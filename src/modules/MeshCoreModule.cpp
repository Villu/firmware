#include "MeshCoreModule.h"

#if MESHCORE_COEXIST

#include "FSCommon.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerStatus.h"
#include "SPILock.h"
#include "concurrency/LockGuard.h"
#include "Router.h"
#include "RoutingModule.h"
#include "Throttle.h"
#include "airtime.h"
#include "gps/RTC.h"
#include "main.h"
#include "meshUtils.h"
#if !MESHTASTIC_EXCLUDE_EXTERNALNOTIFICATION
#include "modules/ExternalNotificationModule.h"
#endif
#include "mesh/Channels.h"
#include "mesh/MeshRadio.h"
#include "mesh/SideChannel.h"
#include "mesh/generated/meshtastic/mesh.pb.h"
#include <MeshCoreHost.h>
#include <esp_random.h>
#include <pb_encode.h>

MeshCoreModule *meshCoreModule;

// The Meshtastic channel that stands for MeshCore's Public channel
static const char *const MC_CHANNEL_NAME = "MeshCore";
// MeshCore's settings in Norway (and its default for the EU and UK): "EU/UK Narrow"
#ifndef MESHCORE_SYNC_WORD
#define MESHCORE_SYNC_WORD 0x12
#endif
// How often this node announces itself to MeshCore neighbours
#ifndef MESHCORE_ADVERT_INTERVAL_MS
#define MESHCORE_ADVERT_INTERVAL_MS (3 * 60 * 60 * 1000UL)
#endif
// The MeshCore phone app's TCP port once Wi-Fi is up; 0 for none (it has no pairing: see OpenLinkGuard in the host)
#ifndef MESHCORE_TCP_PORT
#define MESHCORE_TCP_PORT 5000
#endif
// Written once the "MeshCore" channel has been made: after that, no such channel means the user switched MeshCore off
static const char *const MC_CHANNEL_MADE_FILE = "/meshcore/channel_made";
// A new name costs a slot in Meshtastic's node table, and a name on MeshCore's Public channel costs its sender nothing
#ifndef MESHCORE_NEW_NODES_PER_HOUR
#define MESHCORE_NEW_NODES_PER_HOUR 12
#endif
// Node numbers that stand for MeshCore nodes: this top byte, and a name ending in the tag
static const uint32_t MC_NODE_PREFIX = 0x4D000000;
static const char *const MC_NAME_TAG = " [MC]";
static const size_t MC_NODE_NAME_LEN = sizeof(((meshtastic_NodeInfoLite *)nullptr)->long_name) - 1;

namespace
{

int mcChannelIndex()
{
    for (int i = 0; i < channels.getNumChannels(); i++) {
        const meshtastic_Channel &ch = channels.getByIndex(i);
        if (ch.role != meshtastic_Channel_Role_DISABLED && strcmp(ch.settings.name, MC_CHANNEL_NAME) == 0)
            return i;
    }
    return -1;
}

// Make the "MeshCore" channel in the first free slot. Its key is random and never used on air: nothing sent to this
// channel reaches the Meshtastic radio path, and nothing received there can match it.
int ensureMcChannel()
{
    int idx = mcChannelIndex();
    if (idx >= 0)
        return idx;
    for (int i = 1; i < channels.getNumChannels(); i++) {
        meshtastic_Channel ch = channels.getByIndex(i);
        if (ch.role != meshtastic_Channel_Role_DISABLED)
            continue;
        ch.index = i;
        ch.role = meshtastic_Channel_Role_SECONDARY;
        ch.has_settings = true;
        ch.settings = meshtastic_ChannelSettings_init_zero;
        strncpy(ch.settings.name, MC_CHANNEL_NAME, sizeof(ch.settings.name) - 1);
        ch.settings.psk.size = 32;
        esp_fill_random(ch.settings.psk.bytes, 32);
        channels.setChannel(ch);
        channels.onConfigChanged();
        nodeDB->saveToDisk(SEGMENT_CHANNELS);
        LOG_INFO("MeshCore: made channel %d \"%s\" for MeshCore's Public channel", i, MC_CHANNEL_NAME);
        return i;
    }
    LOG_WARN("MeshCore: no free channel slot for \"%s\"; its messages reach only the MeshCore app", MC_CHANNEL_NAME);
    return -1;
}

NodeNum nodeNumForName(const char *name)
{
    uint32_t h = 2166136261u; // FNV-1a
    for (const char *c = name; *c; c++) {
        h ^= (uint8_t)*c;
        h *= 16777619u;
    }
    return MC_NODE_PREFIX | ((h ^ (h >> 24)) & 0x00FFFFFF);
}

bool newNodeAllowed()
{
    static uint32_t windowStartMs;
    static uint8_t made;
    if (!Throttle::isWithinTimespanMs(windowStartMs, 60 * 60 * 1000UL)) {
        windowStartMs = millis();
        made = 0;
    }
    if (made >= MESHCORE_NEW_NODES_PER_HOUR)
        return false;
    made++;
    return true;
}

// The Meshtastic node that stands for a MeshCore node of this name; made or refreshed, and told to the phone.
// 0 if it would be a new node and this hour's new ones are used up (never with always).
NodeNum touchNode(const char *name, uint8_t hops, bool always = false)
{
    const NodeNum num = nodeNumForName(name);
    if (!always && !nodeDB->getMeshNode(num) && !newNodeAllowed())
        return 0;
    meshtastic_User user = meshtastic_User_init_zero;
    snprintf(user.id, sizeof(user.id), "!%08x", (unsigned)num);
    // The node table keeps MC_NODE_NAME_LEN bytes of a name, and the tag has to be inside them. The name comes off the
    // air: only what a protobuf string may carry, and cut between characters, not inside one.
    char clean[sizeof(user.long_name)];
    strncpy(clean, name, sizeof(clean) - 1);
    clean[sizeof(clean) - 1] = 0;
    sanitizeUtf8(clean, sizeof(clean));
    size_t keep = strlen(clean);
    const size_t room = MC_NODE_NAME_LEN - strlen(MC_NAME_TAG);
    if (keep > room) {
        keep = room;
        while (keep > 0 && ((uint8_t)clean[keep] & 0xC0) == 0x80)
            keep--;
    }
    snprintf(user.long_name, sizeof(user.long_name), "%.*s%s", (int)keep, clean, MC_NAME_TAG);
    size_t n = 0;
    for (const char *c = name; *c && n < 4; c++) // short names are plain ASCII
        if ((uint8_t)*c > 0x20 && (uint8_t)*c < 0x7F)
            user.short_name[n++] = *c;
    if (n == 0)
        strcpy(user.short_name, "MC");
    user.hw_model = meshtastic_HardwareModel_PRIVATE_HW;
    user.role = meshtastic_Config_DeviceConfig_Role_CLIENT;

    const int ch = mcChannelIndex();
    const bool changed = nodeDB->updateUser(num, user, ch < 0 ? 0 : (uint8_t)ch);
    if (meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(num)) {
        node->last_heard = getValidTime(RTCQualityFromNet);
        node->has_hops_away = true;
        node->hops_away = hops;
    }
    if (changed) {
        if (meshtastic_MeshPacket *p = packetPool.allocZeroed()) {
            p->from = num;
            p->to = NODENUM_BROADCAST;
            p->id = generatePacketId();
            p->channel = ch < 0 ? 0 : (uint8_t)ch;
            p->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
            p->decoded.portnum = meshtastic_PortNum_NODEINFO_APP;
            p->decoded.payload.size =
                pb_encode_to_bytes(p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes), &meshtastic_User_msg, &user);
            p->rx_time = getValidTime(RTCQualityFromNet);
            p->hop_start = hops;
            service->sendToPhone(p);
        }
    }
    return num;
}

void deliverText(NodeNum from, NodeNum to, uint8_t channel, const char *text, float snr, uint8_t hops)
{
    meshtastic_MeshPacket *p = packetPool.allocZeroed();
    if (!p)
        return;
    p->from = from;
    p->to = to;
    p->id = generatePacketId();
    p->channel = channel;
    p->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    const size_t len = strnlen(text, sizeof(p->decoded.payload.bytes));
    memcpy(p->decoded.payload.bytes, text, len);
    p->decoded.payload.size = len;
    p->rx_time = getValidTime(RTCQualityFromNet);
    p->rx_snr = snr;
    p->hop_start = hops;
    p->transport_mechanism = meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA;
#if !MESHTASTIC_EXCLUDE_EXTERNALNOTIFICATION
    // The sound, light or buzzer a Meshtastic text gets: that module only sees what the router hands it
    if (externalNotificationModule)
        externalNotificationModule->alertFor(*p);
#endif
    service->sendToPhone(p);
}

class Host : public meshcore_host::Platform, public meshcore_host::Events
{
  public:
    SideChannelRadio *radio = nullptr;
    bool enabled = true; // false: the radio stays on Meshtastic (see the class comment in MeshCoreModule.h)

    void setEnabled(bool on)
    {
        enabled = on;
        if (!on)
            radio->sideChannelStop();
        else if (haveCfg)
            radioSetParams(lastFreqMHz, lastBwKHz, lastSf, lastCr);
    }

    // ---- Platform: radio ----
    static SideChannelConfig sideConfig(float freqMHz, float bwKHz, uint8_t sf, uint8_t cr)
    {
        SideChannelConfig cfg = {};
        cfg.freqMHz = freqMHz;
        cfg.bwKHz = bwKHz;
        cfg.sf = sf;
        cfg.cr = cr;
        cfg.syncWord = MESHCORE_SYNC_WORD;
        cfg.preamble = sf <= 8 ? 32 : 16; // MeshCore's rule: the fast settings get the long preamble
        return cfg;
    }
    bool radioParamsAllowed(float freqMHz, float bwKHz, uint8_t sf, uint8_t cr) override
    {
        // MeshCore goes out through Meshtastic's radio at Meshtastic's power: only inside Meshtastic's region
        if (!myRegion || freqMHz - bwKHz / 2000.0f < myRegion->freqStart || freqMHz + bwKHz / 2000.0f > myRegion->freqEnd)
            return false;
        return radio && radio->sideChannelFits(sideConfig(freqMHz, bwKHz, sf, cr));
    }
    void radioSetParams(float freqMHz, float bwKHz, uint8_t sf, uint8_t cr) override
    {
        const SideChannelConfig cfg = sideConfig(freqMHz, bwKHz, sf, cr);
        lastFreqMHz = freqMHz;
        lastBwKHz = bwKHz;
        lastSf = sf;
        lastCr = cr;
        haveCfg = true;
        if (!enabled)
            return; // kept for when it is switched on
        if (!radioParamsAllowed(freqMHz, bwKHz, sf, cr))
            LOG_ERROR("MeshCore: %.3f MHz SF%u %.1f kHz is outside the region's band or does not fit beside Meshtastic",
                      freqMHz, sf, bwKHz);
        else if (!radio->sideChannelStart(cfg))
            LOG_ERROR("MeshCore: the radio did not take %.3f MHz SF%u %.1f kHz", freqMHz, sf, bwKHz);
    }
    bool radioSend(const uint8_t *data, size_t len) override
    {
        // Both networks transmit in the same band and share its duty cycle. Meshtastic's router stops at the limit;
        // MeshCore's own airtime budget knows nothing of Meshtastic's share, so the same test is made here.
        const float limit = getEffectiveDutyCycle();
        if (!config.lora.override_duty_cycle && limit < 100 && airTime->utilizationTXPercent() > limit) {
            static uint32_t lastLogMs;
            if (!lastLogMs || !Throttle::isWithinTimespanMs(lastLogMs, 60 * 1000UL)) {
                LOG_WARN("MeshCore: not sent, this hour's share of the band (%.0f %%) is used up", limit);
                lastLogMs = millis() | 1;
            }
            return false;
        }
        if (!radio->sideChannelSend(data, len))
            return false;
        sendingMs = radio->sideChannelAirtimeMs(len);
        return true;
    }
    bool radioTakeSendResult(bool &ok) override
    {
        if (!radio->sideChannelTakeSendResult(ok))
            return false;
        if (ok)
            airTime->logAirtime(TX_LOG, sendingMs); // both networks share the band's duty cycle
        return true;
    }
    void radioCancelSend() override { radio->sideChannelCancelSend(); }
    int radioRecv(uint8_t *buf, size_t max, float &snr, float &rssi) override
    {
        static SideChannelFrame frame; // main loop only
        if (!radio->sideChannelRecv(frame))
            return 0;
        const size_t len = frame.len < max ? frame.len : max;
        memcpy(buf, frame.data, len);
        snr = frame.snr;
        rssi = frame.rssi;
        airTime->logAirtime(RX_ALL_LOG, radio->sideChannelAirtimeMs(len));
        return (int)len;
    }
    bool radioBusy() override { return radio->sideChannelReceiving(); }
    uint32_t radioAirtimeMs(size_t len) override { return radio->sideChannelAirtimeMs(len); }
    int8_t radioTxPower() override { return radio->sideChannelTxPower(); }

    // ---- Platform: clock and board ----
    uint32_t getTime() override { return getValidTime(RTCQualityFromNet); }
    void setTime(uint32_t epoch) override
    {
        struct timeval tv = {(time_t)epoch, 0};
        perhapsSetRTC(RTCQualityFromNet, &tv);
    }
    uint16_t battMilliVolts() override { return powerStatus ? (uint16_t)powerStatus->getBatteryVoltageMv() : 0; }
    const char *boardName() override { return "LilyGo T-Deck (Meshtastic + MeshCore)"; }
    void reboot() override { rebootAtMsec = millis() + 2000; }
    bool getPosition(double &lat, double &lon) override
    {
        if (localPosition.latitude_i == 0 && localPosition.longitude_i == 0)
            return false;
        lat = localPosition.latitude_i * 1e-7;
        lon = localPosition.longitude_i * 1e-7;
        return true;
    }
    const char *defaultNodeName() override { return owner.long_name; }
    void log(const char *line) override { LOG_INFO("%s", line); }
    uint32_t storageTotalKb() override { return FSCom.totalBytes() / 1024; }
    uint32_t storageUsedKb() override { return FSCom.usedBytes() / 1024; }

    // ---- Events: MeshCore traffic shown through Meshtastic ----
    void onChannelMessage(uint8_t channelIdx, const char *channelName, const char *text, uint32_t timestamp, float snr,
                          uint8_t hops) override
    {
        const int ch = mcChannelIndex();
        if (channelIdx != 0 || ch < 0) // only Public has a Meshtastic channel; the others reach the MeshCore app
            return;
        // MeshCore group text is "Sender: message"
        char sender[32] = "MeshCore";
        const char *body = text;
        if (const char *sep = strstr(text, ": ")) {
            const size_t n = (size_t)(sep - text);
            if (n > 0 && n < sizeof(sender)) {
                memcpy(sender, text, n);
                sender[n] = 0;
                body = sep + 2;
            }
        }
        NodeNum from = touchNode(sender, hops);
        if (!from) {
            // Too many new names this hour: under the one node that stands for the channel, with the name in the text
            from = touchNode("MeshCore", hops, true);
            body = text;
        }
        deliverText(from, NODENUM_BROADCAST, (uint8_t)ch, body, snr, hops);
    }
    void onChannelMessageSent(uint8_t channelIdx, const char *text) override
    {
        // Typed in the MeshCore app: show it on the screen and in the Meshtastic app too, as this node's own
        const int ch = mcChannelIndex();
        if (channelIdx == 0 && ch >= 0)
            deliverText(nodeDB->getNodeNum(), NODENUM_BROADCAST, (uint8_t)ch, text, 0, 0);
    }
    void onDirectMessage(const uint8_t *pubKey, const char *name, const char *text, uint32_t timestamp, float snr) override
    {
        const int ch = mcChannelIndex();
        // From a contact: someone this node exchanged keys with, so always shown
        deliverText(touchNode(name, 0, true), nodeDB->getNodeNum(), ch < 0 ? 0 : (uint8_t)ch, text, snr, 0);
    }
    void onContact(const uint8_t *pubKey, const char *name, uint8_t type, int32_t latE6, int32_t lonE6, bool isNew,
                   uint8_t hops) override
    {
        const NodeNum num = touchNode(name, hops);
        if (!num || (latE6 == 0 && lonE6 == 0)) // not shown this hour (it is in MeshCore's own contact list), or no position
            return;
        meshtastic_Position pos = meshtastic_Position_init_zero;
        pos.has_latitude_i = pos.has_longitude_i = true;
        pos.latitude_i = latE6 * 10;
        pos.longitude_i = lonE6 * 10;
        pos.time = getValidTime(RTCQualityFromNet);
        pos.location_source = meshtastic_Position_LocSource_LOC_EXTERNAL;
        nodeDB->updatePosition(num, pos);
        if (meshtastic_MeshPacket *p = packetPool.allocZeroed()) {
            const int ch = mcChannelIndex();
            p->from = num;
            p->to = NODENUM_BROADCAST;
            p->id = generatePacketId();
            p->channel = ch < 0 ? 0 : (uint8_t)ch;
            p->which_payload_variant = meshtastic_MeshPacket_decoded_tag;
            p->decoded.portnum = meshtastic_PortNum_POSITION_APP;
            p->decoded.payload.size =
                pb_encode_to_bytes(p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes), &meshtastic_Position_msg, &pos);
            p->rx_time = pos.time;
            p->hop_start = hops;
            service->sendToPhone(p);
        }
    }

  private:
    uint32_t sendingMs = 0;
    float lastFreqMHz = 0, lastBwKHz = 0;
    uint8_t lastSf = 0, lastCr = 0;
    bool haveCfg = false;
};

Host host;

} // namespace

MeshCoreModule::MeshCoreModule() : concurrency::OSThread("MeshCore") {}

bool MeshCoreModule::isMeshCoreNode(NodeNum n)
{
    if ((n & 0xFF000000) != MC_NODE_PREFIX)
        return false;
    const meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(n);
    if (!node)
        return false;
    const size_t len = strlen(node->long_name), tag = strlen(MC_NAME_TAG);
    return len > tag && strcmp(node->long_name + len - tag, MC_NAME_TAG) == 0;
}

void MeshCoreModule::start()
{
    host.radio = SideChannelRadio::instance;
    if (!host.radio) {
        LOG_WARN("MeshCore: this radio cannot share its time with a second network, MeshCore is off");
        failed = true;
        return;
    }
    {
        concurrency::LockGuard g(spiLock);
        channelWasMade = FSCom.exists(MC_CHANNEL_MADE_FILE);
    }
    int ch = mcChannelIndex();
    if (ch < 0 && !channelWasMade)
        ch = ensureMcChannel();
    // Made once and gone now: switched off. (Never made for want of a free slot: on, for the MeshCore app alone.)
    host.enabled = ch >= 0 || !channelWasMade;
    if (!meshcore_host::begin(host, &host, FSCom, "/meshcore")) {
        LOG_ERROR("MeshCore: did not start");
        failed = true;
        return;
    }
    if (ch >= 0 && !channelWasMade) {
        concurrency::LockGuard g(spiLock);
        auto f = FSCom.open(MC_CHANNEL_MADE_FILE, FILE_O_WRITE);
        if (f) {
            f.print("1");
            f.close();
            channelWasMade = true;
        }
    }
    if (!host.enabled)
        LOG_INFO("MeshCore: off, there is no channel named \"%s\"; add one to switch it on", MC_CHANNEL_NAME);
    if (MESHCORE_TCP_PORT)
        meshcore_host::startTcpLink(MESHCORE_TCP_PORT);
    started = true;
    nextAdvertMs = millis() + 30 * 1000;
    nextStatsMs = millis() + 60 * 1000;
}

int32_t MeshCoreModule::runOnce()
{
    if (failed)
        return disable();
    if (!started) {
        // The radio is set up after the modules; give it and the first NodeDB save a moment
        if (millis() < 8000)
            return 500;
        start();
        return 100;
    }

    meshcore_host::loop();

    const uint32_t now = millis();
    if ((int32_t)(now - nextSwitchMs) >= 0) {
        nextSwitchMs = now + 2000;
        const bool want = mcChannelIndex() >= 0 || !channelWasMade;
        if (want != host.enabled) {
            host.setEnabled(want);
            LOG_INFO("MeshCore: switched %s (%s channel named \"%s\")", want ? "on" : "off", want ? "there is a" : "no",
                     MC_CHANNEL_NAME);
            if (want)
                nextAdvertMs = now + 5000;
        }
    }
    if (!host.enabled)
        nextAdvertMs = now + 5000; // nothing to announce on
    const bool clockKnown = getValidTime(RTCQualityFromNet) != 0;
    if ((int32_t)(now - nextAdvertMs) >= 0) {
        // A node ignores an advert stamped older than the last one it kept from us, and without a clock ours is stamped
        // 2024: wait for the clock (GPS, a phone, NTP) at first, but not for ever.
        if (!clockKnown && now < 10 * 60 * 1000UL) {
            nextAdvertMs = now + 5000;
        } else {
            nextAdvertMs = now + MESHCORE_ADVERT_INTERVAL_MS;
            advertWithoutClock = !clockKnown;
            LOG_INFO("MeshCore: advert %s%s", meshcore_host::sendAdvert(false) ? "queued" : "not sent",
                     clockKnown ? "" : " (clock not set)");
        }
    } else if (advertWithoutClock && clockKnown) {
        advertWithoutClock = false;
        nextAdvertMs = now + 5000; // again, now with a time others accept
    }
    if ((int32_t)(now - nextStatsMs) >= 0) {
        nextStatsMs = now + 5 * 60 * 1000;
        SideChannelStats s;
        host.radio->sideChannelGetStats(s);
        meshcore_host::Status st;
        meshcore_host::getStatus(st);
        // Two lines: the log cuts a long one
        LOG_INFO("MeshCore: %u looks (last %u us, max %u us, %u over 20 ms, scan %u us, gap max %u ms, %u over 100 ms), "
                 "%u put off",
                 (unsigned)s.looks, (unsigned)s.lookUsLast, (unsigned)s.lookUsMax, (unsigned)s.looksLong,
                 (unsigned)s.cadUsLast, (unsigned)s.gapMsMax, (unsigned)s.gapsLong, (unsigned)s.homeBusy);
        LOG_INFO("MeshCore: %u hits, %u frames (preamble after %u ms, max %u), %u bad, %u no header, %u no preamble "
                 "(%u of them strong, last %d dBm); sent %u, failed %u; %d contacts, app %s",
                 (unsigned)s.cadHits, (unsigned)s.frames, (unsigned)s.preambleMsLast, (unsigned)s.preambleMsMax,
                 (unsigned)s.crcErrors, (unsigned)s.noHeader, (unsigned)s.noPreamble, (unsigned)s.noPreambleStrong,
                 (int)s.noPreambleRssi, (unsigned)s.txOk, (unsigned)s.txFail, st.contacts,
                 st.appConnected ? "connected" : "not connected");
        LOG_INFO("MeshCore: frames by how far into the preamble the scan began, 10 ms steps from before it: "
                 "%u %u %u %u %u %u %u %u %u %u %u %u %u %u",
                 s.phaseHist[0], s.phaseHist[1], s.phaseHist[2], s.phaseHist[3], s.phaseHist[4], s.phaseHist[5],
                 s.phaseHist[6], s.phaseHist[7], s.phaseHist[8], s.phaseHist[9], s.phaseHist[10], s.phaseHist[11],
                 s.phaseHist[12], s.phaseHist[13]);
    }
    {
        // A frame that was there and did not arrive, said when it happens
        static uint32_t lostSeen = 0, framesSeen = 0, clearSeen = 0;
        SideChannelStats s;
        host.radio->sideChannelGetStats(s);
        const uint32_t lost = s.crcErrors + s.noHeader + s.noPreambleStrong;
        if (lost != lostSeen) {
            lostSeen = lost;
            LOG_DEBUG("MeshCore: no frame from a hit: %u bad, %u no header, %u strong with no preamble (%d dBm), scan at "
                      "%u ms, %u after the one before",
                      (unsigned)s.crcErrors, (unsigned)s.noHeader, (unsigned)s.noPreambleStrong, (int)s.noPreambleRssi,
                      (unsigned)s.strongAtMs, (unsigned)s.strongGapMs);
        }
        if (s.clearAtMs != clearSeen) {
            clearSeen = s.clearAtMs;
            LOG_DEBUG("MeshCore: channel clear again at the scan at %u ms", (unsigned)s.clearAtMs);
        }
        if (s.frames != framesSeen) {
            framesSeen = s.frames;
            LOG_DEBUG("MeshCore: frame found by a scan %d ms into its preamble, preamble flag %u ms after the hit",
                      (int)s.phaseMsLast, (unsigned)s.preambleMsLast);
        }
    }
    return 15;
}

bool MeshCoreModule::handleFromPhone(meshtastic_MeshPacket &p)
{
    if (!started || p.which_payload_variant != meshtastic_MeshPacket_decoded_tag)
        return false;
    const int ch = mcChannelIndex();
    const bool toChannel = ch >= 0 && p.channel == (uint8_t)ch && (p.to == NODENUM_BROADCAST || p.to == 0);
    const bool toNode = isMeshCoreNode(p.to);
    if (!toChannel && !toNode)
        return false;
    // From here on the packet is MeshCore's: whatever happens, it must not go out as Meshtastic
    if (p.decoded.portnum != meshtastic_PortNum_TEXT_MESSAGE_APP)
        return true;

    char text[sizeof(p.decoded.payload.bytes) + 1];
    memcpy(text, p.decoded.payload.bytes, p.decoded.payload.size);
    text[p.decoded.payload.size] = 0;

    bool ok;
    if (toNode) {
        // The node's number was made from the MeshCore name in full: the contact whose name gives the same number.
        // (The node's own long name may be a cut or cleaned-up copy, so it is not what to search by.)
        NodeNum target = p.to;
        ok = meshcore_host::sendDirectTextWhere(
            [](const char *name, const uint8_t *, void *ctx) { return nodeNumForName(name) == *(const NodeNum *)ctx; }, &target,
            text);
        LOG_INFO("MeshCore: direct message to !%08x %s", (unsigned)p.to, ok ? "queued" : "not sent (no such contact yet)");
    } else {
        ok = meshcore_host::sendChannelText(0, text);
        LOG_INFO("MeshCore: message to Public %s", ok ? "queued" : "not sent");
    }
    // Tell the sender how it went, as a routing answer from this node
    if (routingModule)
        routingModule->sendAckNak(ok ? meshtastic_Routing_Error_NONE : meshtastic_Routing_Error_NO_RESPONSE, nodeDB->getNodeNum(),
                                  p.id, p.channel);
    return true;
}

#endif
