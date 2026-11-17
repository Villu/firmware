#include "MeshCoreModule.h"

#if MESHCORE_COEXIST

#include "FSCommon.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerStatus.h"
#include "Router.h"
#include "RoutingModule.h"
#include "airtime.h"
#include "gps/RTC.h"
#include "main.h"
#include "mesh/Channels.h"
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

// The Meshtastic node that stands for a MeshCore node of this name; made or refreshed, and told to the phone
NodeNum touchNode(const char *name, uint8_t hops)
{
    const NodeNum num = nodeNumForName(name);
    meshtastic_User user = meshtastic_User_init_zero;
    snprintf(user.id, sizeof(user.id), "!%08x", (unsigned)num);
    // The node table keeps MC_NODE_NAME_LEN characters of a name: the tag has to be inside them
    snprintf(user.long_name, sizeof(user.long_name), "%.*s%s", (int)(MC_NODE_NAME_LEN - strlen(MC_NAME_TAG)), name, MC_NAME_TAG);
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
    service->sendToPhone(p);
}

class Host : public meshcore_host::Platform, public meshcore_host::Events
{
  public:
    SideChannelRadio *radio = nullptr;

    // ---- Platform: radio ----
    void radioSetParams(float freqMHz, float bwKHz, uint8_t sf, uint8_t cr) override
    {
        SideChannelConfig cfg = {};
        cfg.freqMHz = freqMHz;
        cfg.bwKHz = bwKHz;
        cfg.sf = sf;
        cfg.cr = cr;
        cfg.syncWord = MESHCORE_SYNC_WORD;
        cfg.preamble = sf <= 8 ? 32 : 16; // MeshCore's rule: the fast settings get the long preamble
        if (!radio->sideChannelStart(cfg))
            LOG_ERROR("MeshCore: the radio did not take %.3f MHz SF%u %.1f kHz", freqMHz, sf, bwKHz);
    }
    bool radioSend(const uint8_t *data, size_t len) override
    {
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
        deliverText(touchNode(sender, hops), NODENUM_BROADCAST, (uint8_t)ch, body, snr, hops);
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
        deliverText(touchNode(name, 0), nodeDB->getNodeNum(), ch < 0 ? 0 : (uint8_t)ch, text, snr, 0);
    }
    void onContact(const uint8_t *pubKey, const char *name, uint8_t type, int32_t latE6, int32_t lonE6, bool isNew,
                   uint8_t hops) override
    {
        const NodeNum num = touchNode(name, hops);
        if (latE6 == 0 && lonE6 == 0)
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
    ensureMcChannel();
    if (!meshcore_host::begin(host, &host, FSCom, "/meshcore")) {
        LOG_ERROR("MeshCore: did not start");
        failed = true;
        return;
    }
    meshcore_host::startTcpLink(5000);
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
        LOG_INFO("MeshCore: %u looks (last %u us, max %u us), %u hits, %u frames, %u bad, %u/%u no preamble/header, "
                 "%u put off; sent %u, failed %u; %d contacts, app %s",
                 (unsigned)s.looks, (unsigned)s.lookUsLast, (unsigned)s.lookUsMax, (unsigned)s.cadHits, (unsigned)s.frames,
                 (unsigned)s.crcErrors, (unsigned)s.noPreamble, (unsigned)s.noHeader, (unsigned)s.homeBusy, (unsigned)s.txOk,
                 (unsigned)s.txFail, st.contacts, st.appConnected ? "connected" : "not connected");
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
        // The node's name without the tag is the MeshCore contact's name, or the start of a long one
        const meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(p.to);
        char name[sizeof(node->long_name)];
        strcpy(name, node->long_name);
        name[strlen(name) - strlen(MC_NAME_TAG)] = 0;
        ok = meshcore_host::sendDirectTextByName(name, text);
        LOG_INFO("MeshCore: direct message to %s %s", name, ok ? "queued" : "not sent (no such contact yet)");
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
