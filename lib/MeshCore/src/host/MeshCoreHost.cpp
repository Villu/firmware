#include <target.h>

#include "../companion/MyMesh.h"
#include <FS.h>
#include <FSImpl.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <helpers/MultiSerialInterface.h>
#include <helpers/wifi/SerialWifiInterface.h>
#include <new>
#include <stdarg.h>

namespace meshcore_host
{

Platform *platform = nullptr;

namespace
{
Events *events = nullptr;
MyMesh *the_mesh = nullptr;
DataStore *store = nullptr;
HostRNG mesh_rng;
SimpleMeshTables *tables = nullptr;
MultiSerialInterface links;
int queued_for_app = 0;

void logf(const char *fmt, ...)
{
    if (!platform)
        return;
    char line[200];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    platform->log(line);
}

// ---- storage: the host's file system, with every MeshCore path under one directory ----

struct FileImplOf : public fs::File {
    explicit FileImplOf(const fs::File &f) : fs::File(f) {}
    fs::FileImplPtr impl() const { return _p; }
};

class SubdirFS : public fs::FSImpl
{
    fs::FS &_fs;
    String _dir;

    String full(const char *path) const
    {
        String s = _dir;
        if (path[0] != '/')
            s += "/";
        s += path;
        return s;
    }

  public:
    SubdirFS(fs::FS &fs, const char *dir) : _fs(fs), _dir(dir) {}
    fs::FileImplPtr open(const char *path, const char *mode, const bool create) override
    {
        return FileImplOf(_fs.open(full(path), mode, create)).impl();
    }
    bool exists(const char *path) override { return _fs.exists(full(path)); }
    bool rename(const char *from, const char *to) override { return _fs.rename(full(from), full(to)); }
    bool remove(const char *path) override { return _fs.remove(full(path)); }
    bool mkdir(const char *path) override { return _fs.mkdir(full(path)); }
    bool rmdir(const char *path) override { return _fs.rmdir(full(path)); }
};

fs::FS *host_fs = nullptr;
String host_dir;
fs::FS *mc_fs = nullptr;

bool eraseDir(const String &dir, int depth)
{
    fs::File root = host_fs->open(dir);
    if (!root || !root.isDirectory())
        return false;
    bool ok = true;
    for (fs::File f = root.openNextFile(); f; f = root.openNextFile()) {
        const String path = dir + "/" + f.name();
        const bool isDir = f.isDirectory();
        f.close();
        if (isDir) {
            if (depth < 3)
                ok = eraseDir(path, depth + 1) && ok;
            ok = host_fs->rmdir(path) && ok;
        } else {
            ok = host_fs->remove(path) && ok;
        }
    }
    return ok;
}

// ---- the phone app's links ----

class LinkAdapter : public BaseSerialInterface
{
    AppLink *_link;
    bool _enabled = false;

  public:
    explicit LinkAdapter(AppLink *link) : _link(link) {}
    void enable() override { _enabled = true; }
    void disable() override { _enabled = false; }
    bool isEnabled() const override { return _enabled; }
    bool isConnected() const override { return _enabled && _link->isConnected(); }
    void loop() override { _link->loop(); }
    bool isWriteBusy() const override { return !_link->canWrite(); }
    size_t writeFrame(const uint8_t src[], size_t len) override { return _link->writeFrame(src, len); }
    size_t checkRecvFrame(uint8_t dest[]) override { return _enabled ? _link->readFrame(dest) : 0; }
};

// A link without pairing or encryption (TCP): anyone on the network can open it. It carries the app's everyday traffic,
// but not the commands that hand out or replace the node's identity, sign with it, wipe or restart the node, or run
// its command line. Those are answered "disabled", as a build without them would.
class OpenLinkGuard : public BaseSerialInterface
{
    BaseSerialInterface *_inner;

    // Command and response codes of the companion protocol (companion/MyMesh.cpp)
    static bool refused(uint8_t cmd)
    {
        switch (cmd) {
        case 19: // CMD_REBOOT
        case 23: // CMD_EXPORT_PRIVATE_KEY
        case 24: // CMD_IMPORT_PRIVATE_KEY
        case 33: // CMD_SIGN_START
        case 34: // CMD_SIGN_DATA
        case 35: // CMD_SIGN_FINISH
        case 37: // CMD_SET_DEVICE_PIN
        case 51: // CMD_FACTORY_RESET
        case 66: // CMD_RUN_CLI_COMMAND
            return true;
        default:
            return false;
        }
    }

  public:
    explicit OpenLinkGuard(BaseSerialInterface *inner) : _inner(inner) {}
    void enable() override { _inner->enable(); }
    void disable() override { _inner->disable(); }
    bool isEnabled() const override { return _inner->isEnabled(); }
    bool isConnected() const override { return _inner->isConnected(); }
    void loop() override { _inner->loop(); }
    bool isWriteBusy() const override { return _inner->isWriteBusy(); }
    size_t writeFrame(const uint8_t src[], size_t len) override { return _inner->writeFrame(src, len); }
    size_t checkRecvFrame(uint8_t dest[]) override
    {
        const size_t len = _inner->checkRecvFrame(dest);
        if (len == 0 || !refused(dest[0]))
            return len;
        logf("MeshCore: command %u refused on the TCP link (Bluetooth only)", dest[0]);
        const uint8_t disabled[1] = {15}; // RESP_CODE_DISABLED
        _inner->writeFrame(disabled, sizeof(disabled));
        return 0;
    }
};

LinkAdapter *ble_adapter = nullptr;
SerialWifiInterface *tcp_link = nullptr;
uint16_t tcp_port = 0;
bool tcp_started = false;

// ---- what the mesh tells the host ----

class HostListener : public MyMesh::Listener
{
  public:
    void onMessageRecv(mesh::Packet *pkt, const ContactInfo &from, uint8_t txt_type, uint32_t sender_timestamp,
                       const char *text) override
    {
        logf("MeshCore: direct message from %s: %s", from.name, text);
        if (events && txt_type == TXT_TYPE_PLAIN)
            events->onDirectMessage(from.id.pub_key, from.name, text, sender_timestamp, pkt->getSNR());
    }
    void onChannelMessageRecv(mesh::Packet *pkt, ChannelDetails &channel, const char *text) override
    {
        const int idx = the_mesh->findChannelIdx(channel.channel);
        const uint8_t hops = pkt->isRouteFlood() ? (uint8_t)(pkt->path_len & 63) : 0;
        logf("MeshCore: [%s] %s (SNR %.1f, %u hops)", channel.name, text, pkt->getSNR(), hops);
        if (events)
            events->onChannelMessage(idx < 0 ? 0xFF : (uint8_t)idx, channel.name, text,
                                     the_mesh->getRTCClock()->getCurrentTime(), pkt->getSNR(), hops);
    }
    void onQueueSizeChanged(int msgcount) override { queued_for_app = msgcount; }
    void onChannelMessageSent(uint8_t channel_idx, const char *text) override
    {
        if (events)
            events->onChannelMessageSent(channel_idx, text);
    }
    void onDiscoveredContact(ContactInfo &contact, bool is_new, uint8_t path_len, const uint8_t *path) override
    {
        logf("MeshCore: advert from %s (type %u, %u hops)%s", contact.name, contact.type, path_len & 63,
             is_new ? ", new" : "");
        if (events)
            events->onContact(contact.id.pub_key, contact.name, contact.type, contact.gps_lat, contact.gps_lon, is_new,
                              path_len & 63);
    }
};

HostListener listener;

// The mesh object holds the contact table and the offline message queue: far too big for internal RAM
void *bigAlloc(size_t size)
{
    void *mem = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return mem ? mem : malloc(size);
}
} // namespace

bool begin(Platform &p, Events *e, fs::FS &storage, const char *dir)
{
    if (the_mesh)
        return true;
    platform = &p;
    events = e;
    host_fs = &storage;
    host_dir = dir;
    if (!storage.exists(dir))
        storage.mkdir(dir);
    mc_fs = new fs::FS(fs::FSImplPtr(new SubdirFS(storage, dir)));

    store = new DataStore(*mc_fs, rtc_clock);
    void *tablesMem = bigAlloc(sizeof(SimpleMeshTables));
    void *meshMem = bigAlloc(sizeof(MyMesh));
    if (!store || !tablesMem || !meshMem) {
        logf("MeshCore: no memory for the mesh (%u bytes)", (unsigned)sizeof(MyMesh));
        return false;
    }
    tables = new (tablesMem) SimpleMeshTables();
    the_mesh = new (meshMem) MyMesh(radio_driver, mesh_rng, rtc_clock, *tables, *store);

    store->begin();
    the_mesh->begin();

    // A node that was never named carries the first bytes of its key as a name: give it the host's
    NodePrefs *prefs = the_mesh->getNodePrefs();
    char keyName[10];
    mesh::Utils::toHex(keyName, the_mesh->self_id.pub_key, 4);
    if (strcmp(prefs->node_name, keyName) == 0) {
        const char *name = platform->defaultNodeName();
        if (name && name[0]) {
            strncpy(prefs->node_name, name, sizeof(prefs->node_name) - 1);
            prefs->node_name[sizeof(prefs->node_name) - 1] = 0;
            the_mesh->savePrefs();
        }
    }

    the_mesh->setBLEPin(0);
    the_mesh->setListener(&listener);
    the_mesh->startInterface(links);

    char key[2 * 8 + 1];
    mesh::Utils::toHex(key, the_mesh->self_id.pub_key, 8);
    logf("MeshCore: node \"%s\", key %s..., %d contacts, %.3f MHz SF%u %.1f kHz CR 4/%u", prefs->node_name, key,
         the_mesh->getNumContacts(), prefs->freq, prefs->sf, prefs->bw, prefs->cr);
    return true;
}

bool running()
{
    return the_mesh != nullptr;
}

void loop()
{
    if (!the_mesh)
        return;
    // The TCP link opens once the host's Wi-Fi is up
    if (tcp_port && !tcp_started && WiFi.status() == WL_CONNECTED) {
        tcp_link = new SerialWifiInterface();
        tcp_link->begin(tcp_port);
        links.addInterface(InterfaceType::WiFi, new OpenLinkGuard(tcp_link));
        tcp_link->enable();
        tcp_started = true;
        logf("MeshCore: app link on TCP port %u", tcp_port);
    }
    static uint32_t lastPos = 0;
    if (millis() - lastPos > 30000) {
        lastPos = millis();
        double lat = 0, lon = 0;
        if (platform->getPosition(lat, lon)) {
            sensors.node_lat = lat;
            sensors.node_lon = lon;
        }
    }
    the_mesh->loop();
    links.loop();
    rtc_clock.tick();
}

void setBleLink(AppLink *link)
{
    if (ble_adapter || !link)
        return;
    ble_adapter = new LinkAdapter(link);
    links.addInterface(InterfaceType::Bluetooth, ble_adapter);
    if (links.isEnabled())
        ble_adapter->enable();
}

void startTcpLink(uint16_t port)
{
    tcp_port = port;
}

void setBlePin(uint32_t pin)
{
    if (the_mesh)
        the_mesh->setBLEPin(pin);
}

bool sendChannelText(uint8_t channelIdx, const char *text)
{
    if (!the_mesh || !text)
        return false;
    ChannelDetails channel;
    if (!the_mesh->getChannel(channelIdx, channel))
        return false;
    const uint32_t now = the_mesh->getRTCClock()->getCurrentTimeUnique();
    return the_mesh->sendGroupMessage(now, channel.channel, the_mesh->getNodePrefs()->node_name, text, strlen(text));
}

bool sendDirectText(const uint8_t *pubKeyPrefix, size_t prefixLen, const char *text)
{
    if (!the_mesh || !text)
        return false;
    ContactInfo *contact = the_mesh->lookupContactByPubKey(pubKeyPrefix, (int)prefixLen);
    if (!contact)
        return false;
    uint32_t expectedAck = 0, estTimeout = 0;
    const uint32_t now = the_mesh->getRTCClock()->getCurrentTimeUnique();
    return the_mesh->sendMessage(*contact, now, 0, text, expectedAck, estTimeout) != MSG_SEND_FAILED;
}

bool sendDirectTextByName(const char *name, const char *text)
{
    if (!the_mesh || !name || !text)
        return false;
    ContactInfo *contact = the_mesh->searchContactsByPrefix(name);
    if (!contact)
        return false;
    uint32_t expectedAck = 0, estTimeout = 0;
    const uint32_t now = the_mesh->getRTCClock()->getCurrentTimeUnique();
    return the_mesh->sendMessage(*contact, now, 0, text, expectedAck, estTimeout) != MSG_SEND_FAILED;
}

bool sendAdvert(bool flood)
{
    if (!the_mesh)
        return false;
    if (!flood)
        return the_mesh->advert();
    NodePrefs *prefs = the_mesh->getNodePrefs();
    mesh::Packet *pkt = prefs->advert_loc_policy == ADVERT_LOC_NONE
                            ? the_mesh->createSelfAdvert(prefs->node_name)
                            : the_mesh->createSelfAdvert(prefs->node_name, sensors.node_lat, sensors.node_lon);
    if (!pkt)
        return false;
    the_mesh->sendFlood(pkt);
    return true;
}

bool setNodeName(const char *name)
{
    if (!the_mesh || !name || !name[0])
        return false;
    NodePrefs *prefs = the_mesh->getNodePrefs();
    strncpy(prefs->node_name, name, sizeof(prefs->node_name) - 1);
    prefs->node_name[sizeof(prefs->node_name) - 1] = 0;
    the_mesh->savePrefs();
    return true;
}

void getStatus(Status &out)
{
    memset(&out, 0, sizeof(out));
    if (!the_mesh)
        return;
    const NodePrefs *prefs = the_mesh->getNodePrefs();
    strncpy(out.nodeName, prefs->node_name, sizeof(out.nodeName) - 1);
    memcpy(out.pubKey, the_mesh->self_id.pub_key, sizeof(out.pubKey));
    out.freqMHz = radio_driver.getFreq();
    out.bwKHz = radio_driver.getBw();
    out.sf = radio_driver.getSf();
    out.cr = radio_driver.getCr();
    out.packetsRecv = radio_driver.getPacketsRecv();
    out.packetsSent = radio_driver.getPacketsSent();
    out.contacts = the_mesh->getNumContacts();
    out.queuedForApp = queued_for_app;
    out.appConnected = links.isConnected();
}

// Storage hooks for the vendored DataStore (see MESHCORE_HOSTED there)
uint32_t storageUsedKb()
{
    return platform ? platform->storageUsedKb() : 0;
}
uint32_t storageTotalKb()
{
    return platform ? platform->storageTotalKb() : 0;
}
bool storageErase()
{
    return host_fs && eraseDir(host_dir, 0);
}

} // namespace meshcore_host

// Asked by the vendored MyMesh before it stores radio settings from the app (see MESHCORE_HOSTED there)
bool mc_host_radio_params_ok(float freq_mhz, float bw_khz, uint8_t sf, uint8_t cr)
{
    return !meshcore_host::platform || meshcore_host::platform->radioParamsAllowed(freq_mhz, bw_khz, sf, cr);
}

uint32_t mc_host_storage_used_kb()
{
    return meshcore_host::storageUsedKb();
}

uint32_t mc_host_storage_total_kb()
{
    return meshcore_host::storageTotalKb();
}

bool mc_host_storage_erase()
{
    return meshcore_host::storageErase();
}
