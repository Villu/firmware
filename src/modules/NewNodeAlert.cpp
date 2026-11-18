#include "NewNodeAlert.h"

#if MESHCORE_COEXIST
#include "FSCommon.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "SPILock.h"
#include "concurrency/LockGuard.h"
#include "concurrency/OSThread.h"
#include "configuration.h"
#include "main.h"
#if !MESHTASTIC_EXCLUDE_EXTERNALNOTIFICATION
#include "modules/ExternalNotificationModule.h"
#endif

namespace
{
// A new node waits here until its name is known (its node info usually comes with or soon after its first packet), at
// most WAIT_MS. Then: heard directly or a router -> alert; anything else is dropped without a word.
constexpr uint32_t WAIT_MS = 120 * 1000;
constexpr uint32_t MIN_GAP_MS = 15 * 1000; // between two alerts; the next one waits its turn
constexpr int SLOTS = 8;

struct Pending {
    NodeNum num;
    uint32_t since;
    int8_t hops; // -1: not heard yet, or the packet could not say
    float snr;
    bool heard;    // the first packet has been seen
    bool viaMqtt;  // ...and it came through the internet
    bool meshcore; // a MeshCore node
};

Pending pending[SLOTS];
uint32_t lastAlert = 0;

// What an alert does, chosen on the screen (Settings): bit 0 the sound, bit 1 the banner. Kept in a one-character file.
constexpr int MODE_SOUND = 1, MODE_BANNER = 2, MODE_DEFAULT = MODE_SOUND | MODE_BANNER;
const char *const MODE_FILE = "/prefs/new_node_alert";
int mode = -1; // not read yet

int getMode()
{
    if (mode < 0) {
        mode = MODE_DEFAULT;
        concurrency::LockGuard g(spiLock);
        if (FSCom.exists(MODE_FILE)) {
            auto f = FSCom.open(MODE_FILE, FILE_O_READ);
            if (f) {
                const int c = f.read();
                if (c >= '0' && c <= '3')
                    mode = c - '0';
                f.close();
            }
        }
    }
    return mode;
}

void setMode(int m)
{
    mode = m;
    concurrency::LockGuard g(spiLock);
    auto f = FSCom.open(MODE_FILE, FILE_O_WRITE);
    if (f) {
        f.print((char)('0' + m));
        f.close();
    }
}

bool isRouter(meshtastic_Config_DeviceConfig_Role role)
{
    return role == meshtastic_Config_DeviceConfig_Role_ROUTER || role == meshtastic_Config_DeviceConfig_Role_ROUTER_LATE ||
           role == meshtastic_Config_DeviceConfig_Role_ROUTER_CLIENT || role == meshtastic_Config_DeviceConfig_Role_REPEATER;
}

Pending *find(NodeNum n)
{
    for (Pending &p : pending)
        if (p.num == n)
            return &p;
    return nullptr;
}

void alert(const Pending &p, const meshtastic_NodeInfoLite *node, bool direct, bool router)
{
    char name[48];
    if (node && nodeInfoLiteHasUser(node) && node->long_name[0])
        snprintf(name, sizeof(name), "%s (%s)", node->long_name, node->short_name);
    else
        snprintf(name, sizeof(name), "!%08x", (unsigned)p.num);

    char text[160];
    if (direct && router)
        snprintf(text, sizeof(text), "New router nearby: %s, heard directly, snr %.1f", name, p.snr);
    else if (direct && p.meshcore)
        snprintf(text, sizeof(text), "New MeshCore node nearby: %s, heard directly", name);
    else if (direct)
        snprintf(text, sizeof(text), "New node nearby: %s, heard directly, snr %.1f", name, p.snr);
    else {
        const int hops = node && node->has_hops_away ? node->hops_away : p.hops;
        if (hops > 0)
            snprintf(text, sizeof(text), "New router: %s, %d hop%s away", name, hops, hops == 1 ? "" : "s");
        else
            snprintf(text, sizeof(text), "New router: %s", name);
    }
    const int m = getMode();
    LOG_INFO("New node alert (%s): %s", m == 0 ? "off" : m == MODE_SOUND ? "sound" : m == MODE_BANNER ? "banner" : "sound, banner",
             text);
    if (m == 0)
        return;

#if SCREEN_HAS_OWN_QUEUE
    if (service && (m & MODE_BANNER))
        service->showOnScreen(text); // the screen shows a text from this node to itself as an alert box
#endif
#if !MESHTASTIC_EXCLUDE_EXTERNALNOTIFICATION
    // The message sound, as the user set it up (on/off, buzzer, muted): a text "from" the new node to us
    if (externalNotificationModule && (m & MODE_SOUND)) {
        meshtastic_MeshPacket mp = meshtastic_MeshPacket_init_zero;
        mp.from = p.num;
        mp.to = nodeDB->getNodeNum();
        mp.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
        mp.decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
        const size_t len = strnlen(text, sizeof(mp.decoded.payload.bytes));
        memcpy(mp.decoded.payload.bytes, text, len);
        mp.decoded.payload.size = len;
        externalNotificationModule->alertFor(mp);
    }
#endif
}

class Ticker : public concurrency::OSThread
{
  public:
    Ticker() : OSThread("NewNodeAlert") {}

  protected:
    int32_t runOnce() override
    {
        const uint32_t now = millis();
        bool any = false;
        for (Pending &p : pending) {
            if (!p.num)
                continue;
            any = true;
            const meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(p.num);
            if (!node) { // gone again meanwhile
                p.num = 0;
                continue;
            }
            const bool named = nodeInfoLiteHasUser(node);
            if (!named && now - p.since < WAIT_MS)
                continue; // its node info may still come
            const bool direct = p.heard && !p.viaMqtt && p.hops == 0;
            const bool router = named && isRouter(node->role) && !p.viaMqtt;
            if (!direct && !router) {
                p.num = 0;
                continue;
            }
            if (lastAlert && now - lastAlert < MIN_GAP_MS)
                continue; // one at a time
            alert(p, node, direct, router);
            lastAlert = now;
            p.num = 0;
        }
        return any ? 2000 : 10000;
    }
};

Ticker *ticker = nullptr;
} // namespace

/// For the screen's Settings: what a new-node alert does, 0 off, 1 sound, 2 banner, 3 both. set 0-3 changes and keeps
/// it; anything else only reads it.
extern "C" int firmwareNewNodeAlertMode(int set)
{
    if (set >= 0 && set <= 3 && set != getMode())
        setMode(set);
    return getMode();
}

namespace NewNodeAlert
{
void created(NodeNum n)
{
    if (!n || !nodeDB || n == nodeDB->getNodeNum() || find(n))
        return;
    for (Pending &p : pending) {
        if (!p.num) {
            p = Pending{n, millis(), -1, 0, false, false, false};
            if (!ticker)
                ticker = new Ticker();
            else
                ticker->setIntervalFromNow(1000);
            return;
        }
    }
    LOG_DEBUG("New node alert: no room for 0x%08x", (unsigned)n);
}

void heard(NodeNum n, int8_t hops, bool viaMqtt, float snr)
{
    Pending *p = find(n);
    if (!p || p->heard)
        return;
    p->heard = true;
    p->hops = hops;
    p->viaMqtt = viaMqtt;
    p->snr = snr;
}

void heardMeshCore(NodeNum n, uint8_t hops)
{
    Pending *p = find(n);
    if (!p || p->heard)
        return;
    p->heard = true;
    p->meshcore = true;
    p->hops = (int8_t)hops;
}
} // namespace NewNodeAlert
#endif
