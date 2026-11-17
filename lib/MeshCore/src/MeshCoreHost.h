#pragma once

// MeshCore's companion firmware running inside another firmware.
//
// This header is the whole boundary: it names no MeshCore type and no type of the host, so each side compiles against
// its own headers only. The host implements Platform (radio, clock, board) and, if it wants to show traffic itself,
// Events; it may add AppLinks, over which the MeshCore phone app talks the ordinary companion protocol.

#include <stddef.h>
#include <stdint.h>

namespace fs
{
class FS;
}

namespace meshcore_host
{

/// What the host firmware supplies
class Platform
{
  public:
    // --- radio: one LoRa channel that carries whole frames ---
    /// Tune the MeshCore channel
    virtual void radioSetParams(float freqMHz, float bwKHz, uint8_t sf, uint8_t cr) = 0;
    /// Hand over one frame to send; false if it cannot be taken now
    virtual bool radioSend(const uint8_t *data, size_t len) = 0;
    /// True once per frame given to radioSend(), when it has left the radio or failed
    virtual bool radioTakeSendResult(bool &ok) = 0;
    /// Forget a frame that has not gone out yet
    virtual void radioCancelSend() = 0;
    /// The next received frame, or 0
    virtual int radioRecv(uint8_t *buf, size_t max, float &snr, float &rssi) = 0;
    /// A frame is arriving, or the radio is busy with the host's own network
    virtual bool radioBusy() = 0;
    virtual uint32_t radioAirtimeMs(size_t len) = 0;
    virtual int8_t radioTxPower() = 0;

    // --- clock ---
    /// UNIX time, or 0 while the host does not know it
    virtual uint32_t getTime() = 0;
    /// The MeshCore app set the clock
    virtual void setTime(uint32_t epoch) = 0;

    // --- board ---
    virtual uint16_t battMilliVolts() = 0;
    virtual const char *boardName() = 0;
    virtual void reboot() = 0;
    /// The host's own position, for adverts that share it. False if unknown.
    virtual bool getPosition(double &lat, double &lon) = 0;
    /// Name for a node that has none stored yet
    virtual const char *defaultNodeName() = 0;
    virtual void log(const char *line) = 0;
    /// Size and use of the file system the MeshCore files live on, in KiB
    virtual uint32_t storageTotalKb() { return 0; }
    virtual uint32_t storageUsedKb() { return 0; }
};

/// What the host is told about, if it wants to show MeshCore traffic itself
class Events
{
  public:
    /// A group-channel text. text is as sent: "Sender: message".
    virtual void onChannelMessage(uint8_t channelIdx, const char *channelName, const char *text, uint32_t timestamp, float snr,
                                  uint8_t hops) = 0;
    /// The MeshCore app sent a text to a group channel. text is the message alone.
    virtual void onChannelMessageSent(uint8_t channelIdx, const char *text) {}
    /// A direct text from a contact
    virtual void onDirectMessage(const uint8_t *pubKey, const char *name, const char *text, uint32_t timestamp, float snr) = 0;
    /// An advert was heard. type: 1 chat, 2 repeater, 3 room server, 4 sensor. lat/lon in 1e-6 degrees, 0 if not shared.
    virtual void onContact(const uint8_t *pubKey, const char *name, uint8_t type, int32_t latE6, int32_t lonE6, bool isNew,
                           uint8_t hops) = 0;
};

/// A link to the MeshCore phone app that carries whole companion-protocol frames (BLE, for one)
class AppLink
{
  public:
    static const size_t MAX_FRAME = 176;
    virtual bool isConnected() = 0;
    /// False while the link cannot take another frame
    virtual bool canWrite() = 0;
    virtual size_t writeFrame(const uint8_t *data, size_t len) = 0;
    /// One received frame into dest (MAX_FRAME bytes), or 0
    virtual size_t readFrame(uint8_t *dest) = 0;
    virtual void loop() {}
};

struct Status {
    char nodeName[32];
    uint8_t pubKey[32];
    float freqMHz, bwKHz;
    uint8_t sf, cr;
    uint32_t packetsRecv, packetsSent;
    int contacts;
    int queuedForApp; // messages waiting for the phone app
    bool appConnected;
};

/// Start the MeshCore node. storage is the host's file system; MeshCore keeps its files under dir (created if missing).
bool begin(Platform &platform, Events *events, fs::FS &storage, const char *dir);
/// Call often from the host's main loop
void loop();
bool running();

/// Add the phone-app link over BLE (before or after begin)
void setBleLink(AppLink *link);
/// Serve the phone app on a TCP port once the host has Wi-Fi up (the companion's usual port is 5000)
void startTcpLink(uint16_t port);
/// The pin the MeshCore app is expected to pair with (the host owns the BLE security); for display only
void setBlePin(uint32_t pin);

/// Send a text to a group channel (0 is Public) under this node's name
bool sendChannelText(uint8_t channelIdx, const char *text);
/// Send a text to the contact whose public key starts with these bytes
bool sendDirectText(const uint8_t *pubKeyPrefix, size_t prefixLen, const char *text);
/// Send a text to the contact with this name (or the first whose name starts with it)
bool sendDirectTextByName(const char *name, const char *text);
/// Announce this node: to direct neighbours, or flooded through repeaters
bool sendAdvert(bool flood);
bool setNodeName(const char *name);
void getStatus(Status &out);

} // namespace meshcore_host
