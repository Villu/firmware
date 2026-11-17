#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * A second LoRa network on the same radio.
 *
 * The radio stays in continuous receive on the Meshtastic ("home") settings. Between home frames a driver task retunes
 * to the side settings for one channel-activity check, a few symbols long, and comes straight back unless the check
 * finds a preamble; then it stays for that one frame. The side network needs a longer preamble than the look period,
 * and the home preamble has to outlast one look. Frames for the side network go out after a look that found it clear.
 */
struct SideChannelConfig {
    float freqMHz;
    float bwKHz;
    uint8_t sf;
    uint8_t cr; // denominator, 5..8
    uint8_t syncWord;
    uint16_t preamble; // symbols
};

struct SideChannelFrame {
    uint16_t len;
    int16_t rssi;
    float snr;
    uint8_t data[255];
};

struct SideChannelStats {
    uint32_t looks;      // channel-activity checks made on the side settings
    uint32_t cadHits;    // ...that reported a preamble
    uint32_t frames;     // frames received with a good CRC
    uint32_t crcErrors;  // frames received with a bad CRC or a damaged header
    uint32_t noPreamble; // hits with no preamble behind them
    uint32_t noHeader;   // hits whose preamble was not followed by a header
    uint32_t rxDropped;  // good frames lost because nobody took the earlier ones
    uint32_t txOk, txFail;
    uint32_t homeBusy;    // looks put off because a home frame was arriving
    uint32_t lookUsLast;  // how long the last empty look kept the radio off the home settings
    uint32_t lookUsMax;   // ...and the longest so far
    uint32_t looksLong;   // empty looks that took over 20 ms
    uint32_t cadUsLast;   // from starting the last scan to seeing it done
    uint32_t preambleMsLast; // from a hit to the receiver's preamble flag, for the last frame that arrived
    uint32_t preambleMsMax;  // ...and the longest so far
    uint32_t gapMsMax;       // the longest time between two scans with nothing but home in between
    uint32_t gapsLong;       // such gaps over 100 ms
    uint32_t noPreambleStrong; // hits given up on with a signal over -95 dBm on the channel: a frame, not noise
    int16_t noPreambleRssi;    // the level at the last hit given up on, dBm
    int16_t phaseMsLast;       // how far into its preamble the last frame was when the scan that found it began
    uint8_t phaseHist[14];     // ...counted in steps of 10 ms: before the preamble, 0-9, ..., 120 and later
    uint32_t strongAtMs;       // millis() at the start of the last scan that hit a strong signal with no preamble
    uint32_t strongGapMs;      // ...and how long after the scan before it that was
    uint32_t clearAtMs;        // start of the first empty scan after such a hit
    uint32_t homeRestoreFail;
};

class SideChannelRadio
{
  public:
    /// The radio driver that can do this, or null
    static SideChannelRadio *instance;

    /// Whether these settings can share the radio with the home network as it is set now: a look has to be short
    /// against a home preamble, and the looks must not take too much of the home network's listening time
    virtual bool sideChannelFits(const SideChannelConfig &cfg) const = 0;
    /// Start looking at (or move to) these settings. False if they do not fit or the driver could not start.
    virtual bool sideChannelStart(const SideChannelConfig &cfg) = 0;
    virtual void sideChannelStop() = 0;
    virtual bool sideChannelRunning() const = 0;

    /// Queue one frame; it goes out after the next look that finds the side channel clear. False while one is waiting.
    virtual bool sideChannelSend(const uint8_t *data, size_t len) = 0;
    /// True once, when the frame given to sideChannelSend() has left the radio (ok) or could not be sent
    virtual bool sideChannelTakeSendResult(bool &ok) = 0;
    /// Forget a queued frame that has not started to go out
    virtual void sideChannelCancelSend() = 0;
    /// The oldest received frame, if any
    virtual bool sideChannelRecv(SideChannelFrame &out) = 0;
    /// A side frame is arriving right now, or the radio is taken up by the home network
    virtual bool sideChannelReceiving() const = 0;
    virtual void sideChannelGetStats(SideChannelStats &out) const = 0;
    /// Airtime of a frame of len bytes on the side settings
    virtual uint32_t sideChannelAirtimeMs(size_t len) const = 0;
    /// The power side frames go out at, dBm (the home network's setting)
    virtual int8_t sideChannelTxPower() const = 0;
};
