// A second LoRa network on the SX126x, received between Meshtastic frames. Included by SX126xInterface.cpp.
//
// The side settings are written to the chip with raw commands, never through RadioLib's setters, so RadioLib's own
// record of the modem settings always describes the home network: everything the rest of the driver computes from it
// (airtime, packet parameters restored by startReceive()) stays right, and coming home is three commands plus the
// driver's ordinary RX start. The whole excursion runs under the radio-sequence lock with DIO1 masked, so neither the
// radio thread nor the readout task sees the chip anywhere but at home.

#include "SPILock.h"

namespace
{
// Flags a side excursion reads by polling; nothing is routed to DIO1 while away
const uint16_t SIDE_IRQS = RADIOLIB_SX126X_IRQ_CAD_DONE | RADIOLIB_SX126X_IRQ_CAD_DETECTED | RADIOLIB_SX126X_IRQ_RX_DONE |
                           RADIOLIB_SX126X_IRQ_TX_DONE | RADIOLIB_SX126X_IRQ_TIMEOUT | RADIOLIB_SX126X_IRQ_CRC_ERR |
                           RADIOLIB_SX126X_IRQ_HEADER_VALID | RADIOLIB_SX126X_IRQ_HEADER_ERR |
                           RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED;

uint8_t sideBwCode(float bwKHz)
{
    static const struct {
        float khz;
        uint8_t code;
    } table[] = {{7.8f, 0x00},  {10.4f, 0x08}, {15.6f, 0x01},  {20.8f, 0x09},  {31.25f, 0x02},
                 {41.7f, 0x0A}, {62.5f, 0x03}, {125.0f, 0x04}, {250.0f, 0x05}, {500.0f, 0x06}};
    uint8_t best = 0x04;
    float bestDiff = 1e9f;
    for (const auto &row : table) {
        const float d = fabsf(row.khz - bwKHz);
        if (d < bestDiff) {
            bestDiff = d;
            best = row.code;
        }
    }
    return best;
}

uint32_t sideSymbolUs(uint8_t sf, float bwKHz)
{
    return (uint32_t)((float)(1UL << sf) * 1000.0f / bwKHz);
}

bool sideLdro(uint8_t sf, float bwKHz)
{
    return sideSymbolUs(sf, bwKHz) >= 16000; // as RadioLib's automatic choice
}

// How often the home receiver's flags are read while the radio waits at home
const uint32_t SIDE_GLANCE_MS = 8;

// Semtech's LoRa time-on-air formula (explicit header, CRC on), SF7 and up
uint32_t sideAirtimeMs(const SideChannelConfig &c, size_t len)
{
    const uint32_t symUs = sideSymbolUs(c.sf, c.bwKHz);
    const int de = sideLdro(c.sf, c.bwKHz) ? 2 : 0;
    const int num = 8 * (int)len - 4 * c.sf + 28 + 16;
    const int den = 4 * (c.sf - de);
    int payloadSyms = 8;
    if (num > 0)
        payloadSyms += ((num + den - 1) / den) * c.cr;
    const uint64_t us = (uint64_t)symUs * (4 * (c.preamble + payloadSyms) + 17) / 4; // + 4.25 symbols of sync
    return (uint32_t)((us + 999) / 1000);
}

// Semtech AN1200.48: two symbols and a peak of 22 for SF7/SF8, four symbols and a rising peak above
uint8_t sideCadSymbols(uint8_t sf)
{
    return sf <= 8 ? 2 : 4;
}
uint8_t sideCadPeak(uint8_t sf)
{
    return sf <= 8 ? 22 : (uint8_t)(sf + 14);
}
uint32_t sideCadMs(const SideChannelConfig &c)
{
    return (sideSymbolUs(c.sf, c.bwKHz) * (sideCadSymbols(c.sf) + 1u)) / 1000 + 1;
}

// How long to stay home between looks. A frame whose preamble began just after one scan is found by the next, which
// ends a dwell, a look's retuning and a scan later: eight symbols of the preamble proper (the sync symbols behind it
// do not count) must still be left then for the receiver to lock on.
uint32_t sideHomeDwellMs(const SideChannelConfig &c)
{
    const uint32_t symUs = sideSymbolUs(c.sf, c.bwKHz);
    const int32_t preambleMs = (int32_t)((symUs * c.preamble) / 1000);
    const int32_t lockMs = (int32_t)((symUs * (sideCadSymbols(c.sf) + 8u)) / 1000);
    const int32_t ms = preambleMs - lockMs - (int32_t)sideCadMs(c) - 10;
    return ms < 20 ? 20u : (uint32_t)ms;
}
} // namespace

template <typename T> uint32_t SX126xInterface<T>::sideChannelAirtimeMs(size_t len) const
{
    return sideAirtimeMs(sideCfg, len);
}

template <typename T> bool SX126xInterface<T>::sideChannelFits(const SideChannelConfig &cfg) const
{
    if (cfg.sf < 7 || cfg.sf > 12 || cfg.cr < 5 || cfg.cr > 8 || cfg.bwKHz <= 0 || cfg.preamble < 8)
        return false;
    // A look (the scan plus about 3 ms of retuning) takes that much of a home preamble away from the home receiver:
    // past 60 % too little is left for it to lock. And the side preamble has to be long enough that looks which catch
    // every frame still leave the home network at least 70 % of the time.
    const uint32_t lookMs = sideCadMs(cfg) + 3;
    const uint32_t homePreambleMs = (sideSymbolUs(sf, bw) * preambleLength) / 1000;
    const uint32_t dwellMs = sideHomeDwellMs(cfg);
    return lookMs * 10 <= homePreambleMs * 6 && dwellMs > 20 && lookMs * 10 <= (dwellMs + lookMs) * 3;
}

template <typename T> bool SX126xInterface<T>::sideChannelStart(const SideChannelConfig &cfg)
{
    if (!sideChannelFits(cfg)) {
        LOG_ERROR("Side channel: SF%u at %.1f kHz with a preamble of %u does not fit beside the home network (SF%u, %.1f kHz)",
                  cfg.sf, cfg.bwKHz, cfg.preamble, sf, bw);
        return false;
    }
    {
        RadioSequence seq(this); // never while a look is using the old settings
        sideCfg = cfg;
    }
    if (!sideRxQueue)
        sideRxQueue = xQueueCreate(6, sizeof(SideChannelFrame));
    if (!sideRxQueue)
        return false;
    if (!sideTask) {
        // Above the main loop so a look is on time, below the readout task so a home frame is read out first
        UBaseType_t priority = uxTaskPriorityGet(nullptr) + 1;
        if (priority > configMAX_PRIORITIES - 2)
            priority = configMAX_PRIORITIES - 2;
        TaskHandle_t task = nullptr;
#ifdef ARCH_ESP32
        if (xTaskCreatePinnedToCore(sideTaskMain, "SideChan", 6144, this, priority, &task, xPortGetCoreID()) != pdPASS)
            task = nullptr;
#else
        if (xTaskCreate(sideTaskMain, "SideChan", 1536, this, priority, &task) != pdPASS)
            task = nullptr;
#endif
        sideTask = task;
        if (!sideTask) {
            LOG_ERROR("Side channel: task not started");
            return false;
        }
    }
    sideEnabled = true;
    LOG_INFO("Side channel: %.3f MHz, SF%u, %.1f kHz, CR 4/%u, sync 0x%02x, preamble %u; a %u-symbol look every %u ms",
             cfg.freqMHz, cfg.sf, cfg.bwKHz, cfg.cr, cfg.syncWord, cfg.preamble, sideCadSymbols(cfg.sf),
             (unsigned)(sideHomeDwellMs(cfg) + sideCadMs(cfg) + 3));
    return true;
}

template <typename T> bool SX126xInterface<T>::sideChannelSend(const uint8_t *data, size_t len)
{
    const uint8_t state = sideTxState;
    if (!sideEnabled || !sideTask || len == 0 || len > sizeof(sideTxBuf) || state == SIDE_TX_PENDING ||
        state == SIDE_TX_SENDING)
        return false;
    if (disabled || !config.lora.tx_enabled || config.lora.region == meshtastic_Config_LoRaConfig_RegionCode_UNSET)
        return false;
    memcpy(sideTxBuf, data, len);
    sideTxLen = (uint16_t)len;
    sideTxState = SIDE_TX_PENDING; // last: the task takes the frame once it sees this
    xTaskNotifyGive(sideTask);     // and looks at once, not at the end of its dwell: the sender is timing this
    return true;
}

template <typename T> bool SX126xInterface<T>::sideChannelTakeSendResult(bool &ok)
{
    const uint8_t state = sideTxState;
    if (state != SIDE_TX_SENT && state != SIDE_TX_FAILED)
        return false;
    ok = state == SIDE_TX_SENT;
    uint8_t taken = state;
    sideTxState.compare_exchange_strong(taken, SIDE_TX_IDLE);
    return true;
}

template <typename T> bool SX126xInterface<T>::sideChannelRecv(SideChannelFrame &out)
{
    return sideRxQueue && xQueueReceive(sideRxQueue, &out, 0) == pdTRUE;
}

template <typename T> void SX126xInterface<T>::sideTaskMain(void *arg)
{
    static_cast<SX126xInterface<T> *>(arg)->sideTaskLoop();
}

template <typename T> void SX126xInterface<T>::sideTaskLoop()
{
    uint32_t homeSince = millis();
    for (;;) {
        // The home settings can change under a running side channel (another preset): stay home while it does not fit
        const bool fits = sideEnabled && sideChannelFits(sideCfg);
        if (sideEnabled && fits == sideSuspended) {
            sideSuspended = !fits;
            if (fits)
                LOG_INFO("Side channel: fits beside the home network again, looking");
            else
                LOG_WARN("Side channel: does not fit beside the home network as it is set now (SF%u, %.1f kHz), not looking",
                         sf, bw);
        }
        if (!fits) {
            uint8_t waiting = SIDE_TX_PENDING;
            sideTxState.compare_exchange_strong(waiting, SIDE_TX_FAILED);
            vTaskDelay(pdMS_TO_TICKS(50));
            homeSince = millis();
            continue;
        }
        const uint32_t dwell = sideHomeDwellMs(sideCfg);
        const uint32_t waited = millis() - homeSince;
        if (waited < dwell) {
            // Woken early by a frame handed in for sending: it goes out after a look made right now.
            // The wait is cut into slices with a glance at the home receiver's flags between them: a preamble
            // sighting puts the next look off for a header's time, and that has to count from when the preamble
            // showed, not from when the look fell due up to a dwell later. Counted from then it cost one side frame
            // in ten: a strong side transmission makes the home receiver report a preamble as it starts, and the
            // look came after the side preamble was over.
            const uint32_t left = dwell - waited;
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(left < SIDE_GLANCE_MS ? left : SIDE_GLANCE_MS)) == 0 ||
                sideTxState != SIDE_TX_PENDING) {
                if (left > SIDE_GLANCE_MS) {
                    RadioSequence seq(this);
                    (void)sideHomeState();
                }
                continue;
            }
        }
        const SideLook result = sideLook();
        sideHomeBusy = result != SIDE_LOOK_DONE;
        switch (result) {
        case SIDE_LOOK_DONE:
            homeSince = millis();
            // A false alarm kept the radio away for longer than a look: the next scan still comes one period after
            // the scan that raised it, or a preamble starting behind that scan is over before anything looks again.
            // Home gets 20 ms at least.
            if (sideFalseHitMs) {
                const uint32_t cut = sideFalseHitMs < dwell - 20 ? sideFalseHitMs : dwell - 20;
                homeSince -= cut;
                sideFalseHitMs = 0;
            }
            break;
        case SIDE_LOOK_HOME_BUSY:
            sideStats.homeBusy++;
            vTaskDelay(pdMS_TO_TICKS(4));
            break;
        default: // transmitting, scanning or asleep: the radio thread has the chip
            vTaskDelay(pdMS_TO_TICKS(10));
            break;
        }
    }
}

template <typename T> typename SX126xInterface<T>::SideLook SX126xInterface<T>::sideHomeState()
{
    // Only from plain continuous RX with nothing in flight: then the look can put back exactly what it found
    if (disabled || rxOffline || !isReceiving || sendingPacket != NULL || !rxArmedContinuous || cadHandedToRx ||
        cadHandoffRxStart || rearmState != REARM_NONE || rxArmedBeforeTxDone)
        return SIDE_LOOK_RADIO_OFF;

    const uint16_t irq = lora.getIrqFlags();
    // A frame is arriving or waits to be read out
    if (irq & (RADIOLIB_SX126X_IRQ_RX_DONE | RADIOLIB_SX126X_IRQ_CRC_ERR | RADIOLIB_SX126X_IRQ_HEADER_VALID)) {
        sideBareSince = 0;
        return SIDE_LOOK_HOME_BUSY;
    }
    // A bare preamble may be a home frame starting, and a look now would come back after its preamble: no look until
    // its header would have shown. Meshtastic's transmit path holds on the same flag and clears it on sight too, so
    // whichever of the two reads it first has to tell the other: here through the stock hold, in both directions.
    // Bare preambles with never a header behind them (noise, or LoRa that is not ours) put looks off for four such
    // waits at most; after that they are looked past until the flag has been quiet once.
    const uint32_t now = millis();
    const uint32_t graceMs = preambleHeaderGraceMsec(sf, bw, preambleLength);
    const bool capped = sideBareSince && now - sideBareSince >= 4 * graceMs;
    if (irq & RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED) {
        if (!preambleHoldActive())
            holdOnPreamble(); // clears the flag and holds Meshtastic's transmissions, as its own look would have
        else
            lora.clearIrqFlags(RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED); // a hold is running; this sighting is new to us
        if (!sideBareSince)
            sideBareSince = now ? now : 1;
        if (!capped) {
            sideHomeHoldUntil = now + graceMs;
            return SIDE_LOOK_HOME_BUSY;
        }
        sideHomeHoldUntil = 0;
    } else {
        if (!capped && preambleHoldActive() && !preambleHoldHeaderSeen &&
            Throttle::isWithinTimespanMs(preambleHoldStart, graceMs)) {
            // The transmit path saw a preamble first and cleared it
            if (!sideBareSince)
                sideBareSince = now ? now : 1;
            return SIDE_LOOK_HOME_BUSY;
        }
        if (sideHomeHoldUntil && (int32_t)(sideHomeHoldUntil - now) > 0)
            return SIDE_LOOK_HOME_BUSY;
        sideBareSince = 0; // quiet
    }
    sideHomeHoldUntil = 0;
    return SIDE_LOOK_DONE;
}

// The display, the SD card and the radio share one SPI bus behind spiLock, taken per transfer. A look is some twenty
// transfers, and the display getting in between two of them keeps the radio off the home network for as long as it
// draws. So a look takes the bus once, before it leaves home, and keeps it until the radio is back.
template <typename T> void SX126xInterface<T>::sideBusTake()
{
    spiLock->lock();
    LockingArduinoHal::busHolder = (void *)xTaskGetCurrentTaskHandle();
    sideBusHeld = true;
}

template <typename T> void SX126xInterface<T>::sideBusRelease()
{
    if (!sideBusHeld)
        return;
    sideBusHeld = false;
    LockingArduinoHal::busHolder = nullptr;
    spiLock->unlock();
}

template <typename T> int16_t SX126xInterface<T>::sideStandby()
{
    // On the oscillator the chip was left on, so the next command does not wait for a TCXO to start
    const uint8_t mode = tcxoVoltage > 0 ? RADIOLIB_SX126X_STANDBY_XOSC : RADIOLIB_SX126X_STANDBY_RC;
    return module.SPIwriteStream(RADIOLIB_SX126X_CMD_SET_STANDBY, &mode, 1);
}

template <typename T> int16_t SX126xInterface<T>::sideTune(float freqMHz, float bwKHz, uint8_t sfv, uint8_t crv, uint8_t sw)
{
    const uint32_t frf = (uint32_t)((double)freqMHz * 1048576.0); // Hz * 2^25 / 32 MHz
    const uint8_t freq[4] = {(uint8_t)(frf >> 24), (uint8_t)(frf >> 16), (uint8_t)(frf >> 8), (uint8_t)frf};
    int16_t err = module.SPIwriteStream(RADIOLIB_SX126X_CMD_SET_RF_FREQUENCY, freq, sizeof(freq));
    if (err != RADIOLIB_ERR_NONE)
        return err;
    const uint8_t mod[4] = {sfv, sideBwCode(bwKHz), (uint8_t)(crv - 4), (uint8_t)(sideLdro(sfv, bwKHz) ? 1 : 0)};
    err = module.SPIwriteStream(RADIOLIB_SX126X_CMD_SET_MODULATION_PARAMS, mod, sizeof(mod));
    if (err != RADIOLIB_ERR_NONE)
        return err;
    // The one-byte sync word spread over the two registers, with RadioLib's control nibbles (0x44)
    const uint8_t sync[2] = {(uint8_t)((sw & 0xF0) | 0x04), (uint8_t)(((sw & 0x0F) << 4) | 0x04)};
    const uint8_t cmd[3] = {RADIOLIB_SX126X_CMD_WRITE_REGISTER, (uint8_t)(RADIOLIB_SX126X_REG_LORA_SYNC_WORD_MSB >> 8),
                            (uint8_t)(RADIOLIB_SX126X_REG_LORA_SYNC_WORD_MSB & 0xFF)};
    return module.SPIwriteStream(cmd, sizeof(cmd), sync, sizeof(sync));
}

template <typename T> int16_t SX126xInterface<T>::sidePacketParams(uint16_t preamble, uint8_t len)
{
    // Explicit header, CRC on, standard IQ
    const uint8_t pkt[6] = {(uint8_t)(preamble >> 8), (uint8_t)preamble, 0x00, len, 0x01, 0x00};
    return module.SPIwriteStream(RADIOLIB_SX126X_CMD_SET_PACKET_PARAMS, pkt, sizeof(pkt));
}

template <typename T> typename SX126xInterface<T>::SideLook SX126xInterface<T>::sideLook()
{
    RadioSequence seq(this);
    sideBusTake(); // waited for at home
    const SideLook home = sideHomeState();
    if (home != SIDE_LOOK_DONE) {
        sideBusRelease();
        return home;
    }

    const SideChannelConfig c = sideCfg;
    const uint32_t t0 = micros();
    sideChannelAway = true;
    int16_t err = sideStandby();
    // A home frame that completed between the check above and the standby is still in the chip, and the readout
    // task is already waiting for this lock: put RX back as it was and let it read.
    if (err == RADIOLIB_ERR_NONE && (lora.getIrqFlags() & RADIOLIB_SX126X_IRQ_RX_DONE)) {
        const uint8_t forever[3] = {0xFF, 0xFF, 0xFF};
        module.SPIwriteStream(RADIOLIB_SX126X_CMD_SET_RX, forever, sizeof(forever));
        sideChannelAway = false;
        sideBusRelease();
        return SIDE_LOOK_HOME_BUSY;
    }

    if (err == RADIOLIB_ERR_NONE)
        err = sideTune(c.freqMHz, c.bwKHz, c.sf, c.cr, c.syncWord);
    if (err == RADIOLIB_ERR_NONE)
        err = sidePacketParams(c.preamble, 0xFF);
    if (err == RADIOLIB_ERR_NONE) {
        const uint8_t dio[8] = {(uint8_t)(SIDE_IRQS >> 8), (uint8_t)(SIDE_IRQS & 0xFF), 0, 0, 0, 0, 0, 0};
        err = module.SPIwriteStream(RADIOLIB_SX126X_CMD_SET_DIO_IRQ_PARAMS, dio, sizeof(dio));
    }
    if (err == RADIOLIB_ERR_NONE)
        err = lora.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
    if (err == RADIOLIB_ERR_NONE) {
        // On a detection the chip goes straight into RX for at most one maximum-length frame (15.625 us steps)
        const uint32_t rxSteps = (sideAirtimeMs(c, 255) + 50) * 64;
        const uint8_t symCode = sideCadSymbols(c.sf) == 2 ? RADIOLIB_SX126X_CAD_ON_2_SYMB : RADIOLIB_SX126X_CAD_ON_4_SYMB;
        const uint8_t cad[7] = {symCode,
                                sideCadPeak(c.sf),
                                RADIOLIB_SX126X_CAD_PARAM_DET_MIN,
                                RADIOLIB_SX126X_CAD_GOTO_RX,
                                (uint8_t)(rxSteps >> 16),
                                (uint8_t)(rxSteps >> 8),
                                (uint8_t)rxSteps};
        cadParamsValid = false; // not the pre-TX scan's
        err = module.SPIwriteStream(RADIOLIB_SX126X_CMD_SET_CAD_PARAMS, cad, sizeof(cad));
    }
    if (err == RADIOLIB_ERR_NONE) {
        module.setRfSwitchState(Module::MODE_RX);
        err = module.SPIwriteStream(RADIOLIB_SX126X_CMD_SET_CAD, nullptr, 0);
    }

    uint16_t irq = 0;
    uint32_t scanStartMs = millis();
    if (err == RADIOLIB_ERR_NONE) {
        sideStats.looks++;
        // The scan takes its symbols and then the chip's own reckoning: sleep through the symbols, then ask every ms
        const uint32_t cadStart = micros();
        const uint32_t nowMs = millis();
        if (sideLastScanMs) {
            const uint32_t gap = nowMs - sideLastScanMs;
            if (gap > sideStats.gapMsMax)
                sideStats.gapMsMax = gap;
            if (gap > 100)
                sideStats.gapsLong++;
        }
        sideLastScanMs = nowMs ? nowMs : 1;
        scanStartMs = nowMs;
        vTaskDelay(pdMS_TO_TICKS((sideSymbolUs(c.sf, c.bwKHz) * sideCadSymbols(c.sf)) / 1000));
        for (uint32_t extra = 0; extra < 40; extra++) {
            irq = lora.getIrqFlags();
            if (irq & RADIOLIB_SX126X_IRQ_CAD_DONE)
                break;
            vTaskDelay(1);
        }
        sideStats.cadUsLast = micros() - cadStart;
    }

    bool empty = true;
    if (irq & RADIOLIB_SX126X_IRQ_CAD_DETECTED) {
        empty = false;
        sideStats.cadHits++;
        lora.clearIrqFlags(RADIOLIB_SX126X_IRQ_CAD_DONE | RADIOLIB_SX126X_IRQ_CAD_DETECTED);
        sideBusRelease(); // a frame takes long: the display may draw between the polls
        sideRxActive = true;
        const uint32_t strongBefore = sideStats.noPreambleStrong;
        const bool something = sideCatchFrame(c, scanStartMs);
        sideRxActive = false;
        const uint32_t tookMs = (micros() - t0) / 1000 + 3; // and the way home
        if (sideStats.noPreambleStrong != strongBefore) {
            // Something strong with no preamble. The first time it may be a transmitter keying up, its preamble
            // about to start: look again as soon as after a false alarm. Again right after that it is a frame
            // whose preamble was missed, and the home network gets its full share until the channel is clear.
            sideStats.strongAtMs = scanStartMs;
            sideStats.strongGapMs = sidePrevScanMs ? scanStartMs - sidePrevScanMs : 0;
            if (!sideStrongPending)
                sideFalseHitMs = tookMs;
            sideStrongPending = true;
        } else if (!something) {
            sideFalseHitMs = tookMs;
        }
    } else if ((irq & RADIOLIB_SX126X_IRQ_CAD_DONE) && sideTxState == SIDE_TX_PENDING) {
        empty = false;
        sideBusRelease();
        sideTransmit(c); // the scan just found the channel clear
    }
    if (empty && (irq & RADIOLIB_SX126X_IRQ_CAD_DONE) && sideStrongPending) {
        sideStrongPending = false;
        sideStats.clearAtMs = scanStartMs;
    }
    sidePrevScanMs = scanStartMs;

    sideGoHome();
    sideBusRelease();
    if (!empty)
        sideLastScanMs = 0; // a frame came or went: the time to the next scan is no gap
    if (empty) {
        const uint32_t us = micros() - t0;
        sideStats.lookUsLast = us;
        if (us > sideStats.lookUsMax)
            sideStats.lookUsMax = us;
        if (us > 20000)
            sideStats.looksLong++;
    }
    return SIDE_LOOK_DONE;
}

template <typename T> bool SX126xInterface<T>::sideCatchFrame(const SideChannelConfig &c, uint32_t scanStartMs)
{
    // The chip is in RX on the side settings. A real frame raises PREAMBLE_DETECTED some symbols later and its
    // header a preamble later; a hit with neither was noise, and is given up on rather than waited out. Five
    // symbols and 6 ms was enough only while the scan was waited out for longer than it takes: measured from the
    // scan's real end it gave up on four frames in ten.
    const uint32_t symUs = sideSymbolUs(c.sf, c.bwKHz);
    const uint32_t preambleBy = (symUs * 10u) / 1000 + 6;
    const uint32_t headerBy = (symUs * (c.preamble + 13u)) / 1000 + 20;
    const uint32_t frameBy = sideAirtimeMs(c, 255) + 50;
    const uint32_t start = millis();
    bool preamble = false, header = false;
    uint32_t preambleAfter = 0;
    for (;;) {
        const uint16_t irq = lora.getIrqFlags();
        if (!preamble && (irq & (RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED | RADIOLIB_SX126X_IRQ_HEADER_VALID)))
            preambleAfter = millis() - start;
        if (irq & RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED)
            preamble = true;
        if (irq & RADIOLIB_SX126X_IRQ_HEADER_VALID)
            preamble = header = true;
        if (irq & RADIOLIB_SX126X_IRQ_RX_DONE) {
            static SideChannelFrame frame; // only this task uses it
            const size_t len = lora.getPacketLength();
            int16_t state = RADIOLIB_ERR_UNKNOWN;
            if (len > 0 && len <= sizeof(frame.data))
                state = lora.readData(frame.data, len); // checks the CRC flag and clears the flags
            if (state == RADIOLIB_ERR_NONE) {
                frame.len = (uint16_t)len;
                frame.snr = lora.getSNR();
                frame.rssi = (int16_t)lround(lora.getRSSI());
                sideStats.frames++;
                const int32_t phase = (int32_t)(scanStartMs - (millis() - sideAirtimeMs(c, len)));
                sideStats.phaseMsLast = (int16_t)phase;
                sideStats.phaseHist[phase < 0 ? 0 : phase >= 120 ? 13 : 1 + phase / 10]++;
                sideStats.preambleMsLast = preambleAfter;
                if (preambleAfter > sideStats.preambleMsMax)
                    sideStats.preambleMsMax = preambleAfter;
                if (xQueueSend(sideRxQueue, &frame, 0) != pdTRUE)
                    sideStats.rxDropped++;
            } else {
                sideStats.crcErrors++;
            }
            return true;
        }
        if (irq & (RADIOLIB_SX126X_IRQ_HEADER_ERR | RADIOLIB_SX126X_IRQ_TIMEOUT)) {
            sideStats.crcErrors++;
            return true;
        }
        const uint32_t elapsed = millis() - start;
        if (!preamble && elapsed >= preambleBy) {
            sideStats.noPreamble++;
            const float level = lora.getRSSI(false); // right now, not the last packet's
            sideStats.noPreambleRssi = (int16_t)level;
            if (level > -95.0f)
                sideStats.noPreambleStrong++;
            return false;
        }
        if (!header && elapsed >= headerBy) {
            sideStats.noHeader++;
            return true;
        }
        if (elapsed >= frameBy)
            return true;
        vTaskDelay(pdMS_TO_TICKS(header ? 4 : 2));
    }
}

template <typename T> void SX126xInterface<T>::sideTransmit(const SideChannelConfig &c)
{
    const uint16_t len = sideTxLen;
    // From here the frame is on its way: a cancel no longer reaches it, and a new frame is not taken until it is done
    uint8_t waiting = SIDE_TX_PENDING;
    if (len == 0 || !sideTxState.compare_exchange_strong(waiting, SIDE_TX_SENDING)) // cancelled meanwhile
        return;
    int16_t err = sideStandby();
    if (err == RADIOLIB_ERR_NONE)
        err = sidePacketParams(c.preamble, (uint8_t)len);
    if (err == RADIOLIB_ERR_NONE) {
        const uint8_t base[2] = {0x00, 0x00};
        err = module.SPIwriteStream(RADIOLIB_SX126X_CMD_SET_BUFFER_BASE_ADDRESS, base, sizeof(base));
    }
    if (err == RADIOLIB_ERR_NONE) {
        const uint8_t cmd[2] = {RADIOLIB_SX126X_CMD_WRITE_BUFFER, 0x00};
        err = module.SPIwriteStream(cmd, sizeof(cmd), sideTxBuf, len);
    }
    if (err == RADIOLIB_ERR_NONE) {
        // Datasheet 15.1: the sensitivity register follows the bandwidth for TX. RadioLib sets it for the home
        // bandwidth with each of its own transmissions.
        const uint8_t rd[3] = {RADIOLIB_SX126X_CMD_READ_REGISTER, (uint8_t)(RADIOLIB_SX126X_REG_SENSITIVITY_CONFIG >> 8),
                               (uint8_t)(RADIOLIB_SX126X_REG_SENSITIVITY_CONFIG & 0xFF)};
        uint8_t value = 0;
        err = module.SPIreadStream(rd, sizeof(rd), &value, 1);
        if (err == RADIOLIB_ERR_NONE) {
            if (c.bwKHz > 400.0f)
                value &= 0xFB;
            else
                value |= 0x04;
            const uint8_t wr[3] = {RADIOLIB_SX126X_CMD_WRITE_REGISTER, rd[1], rd[2]};
            err = module.SPIwriteStream(wr, sizeof(wr), &value, 1);
        }
    }
    if (err == RADIOLIB_ERR_NONE)
        err = lora.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
    bool sent = false;
    if (err == RADIOLIB_ERR_NONE) {
        setTransmitEnable(true);
        module.setRfSwitchState(Module::MODE_TX);
        const uint8_t noTimeout[3] = {0x00, 0x00, 0x00};
        err = module.SPIwriteStream(RADIOLIB_SX126X_CMD_SET_TX, noTimeout, sizeof(noTimeout));
    }
    if (err == RADIOLIB_ERR_NONE) {
        const uint32_t airMs = sideAirtimeMs(c, len);
        const uint32_t start = millis();
        if (airMs > 4)
            vTaskDelay(pdMS_TO_TICKS(airMs - 3));
        while (millis() - start < airMs + 200) {
            if (lora.getIrqFlags() & RADIOLIB_SX126X_IRQ_TX_DONE) {
                sent = true;
                break;
            }
            vTaskDelay(1);
        }
    }
    setTransmitEnable(false);
    lora.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
    if (sent)
        sideStats.txOk++;
    else
        sideStats.txFail++;
    sideTxLen = 0;
    sideTxState = (uint8_t)(sent ? SIDE_TX_SENT : SIDE_TX_FAILED);
}

template <typename T> void SX126xInterface<T>::sideGoHome()
{
    int16_t err = RADIOLIB_ERR_NONE;
    for (int attempt = 0; attempt < 2; attempt++) {
        err = sideStandby();
        if (err == RADIOLIB_ERR_NONE)
            err = sideTune(getFreq(), bw, sf, cr, syncWord);
        if (err == RADIOLIB_ERR_NONE) {
            setTransmitEnable(false);
            // RadioLib's RX start: the home packet parameters, the driver's flag set, IRQs cleared, continuous RX.
            // The RX interrupt was never detached and DIO1 could not rise while away.
            err = startRxCommand();
        }
        if (err == RADIOLIB_ERR_NONE)
            break;
        vTaskDelay(2);
    }
    if (err != RADIOLIB_ERR_NONE) {
        // Leave it to the driver's own recovery: periodicRadioMaintenance() re-arms a radio marked offline
        sideStats.homeRestoreFail++;
        rxArmedContinuous = false;
        rxOffline = true;
    }
    sideChannelAway = false;
}
