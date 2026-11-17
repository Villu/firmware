#include <target.h>

#include <esp_random.h>
#include <esp_timer.h>

using meshcore_host::platform;

HostBoard board;
HostRadio radio_driver;
HostRTC rtc_clock;
SensorManager sensors;

static HostRNG identity_rng;

mesh::LocalIdentity radio_new_identity()
{
    return mesh::LocalIdentity(&identity_rng);
}

// ---- board ----

uint16_t HostBoard::getBattMilliVolts()
{
    return platform ? platform->battMilliVolts() : 0;
}

const char *HostBoard::getManufacturerName() const
{
    return platform ? platform->boardName() : "hosted";
}

void HostBoard::reboot()
{
    if (platform)
        platform->reboot();
}

// ---- radio ----

void HostRadio::setParams(float freq, float bw, uint8_t sf, uint8_t cr)
{
    // Stored settings the host does not allow (from before it checked, or its own radio plan changed): the defaults
    if (platform && !platform->radioParamsAllowed(freq, bw, sf, cr)) {
        char line[120];
        snprintf(line, sizeof(line), "MeshCore: %.3f MHz SF%u %.1f kHz is not allowed here, using %.3f MHz SF%u %.1f kHz",
                 freq, sf, bw, (float)LORA_FREQ, (unsigned)LORA_SF, (float)LORA_BW);
        platform->log(line);
        freq = LORA_FREQ;
        bw = LORA_BW;
        sf = LORA_SF;
        cr = LORA_CR;
    }
    _freq = freq;
    _bw = bw;
    _sf = sf;
    _cr = cr;
    if (platform)
        platform->radioSetParams(freq, bw, sf, cr);
}

int HostRadio::recvRaw(uint8_t *bytes, int sz)
{
    if (!platform)
        return 0;
    float snr = 0, rssi = 0;
    const int len = platform->radioRecv(bytes, (size_t)sz, snr, rssi);
    if (len > 0) {
        _lastSnr = snr;
        _lastRssi = rssi;
        _recv++;
    }
    return len;
}

uint32_t HostRadio::getEstAirtimeFor(int len_bytes)
{
    return platform ? platform->radioAirtimeMs((size_t)len_bytes) : 0;
}

// As RadioLibWrapper::packetScoreInt(): the chance this packet got through, from its SNR margin and its length
float HostRadio::packetScore(float snr, int packet_len)
{
    static const float threshold[] = {-7.5f, -10.0f, -12.5f, -15.0f, -17.5f, -20.0f};
    if (_sf < 7 || _sf > 12)
        return 0.0f;
    if (snr < threshold[_sf - 7])
        return 0.0f;
    const float successRate = (snr - threshold[_sf - 7]) / 10.0f;
    const float collisionPenalty = 1.0f - ((float)packet_len / 256.0f);
    const float score = successRate * collisionPenalty;
    return score < 0.0f ? 0.0f : score > 1.0f ? 1.0f : score;
}

bool HostRadio::startSendRaw(const uint8_t *bytes, int len)
{
    if (!platform || !platform->radioSend(bytes, (size_t)len))
        return false;
    _sending = true;
    return true;
}

bool HostRadio::isSendComplete()
{
    bool ok = false;
    if (!_sending || !platform || !platform->radioTakeSendResult(ok))
        return false;
    _sending = false;
    if (ok)
        _sent++;
    return true;
}

void HostRadio::onSendFinished()
{
    // Also called when the dispatcher gave up waiting: then the frame must not go out behind its back
    if (_sending && platform)
        platform->radioCancelSend();
    _sending = false;
}

bool HostRadio::isReceiving()
{
    return platform && platform->radioBusy();
}

// ---- clock ----

uint32_t HostRTC::getCurrentTime()
{
    const uint32_t hostTime = platform ? platform->getTime() : 0;
    if (hostTime)
        return hostTime;
    return _base + (uint32_t)((esp_timer_get_time() - _baseUs) / 1000000);
}

void HostRTC::setCurrentTime(uint32_t time)
{
    _base = time;
    _baseUs = esp_timer_get_time();
    if (platform)
        platform->setTime(time);
}

void HostRNG::random(uint8_t *dest, size_t sz)
{
    esp_fill_random(dest, sz);
}
