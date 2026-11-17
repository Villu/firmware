#pragma once

// The "board" MeshCore's companion code is built for here: another firmware. Everything a MeshCore variant normally
// supplies as globals (board, radio, clock, sensors) is a thin object that forwards to the host's Platform.

#include <Arduino.h>
#include <Mesh.h>
#include <helpers/KeyValueStore.h>
#include <helpers/SensorManager.h>

#include "MeshCoreHost.h"

// The companion sources name these globals plainly; give them names that cannot meet the host's own symbols.
#define board mc_host_board
#define radio_driver mc_host_radio
#define rtc_clock mc_host_rtc
#define sensors mc_host_sensors

class HostBoard : public mesh::MainBoard
{
  public:
    uint16_t getBattMilliVolts() override;
    const char *getManufacturerName() const override;
    void reboot() override;
    uint8_t getStartupReason() const override { return BD_STARTUP_NORMAL; }
    void attachDynamicPrefs(KeyValueStore *) {}
};

class HostRadio : public mesh::Radio
{
    float _freq = 0, _bw = 0;
    uint8_t _sf = 0, _cr = 0;
    bool _sending = false;
    float _lastSnr = 0, _lastRssi = 0;
    uint32_t _recv = 0, _sent = 0, _recvErrors = 0;

  public:
    int recvRaw(uint8_t *bytes, int sz) override;
    uint32_t getEstAirtimeFor(int len_bytes) override;
    float packetScore(float snr, int packet_len) override;
    bool startSendRaw(const uint8_t *bytes, int len) override;
    bool isSendComplete() override;
    void onSendFinished() override;
    bool isInRecvMode() const override { return !_sending; }
    bool isReceiving() override;
    float getLastRSSI() const override { return _lastRssi; }
    float getLastSNR() const override { return _lastSnr; }

    // what the companion code asks of its radio driver beyond mesh::Radio
    void setParams(float freq, float bw, uint8_t sf, uint8_t cr);
    void setTxPower(int8_t) {} // the host's own setting applies to both networks
    bool setRxBoostedGainMode(bool) { return true; } // the host owns the receiver gain
    bool getRxBoostedGainMode() const { return true; }
    uint32_t getPacketsRecv() const { return _recv; }
    uint32_t getPacketsSent() const { return _sent; }
    uint32_t getPacketsRecvErrors() const { return _recvErrors; }
    float getFreq() const { return _freq; }
    float getBw() const { return _bw; }
    uint8_t getSf() const { return _sf; }
    uint8_t getCr() const { return _cr; }
};

class HostRTC : public mesh::RTCClock
{
    uint32_t _base = 1715770351; // MeshCore's own "clock not set" start, 15 May 2024
    int64_t _baseUs = 0;

  public:
    uint32_t getCurrentTime() override;
    void setCurrentTime(uint32_t time) override;
};

class HostRNG : public mesh::RNG
{
  public:
    void random(uint8_t *dest, size_t sz) override;
};

extern HostBoard board;
extern HostRadio radio_driver;
extern HostRTC rtc_clock;
extern SensorManager sensors;

mesh::LocalIdentity radio_new_identity();

namespace meshcore_host
{
extern Platform *platform; // set by begin()
}
