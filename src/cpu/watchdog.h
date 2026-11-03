#pragma once

#include "io.h"
#include <functional>

// Watchdog timer (WDOG_CTL/CNT/STAT at 0xFFC00200). Counts down at SCLK while enabled; on expiry it sets
// WDRO and raises the event WDEV selects. R246 reboots through it (after a disk-mode eject or a firmware
// update: WDOG_CNT = 1, enable, spin), so the reset event asks the host to rebuild the machine (see
// BlackFinCpu::ResetRequested).
class Watchdog : public RegisterDevice {
public:
    explicit Watchdog(u32 baseAddr);

    void Advance(u64 cycles);   // core clock cycles since power-on: after every run (HostWork)
    void SetResetCallback(std::function<void()> cb) { onReset_ = std::move(cb); }

private:
    static constexpr u32 kDisable = 0xAD;
    bool Enabled() const { return wden_ != kDisable; }
    void Expire();

    u32 wdev_ = 0, wden_ = kDisable, wdro_ = 0;
    u32 cnt_ = 0, stat_ = 0;
    u64 startCycles_ = 0;
    bool running_ = false, started_ = false;
    std::function<void()> onReset_;
};
