#pragma once

#include "io.h"
#include <functional>

class TimeSource;

// Watchdog timer (WDOG_CTL/CNT/STAT at 0xFFC00200). Counts down at SCLK while enabled; on expiry it sets
// WDRO and raises the event WDEV selects. R246 reboots through it (after a disk-mode eject: WDOG_CNT = 1,
// enable, spin), so the reset event asks the host to rebuild the machine (see BlackFinCpu::ResetRequested).
class Watchdog : public RegisterDevice {
public:
    Watchdog(u32 baseAddr, const TimeSource& time);

    void UpdateCycles(u64 cycles);   // core clock cycles since power-on
    void SetResetCallback(std::function<void()> cb) { onReset_ = std::move(cb); }
    bool NeedsProcess() const override { return false; }

private:
    static constexpr u32 kDisable = 0xAD;
    bool Enabled() const { return wden_ != kDisable; }
    void Expire();

    u32 wdev_ = 0, wden_ = kDisable, wdro_ = 0;
    u32 cnt_ = 0, stat_ = 0;
    u64 startCycles_ = 0;
    bool running_ = false;
    const TimeSource& time_;
    std::function<void()> onReset_;
};
