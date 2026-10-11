#include "watchdog.h"
#include "time_source.h"
#include "utils/log.h"

Watchdog::Watchdog(u32 baseAddr, const TimeSource& time)
    : RegisterDevice("Watchdog", baseAddr, 0x10), time_(time) {
    REG32(WDOG_CTL, 0x00);
    FIELD(WDOG_CTL, WDEV, 1, 2, R(wdev_), W(wdev_));
    FIELD(WDOG_CTL, WDEN, 4, 8, R(wden_), W(wden_));
    FIELD(WDOG_CTL, WDRO, 15, 1, R(wdro_), [this](u32 v) { if (v) wdro_ = 0; });   // write 1 to clear
    WDOG_CTL.writeCallback = [this](u32) {
        if (Enabled() && !running_) {   // enabling loads the count
            running_ = true;
            stat_ = cnt_;
            startCycles_ = 0;
        } else if (!Enabled()) {
            running_ = false;
        }
    };

    REG32(WDOG_CNT, 0x04);
    FIELD(WDOG_CNT, VAL, 0, 32, R(cnt_), [this](u32 v) { if (!running_) cnt_ = v; });

    // Reading gives the current count; any write reloads it from WDOG_CNT (the firmware's "kick").
    REG32(WDOG_STAT, 0x08);
    FIELD(WDOG_STAT, VAL, 0, 32, R(stat_), [this](u32) { stat_ = cnt_; startCycles_ = 0; });
}

void Watchdog::UpdateCycles(u64 cycles) {
    if (!running_) return;
    if (startCycles_ == 0) {
        startCycles_ = cycles;
        if (stat_ == 0) Expire();
        return;
    }
    // SCLK = CCLK / 4; wall mode runs guest time slower like the other timers.
    const u64 ticks = (cycles - startCycles_) / 4 / time_.Slowdown();
    if (ticks < stat_) return;
    stat_ = 0;
    Expire();
}

void Watchdog::Expire() {
    wdro_ = 1;
    running_ = false;
    if (wdev_ == 0) {
        LogInfo("Watchdog expired: system reset");
        if (onReset_) onReset_();
    } else {
        LogInfo("Watchdog expired (event %u not emulated)", wdev_);
    }
}
