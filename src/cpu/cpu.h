#pragma once
#include <map>
#include <unordered_map>
#include <set>

#include "emu.h"
#include "time_source.h"
#include <atomic>
#include <thread>
#include <memory>
#include <vector>
#include <chrono>

// Forward declarations for bcore types (headers included in cpu.cpp only)
struct CpuState;
class Core;
class EmulatorMemory;

enum RegIndex {
    FP,
    SP,
    RETS,
    R0,
    R1,
    R2,
    P1,
};

class SIC;
class CoreTimer;
class Watchdog;
class GPIO;
class PPI;
class Display;
class OLED;
class NFC;
class GPTimer;
class USBDevice;
class NandFlash;
class Keyboard;
class MCP230XX;
class ADXL345;
class Potentiometer;
class GPIOPeripheral;
class SPORT;
class AudioOutput;
class Profiler;
struct BcoreStats;
class BcoreEventSink;

class BlackFinCpu : public CpuInterface {
public:
    // deterministic selects the guest-time model; see TimeSource.
    explicit BlackFinCpu(bool deterministic = false);
    ~BlackFinCpu() override;

    void HaltExecution(HaltReason reason) override;
    void SaveContext() override;
    void RestoreContext() override;
    HaltReason Run() override;

    void SetRegister(int index, u32 value) override;
    u32 GetRegister(int index) override;

    void SetPC(u32 value) override;
    u32 PC() override;

    Emulator& GetEmulator() { return emulator; }
    USBDevice& GetUSB() { return *usb; }

    void SetBootMode(int mode);
    // Persistent JIT object cache directory ("" = off). See bcore Core::set_object_cache_dir.
    void SetJitCacheDir(const std::string& dir);
    // Pace deterministic guest time to the host clock: sleep whenever the guest is ahead. For interactive
    // use once the emulator is faster than real time (guest time stays instruction-driven).
    void SetRealtime(bool on) { realtime_ = on; UpdateAsyncCompile(); }
    // Compile uncached blocks on a worker thread and interpret them meanwhile (see interp.h). On when guest
    // time is paced to the host (--realtime, wall mode), off for --deterministic so runs stay reproducible.
    // OP1EMU_ASYNC_JIT=0/1 overrides.
    void UpdateAsyncCompile();
    uint64_t InterpretedSteps() const { return interpSteps_; }
private:
    void AfterBlock(uint64_t cyclesElapsed);
    // Guest cycles per batch between device/interrupt polls (OP1EMU_BATCH overrides; 1 = every block).
    // Batch size once the firmware is up (see Run); 1 = poll devices after every block.
    uint64_t batchCycles_ = 64;
    uint64_t lastSclkTicks_ = 0;
    uint64_t batchAfterCycles_ = 0;  // batching starts at this guest cycle (OP1EMU_BATCH_AFTER)
    bool panicLog_ = false;
    bool realtime_ = false;
    u32 verifyLo_ = 0, verifyHi_ = 0;
    uint64_t verifyOk_ = 0, verifyBad_ = 0;
    void VerifyAgainst(u32 pc, const CpuState& interp, const std::map<uint32_t, uint8_t>& writes);
    uint32_t interpFirst_ = 0; u32 interpLo_ = 0, interpHi_ = 0xFFFFFFFF;
    std::unordered_map<u32, uint32_t> interpVisits_;
    uint64_t interpNs_ = 0, syncWaitNs_ = 0, syncWaitMaxNs_ = 0;
    uint64_t interpSteps_ = 0, interpBails_ = 0, statsInterpSteps_ = 0, statsInterpBails_ = 0;
    void Pace();
    void PrefetchPack();
    void PrefetchStep();
    size_t prefetchQueued_ = 0;
    uint64_t prefetchScanNs_ = 0;
    uint64_t prefetchAgainNs_ = 0;
    uint64_t lagPrevHostNs_ = 0, lagPrevGuestNs_ = 0, lagPrevBatches_ = 0, lagPrevMmr_ = 0;   // OP1EMU_LAG_LOG
    bool lagLogOn_ = false; bool pcProf_ = false; std::unordered_map<uint32_t, std::pair<uint64_t, uint64_t>> pcProfNs_;   // OP1EMU_PC_PROF
    uint64_t lagPrevInterpNs_ = 0, lagPrevInterpSteps_ = 0; uint64_t lagRunMaxNs_ = 0, lagAfterMaxNs_ = 0, lagRunSumNs_ = 0; uint32_t lagRunMaxPc_ = 0;
    uint64_t paceStartNs_ = 0, paceCalls_ = 0, statsNextNs_ = 0, statsSleptNs_ = 0, statsMaxLagNs_ = 0;
    u32 breakPc_ = 0;
    u32 stopPc_ = 0;
    std::set<u32> breakPcs_;
    uint32_t lastIpend_ = 0;
    unsigned breakHitsLeft_ = 4, breakHit_ = 0;
    void OnBreak();
    void ApplyOverlay();
    uint64_t statsTranslateNs_ = 0, statsTranslateN_ = 0;
    uint64_t statsLastGuestNs_ = 0, statsLastHostNs_ = 0;
    bool stats_ = false;
    uint64_t idleStepCycles_ = 256;  // guest cycles per idle step (OP1EMU_IDLE_STEP)
    void IdleUntilInterrupt();
public:
    void LogJitCacheStats();

    // True once the JIT could not translate or run a block. The CPU state is
    // then no longer advancing, so the host should stop instead of retrying.
    bool Failed() const { return failed_.load(std::memory_order_acquire); }
    // Run() returns as soon as the pc reaches this address (0 = off). The LDR loader's return address for
    // an init block: without it the stop depended on where a batch happened to end.
    void SetStopPc(u32 pc) { stopPc_ = pc; }
    // True once the watchdog reset the system: Run() returns and the host rebuilds the machine (a real
    // reset reloads everything from the boot ROM; the NAND keeps its contents).
    bool ResetRequested() const { return resetRequested_.load(std::memory_order_acquire); }

    void QueueEvent(const std::function<void()>& event, std::chrono::nanoseconds delay = std::chrono::nanoseconds(1));

    void AttachDisplay(const std::shared_ptr<Display>& display);
    void AttachKeyboard(const std::shared_ptr<Keyboard>& keyboard);
    void AttachNandFlash(const std::shared_ptr<NandFlash>& nandFlash);
    void AttachAudioOutput(const std::shared_ptr<AudioOutput>& audioOutput);
    void SetAcceleration(int16_t x, int16_t y, int16_t z);
    void SetPotentiometerValue(u8 value);

    const TimeSource& Time() const { return time_; }
#ifdef ENABLE_PROFILING
    // Profiling hooks (see profiling/profiler.h). Not owned.
    void SetProfiler(Profiler* profiler);
    BcoreStats CoreStats() const;
    void SetCoreEventSink(BcoreEventSink* sink);
    void SetPerfJitdump(bool enable);
    uint64_t EventsDelivered() const { return eventsDelivered_; }
#endif
    // Deterministic mode only: RTC calendar time at guest time zero.
    void SetRtcEpoch(std::chrono::system_clock::time_point epoch) { time_.SetEpoch(epoch); }

protected:
    void ProcessInterrupt(int pin, int level);
    void ServicePanel();
    void ProcessEvents();

    // Declared before the devices, which keep a reference to it.
    TimeSource time_;
    PanelOscillator panel_;

    std::shared_ptr<SIC> sic;
    std::shared_ptr<CoreTimer> coreTimer;
    std::shared_ptr<Watchdog> watchdog_;
    std::atomic<bool> resetRequested_{false};
    std::shared_ptr<GPIO> portG;
    std::shared_ptr<PPI> ppi;
    std::shared_ptr<OLED> oled;
    std::shared_ptr<NFC> nfc;
    std::shared_ptr<GPTimer> gptimer;
    std::shared_ptr<USBDevice> usb;
    std::shared_ptr<SPORT> sport0;
    std::shared_ptr<SPORT> sport1;
    std::shared_ptr<ADXL345> adxl345;
    std::shared_ptr<Potentiometer> potentiometer;
    std::shared_ptr<GPIOPeripheral> gpioOrConnection;
    std::vector<std::shared_ptr<MCP230XX>> gpioExpanders;
    std::vector<std::shared_ptr<Device>> devices;
    std::vector<Device*> processDevices_;  // devices with NeedsProcess(), in devices order
    size_t processDevicesFor_ = 0;          // devices.size() processDevices_ was built for
    // Pending host events in order. raiseIvg >= 0 is a cec_raise(ivg) (the common case: every SIC level
    // update re-forwards the pending interrupts), which needs no std::function.
    struct Event {
        std::chrono::nanoseconds delay;
        int raiseIvg;
        std::function<void()> fn;
    };
    std::vector<Event> eventQueue;
    std::vector<Event> dueEvents_;  // scratch for ProcessEvents
    // Events queued by the CPU thread itself (raises from device polling and MMR writes, NAND busy, ...)
    // go to this unlocked lane, in order; other threads use eventQueue under the mutex. Each lane keeps
    // its own order; how the two interleave depends on host timing anyway.
    std::vector<Event> cpuLane_;
    std::atomic<bool> cpuThreadKnown_{false};
    std::thread::id cpuThreadId_;
    bool OnCpuThread() const {
        return cpuThreadKnown_.load(std::memory_order_acquire) && std::this_thread::get_id() == cpuThreadId_;
    }
    void ProcessLane(std::vector<Event>& lane);
    // IVGs with a raise already in cpuLane_ for the next pass. All raises queued between two passes run in
    // the same pass, and in supervisor mode cec_raise(ivg) only sets ILAT bit ivg and the pending flag, so a
    // second one there is a no-op and is not queued. Off for good once the guest is seen in user mode, where
    // cec_raise can raise an exception instead.
    uint32_t laneRaiseMask_ = 0;
    bool laneUrgent_ = false;
    uint64_t batches_ = 0, batchOne_ = 0, endMmr_ = 0, endPend_ = 0, endIlat_ = 0, endUrg_ = 0, endIdle_ = 0;  // OP1EMU_STATS  // cpuLane_ holds an event that changes state when delivered
    bool sawUserMode_ = false;
    void QueueRaise(int ivg);
    std::recursive_mutex eventQueueMutex;
    std::atomic<size_t> eventCount_{0};     // eventQueue.size(), readable without the lock
    std::chrono::nanoseconds elapsedTime{0};
#ifdef ENABLE_PROFILING
    Profiler* profiler_ = nullptr;
    uint64_t eventsDelivered_ = 0;
#endif
    // Last cyclesElapsed value the GP timers were advanced to (see Run()).
    uint64_t lastTimerCycles_ = 0;
    Emulator emulator;
    std::unique_ptr<CpuState> cpuState_;
    std::unique_ptr<EmulatorMemory> bcoreMemory_;
    std::shared_ptr<Core> core_;
    uint32_t pc;
    std::atomic<bool> failed_{false};
};