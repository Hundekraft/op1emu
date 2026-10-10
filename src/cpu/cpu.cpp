#include "cpu.h"
#include "bcore_memory.h"
#include "ebiu.h"
#include "bootrom.h"
#include "twi.h"
#include "sic.h"
#include "coretimer.h"
#include "otp.h"
#include "gptimer.h"
#include "dma.h"
#include "ppi.h"
#include "gpio.h"
#include "nand.h"
#include "jtag.h"
#include "rtc.h"
#include "usb.h"
#include "sport.h"
#include "emu.h"
#ifdef ENABLE_PROFILING
#include "profiling/profiler.h"
#endif
#include "peripheral/mcp230xx.h"
#include "peripheral/adxl345.h"
#include "peripheral/oled.h"
#include "peripheral/display.h"
#include "peripheral/keyboard.h"
#include "peripheral/potentiometer.h"
#include "peripheral/audio_output.h"
#include "utils/log.h"

#include "core.h"
#include "cpu_state.h"
#include "mmr.h"
#include <cstring>

// bcore CEC functions (extern "C" in bcore's src/cec.h)
extern "C" void cec_raise(CpuState* cpu, uint32_t ivg);
extern "C" void cec_check_pending(CpuState* cpu);
extern "C" bool cec_is_user_mode();

// bcore EVT init (extern in bcore's src/evt.h)
void evt_init();

static constexpr int IVG_IVTMR = 6;

static constexpr int IRQ_TWI = 20;
static constexpr int IRQ_PORTF_A = 45;
static constexpr int IRQ_PORTF_B = 46;
static constexpr int IRQ_PORTG_A = 40;
static constexpr int IRQ_PORTG_B = 41;
static constexpr int IRQ_PORTH_A = 29;
static constexpr int IRQ_PORTH_B = 31;
static constexpr int IRQ_DMA0 = 15;
static constexpr int IRQ_DMA1 = 28;
static constexpr int IRQ_DMA2 = 30;
static constexpr int IRQ_DMA3 = 16;
static constexpr int IRQ_RTC = 14;
static constexpr int IRQ_NFC = 48;
static constexpr int IRQ_USB_INT0 = 52;
static constexpr int IRQ_USB_INT1 = 53;
static constexpr int IRQ_USB_INT2 = 54;
static constexpr int IRQ_USB_DMAINT = 55;
static constexpr int IRQ_GPTIMER_0 = 32;

// Thin CEC device shim so the Emulator device map covers the CEC MMR range.
// bcore intercepts these internally during JIT execution, but the Emulator
// needs a device registered to avoid "unmapped address" warnings.
// Note: CEC state is process-global (module-level statics in bcore's cec.cpp).
// Only one BlackFinCpu instance may exist per process.
class BcoreCECDevice : public Device {
public:
    BcoreCECDevice(CpuState* cpu)
        : Device("CEC", CEC_MMR_BASE, CEC_MMR_SIZE), cpu_(cpu) {}
    void Read(u32 offset, void* buffer, u32 length) override {
        u32 val = cec_mmr_read(cpu_, baseAddress + offset);
        memcpy(buffer, &val, std::min(length, (u32)sizeof(val)));
    }
    void Write(u32 offset, const void* buffer, u32 length) override {
        u32 val = 0;
        memcpy(&val, buffer, std::min(length, (u32)sizeof(val)));
        cec_mmr_write(cpu_, baseAddress + offset, val);
    }
    u32 Read32(u32 offset) override {
        return cec_mmr_read(cpu_, baseAddress + offset);
    }
    void Write32(u32 offset, u32 value) override {
        cec_mmr_write(cpu_, baseAddress + offset, value);
    }
private:
    CpuState* cpu_;
};

// Thin EVT device shim for the same reason.
class BcoreEVTDevice : public Device {
public:
    BcoreEVTDevice()
        : Device("EVT", EVT_BASE, EVT_SIZE) {}
    void Read(u32 offset, void* buffer, u32 length) override {
        u32 val = evt_read(baseAddress + offset);
        memcpy(buffer, &val, std::min(length, (u32)sizeof(val)));
    }
    void Write(u32 offset, const void* buffer, u32 length) override {
        u32 val = 0;
        memcpy(&val, buffer, std::min(length, (u32)sizeof(val)));
        evt_write(baseAddress + offset, val);
    }
    u32 Read32(u32 offset) override {
        return evt_read(baseAddress + offset);
    }
    void Write32(u32 offset, u32 value) override {
        evt_write(baseAddress + offset, value);
    }
};

BlackFinCpu::BlackFinCpu(bool deterministic) : time_(deterministic), pc(0) {
    if (const char* b = getenv("OP1EMU_BATCH")) batchCycles_ = std::max(1ull, strtoull(b, nullptr, 10));
    if (const char* b = getenv("OP1EMU_BATCH_AFTER")) batchAfterCycles_ = strtoull(b, nullptr, 10);
    panicLog_ = getenv("OP1_PANIC_LOG") != nullptr;
    if (const char* b = getenv("OP1EMU_IDLE_STEP")) idleStepCycles_ = std::max(1ull, strtoull(b, nullptr, 10));
    cpuState_ = std::make_unique<CpuState>();
    memset(cpuState_.get(), 0, sizeof(CpuState));

    auto irqHandler = [this](int q, int level) { this->ProcessInterrupt(q, level); };
    devices.emplace_back(std::make_shared<MemoryDevice>("L1 SRAM", 0xFFB00000, 0x1000));
    devices.emplace_back(std::make_shared<MemoryDevice>("PORT_MUX", 0xFFC03200, 0x100));
    devices.emplace_back(std::make_shared<MemoryDevice>("Data A",   0xFF800000, 0x4000));
    devices.emplace_back(std::make_shared<MemoryDevice>("Data A Cache", 0xFF804000, 0x4000));
    devices.emplace_back(std::make_shared<MemoryDevice>("Data B",   0xFF900000, 0x4000));
    devices.emplace_back(std::make_shared<MemoryDevice>("Data B Cache", 0xFF904000, 0x4000));
    devices.emplace_back(std::make_shared<MemoryDevice>("Inst A",   0xFFA00000, 0x8000));
    devices.emplace_back(std::make_shared<MemoryDevice>("Inst B",   0xFFA08000, 0x4000));
    devices.emplace_back(std::make_shared<MemoryDevice>("Inst Cache", 0xFFA10000, 0x4000));
    devices.emplace_back(std::make_shared<MemoryDevice>("SDRAM", 0, 0x8000000));

    // CEC/EVT shim devices for emulator device map coverage
    devices.emplace_back(std::make_shared<BcoreEVTDevice>());
    devices.emplace_back(std::make_shared<BcoreCECDevice>(cpuState_.get()));
    // MMU region — register as plain memory (bcore does not use a separate MMU device)
    devices.emplace_back(std::make_shared<MemoryDevice>("MMU", 0xFFE00000, 0x2000));

    devices.emplace_back(std::make_shared<EBIU>(0xFFC00A00));
    devices.emplace_back(std::make_shared<OTP>(0xFFC03600, "otp.bin"));
    std::shared_ptr<USB> usb = std::make_shared<USB>(0xFFC03800);
    usb->BindInterrupt(IRQ_USB_INT0, IRQ_USB_INT1, IRQ_USB_INT2, IRQ_USB_DMAINT, irqHandler);
    devices.emplace_back(usb);
    this->usb = usb;
    sport0 = std::make_shared<SPORT>(0xFFC00800, 0, time_);
    devices.emplace_back(sport0);
    sport1 = std::make_shared<SPORT>(0xFFC00900, 1, time_);
    devices.emplace_back(sport1);
    // OP-1 seems only use last byte of DSPID, which is 0x02 for BF524 rev 02
    devices.emplace_back(std::make_shared<Jtag>(0xFFE05000, 0x02));
    devices.emplace_back(std::make_shared<BootROM>(0xEF000000));
    gptimer = std::make_shared<GPTimer>(0xFFC00600);
    for (int i = 0; i < 8; i++) {
        gptimer->BindInterrupt(i, IRQ_GPTIMER_0 + i, irqHandler);
    }
    devices.emplace_back(gptimer);
    nfc = std::make_shared<NFC>(0xFFC03700);
    nfc->BindInterrupt(IRQ_NFC, irqHandler);
    devices.emplace_back(nfc);

    std::shared_ptr<RTC> rtc = std::make_shared<RTC>(0xFFC00300, time_);
    rtc->BindInterrupt(IRQ_RTC, irqHandler);
    devices.emplace_back(rtc);

    ppi = std::make_shared<PPI>(0xFFC01000);
    devices.emplace_back(ppi);
    std::shared_ptr<DMA> dma = std::make_shared<DMA>(0xFFC00C00, emulator);
    devices.emplace_back(dma);
    dma->AttachDMABus(DMAPeripheralType::DMAPeripheralPPI, ppi);
    dma->AttachDMABus(DMAPeripheralType::DMAPeripheralNFC, nfc);
    dma->AttachDMABus(DMAPeripheralType::DMAPeripheralSPORT0Rx, sport0);
    dma->AttachDMABus(DMAPeripheralType::DMAPeripheralSPORT0Tx, sport0);
    dma->AttachDMABus(DMAPeripheralType::DMAPeripheralSPORT1Rx, sport1);
    dma->AttachDMABus(DMAPeripheralType::DMAPeripheralSPORT1Tx, sport1);
    auto memorySrcBus0 = std::make_shared<MemorySrcDMABus>();
    auto memoryDestBus0 = std::make_shared<MemoryDestDMABus>(*memorySrcBus0);
    dma->AttachDMABus(DMAPeripheralType::DMAPeripheralMDMADest0, memoryDestBus0);
    dma->AttachDMABus(DMAPeripheralType::DMAPeripheralMDMASrc0, memorySrcBus0);
    auto memorySrcBus1 = std::make_shared<MemorySrcDMABus>();
    auto memoryDestBus1 = std::make_shared<MemoryDestDMABus>(*memorySrcBus1);
    dma->AttachDMABus(DMAPeripheralType::DMAPeripheralMDMADest1, memoryDestBus1);
    dma->AttachDMABus(DMAPeripheralType::DMAPeripheralMDMASrc1, memorySrcBus1);
    dma->BindInterrupt(0, IRQ_DMA0, irqHandler);
    dma->BindInterrupt(1, IRQ_DMA1, irqHandler);
    dma->BindInterrupt(2, IRQ_DMA2, irqHandler);
    dma->BindInterrupt(3, IRQ_DMA3, irqHandler);
    std::shared_ptr<GPIO> portF = std::make_shared<GPIO>("PORTF", 0xFFC00700);
    portF->BindInterruptA(IRQ_PORTF_A, irqHandler);
    portF->BindInterruptB(IRQ_PORTF_B, irqHandler);
    devices.emplace_back(portF);
    portG = std::make_shared<GPIO>("PORTG", 0xFFC01500);
    portG->BindInterruptA(IRQ_PORTG_A, irqHandler);
    portG->BindInterruptB(IRQ_PORTG_B, irqHandler);
    devices.emplace_back(portG);
    std::shared_ptr<GPIO> portH = std::make_shared<GPIO>("PORTH", 0xFFC01700);
    portH->BindInterruptA(IRQ_PORTH_A, irqHandler);
    portH->BindInterruptB(IRQ_PORTH_B, irqHandler);
    devices.emplace_back(portH);

    sic = std::make_shared<SIC>(0xFFC00100);
    sic->SetInterruptForwardCallback([this](int ivg, int level) {
        if (level) QueueRaise(ivg);
    });
    devices.push_back(sic);
    nfc->SetIVGResolver([this]() { return sic->GetIVG(IRQ_NFC); });
    coreTimer = std::make_shared<CoreTimer>(0xFFE03000, time_);
    coreTimer->BindInterrupt(IVG_IVTMR, [this](int ivg, int level) {
        if (level) QueueRaise(ivg);
    });
    devices.emplace_back(coreTimer);

    std::shared_ptr<TWI> twi = std::make_shared<TWI>(0xFFC01400);
    devices.push_back(twi);

    for (int i = 0; i < 8; i++) {
        u32 addr = 0x20 + i;
        gpioExpanders.push_back(std::make_shared<MCP230XX>(addr, MCP230XXModel::MCP23017));
        twi->AttachPeripheral(gpioExpanders.back());
    }
    // TODO: this bit controls DAT_01f007e0, figure out its function
    gpioExpanders[0]->SetPinInput(6, GPIOPinLevel::High); // Set MCP23017 #6 high
    // Connect gpio expanders
    gpioExpanders[3]->Connect(16, {*gpioExpanders[2], 0}); // Connect gpio3 INTA to gpio2 #0
    gpioExpanders[4]->Connect(16, {*gpioExpanders[2], 1}); // Connect gpio4 INTA to gpio2 #1
    gpioExpanders[6]->Connect(16, {*gpioExpanders[2], 2}); // Connect gpio6 INTA to gpio2 #2
    gpioExpanders[5]->Connect(16, {*gpioExpanders[2], 3}); // Connect gpio5 INTA to gpio2 #3

    gpioOrConnection = std::make_shared<GPIOOrGate>(true); // active low
    gpioOrConnection->Connect(2, {*portG, 0}); // Connect GPIOOr output to portG #0
    gpioExpanders[2]->Connect(16, {*gpioOrConnection, 0}); // Connect gpio2 INTA to GPIOOr #0
    // FIXME: figure out how gpio2 INTA and gpio0 INTA are connected to portG #0
    gpioExpanders[0]->Connect(16, {*gpioOrConnection, 1}); // Connect gpio0 INTA to GPIOOr #1

    adxl345 = std::make_shared<ADXL345>(0x53);
    adxl345->Connect(0, {*gpioExpanders[0], 1}); // Connect ADXL345 INT1 to gpio0 #1

    twi->AttachPeripheral(adxl345);
    // This one byte controls DAT_ff802894, which seams to be "is_not_display_flip"
    twi->AttachPeripheral(std::make_shared<DummyI2CPeripheral>(0x1a, 0x0)); // Dummy I2C device
    twi->AttachPeripheral(std::make_shared<DummyI2CPeripheral>(0x18, 0x0)); // Dummy I2C device
    twi->AttachPeripheral(std::make_shared<DummyI2CPeripheral>(0x58, 0x0)); // Dummy I2C device
    twi->AttachPeripheral(std::make_shared<DummyI2CPeripheral>(0x09, 0x0)); // Dummy I2C device
    twi->AttachPeripheral(std::make_shared<DummyI2CPeripheral>(0x4a, 0x0)); // probably ADC?
    twi->AttachPeripheral(std::make_shared<DummyI2CPeripheral>(0x64, 0x3C)); // LTC2941 battgauge
    twi->AttachPeripheral(std::make_shared<DummyI2CPeripheral>(0x11, 0x80)); // Si4713? FM radio
    // TODO: figure out the exact device of the potentiometer
    potentiometer = std::make_shared<Potentiometer>(0x54);
    twi->AttachPeripheral(potentiometer);
    twi->BindInterrupt(IRQ_TWI, irqHandler);

    for (const auto& device : devices) {
        emulator.BindDevice(device.get());
    }

    std::array<GPIOPeripheral::GPIOConnection, OLED::DATA_PINS> oledDatabus{
        std::tuple<GPIOPeripheral&, int>{*portF, 0},
        std::tuple<GPIOPeripheral&, int>{*portF, 1},
        std::tuple<GPIOPeripheral&, int>{*portF, 2},
        std::tuple<GPIOPeripheral&, int>{*portF, 3},
        std::tuple<GPIOPeripheral&, int>{*portF, 4},
        std::tuple<GPIOPeripheral&, int>{*portF, 5},
        std::tuple<GPIOPeripheral&, int>{*portF, 6},
        std::tuple<GPIOPeripheral&, int>{*portF, 7},
        std::tuple<GPIOPeripheral&, int>{*portF, 8},
        std::tuple<GPIOPeripheral&, int>{*portF, 9},
        std::tuple<GPIOPeripheral&, int>{*portF, 10},
        std::tuple<GPIOPeripheral&, int>{*portF, 11},
        std::tuple<GPIOPeripheral&, int>{*portF, 12},
        std::tuple<GPIOPeripheral&, int>{*portF, 13},
        std::tuple<GPIOPeripheral&, int>{*portF, 14},
        std::tuple<GPIOPeripheral&, int>{*portF, 15},
    };
    std::tuple<GPIOPeripheral&, int> oledWr {*portG, 11};
    std::tuple<GPIOPeripheral&, int> oledCs {*portG, 5};
    std::tuple<GPIOPeripheral&, int> oledRd {*portG, 4};
    std::tuple<GPIOPeripheral&, int> oledRs {*portG, 2};
    oled = std::make_shared<OLED>(oledDatabus, oledCs, oledRs, oledRd, oledWr);

    // Initialize bcore after all devices are bound
    bcoreMemory_ = std::make_unique<EmulatorMemory>(emulator);
    core_ = std::make_shared<Core>(cpuState_.get(), bcoreMemory_.get());
    // IR pipeline at O1, backend at CodeGenOpt::Default. Until bcore wired
    // optimize_module into compilation, the IR level only ever affected the
    // IR-dump output, so init(2) compiled with the backend alone. Measured on
    // a boot to the main display with the backend at CodeGenOpt::Less (this
    // exact pairing with Default was not timed): O1 -6% execution for +43%
    // compile; O2 no faster than O1 and +69% compile.
    core_->init(/*opt_level=*/1, /*codegen_level=*/2);

    // Initialize bcore CEC and EVT
    cec_init();
    evt_init();

    SetRegister(RegIndex::SP, 0x7000000); // Set stack pointer to top of SDRAM
    SetRegister(RegIndex::FP, 0x7000000); // Set frame pointer to top of SDRAM
    cpuState_->ksp = 0x7000000;
    cpuState_->usp = 0x7000000;
    cpuState_->syscfg = 0x30;

    time_.StartWallClock();
}

BlackFinCpu::~BlackFinCpu() {
}

void BlackFinCpu::ProcessInterrupt(int pin, int level) {
    sic->SetInterruptLevel(pin, level);
}

void BlackFinCpu::HaltExecution(HaltReason reason) {
    // TODO
}

void BlackFinCpu::SaveContext() {
    // TODO
}

void BlackFinCpu::RestoreContext() {
    // TODO
}

static void SetBfinCycles(CpuState& cpu_state, u64 cycles) {
    cpu_state.cycles[0] = (u32)(cycles & 0xffffffff);
    cpu_state.cycles[1] = (u32)(cycles >> 32);
    cpu_state.cycles[2] = cpu_state.cycles[1];
}

HaltReason BlackFinCpu::Run() {
    // Runs guest blocks back to back and does the between-block work (interrupts, device polling,
    // timers, events) once per batch of at least batchCycles_ guest cycles instead of after every
    // block: most blocks are 2-6 instructions, and the per-block device work cost more than the guest
    // code. CYCLES and the core timer still update per block. A batch ends early at IDLE, and after any
    // block that touched an MMR or raised an interrupt: device-facing code keeps per-block behavior
    // (some drivers depend on it, e.g. the NFC write-done handshake), compute and DSP loops batch.
    uint64_t batchStart = 0;
    bool idle = false;
    // Batching starts once the firmware is up: 1 s of guest time after it first enters its main display
    // routine (R246), or after 20 s of guest time for firmware where that address means nothing. During
    // boot it stays per-block: R246's startup has a thread race that coarser device polling loses (an
    // early allocation panics once the "panics are fatal" flag is already set).
    if (!batchArmed_) {
        const uint64_t now = time_.Cycles();
        if (cpuState_->pc == kMainDisplayPc && batchAfterCycles_ == UINT64_MAX) batchAfterCycles_ = now + TimeSource::kHz;
        if (now >= batchAfterCycles_ || now >= 20 * TimeSource::kHz) batchArmed_ = true;
    }
    const uint64_t batch = batchArmed_ ? batchCycles_ : 1;
    for (uint64_t executed = 0; executed < batch && !idle;) {
        // Guest time at the start of this block: host time, or in deterministic
        // mode the packets every earlier block executed (see TimeSource).
        const uint64_t cyclesElapsed = time_.Cycles();
        if (executed == 0) batchStart = cyclesElapsed;
        SetBfinCycles(*cpuState_, cyclesElapsed);
        coreTimer->UpdateCycles(cyclesElapsed);

#ifdef ENABLE_PROFILING
        const uint32_t blockPc = cpuState_->pc;
        if (profiler_) profiler_->BeforeRun(blockPc);
#endif
        // R246's panic(fmt, ...) at 0x01919AA8: log the format and first arguments (OP1_PANIC_LOG=1).
        // Most early-boot panics are suppressed by the firmware itself; a fatal one halts at 0x0191995A.
        if (cpuState_->pc == 0x01919AA8 && panicLog_) {
            auto str = [&](u32 a) { std::string m; for (int k = 0; k < 200; ++k) { char c = (char)emulator.MemoryRead8(a + k); if (!c) break; m += c; } return m; };
            fprintf(stderr, "FIRMWARE PANIC: fmt='%s' r1=0x%08x ('%s') r2=0x%08x r3=0x%08x rets=0x%08x ipend_ivg=%d\n", str(cpuState_->dpregs[0]).c_str(),
                    cpuState_->dpregs[1], str(cpuState_->dpregs[1]).c_str(), cpuState_->dpregs[2], cpuState_->dpregs[3], cpuState_->rets, cec_current_ivg());
        }
        // Hit entry point, invalidating core to reset cached translations
        if (cpuState_->pc == 0xFFA00000) {
            core_->invalidate();
        }
        // Execute one basic block — bcore updates cpuState_->pc internally.
        // Hardware loops, PC advance, and hwloop counters are all handled by bcore.
        if (!core_->run(cpuState_->pc)) {
            // Translation/JIT failure: PC is unchanged, so retrying would loop
            // forever. Skip device processing and let the host stop the run.
            LogError("JIT execution failed at PC 0x%08x", cpuState_->pc);
            failed_.store(true, std::memory_order_release);
            return HaltReason::Break;
        }
        time_.Retire(cpuState_->packets);
        executed += cpuState_->packets ? cpuState_->packets : 1;
#ifdef ENABLE_PROFILING
        if (profiler_) profiler_->AfterRun(blockPc, cpuState_->packets);
#endif
        cpuState_->did_jump = false; // Clear jump flag set by bcore, since we handle it in the emulator loop
        idle = cec_take_idle();
        if (bcoreMemory_->TakeMmrTouched() || cec_pending()) break;
    }
    AfterBlock(batchStart);
    if (idle) IdleUntilInterrupt();
    return HaltReason::Break;
}

// IDLE stalls the core until an interrupt. Instead of executing the firmware's idle loop over and over,
// let guest time pass in small steps with the same per-block device work, until a wake-up: an interrupt
// is taken (cec_check_pending in AfterBlock moves pc to the handler), one is latched in ILAT, or a
// peripheral asserts one enabled in SIC_IWR. The last two wake the core even when IMASK/SIC_IMASK keep
// the interrupt from being serviced (the boot ROM idles that way, often with the interrupt already
// pending, in which case IDLE does not sleep at all). The step limit returns control to the host loop.
// In deterministic mode the steps add guest cycles; in wall mode guest time is host time anyway.
void BlackFinCpu::IdleUntilInterrupt() {
    const uint64_t kIdleStepCycles = idleStepCycles_;  // bounds the extra interrupt latency
    const int kMaxIdleSteps = (int)std::max<uint64_t>(1, 262144 / idleStepCycles_);  // ~0.65 ms, then back to the host loop
    const u32 idlePc = cpuState_->pc;
    auto awake = [&] { return cpuState_->pc != idlePc || cec_ilat() || sic->WakeupPending(); };
    for (int i = 0; i < kMaxIdleSteps && !awake(); ++i) {
        if (time_.Deterministic()) time_.Retire(kIdleStepCycles);
        const uint64_t cyclesElapsed = time_.Cycles();
        SetBfinCycles(*cpuState_, cyclesElapsed);
        coreTimer->UpdateCycles(cyclesElapsed);
        AfterBlock(cyclesElapsed);
    }
}

// Everything that happens between two guest blocks: interrupts, device polling, timers, events.
void BlackFinCpu::AfterBlock(uint64_t cyclesElapsed) {
    cec_check_pending(cpuState_.get());

    // Get active IVG from CEC
    int ivg = cec_current_ivg();
    if (processDevicesFor_ != devices.size()) {
        processDevices_.clear();
        for (const auto& device : devices)
            if (device->NeedsProcess()) processDevices_.push_back(device.get());
        processDevicesFor_ = devices.size();
    }
    for (Device* device : processDevices_) {
        device->ProcessWithInterrupt(ivg);
    }
    // Drive the GP timers off the *delta* in cyclesElapsed, not off it landing
    // on a multiple. `cyclesElapsed % 10000 == 0` only fires when the sampled
    // value happens to be an exact multiple of 10000, which is essentially
    // never true for a value derived from wall-clock microseconds -- most
    // ticks were silently dropped, and occasionally the value sat on a
    // multiple across calls and ticked repeatedly, so the emulated GP timer
    // rate was erratic (measured: programmed period=200 firing at 10-19 ms
    // instead of the 5 ms that rate implies).
    //
    // An accumulator is both correct and cheap. At most kMaxCatchupTicks are
    // delivered per call and the rest of a long gap is dropped on purpose
    // (the accumulator still advances by the full gap): after a host stall or
    // suspend/resume, replaying the backlog would make the guest storm timer
    // interrupts for as long as the gap was.
    // FIXME: still one shared rate for all three clock domains, which is not
    // how SCLK/TACLK/TMRCLK relate on real hardware.
    constexpr uint64_t kCyclesPerTimerTick = 10000;
    constexpr uint64_t kMaxCatchupTicks = 64;
    uint64_t ticks = ((uint64_t)cyclesElapsed - lastTimerCycles_) / kCyclesPerTimerTick;
    if (ticks) {
        lastTimerCycles_ += ticks * kCyclesPerTimerTick;
        if (ticks > kMaxCatchupTicks) ticks = kMaxCatchupTicks;
        for (uint64_t i = 0; i < ticks; ++i) {
            gptimer->Tick(GPTimerClockTypeTACLK);
        }
    }
    // SCLK-clocked timers run at the system clock, CCLK / kCclkPerSclk (assumed 400/100 MHz: there is no
    // PLL model yet). TMRCLK is derived from timer 7 inside GPTimer::Advance. Only TACLK keeps the 40 kHz
    // tick above. Ticking everything at 40 kHz made the display's frame-sync timers (0/1 on TMRCLK) about
    // 180x too slow, and the firmware spent most of its time polling TIMER_STATUS for them to stop.
    constexpr uint64_t kCclkPerSclk = 4;
    constexpr uint64_t kMaxCatchupSclk = 100000000;  // 1 s; drop the rest of a host stall
    const uint64_t sclk = (uint64_t)cyclesElapsed / kCclkPerSclk;
    if (sclk > lastSclkTicks_) {
        gptimer->Advance(GPTimerClockTypeSCLK, std::min(sclk - lastSclkTicks_, kMaxCatchupSclk));
        lastSclkTicks_ = sclk;
    }

    if (time_.Deterministic()) ServicePanel();
    ProcessEvents();
}

#ifdef ENABLE_PROFILING
void BlackFinCpu::SetProfiler(Profiler* profiler) {
    profiler_ = profiler;
    bcoreMemory_->SetProfiler(profiler);
}

BcoreStats BlackFinCpu::CoreStats() const { return core_->stats(); }

void BlackFinCpu::SetCoreEventSink(BcoreEventSink* sink) { core_->setEventSink(sink); }

void BlackFinCpu::SetPerfJitdump(bool enable) { core_->set_perf_jitdump(enable); }
#endif

void BlackFinCpu::ServicePanel() {
    if (auto low = panel_.Service(time_.Nanoseconds()))
        portG->SetPinInput(3, *low ? GPIOPinLevel::Low : GPIOPinLevel::High);
}

void BlackFinCpu::SetRegister(int index, u32 value) {
    switch (index)
    {
    case RegIndex::FP:
        cpuState_->dpregs[15] = value;
        break;
    case RegIndex::SP:
        cpuState_->dpregs[14] = value;
        break;
    case RegIndex::RETS:
        cpuState_->rets = value;
        break;
    case RegIndex::R0...RegIndex::R2:
        cpuState_->dpregs[index - RegIndex::R0] = value;
        break;
    case RegIndex::P1:
        cpuState_->dpregs[9] = value;
        break;
    default:
        break;
    }
}

u32 BlackFinCpu::GetRegister(int index) {
    switch (index)
    {
    case RegIndex::FP:
        return cpuState_->dpregs[15];
    case RegIndex::SP:
        return cpuState_->dpregs[14];
    case RegIndex::RETS:
        return cpuState_->rets;
    case RegIndex::R0...RegIndex::R2:
        return cpuState_->dpregs[index - RegIndex::R0];
    case RegIndex::P1:
        return cpuState_->dpregs[9];
    default:
        return 0;
    }
}

void BlackFinCpu::SetPC(u32 value) {
    cpuState_->pc = value;
}

u32 BlackFinCpu::PC() {
    return cpuState_->pc;
}

void BlackFinCpu::QueueEvent(const std::function<void()>& event, std::chrono::nanoseconds delay) {
    if (OnCpuThread()) {
        cpuLane_.push_back({delay, -1, event});
        return;
    }
    std::unique_lock<std::recursive_mutex> lock(eventQueueMutex);
    eventQueue.push_back({delay, -1, event});
    eventCount_.store(eventQueue.size(), std::memory_order_release);
}

void BlackFinCpu::QueueRaise(int ivg) {
    // Same 1 ns default delay as QueueEvent: the raise is delivered on the next ProcessEvents.
    if (OnCpuThread()) {
        if (!sawUserMode_ && cec_is_user_mode()) sawUserMode_ = true;
        const uint32_t bit = 1u << (ivg & 31);
        if (!sawUserMode_ && (laneRaiseMask_ & bit)) return;
        laneRaiseMask_ |= bit;
        cpuLane_.push_back({std::chrono::nanoseconds(1), ivg, {}});
        return;
    }
    std::unique_lock<std::recursive_mutex> lock(eventQueueMutex);
    eventQueue.push_back({std::chrono::nanoseconds(1), ivg, {}});
    eventCount_.store(eventQueue.size(), std::memory_order_release);
}

void BlackFinCpu::ProcessEvents() {
    if (!cpuThreadKnown_.load(std::memory_order_relaxed)) {
        cpuThreadId_ = std::this_thread::get_id();
        cpuThreadKnown_.store(true, std::memory_order_release);
    }
    if (!cpuLane_.empty()) ProcessLane(cpuLane_);
    laneRaiseMask_ = 0;  // raises queued from here on run in the pass after this one
    // Fast path: nearly every block runs with no events from other threads. Skip the lock.
    if (eventCount_.load(std::memory_order_acquire) != 0) {
        std::unique_lock<std::recursive_mutex> lock(eventQueueMutex);
        ProcessLane(eventQueue);
        eventCount_.store(eventQueue.size(), std::memory_order_release);
    }
    elapsedTime += std::chrono::nanoseconds(1);
}

// Runs the events of one lane that were due when the call started, in order, then removes the due ones
// and counts the others down by 1 ns. Events queued while running are not run in this pass and, if
// due, are dropped by the removal (the behavior of the original copy-based loop, kept as is). Due events
// are moved out first, so an event that queues another cannot invalidate the one running.
void BlackFinCpu::ProcessLane(std::vector<Event>& lane) {
    constexpr std::chrono::nanoseconds instructionTime(1); // Approximate instruction execution time
    std::vector<Event> due;
    due.swap(dueEvents_);
    due.clear();
    for (auto& e : lane)
        if (e.delay <= std::chrono::nanoseconds(0)) due.push_back(std::move(e));
    for (auto& e : due) {
        if (e.raiseIvg >= 0) cec_raise(cpuState_.get(), e.raiseIvg);
        else e.fn();
#ifdef ENABLE_PROFILING
        ++eventsDelivered_;
#endif
    }
    size_t w = 0;
    for (size_t r = 0; r < lane.size(); ++r) {
        if (lane[r].delay <= std::chrono::nanoseconds(0)) continue;
        lane[r].delay -= instructionTime;
        if (w != r) lane[w] = std::move(lane[r]);
        ++w;
    }
    lane.resize(w);
    due.swap(dueEvents_);
}

void BlackFinCpu::AttachDisplay(const std::shared_ptr<Display>& display) {
    ppi->AttachDisplay(display);
    // Deterministic mode drives TE from guest time (ServicePanel); the
    // frontend only presents pixels. Wall mode keeps the frontend's host poll.
    if (time_.Deterministic()) return;
    display->SetOnFrameStartCallback([this](Display& disp) {
        this->QueueEvent([this]() {
            this->portG->SetPinInput(3, GPIOPinLevel::Low);
        });
        this->QueueEvent([this]() {
            this->portG->SetPinInput(3, GPIOPinLevel::High);
        }, std::chrono::nanoseconds(1000));
    });
}

void BlackFinCpu::AttachNandFlash(const std::shared_ptr<NandFlash>& nandFlash) {
    nfc->AttachNandFlash(nandFlash);
}

void BlackFinCpu::AttachKeyboard(const std::shared_ptr<Keyboard>& keyboard) {
    keyboard->SetKeyEventCallback([this](int bank, int index, bool pressed) {
        this->QueueEvent([this, bank, index, pressed]() {
            // Map keyboard events to GPIO expander pins
            gpioExpanders[bank]->SetPinInput(index, pressed ? GPIOPinLevel::Low : GPIOPinLevel::High);
        });
    });
}

void BlackFinCpu::AttachAudioOutput(const std::shared_ptr<AudioOutput>& audioOutput) {
    auto cb = [audioOutput](const void* data, size_t samples, int channels, int bitsPerSample) {
        audioOutput->WriteSamples(data, samples, channels, bitsPerSample);
    };
    sport0->SetAudioOutputCallback(cb);
    sport1->SetAudioOutputCallback(cb);
}

void BlackFinCpu::SetAcceleration(int16_t x, int16_t y, int16_t z) {
    QueueEvent([this, x, y, z]() {
        this->adxl345->SetAcceleration(x, y, z);
    });
}

void BlackFinCpu::SetPotentiometerValue(u8 value) {
    QueueEvent([this, value]() {
        this->potentiometer->SetValue(value);
    });
}

void BlackFinCpu::SetJitCacheDir(const std::string& dir) {
    core_->set_object_cache_dir(dir);
    if (!dir.empty()) LogInfo("JIT object cache: %s", dir.c_str());
}

void BlackFinCpu::LogJitCacheStats() {
    const uint64_t hits = core_->object_cache_hits(), misses = core_->object_cache_misses();
    if (hits || misses) LogInfo("JIT object cache: %llu hits, %llu compiled", (unsigned long long)hits, (unsigned long long)misses);
}

void BlackFinCpu::SetBootMode(int mode) {
    sic->SetBootMode(mode);
}