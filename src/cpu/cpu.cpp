#include <map>
#include <set>
#include <iterator>
#include <sstream>
#include <fstream>
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
#include "interp.h"
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

namespace {
// Debug (OP1EMU_INTERP_VERIFY): reads see the emulator's memory plus this run's own writes; writes stay here.
class OverlayMemory : public Memory {
public:
    explicit OverlayMemory(Memory* base) : base_(base) {}
    uint32_t base() const override { return 0; }
    uint32_t size() const override { return 0xFFFFFFFF; }
    uint8_t read8(uint32_t a) const override {
        if (a >= 0xFFC00000u) { mmr_ = true; return 0; }   // never touch devices from the check
        auto it = w_.find(a);
        return it != w_.end() ? it->second : base_->read8(a);
    }
    bool touchedMmr() const { return mmr_; }
    uint16_t read16(uint32_t a) const override { return (uint16_t)(read8(a) | read8(a + 1) << 8); }
    uint32_t read32(uint32_t a) const override { return (uint32_t)read16(a) | (uint32_t)read16(a + 2) << 16; }
    void write8(uint32_t a, uint8_t v) override { if (a >= 0xFFC00000u) mmr_ = true; w_[a] = v; }
    void write16(uint32_t a, uint16_t v) override { write8(a, (uint8_t)v); write8(a + 1, (uint8_t)(v >> 8)); }
    void write32(uint32_t a, uint32_t v) override { write16(a, (uint16_t)v); write16(a + 2, (uint16_t)(v >> 16)); }
    const uint8_t* raw() const override { return nullptr; }
    uintptr_t fast_base() const override { return 0; }
    uint32_t rawmem_limit() const override { return 0; }
    const std::map<uint32_t, uint8_t>& writes() const { return w_; }
private:
    Memory* base_;
    std::map<uint32_t, uint8_t> w_;
    mutable bool mmr_ = false;
};
}  // namespace

BlackFinCpu::BlackFinCpu(bool deterministic) : time_(deterministic), pc(0) {
    pcProf_ = getenv("OP1EMU_PC_PROF") != nullptr;
    if (const char* b = getenv("OP1EMU_BATCH")) batchCycles_ = std::max(1ull, strtoull(b, nullptr, 10));
    if (const char* b = getenv("OP1EMU_BATCH_AFTER")) batchAfterCycles_ = strtoull(b, nullptr, 10);
    panicLog_ = getenv("OP1_PANIC_LOG") != nullptr;
    stats_ = getenv("OP1EMU_STATS") != nullptr;
    if (const char* v = getenv("OP1EMU_INTERP_VERIFY")) {
        char* end = nullptr;
        verifyLo_ = (u32)strtoul(v, &end, 16);
        verifyHi_ = (end && *end == '-') ? (u32)strtoul(end + 1, nullptr, 16) : 0xFFFFFFFF;
    }
    if (const char* f = getenv("OP1EMU_INTERP_FIRST")) {
        interpFirst_ = (uint32_t)strtoul(f, nullptr, 10);
        if (const char* c = strchr(f, ',')) {
            char* end = nullptr;
            interpLo_ = (u32)strtoul(c + 1, &end, 16);
            if (end && *end == '-') interpHi_ = (u32)strtoul(end + 1, nullptr, 16);
        }
    }
    if (const char* b = getenv("OP1EMU_BREAK")) {  // PC[+PC...][,HITS]: see OnBreak
        for (const char* q = b; *q && *q != ',';) {
            char* end = nullptr;
            const u32 pc = strtoul(q, &end, 16);
            if (end == q) break;
            breakPcs_.insert(pc);
            q = (*end == '+') ? end + 1 : end;
        }
        breakPc_ = breakPcs_.empty() ? 0 : *breakPcs_.begin();
        if (const char* c = strchr(b, ',')) breakHitsLeft_ = strtoul(c + 1, nullptr, 10);
    }
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
    // System interrupts are levels; the core latches one into ILAT only while its IVG is not being serviced.
    // Forwarding a still-asserted source during its own handler (SIC re-forwards on every line change) latched
    // it again, so the handler ran twice: the TX DMA handler rendered two audio blocks per 64-frame buffer.
    // AfterBlock re-forwards when an IPEND bit clears (RTI), so a level that is still asserted is taken again.
    sic->SetInterruptForwardCallback([this](int ivg, int level) {
        if (level && !(cec_ipend() & (1u << (ivg & 31)))) QueueRaise(ivg);
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

    // Debug: OP1EMU_WATCH=ADDR[,WORDS] (hex address) logs the guest block pc of 32-bit reads of those words.
    // A hook in SDRAM turns off the JIT's inline SDRAM access (and so changes the JIT cache salt).
    if (const char* w = getenv("OP1EMU_WATCH")) {
        const u32 base = strtoul(w, nullptr, 16);
        const char* c = strchr(w, ',');
        const u32 words = c ? strtoul(c + 1, nullptr, 10) : 1;
        for (u32 i = 0; i < words; ++i) {
            emulator.AddReadHook(base + i * 4, [this](u32 addr) {
                u32 v = 0;
                emulator.MemoryRead(addr, &v, 4);
                static unsigned hits = 0;
                if (hits++ < 200) LogInfo("watch: read [%#x] = %#x, block pc %#x", addr, v, cpuState_->pc);
                return v;
            });
        }
    }

    // Debug: OP1EMU_WATCHW=ADDR[,WORDS] (hex address) logs the guest block pc of 32-bit writes to those words.
    if (const char* w = getenv("OP1EMU_WATCHW")) {
        const u32 base = strtoul(w, nullptr, 16);
        const char* c = strchr(w, ',');
        const u32 words = c ? strtoul(c + 1, nullptr, 10) : 1;
        for (u32 i = 0; i < words; ++i) {
            emulator.AddWriteHook(base + i * 4, [this](u32 addr, u32 v) {
                emulator.MemoryWrite(addr, &v, 4);
                static unsigned hits = 0;
                if (hits++ < 200) LogInfo("watchw: write [%#x] = %#x, block pc %#x", addr, v, cpuState_->pc);
            });
        }
    }

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
    if (!pcProf_ || pcProfNs_.empty()) return;
    std::vector<std::pair<uint32_t, std::pair<uint64_t, uint64_t>>> v(pcProfNs_.begin(), pcProfNs_.end());
    std::sort(v.begin(), v.end(), [](const auto& a, const auto& b) { return a.second.first > b.second.first; });
    uint64_t total = 0;
    for (const auto& e : v) total += e.second.first;
    fprintf(stderr, "pc profile: %.1f ms in block runs\n", total / 1e6);
    for (size_t i = 0; i < v.size() && i < 60; ++i)
        fprintf(stderr, "  %#010x %8.2f ms %5.1f%% %10llu packets %6.1f ns/packet\n", v[i].first, v[i].second.first / 1e6,
                100.0 * v[i].second.first / total, (unsigned long long)v[i].second.second,
                v[i].second.second ? (double)v[i].second.first / v[i].second.second : 0.0);
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

static uint64_t HostNowNs();

HaltReason BlackFinCpu::Run() {
    // Runs guest blocks back to back and does the between-block work (interrupts, device polling,
    // timers, events) once per batch of at least batchCycles_ guest cycles instead of after every
    // block: most blocks are 2-6 instructions, and the per-block device work cost more than the guest
    // code. CYCLES and the core timer still update per block. A batch ends early at IDLE, and after any
    // block that touched an MMR or raised an interrupt: device-facing code keeps per-block behavior
    // (some drivers depend on it, e.g. the NFC write-done handshake), compute and DSP loops batch.
    uint64_t batchStart = 0;
    bool idle = false;
    // An event waiting in the CPU lane (a raise that will latch a new interrupt, NAND busy, ...) is delivered
    // on the next poll. Keep that poll one block away, as in per-block mode: R246 relies on that latency
    // (the NAND ISR clears a request struct on the stack of the function that started the request; one
    // batch later the function has returned and the ISR clears its own saved RETI).
    const bool armed = time_.Cycles() >= batchAfterCycles_;
    const uint64_t batch = (armed && !laneUrgent_) ? batchCycles_ : 1;
    for (uint64_t executed = 0; executed < batch && !idle;) {
        if (stopPc_ && cpuState_->pc == stopPc_) break;
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
        if (panicLog_) {  // with OP1_PANIC_LOG: the last 64 blocks before R246 reaches its halt loop
            static u32 ring[64]; static unsigned ri = 0; static bool dumped = false;
            ring[ri++ & 63] = cpuState_->pc;
            if (!dumped && (cpuState_->pc == 0x01919954 || cpuState_->pc == 0x0191995A)) {
                dumped = true;
                fprintf(stderr, "HALT reached; last blocks:");
                for (unsigned k = 0; k < 64; ++k) fprintf(stderr, " %08x", ring[(ri + k) & 63]);
                fprintf(stderr, "\nHALT ivg=%d rets=%08x retx=%08x reti=%08x seqstat=%08x\n", cec_current_ivg(), cpuState_->rets,
                        cpuState_->retx, cpuState_->reti, cpuState_->seqstat);
            }
        }
        // R246's panic(fmt, ...) at 0x01919AA8: log the format and first arguments (OP1_PANIC_LOG=1).
        // Most early-boot panics are suppressed by the firmware itself; a fatal one halts at 0x0191995A.
        if (cpuState_->pc == 0x01919AA8 && panicLog_) {
            auto str = [&](u32 a) { std::string m; for (int k = 0; k < 200; ++k) { char c = (char)emulator.MemoryRead8(a + k); if (!c) break; m += c; } return m; };
            fprintf(stderr, "FIRMWARE PANIC: fmt='%s' r1=0x%08x ('%s') r2=0x%08x r3=0x%08x rets=0x%08x ipend_ivg=%d\n", str(cpuState_->dpregs[0]).c_str(),
                    cpuState_->dpregs[1], str(cpuState_->dpregs[1]).c_str(), cpuState_->dpregs[2], cpuState_->dpregs[3], cpuState_->rets, cec_current_ivg());
        }
        if (breakPc_ && breakPcs_.count(cpuState_->pc)) OnBreak();
        // Hit entry point, invalidating core to reset cached translations
        if (cpuState_->pc == 0xFFA00000) {
            ApplyOverlay();
            core_->invalidate();
            PrefetchPack();
            prefetchAgainNs_ = time_.Nanoseconds() + 10'000'000'000ull;   // and once more when init is done
        }
        if (prefetchAgainNs_ && time_.Nanoseconds() >= prefetchAgainNs_) {
            prefetchAgainNs_ = 0;
            PrefetchPack();
        }
        // Execute one basic block — bcore updates cpuState_->pc internally.
        // Hardware loops, PC advance, and hwloop counters are all handled by bcore.
        core_->set_chain_budget(batch > 1 && executed < batch ? (uint32_t)(batch - executed) : 0);
        // Test mode (deterministic): OP1EMU_INTERP_FIRST=N[,LO-HI] interprets each block start in [LO, HI) the
        // first N times it is reached, then the JIT runs it. Exercises the interpreter reproducibly.
        bool interpreted = false;
        if (interpFirst_) {
            const u32 pc0 = cpuState_->pc;
            if (pc0 >= interpLo_ && pc0 < interpHi_ && interpVisits_[pc0]++ < interpFirst_) {
                const uint32_t steps = Interpreter::Run(cpuState_.get(), bcoreMemory_.get(), cec_is_user_mode(), 256);
                interpSteps_ += steps;
                interpreted = steps != 0;
            }
        }
        // Debug: OP1EMU_INTERP_VERIFY=LO-HI runs the interpreter on a copy before each JIT block starting in
        // [LO, HI) and compares registers and stores when both stop at the same pc.
        CpuState verifyState;
        std::unique_ptr<OverlayMemory> verifyMem;
        uint32_t verifySteps = 0;
        const u32 verifyPc = cpuState_->pc;
        if (verifyHi_ && verifyPc >= verifyLo_ && verifyPc < verifyHi_ && !interpreted) {
            verifyState = *cpuState_;
            verifyMem = std::make_unique<OverlayMemory>(bcoreMemory_.get());
            verifySteps = Interpreter::Run(&verifyState, verifyMem.get(), cec_is_user_mode(), 256);
            core_->set_chain_budget(0);
        }
        const uint64_t lagT0 = (lagLogOn_ || pcProf_) ? HostNowNs() : 0;
        const u32 lagPc = cpuState_->pc;
        const bool ranJit = !interpreted && core_->run(cpuState_->pc);
        if (pcProf_) {   // OP1EMU_PC_PROF: host time and packets per block-run entry pc, printed at exit
            auto& e = pcProfNs_[lagPc];
            e.first += HostNowNs() - lagT0;
            e.second += cpuState_->packets;
        }
        if (lagLogOn_) {
            const uint64_t d = HostNowNs() - lagT0;
            lagRunSumNs_ += d;
            if (d > lagRunMaxNs_) { lagRunMaxNs_ = d; lagRunMaxPc_ = lagPc; }
        }
        if (!interpreted && !ranJit) {
            // Async compile: the block is still compiling. Interpret one instruction meanwhile; what the
            // interpreter leaves to the JIT (CEC/MMU work, exceptions, IDLE) waits for the compile.
            // Straight-line code is interpreted up to the next jump (or loop back), so only real block
            // starts come back to the JIT; stepping alone would hand it every instruction address.
            bool ok = false;
            if (core_->not_ready()) {
                const auto t0 = std::chrono::steady_clock::now();
                const uint32_t steps = Interpreter::Run(cpuState_.get(), bcoreMemory_.get(), cec_is_user_mode(), 256);
                interpSteps_ += steps;
                if (steps) {
                    ok = true;
                    if (stats_) interpNs_ += (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0).count();
                } else {
                    ++interpBails_;
                    ok = core_->run_sync(cpuState_->pc);
                    if (stats_) {
                        const uint64_t w = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0).count();
                        syncWaitNs_ += w;
                        syncWaitMaxNs_ = std::max(syncWaitMaxNs_, w);
                    }
                }
            }
            if (!ok) {
                // Translation/JIT failure: PC is unchanged, so retrying would loop
                // forever. Skip device processing and let the host stop the run.
                LogError("JIT execution failed at PC 0x%08x", cpuState_->pc);
                failed_.store(true, std::memory_order_release);
                return HaltReason::Break;
            }
        }
        if (verifySteps && !verifyMem->touchedMmr() && verifyState.pc == cpuState_->pc && cpuState_->packets == verifySteps) VerifyAgainst(verifyPc, verifyState, verifyMem->writes());
        time_.Retire(cpuState_->packets);
        executed += cpuState_->packets ? cpuState_->packets : 1;
#ifdef ENABLE_PROFILING
        if (profiler_) profiler_->AfterRun(blockPc, cpuState_->packets);
#endif
        cpuState_->did_jump = false; // Clear jump flag set by bcore, since we handle it in the emulator loop
        idle = cec_take_idle();
        // Also stop when a latched interrupt became deliverable (after RTI/STI/IMASK changes): hardware takes
        // it before the next instruction, and VDK's rescheduling (RAISE 14 from an ISR) depends on that.
        {
            const bool mmr = bcoreMemory_->TakeMmrTouched(), pend = cec_pending(), il = cec_deliverable(cpuState_.get()), urg = laneUrgent_;
            if (stats_) {  // batch-end reasons for OP1EMU_STATS
                if (mmr) ++endMmr_; else if (pend) ++endPend_; else if (il) ++endIlat_; else if (urg) ++endUrg_;
            }
            if (mmr || pend || il || urg) break;
        }
    }
    if (stats_) { ++batches_; if (idle) ++endIdle_; if (batch == 1) ++batchOne_; }
    {
        const uint64_t lagT0 = lagLogOn_ ? HostNowNs() : 0;
        AfterBlock(batchStart);
        if (lagLogOn_) lagAfterMaxNs_ = std::max(lagAfterMaxNs_, HostNowNs() - lagT0);
    }
    if (core_->prefetch_active()) PrefetchStep();
    if (idle) IdleUntilInterrupt();
    if (realtime_ && (++paceCalls_ & 255) == 0) Pace();
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

// Debug: OP1EMU_BREAK=PC[,HITS] logs the registers each time a block starts at PC (first HITS times, default 4).
// OP1EMU_BREAK_MEM=ADDR:LEN[,...] (hex) also writes those guest ranges to OP1EMU_BREAK_DIR/hitN_ADDR.bin; an ADDR
// of rN / pN / iN dumps from the address in that data, pointer or index register.
void BlackFinCpu::OnBreak() {
    if (!breakHitsLeft_) return;
    const unsigned hit = breakHit_++;
    --breakHitsLeft_;
    const auto& r = cpuState_->dpregs;
    LogInfo("break %#x hit %u: R0=%08x R1=%08x R2=%08x R3=%08x R4=%08x R5=%08x R6=%08x R7=%08x", cpuState_->pc, hit,
            r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]);
    LogInfo("  P0=%08x P1=%08x P2=%08x P3=%08x P4=%08x P5=%08x SP=%08x FP=%08x RETS=%08x guest_ns=%llu", r[8], r[9], r[10],
            r[11], r[12], r[13], r[14], r[15], cpuState_->rets, (unsigned long long)time_.Nanoseconds());
    const char* mem = getenv("OP1EMU_BREAK_MEM");
    const char* dir = getenv("OP1EMU_BREAK_DIR");
    if (!mem || !dir) return;
    std::string spec(mem);
    size_t pos = 0;
    while (pos < spec.size()) {
        size_t end = spec.find(',', pos);
        if (end == std::string::npos) end = spec.size();
        const std::string item = spec.substr(pos, end - pos);
        pos = end + 1;
        const size_t colon = item.find(':');
        if (colon == std::string::npos) continue;
        const std::string a = item.substr(0, colon);
        const u32 addr = (a[0] == 'r') ? r[std::stoi(a.substr(1)) & 7]
                       : (a[0] == 'p') ? r[8 + (std::stoi(a.substr(1)) & 7)]
                       : (a[0] == 'i') ? cpuState_->iregs[std::stoi(a.substr(1)) & 3]
                       : (u32)strtoul(a.c_str(), nullptr, 16);
        const u32 len = (u32)strtoul(item.c_str() + colon + 1, nullptr, 16);
        std::vector<u8> buf(len);
        emulator.MemoryRead(addr, buf.data(), (int)len);
        char path[512];
        snprintf(path, sizeof path, "%s/hit%u_%s.bin", dir, hit, a.c_str());
        if (FILE* f = fopen(path, "wb")) { fwrite(buf.data(), 1, len, f); fclose(f); }
    }
}

// Dev aid: OP1EMU_OVERLAY=FILE applies "load ADDR PATH" (raw bytes) and "word ADDR VALUE" lines (hex) each time a
// boot stage jumps to 0xFFA00000, i.e. after the LDR loader has written the stage. Lets a synth engine be tried
// without reinstalling the firmware (tools/engines/build.sh writes the file). '#' starts a comment.
void BlackFinCpu::ApplyOverlay() {
    const char* path = getenv("OP1EMU_OVERLAY");
    if (!path) return;
    std::ifstream in(path);
    if (!in) { LogError("overlay: cannot open %s", path); return; }
    std::string kind, file;
    unsigned n = 0;
    for (std::string line; std::getline(in, line);) {
        std::istringstream ls(line);
        u32 addr = 0;
        if (!(ls >> kind) || kind[0] == '#') continue;
        if (kind == "load" && (ls >> std::hex >> addr >> file)) {
            std::ifstream f(file, std::ios::binary);
            std::vector<char> buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
            if (!f && buf.empty()) { LogError("overlay: cannot read %s", file.c_str()); continue; }
            emulator.MemoryWrite(addr, buf.data(), (int)buf.size());
            ++n;
        } else if (kind == "word") {
            u32 value = 0;
            if (ls >> std::hex >> addr >> value) { emulator.MemoryWrite32(addr, value); ++n; }
        }
    }
    LogInfo("overlay: applied %u entries from %s", n, path);
}

void BlackFinCpu::VerifyAgainst(u32 pc, const CpuState& in, const std::map<uint32_t, uint8_t>& writes) {
    const CpuState& j = *cpuState_;
    std::string diff;
    char b[96], name[8];
    auto cmp = [&](const char* n, uint32_t a, uint32_t c) {
        if (a != c) { snprintf(b, sizeof b, " %s interp=%08x jit=%08x", n, a, c); diff += b; }
    };
    for (int i = 0; i < 16; ++i) { snprintf(name, sizeof name, "%s%d", i < 8 ? "R" : "P", i < 8 ? i : i - 8); cmp(name, in.dpregs[i], j.dpregs[i]); }
    for (int i = 0; i < 4; ++i) { cmp("I", in.iregs[i], j.iregs[i]); cmp("M", in.mregs[i], j.mregs[i]); cmp("B", in.bregs[i], j.bregs[i]); cmp("L", in.lregs[i], j.lregs[i]); }
    for (int i = 0; i < 2; ++i) { cmp("AX", in.ax[i], j.ax[i]); cmp("AW", in.aw[i], j.aw[i]); cmp("LC", in.lc[i], j.lc[i]); cmp("LT", in.lt[i], j.lt[i]); cmp("LB", in.lb[i], j.lb[i]); }
    cmp("ASTAT", astat_compose(&in), astat_compose(&j));
    cmp("RETS", in.rets, j.rets);
    for (const auto& [a, v] : writes) {
        const uint8_t real = emulator.MemoryRead8(a);
        if (real != v) { snprintf(b, sizeof b, " [%08x] interp=%02x jit=%02x", a, v, real); diff += b; }
    }
    if (diff.empty()) { ++verifyOk_; return; }
    ++verifyBad_;
    static std::set<u32> seen;
    if (seen.insert(pc).second) LogInfo("VERIFY mismatch block %#x (%u insns):%s", pc, j.packets, diff.c_str());
}

// Background compile: queue every cached block whose code is in memory as a low-priority load, so screens
// and engines run compiled code the first time (pack loads otherwise happen on first use, thousands at once
// when a new screen comes up). OP1EMU_PREFETCH=0 turns it off.
void BlackFinCpu::PrefetchPack() {
    if (const char* e = getenv("OP1EMU_PREFETCH"); e && e[0] == '0') return;
    core_->prefetch_pack();
    prefetchQueued_ = 0;
    prefetchScanNs_ = 0;
}

// One slice of the scan per Run(): 512 index entries take ~2 ms at most, so the scan never stalls the guest.
void BlackFinCpu::PrefetchStep() {
    const auto t0 = std::chrono::steady_clock::now();
    prefetchQueued_ += core_->prefetch_step(512);
    prefetchScanNs_ += (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0).count();
    if (!core_->prefetch_active() && prefetchQueued_)
        LogInfo("JIT: preloading %zu cached blocks in the background (scan %.1f ms, in slices)", prefetchQueued_,
                prefetchScanNs_ / 1e6);
}

void BlackFinCpu::UpdateAsyncCompile() {
    // Direct memory for the interpreter: SDRAM, L1 data A/B (+cache), L1 code, scratchpad.
    {
        std::vector<Interpreter::Range> ranges;
        for (u32 a : {0x00000000u, 0xFF800000u, 0xFF804000u, 0xFF900000u, 0xFF904000u, 0xFFA00000u, 0xFFA08000u,
                      0xFFA10000u, 0xFFB00000u}) {
            u8* host = nullptr; u32 base = 0, size = 0;
            if (emulator.PlainRange(a, &host, &base, &size)) ranges.push_back({base, size, host});
        }
        Interpreter::SetDirectRanges(ranges.data(), (int)ranges.size());
    }
    bool on = Interpreter::Available() && (realtime_ || !time_.Deterministic());
    if (const char* e = getenv("OP1EMU_ASYNC_JIT")) on = Interpreter::Available() && e[0] == '1';
    if (core_) core_->set_async_compile(on);
    static bool logged = false;
    if (!logged) { logged = true; LogInfo("JIT: %s compile", on ? "background (interpreting meanwhile)" : "synchronous"); }
}

static uint64_t HostNowNs() {
    return (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Keeps guest time (instruction-driven) within ~1-2 ms of host time: sleeps when ahead. When behind it
// runs flat out, and a backlog over 50 ms is dropped (re-anchored) rather than caught up.
void BlackFinCpu::Pace() {
    const uint64_t host = HostNowNs();
    if (!paceStartNs_) {
        paceStartNs_ = host - time_.Nanoseconds();
        statsNextNs_ = host + 5'000'000'000ULL;
        statsLastHostNs_ = host;
        statsLastGuestNs_ = time_.Nanoseconds();
    }
    const uint64_t guest = time_.Nanoseconds();
    const uint64_t hostElapsed = host - paceStartNs_;
    // OP1EMU_LEAD_MS (default 70): let the guest run up to this far ahead of the host clock when it has
    // headroom, so a burst of guest work slower than real time (engine switches, a first visit to a screen)
    // eats the lead instead of falling behind. The audio it renders early queues in the host ring, so the
    // GUI's audio prefill is smaller by the same amount (see gui/audio_output_miniaudio.cpp).
    static const uint64_t leadNs = (uint64_t)(getenv("OP1EMU_LEAD_MS") ? strtoul(getenv("OP1EMU_LEAD_MS"), nullptr, 10) : 70) * 1'000'000ull;
    if (guest > hostElapsed + leadNs + 2'000'000) {
        const uint64_t ahead = guest - hostElapsed - leadNs - 1'000'000;
        std::this_thread::sleep_for(std::chrono::nanoseconds(ahead));
        statsSleptNs_ += ahead;
    } else if (hostElapsed > guest) {
        statsMaxLagNs_ = std::max(statsMaxLagNs_, hostElapsed - guest);
        // OP1EMU_LAG_LOG=MS: log each lag over MS ms with the host and guest time since the previous Pace().
        static const long lagLog = getenv("OP1EMU_LAG_LOG") ? strtol(getenv("OP1EMU_LAG_LOG"), nullptr, 10) : -1;
        if (lagLog >= 0 && hostElapsed - guest > (uint64_t)lagLog * 1'000'000 && host - lagPrevHostNs_ > 1'000'000)
            LogInfo("lag %.1f ms: since last pace host %.2f ms, guest %.2f ms, pc %#x ivg %d, mmr ends %llu; block runs %.2f ms "
                    "(slowest %.2f ms at %#x), slowest AfterBlock %.2f ms, interpreted %llu in %.2f ms",
                    (hostElapsed - guest) / 1e6, (host - lagPrevHostNs_) / 1e6, (guest - lagPrevGuestNs_) / 1e6, cpuState_->pc,
                    cec_current_ivg(), (unsigned long long)(endMmr_ - lagPrevMmr_), lagRunSumNs_ / 1e6, lagRunMaxNs_ / 1e6,
                    lagRunMaxPc_, lagAfterMaxNs_ / 1e6, (unsigned long long)(interpSteps_ - lagPrevInterpSteps_),
                    (interpNs_ - lagPrevInterpNs_) / 1e6);
        // Too far behind (boot, a host stall): drop the backlog instead of running fast to catch up.
        if (hostElapsed - guest > 50'000'000) paceStartNs_ = host - guest;
    }
    lagPrevHostNs_ = HostNowNs(); lagPrevGuestNs_ = guest; lagPrevBatches_ = batches_; lagPrevMmr_ = endMmr_;
    lagPrevInterpNs_ = interpNs_; lagPrevInterpSteps_ = interpSteps_;
    lagLogOn_ = getenv("OP1EMU_LAG_LOG") != nullptr; lagRunMaxNs_ = lagAfterMaxNs_ = lagRunSumNs_ = 0;
    if (stats_ && host >= statsNextNs_) {
        const double hostS = (host - statsLastHostNs_) / 1e9, guestS = (guest - statsLastGuestNs_) / 1e9;
        LogInfo("realtime: %.2fx real time over %.1f s, slept %.0f%%, max lag %.1f ms",
                guestS / hostS, hostS, 100.0 * statsSleptNs_ / 1e9 / hostS, statsMaxLagNs_ / 1e6);
        const uint64_t tns = core_->translate_ns(), tn = core_->translate_count();
        LogInfo("translate: %llu blocks, %.1f ms (byte-index/pack loads included)", (unsigned long long)(tn - statsTranslateN_),
                (tns - statsTranslateNs_) / 1e6);
        statsTranslateNs_ = tns; statsTranslateN_ = tn;
        if (interpSteps_ != statsInterpSteps_ || interpBails_ != statsInterpBails_)
            LogInfo("interpreted %llu instructions in %.1f ms while compiling; %llu left to the JIT, waited %.1f ms (max %.1f)",
                    (unsigned long long)(interpSteps_ - statsInterpSteps_), interpNs_ / 1e6,
                    (unsigned long long)(interpBails_ - statsInterpBails_), syncWaitNs_ / 1e6, syncWaitMaxNs_ / 1e6);
        interpNs_ = syncWaitNs_ = syncWaitMaxNs_ = 0; lagPrevInterpNs_ = 0;
        statsInterpSteps_ = interpSteps_; statsInterpBails_ = interpBails_;
        if (verifyHi_) LogInfo("verify: %llu blocks equal, %llu mismatches", (unsigned long long)verifyOk_, (unsigned long long)verifyBad_);
        LogInfo("now: pc %#x ivg %d ipend %#x ilat %#x imask %#x", cpuState_->pc, cec_current_ivg(), cec_ipend(), cec_ilat(),
                emulator.MemoryRead32(0xFFE02104));
        LogInfo("batches %llu: size1 %llu | ends: mmr %llu pend %llu deliverable %llu urgent %llu idle %llu", (unsigned long long)batches_,
                (unsigned long long)batchOne_, (unsigned long long)endMmr_, (unsigned long long)endPend_, (unsigned long long)endIlat_,
                (unsigned long long)endUrg_, (unsigned long long)endIdle_);
        batches_ = batchOne_ = endMmr_ = endPend_ = endIlat_ = endUrg_ = endIdle_ = 0;
        statsNextNs_ = host + 5'000'000'000ULL;
        statsLastHostNs_ = host; statsLastGuestNs_ = guest; statsSleptNs_ = 0; statsMaxLagNs_ = 0;
    }
}

// Everything that happens between two guest blocks: interrupts, device polling, timers, events.
void BlackFinCpu::AfterBlock(uint64_t cyclesElapsed) {
    cec_check_pending(cpuState_.get());
    {
        const uint32_t ipend = cec_ipend();
        if (lastIpend_ & ~ipend) sic->ForwardInterrupts();
        lastIpend_ = ipend;
    }

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
        laneUrgent_ = true;
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
        // Urgent only for an interrupt that is neither latched nor being serviced: a level source re-raised
        // while its own handler runs just re-latches for after the RTI.
        if (!((cec_ilat() | cec_ipend()) & bit)) laneUrgent_ = true;
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
    if (!cpuLane_.empty()) {
        ProcessLane(cpuLane_);
        // Still urgent if something left in the lane will change state when delivered.
        laneUrgent_ = false;
        const uint32_t busy = cec_ilat() | cec_ipend();
        for (const auto& e : cpuLane_)
            if (e.raiseIvg < 0 || !(busy & (1u << (e.raiseIvg & 31)))) { laneUrgent_ = true; break; }
    }
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
        // A SIC level (IVG 7..15) queued before its handler started is not latched again once the handler runs
        // (see SetInterruptForwardCallback); AfterBlock re-forwards it after the RTI if it is still asserted.
        if (e.raiseIvg >= 0) {
            if (!(e.raiseIvg >= 7 && (cec_ipend() & (1u << (e.raiseIvg & 31))))) cec_raise(cpuState_.get(), e.raiseIvg);
        } else e.fn();
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