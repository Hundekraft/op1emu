#include "bcore_memory.h"
#include "mmr.h"
#include "cpu_state.h"
#include <cstdio>
#include <cstdlib>
#ifdef ENABLE_PROFILING
#include "profiling/profiler.h"

void EmulatorMemory::Mmio(uint32_t addr, bool write) const {
    if (profiler_ && addr >= 0xFFC00000) profiler_->OnMmio(addr, write);
}
#define PROFILE_MMIO(addr, write) Mmio(addr, write)
#else
#define PROFILE_MMIO(addr, write) ((void)0)
#endif

EmulatorMemory::EmulatorMemory(Emulator& emulator)
    : emulator_(emulator) {
    const char* strict = getenv("OP1EMU_STRICT_MEMORY");
    strictData_ = strict && std::string(strict) == "1";
    // SDRAM at guest 0 is plain memory: let the JIT access it directly, unless a read/write hook sits in it.
    u8* host = nullptr;
    u32 size = 0;
    if (emulator_.DirectRange(0, &host, &size)) {
        auto hookBelow = [size](const auto& hooks) { return !hooks.empty() && hooks.begin()->first < size; };
        if (!hookBelow(emulator_.ReadHooks()) && !hookBelow(emulator_.WriteHooks())) {
            rawBase_ = reinterpret_cast<uintptr_t>(host);
            rawLimit_ = size;
        }
    }
    // L1 data A/B (SRAM and cache-as-SRAM halves) and the scratchpad. Not L1 code: writes there must
    // keep going through the emulator.
    for (u32 a : {0xFF800000u, 0xFF804000u, 0xFF900000u, 0xFF904000u, 0xFFB00000u}) {
        u8* h = nullptr; u32 base = 0, sz = 0;
        if (nRanges_ < 8 && emulator_.PlainRange(a, &h, &base, &sz) && base == a)
            ranges_[nRanges_++] = FastRange{base, sz, reinterpret_cast<uintptr_t>(h)};
    }
    if (getenv("OP1EMU_NO_L1_INLINE")) nRanges_ = 0;
}

int EmulatorMemory::fast_ranges(FastRange* out, int max) const {
    int n = 0;
    for (; n < nRanges_ && n < max; ++n) out[n] = ranges_[n];
    return n;
}

uint32_t EmulatorMemory::base() const {
    return 0;
}

uint32_t EmulatorMemory::size() const {
    return 0xFFFFFFFF;
}

uintptr_t EmulatorMemory::fast_base() const {
    return rawBase_;
}

uint8_t EmulatorMemory::read8(uint32_t addr) const {
    PROFILE_MMIO(addr, false);
    if (addr >= 0xFFC00000u) mmrTouched_ = true;
    return emulator_.MemoryRead8(addr);
}

uint16_t EmulatorMemory::read16(uint32_t addr) const {
    PROFILE_MMIO(addr, false);
    if (addr >= 0xFFC00000u) mmrTouched_ = true;
    return emulator_.MemoryRead16(addr);
}

uint32_t EmulatorMemory::read32(uint32_t addr) const {
    PROFILE_MMIO(addr, false);
    if (addr >= 0xFFC00000u) mmrTouched_ = true;
    return emulator_.MemoryRead32(addr);
}

void EmulatorMemory::write8(uint32_t addr, uint8_t val) {
    if ((addr & 0xFFFFF000u) == 0xFFE00000u) cplbDirty_ = true;
    PROFILE_MMIO(addr, true);
    if (addr >= 0xFFC00000u) mmrTouched_ = true;
    emulator_.MemoryWrite8(addr, val);
}

void EmulatorMemory::write16(uint32_t addr, uint16_t val) {
    if ((addr & 0xFFFFF000u) == 0xFFE00000u) cplbDirty_ = true;
    PROFILE_MMIO(addr, true);
    if (addr >= 0xFFC00000u) mmrTouched_ = true;
    emulator_.MemoryWrite16(addr, val);
}

void EmulatorMemory::write32(uint32_t addr, uint32_t val) {
    if ((addr & 0xFFFFF000u) == 0xFFE00000u) cplbDirty_ = true;
    PROFILE_MMIO(addr, true);
    if (addr >= 0xFFC00000u) mmrTouched_ = true;
    emulator_.MemoryWrite32(addr, val);
}

const uint8_t* EmulatorMemory::raw() const {
    return nullptr;
}

void EmulatorMemory::data_alignment_fault(uint32_t pc, uint32_t addr, uint32_t status) {
    emulator_.MemoryWrite32(0xFFE00008, status);
    emulator_.MemoryWrite32(0xFFE0000C, addr);
    emulator_.MemoryWrite32(0xFFE01008, status & (1u << 17));
    emulator_.MemoryWrite32(0xFFE0100C, pc);
    mmrTouched_ = true;
    fprintf(stderr, "Data alignment fault: PC=%08x ADDR=%08x STATUS=%08x\n", pc, addr, status);
}

void EmulatorMemory::refreshDataProtection() {
    if (!cplbDirty_) return;
    dmemControl_ = emulator_.MemoryRead32(0xFFE00004);
    for (unsigned i=0;i<16;++i) {
        cplbAddress_[i]=emulator_.MemoryRead32(0xFFE00100+4*i);
        cplbData_[i]=emulator_.MemoryRead32(0xFFE00200+4*i);
    }
    cplbDirty_=false;
}

uint32_t EmulatorMemory::check_data_access(uint32_t pc, uint32_t addr, uint32_t width, uint32_t flags, bool record) {
    refreshDataProtection();
    const bool supervisor = flags & (1u << 17), write = flags & (1u << 16);
    uint32_t cause = 0, matches = 0;
    // Alignment is handled by the JIT guard below; don't misclassify an odd
    // pointer as an unmapped bus access first.
    if (addr & (width - 1)) return 0;
    if (addr >= 0xFFC00000u) {
        if (!supervisor || (flags & (1u << 18))) cause = 0x23;
    } else if (dmemControl_ & 2u) {
        static const uint32_t pages[] = {1024,4096,1u<<20,4u<<20};
        bool denied = false;
        for (unsigned i=0;i<16;++i) {
            const uint32_t data=cplbData_[i];
            if (!(data&1)) continue;
            const uint32_t base=cplbAddress_[i];
            if (uint64_t(addr)<base || uint64_t(addr)>=uint64_t(base)+pages[(data>>16)&3]) continue;
            matches |= 1u<<i;
            if (write) {
                denied |= !(data & (supervisor ? 16u : 8u));
                denied |= (data & ((1u<<14)|(1u<<12)|(1u<<7))) == (1u<<12);
            } else denied |= !supervisor && !(data&4u);
        }
        if (matches & (matches-1)) cause=0x27;
        else if (denied) cause=0x23;
        else if (!matches && (addr & 0xFF000000u)!=0xFF000000u) cause=0x26;
    }
    if (cause) {
        if (!record) return cause;
        const uint32_t status=flags|matches|(cause==0x26 ? 1u<<19 : 0);
        emulator_.MemoryWrite32(0xFFE00008,status);emulator_.MemoryWrite32(0xFFE0000C,addr);
        emulator_.MemoryWrite32(0xFFE01008,flags&(1u<<17));emulator_.MemoryWrite32(0xFFE0100C,pc);
        mmrTouched_=true;
        fprintf(stderr,"Data protection fault: PC=%08x ADDR=%08x STATUS=%08x CAUSE=%02x\n",pc,addr,status,cause);
        return cause;
    }
    // Missing peripheral models are not proof of an invalid hardware address.
    // Keep these visible separately; don't invent a bus fault for e.g. boot ROM PLL reads.
    if (addr >= 0xFFC00000u) {
        if (record && !emulator_.IsMemoryValid(addr) && unknownMmrs_.insert(addr).second)
            fprintf(stderr,"Unmodeled MMR (diagnostic): PC=%08x ADDR=%08x WIDTH=%u\n",pc,addr,width);
        return 0;
    }
    if (uint64_t(addr)+width > (1ull<<32) || !emulator_.IsMemoryValid(addr) ||
        !emulator_.IsMemoryValid(addr+width-1)) {
        if (record) fprintf(stderr,"Unmapped data access (diagnostic stop): PC=%08x ADDR=%08x WIDTH=%u STATUS=%08x\n",pc,addr,width,flags);
        return UINT32_MAX;
    }
    return 0;
}
