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
    PROFILE_MMIO(addr, true);
    if (addr >= 0xFFC00000u) mmrTouched_ = true;
    emulator_.MemoryWrite8(addr, val);
}

void EmulatorMemory::write16(uint32_t addr, uint16_t val) {
    PROFILE_MMIO(addr, true);
    if (addr >= 0xFFC00000u) mmrTouched_ = true;
    emulator_.MemoryWrite16(addr, val);
}

void EmulatorMemory::write32(uint32_t addr, uint32_t val) {
    PROFILE_MMIO(addr, true);
    if (addr >= 0xFFC00000u) mmrTouched_ = true;
    emulator_.MemoryWrite32(addr, val);
}

const uint8_t* EmulatorMemory::raw() const {
    return nullptr;
}
