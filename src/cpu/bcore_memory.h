#pragma once

// bcore headers (from ext/bcore/include/)
#include "mem.h"

// op1emu headers
#include "emu.h"

class Profiler;

class EmulatorMemory : public Memory {
public:
    explicit EmulatorMemory(Emulator& emulator);

    uint32_t base() const override;
    uint32_t size() const override;
    uintptr_t fast_base() const override;

    uint8_t  read8(uint32_t addr) const override;
    uint16_t read16(uint32_t addr) const override;
    uint32_t read32(uint32_t addr) const override;

    void write8(uint32_t addr, uint8_t val) override;
    void write16(uint32_t addr, uint16_t val) override;
    void write32(uint32_t addr, uint32_t val) override;

    const uint8_t* raw() const override;

    // Guest addresses below rawmem_limit() are plain SDRAM at fast_base() + addr; bcore inlines those
    // loads/stores and calls read*/write* only for the rest (MMRs, L1, boot ROM). 0 = always call.
    uint32_t rawmem_limit() const override { return rawLimit_; }
    // True once after the guest accessed an MMR (>= 0xFFC00000) since the last call.
    bool TakeMmrTouched() { const bool t = mmrTouched_; mmrTouched_ = false; return t; }
    bool allow_chain() const override { return !mmrTouched_; }

#ifdef ENABLE_PROFILING
    // Counts guest accesses to the MMR space (0xFFC00000 and up). Not owned.
    void SetProfiler(Profiler* profiler) { profiler_ = profiler; }
#endif

private:
    Emulator& emulator_;
    mutable bool mmrTouched_ = false;

    uintptr_t rawBase_ = 0;
    uint32_t rawLimit_ = 0;
#ifdef ENABLE_PROFILING
    Profiler* profiler_ = nullptr;
    void Mmio(uint32_t addr, bool write) const;
#endif
};
