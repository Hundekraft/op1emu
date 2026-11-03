#include <string>
#include <cstdlib>
#include <cstdio>
#include "bcore_memory.h"
#include "utils/log.h"
#ifdef ENABLE_PROFILING
#include "profiling/memory_census.h"
#include "profiling/profiler.h"

void EmulatorMemory::Access(uint32_t addr, int widthIndex, bool write) const {
    if (census_) census_->Count(addr, widthIndex, write);
    if (profiler_ && addr >= 0xFFC00000) profiler_->OnMmio(addr, write);
}
#define PROFILE_ACCESS(addr, width, write) Access(addr, width, write)
#else
#define PROFILE_ACCESS(addr, width, write) ((void)0)
#endif

EmulatorMemory::EmulatorMemory(Emulator& emulator)
    : emulator_(emulator) {
    // OP1EMU_STRICT_MEMORY=1: every guest load and store is checked against the DCPLBs, the MMR privilege rules
    // and the memory map first (check_data_access); a run stops at an access to unmapped memory.
    const char* strict = getenv("OP1EMU_STRICT_MEMORY");
    strictData_ = strict && std::string(strict) == "1";
}

// Generated code calls here only for what is not in DirectPages(): the MMRs,
// mostly. After the access, so that the device it reached can say from its
// state then whether the host has work after the block (see
// Emulator::HostWorkAfter, ReadAttended and WriteAttended). The CPU thread is the only one running guest
// code, but other threads set the budget to 0 too.
void EmulatorMemory::Attend(bool hostWork) const {
    if (!chainBudget_ || !hostWork) return;
#ifdef ENABLE_PROFILING
    if (__atomic_load_n(chainBudget_, __ATOMIC_RELAXED)) ++chainStops_;
#endif
    __atomic_store_n(chainBudget_, 0, __ATOMIC_RELAXED);
}

uint32_t EmulatorMemory::base() const {
    return 0;
}

uint32_t EmulatorMemory::size() const {
    return 0xFFFFFFFF;
}

uintptr_t EmulatorMemory::fast_base() const {
    return 0;
}

uint8_t EmulatorMemory::read8(uint32_t addr) const {
    PROFILE_ACCESS(addr, 0, false);
    bool hostWork, stable;
    const uint8_t value = emulator_.ReadAttended<uint8_t>(addr, hostWork, stable);
    Attend(hostWork);
    if (!stable && unstableReads_) ++*unstableReads_;
    return value;
}

uint16_t EmulatorMemory::read16(uint32_t addr) const {
    PROFILE_ACCESS(addr, 1, false);
    bool hostWork, stable;
    const uint16_t value = emulator_.ReadAttended<uint16_t>(addr, hostWork, stable);
    Attend(hostWork);
    if (!stable && unstableReads_) ++*unstableReads_;
    return value;
}

uint32_t EmulatorMemory::read32(uint32_t addr) const {
    PROFILE_ACCESS(addr, 2, false);
    bool hostWork, stable;
    const uint32_t value = emulator_.ReadAttended<uint32_t>(addr, hostWork, stable);
    Attend(hostWork);
    if (!stable && unstableReads_) ++*unstableReads_;
    return value;
}

void EmulatorMemory::write8(uint32_t addr, uint8_t val) {
    PROFILE_ACCESS(addr, 0, true);
    bool hostWork;
    emulator_.WriteAttended<uint8_t>(addr, val, hostWork);
    if ((addr & 0xFFFFF000u) == 0xFFE00000u) cplbDirty_ = true;
    Attend(hostWork);
}

void EmulatorMemory::write16(uint32_t addr, uint16_t val) {
    PROFILE_ACCESS(addr, 1, true);
    bool hostWork;
    emulator_.WriteAttended<uint16_t>(addr, val, hostWork);
    if ((addr & 0xFFFFF000u) == 0xFFE00000u) cplbDirty_ = true;
    Attend(hostWork);
}

void EmulatorMemory::write32(uint32_t addr, uint32_t val) {
    PROFILE_ACCESS(addr, 2, true);
    bool hostWork;
    emulator_.WriteAttended<uint32_t>(addr, val, hostWork);
    if ((addr & 0xFFFFF000u) == 0xFFE00000u) cplbDirty_ = true;
    Attend(hostWork);
}

static_assert(PAGE_BITS == bcore::Memory::kPageBits, "bcore's page table has op1emu's page size");
static_assert(NUM_PAGE_TABLE_ENTRIES == bcore::Memory::kPageTableEntries, "one entry per guest page");

uint8_t* const* EmulatorMemory::page_table() const {
#ifdef ENABLE_PROFILING
    // The census counts in read*/write*. Switching it on drops every block
    // compiled before (BlackFinCpu::EnableMemoryCensus()), so no block that
    // bypasses it runs while it counts.
    if (census_) return nullptr;
#endif
    return emulator_.DirectPages();
}

const uint8_t* EmulatorMemory::raw() const {
    return nullptr;
}

void EmulatorMemory::data_alignment_fault(uint32_t pc, uint32_t addr, uint32_t status) {
    emulator_.MemoryWrite32(0xFFE00008, status);              // DCPLB_FAULT_STATUS
    emulator_.MemoryWrite32(0xFFE0000C, addr);                // DCPLB_FAULT_ADDR
    emulator_.MemoryWrite32(0xFFE01008, status & (1u << 17)); // ICPLB_FAULT_STATUS: supervisor bit only
    emulator_.MemoryWrite32(0xFFE0100C, pc);                  // ICPLB_FAULT_ADDR: the faulting instruction
    LogWarn("Data alignment fault: PC=%08x ADDR=%08x STATUS=%08x", pc, addr, status);
}

void EmulatorMemory::refreshDataProtection() {
    if (!cplbDirty_) return;
    dmemControl_ = emulator_.MemoryRead32(0xFFE00004);
    for (unsigned i = 0; i < 16; ++i) {
        cplbAddress_[i] = emulator_.MemoryRead32(0xFFE00100 + 4 * i);
        cplbData_[i] = emulator_.MemoryRead32(0xFFE00200 + 4 * i);
    }
    cplbDirty_ = false;
}

// Strict mode: the DCPLB, MMR privilege and memory-map check before an access (bcore's cec_check_data). Returns the
// exception cause (0x23 protection, 0x26 miss, 0x27 multiple hits), 0, or UINT32_MAX to stop the run.
uint32_t EmulatorMemory::check_data_access(uint32_t pc, uint32_t addr, uint32_t width, uint32_t flags, bool record) {
    refreshDataProtection();
    const bool supervisor = flags & (1u << 17), write = flags & (1u << 16);
    uint32_t cause = 0, matches = 0;
    // Alignment is the JIT's own check after this one: an odd pointer is not an unmapped access.
    if (addr & (width - 1)) return 0;
    if (addr >= 0xFFC00000u) {
        if (!supervisor || (flags & (1u << 18))) cause = 0x23;
    } else if (dmemControl_ & 2u) {
        static const uint32_t pages[] = {1024, 4096, 1u << 20, 4u << 20};
        bool denied = false;
        for (unsigned i = 0; i < 16; ++i) {
            const uint32_t data = cplbData_[i];
            if (!(data & 1)) continue;
            const uint32_t base = cplbAddress_[i];
            if (uint64_t(addr) < base || uint64_t(addr) >= uint64_t(base) + pages[(data >> 16) & 3]) continue;
            matches |= 1u << i;
            if (write) {
                denied |= !(data & (supervisor ? 16u : 8u));
                denied |= (data & ((1u << 14) | (1u << 12) | (1u << 7))) == (1u << 12);
            } else denied |= !supervisor && !(data & 4u);
        }
        if (matches & (matches - 1)) cause = 0x27;
        else if (denied) cause = 0x23;
        else if (!matches && (addr & 0xFF000000u) != 0xFF000000u) cause = 0x26;
    }
    if (cause) {
        if (!record) return cause;
        const uint32_t status = flags | matches | (cause == 0x26 ? 1u << 19 : 0);
        emulator_.MemoryWrite32(0xFFE00008, status);
        emulator_.MemoryWrite32(0xFFE0000C, addr);
        emulator_.MemoryWrite32(0xFFE01008, flags & (1u << 17));
        emulator_.MemoryWrite32(0xFFE0100C, pc);
        fprintf(stderr, "Data protection fault: PC=%08x ADDR=%08x STATUS=%08x CAUSE=%02x\n", pc, addr, status, cause);
        return cause;
    }
    // A missing peripheral model is no proof of an invalid hardware address: logged once, not a fault.
    if (addr >= 0xFFC00000u) {
        if (record && !emulator_.IsMemoryValid(addr) && unknownMmrs_.insert(addr).second)
            fprintf(stderr, "Unmodeled MMR (diagnostic): PC=%08x ADDR=%08x WIDTH=%u\n", pc, addr, width);
        return 0;
    }
    if (uint64_t(addr) + width > (1ull << 32) || !emulator_.IsMemoryValid(addr) || !emulator_.IsMemoryValid(addr + width - 1)) {
        if (record) fprintf(stderr, "Unmapped data access (diagnostic stop): PC=%08x ADDR=%08x WIDTH=%u STATUS=%08x\n",
                            pc, addr, width, flags);
        return UINT32_MAX;
    }
    return 0;
}
