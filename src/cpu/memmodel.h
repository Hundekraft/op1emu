#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

// OP1EMU_MEMMODEL=FILE: an estimate of what L1 cache misses and SDRAM cost on the real BF524, for the code the
// interpreter runs (use with OP1EMU_INTERP_FIRST=N,LO-HI so the code of interest is interpreted). The emulator
// itself still counts one cycle per instruction; the model only adds up the stalls it would see and writes
// them per block start pc at exit. Four variants run side by side: warm (caches kept) or cold (flushed at the
// OP1EMU_MEMMODEL_FLUSH pcs, e.g. the start of each audio block), each with and without allocating a data
// cache line on a write miss (not documented for Blackfin in what we have).
//
// Geometry (ADI Blackfin memory training slides, ADST-170 sec. 6): data cache 16 KB = 4 sub-banks (address
// bits 13:12) x 2 ways x 64 lines (bits 10:5) x 32 bytes, LRU; instruction cache 16 KB = 4 sub-banks x 4 ways x
// 32 lines x 32 bytes, LRU; line fills are 4 x 64-bit, critical word first, the core continues when its word
// arrives; dirty victims go out through a 4-deep victim buffer without stalling the core.
// R246 setup (docs/firmware/clocks-memory.md): SDRAM below 0x02000000 cacheable write-back, 0x02000000 and up
// write-through; SDGCTL CL 2, tRCD 2, tRP 2, tWR 2; SDBCTL 10-bit columns on a 16-bit bus (2 KB rows),
// 64 MB; SCLK = CCLK / 5. Internal bank = address bits 25:24, row 23:11, column 10:1 (ADST-170 sec. 12 slide 25,
// 16-bit SDRAM, 64 MB, 10-bit columns); the controller keeps up to four pages open (one per internal bank).
// Not modelled: refresh, R246's own code (JIT-run, so not traced: the cold variant bounds it), DMA traffic.
class MemModel {
public:
    explicit MemModel(const std::string& path);
    ~MemModel();
    void BeginBlock(uint32_t pc, uint64_t guestCycles);
    void Access(uint32_t addr, uint32_t len, int kind);   // 0 data read, 1 data write, 2 instruction fetch
    static void Trace(void* self, uint32_t addr, uint32_t len, int kind) {
        static_cast<MemModel*>(self)->Access(addr, len, kind);
    }

private:
    struct Cache {
        int sets, ways;
        std::vector<uint32_t> tag;     // line address + 1 (0 = invalid)
        std::vector<uint8_t> dirty;
        std::vector<uint64_t> used;    // LRU stamp
        uint64_t stamp = 0;
        int Set(uint32_t line) const;
        void Init(int s, int w);
        void Flush();
    };
    struct Counters { uint64_t insns = 0, imiss = 0, rmiss = 0, wmiss = 0, victims = 0, stall = 0; };
    struct Variant {
        bool cold, walloc;
        Cache d, i;
        uint64_t stall = 0, busFree = 0;   // CCLK
        uint32_t openRow[4] = {~0u, ~0u, ~0u, ~0u};
        std::unordered_map<uint32_t, Counters> perPc;
        Counters* cur = nullptr;           // perPc[current block pc] (references survive rehashing)
    };
    enum Region { kFree, kCachedWB, kCachedWT };
    Region Classify(uint32_t addr) const;
    uint64_t Now(const Variant& v) const { return blockCycles_ + index_ + v.stall; }
    uint64_t Bus(Variant& v, uint32_t addr, uint64_t start, unsigned beats, bool write);
    void Fill(Variant& v, Cache& c, uint32_t addr, bool dirty, Counters& n);

    std::string path_;
    std::vector<Variant> variants_;
    std::vector<std::pair<uint32_t, uint32_t>> l1_;   // what-if ranges counted as L1 (no cost)
    struct Remap { uint32_t lo, hi, to; };
    std::vector<Remap> remap_;                          // what-if: [lo, hi) placed at `to` instead
    std::vector<uint32_t> flushPcs_;
    uint32_t blockPc_ = 0;
    uint64_t blockCycles_ = 0, index_ = 0;
};
