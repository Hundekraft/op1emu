#pragma once
// Aggregated execution trace for firmware mapping (OP1EMU_TRACE_DIR=dir). Off unless the variable is set; then
// the CPU loop, the MMR path and interrupt dispatch report here, and each phase is written as TSV files to
// dir/NN_<phase>/ (see WritePhase). Everything is aggregated into unique sets with counts, never a raw stream,
// except the first OP1EMU_TRACE_ORDERED (default 200000) MMR writes of the whole run, kept in order, of which
// each (address, instruction) contributes at most OP1EMU_TRACE_PER_SITE (default 64), so data streams (NAND,
// SPI, TWI data registers) do not crowd out the init writes.
//
// Phases: the run starts in phase "boot-rom"; the first two times the CPU enters 0xFFA00000 (te-boot stage 0
// INIT, then OP1_vdk entry) a new phase "ffa00000-N" starts, because code at the same addresses changes there. The headless
// command "trace NAME" ends the current phase and starts NAME (RequestPhase, any thread).
//
// Block ranges are exact: [start, end of the last packet that ran). The CPU loop disables bcore's self-loop
// chaining while tracing so one run() is one pass through one block (the packet count is then exact).
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <atomic>
#include <functional>

class Tracer {
public:
    // Disassembler: pc -> (text, next pc). Must read the code as it is now.
    using Disasm = std::function<std::pair<std::string, uint32_t>(uint32_t)>;

    static Tracer* Create(Disasm disasm);   // nullptr when OP1EMU_TRACE_DIR is unset
    explicit Tracer(const std::string& dir, Disasm disasm);

    // CPU thread.
    void BeginBlock(uint32_t pc, uint64_t cycles) { curStart_ = pc; curCycles_ = cycles; }
    // After core->run: packets executed, pc the block left to (before any interrupt dispatch).
    void EndBlock(uint32_t packets, uint32_t nextPc);
    // After interrupt dispatch (and after each block): new IPEND bits = an event taken at pc.
    void CheckIpend(uint32_t ipend, uint32_t pc);
    // MMR access (>= 0xFFC00000) from guest code; packet = CpuState::packets at the access (1-based).
    void OnMmr(uint32_t addr, bool write, unsigned width, uint32_t value, uint32_t packet);
    // Called at the top of the CPU loop: applies a pending RequestPhase and the 0xFFA00000 split.
    void Poll(uint32_t pc);
    void Finish();   // write the current phase (at exit)

    // Any thread.
    void RequestPhase(const std::string& name);

private:
    struct BlockKey { uint32_t start, packets; };
    struct BlockStat { uint64_t count = 0; uint32_t end = 0, lastPc = 0; uint8_t kind = 0; };  // kind: see Classify
    struct MmrStat {
        uint64_t count = 0;
        std::vector<uint32_t> values;   // distinct, first 64
        bool valuesOverflow = false;
        std::vector<uint32_t> sites;    // insn pcs, distinct, first 64
        bool sitesOverflow = false;
    };
    struct Ordered { uint32_t addr, value, start, packet; uint64_t cycles; uint8_t width; uint32_t insnPc; };

    void NewPhase(const std::string& name);
    void WritePhase();
    uint32_t InsnPc(uint32_t start, uint32_t packet);   // pc of the packet-th packet of the block at start
    uint8_t Classify(const std::string& text);
    void ResolvePending();

    std::string dir_;
    Disasm disasm_;
    uint32_t curStart_ = 0;
    uint64_t curCycles_ = 0;
    uint32_t lastIpend_ = 0;
    int phaseIndex_ = 0;
    std::string phaseName_;
    int ffaHits_ = 0;
    int firstIndex_ = 0;   // > 0 after a watchdog reset: the new machine continues the numbering
    uint64_t phaseStartCycles_ = 0, phaseBlocks_ = 0;

    std::unordered_map<uint64_t, BlockStat> blocks_;        // (start << 32 | packets)
    std::unordered_map<uint64_t, uint32_t> insnPc_;          // (start << 32 | packet) -> insn pc, per phase
    std::unordered_map<uint64_t, uint64_t> transfers_;       // indirect + direct calls: (src insn pc << 32 | target)
    std::unordered_map<uint64_t, uint8_t> transferKind_;
    std::unordered_map<uint64_t, MmrStat> mmr_;              // addr | write << 32 | width << 33
    struct PendingSite { uint64_t mmrKey; uint32_t start, packet; };
    std::vector<PendingSite> pendingSites_;
    std::unordered_map<uint64_t, uint64_t> irq_;             // (ivg << 32 | handler)
    std::vector<Ordered> ordered_;
    size_t orderedCap_ = 200000, orderedResolved_ = 0;
    uint32_t orderedPerSite_ = 64;                           // per (addr, block, packet): data streams stay short
    std::unordered_map<uint64_t, uint32_t> orderedSeen_;
    uint64_t orderedDropped_ = 0, orderedSkipped_ = 0;

    std::mutex reqMu_;
    std::atomic<bool> reqPending_{false};
    std::string reqName_;
};
