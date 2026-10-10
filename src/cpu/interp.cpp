#include "interp.h"

#ifdef OP1EMU_INTERP
#include "interp_glue.h"
#include <cstring>

namespace {
uint8_t Read8(void* m, uint32_t a) { return static_cast<Memory*>(m)->read8(a); }
uint16_t Read16(void* m, uint32_t a) { return static_cast<Memory*>(m)->read16(a); }
uint32_t Read32(void* m, uint32_t a) { return static_cast<Memory*>(m)->read32(a); }
void Write8(void* m, uint32_t a, uint8_t v) { static_cast<Memory*>(m)->write8(a, v); }
void Write16(void* m, uint32_t a, uint16_t v) { static_cast<Memory*>(m)->write16(a, v); }
void Write32(void* m, uint32_t a, uint32_t v) { static_cast<Memory*>(m)->write32(a, v); }
}  // namespace

bool Interpreter::Available() { return true; }

unsigned Interpreter::Run(CpuState* cpu, Memory* mem, bool userMode, unsigned maxSteps) {
    interp_glue_regs r;
    std::memcpy(r.dpregs, cpu->dpregs, sizeof r.dpregs);
    std::memcpy(r.iregs, cpu->iregs, sizeof r.iregs);
    std::memcpy(r.mregs, cpu->mregs, sizeof r.mregs);
    std::memcpy(r.bregs, cpu->bregs, sizeof r.bregs);
    std::memcpy(r.lregs, cpu->lregs, sizeof r.lregs);
    std::memcpy(r.ax, cpu->ax, sizeof r.ax);
    std::memcpy(r.aw, cpu->aw, sizeof r.aw);
    std::memcpy(r.lt, cpu->lt, sizeof r.lt);
    std::memcpy(r.lc, cpu->lc, sizeof r.lc);
    std::memcpy(r.lb, cpu->lb, sizeof r.lb);
    std::memcpy(r.emudat, cpu->emudat, sizeof r.emudat);
    std::memcpy(r.cycles, cpu->cycles, sizeof r.cycles);
    r.rets = cpu->rets; r.reti = cpu->reti; r.retx = cpu->retx; r.retn = cpu->retn; r.rete = cpu->rete;
    r.pc = cpu->pc; r.usp = cpu->usp; r.ksp = cpu->ksp; r.seqstat = cpu->seqstat; r.syscfg = cpu->syscfg;
    r.astat = astat_compose(cpu);

    const uint32_t limit = mem->rawmem_limit();
    const interp_glue_mem m{mem, Read8, Read16, Read32, Write8, Write16, Write32,
                            limit ? reinterpret_cast<uint8_t*>(mem->fast_base()) : nullptr, limit};
    const unsigned steps = interp_glue_run(&r, &m, userMode ? 1 : 0, maxSteps);
    if (!steps) return 0;

    std::memcpy(cpu->dpregs, r.dpregs, sizeof r.dpregs);
    std::memcpy(cpu->iregs, r.iregs, sizeof r.iregs);
    std::memcpy(cpu->mregs, r.mregs, sizeof r.mregs);
    std::memcpy(cpu->bregs, r.bregs, sizeof r.bregs);
    std::memcpy(cpu->lregs, r.lregs, sizeof r.lregs);
    std::memcpy(cpu->ax, r.ax, sizeof r.ax);
    std::memcpy(cpu->aw, r.aw, sizeof r.aw);
    std::memcpy(cpu->lt, r.lt, sizeof r.lt);
    std::memcpy(cpu->lc, r.lc, sizeof r.lc);
    std::memcpy(cpu->lb, r.lb, sizeof r.lb);
    std::memcpy(cpu->emudat, r.emudat, sizeof r.emudat);
    std::memcpy(cpu->cycles, r.cycles, sizeof r.cycles);
    cpu->rets = r.rets; cpu->reti = r.reti; cpu->retx = r.retx; cpu->retn = r.retn; cpu->rete = r.rete;
    cpu->pc = r.pc; cpu->usp = r.usp; cpu->ksp = r.ksp; cpu->seqstat = r.seqstat; cpu->syscfg = r.syscfg;
    astat_decompose(cpu, r.astat);
    cpu->did_jump = false;
    cpu->packets = steps;
    return steps;
}
#else
bool Interpreter::Available() { return false; }
unsigned Interpreter::Run(CpuState*, Memory*, bool, unsigned) { return 0; }
#endif
