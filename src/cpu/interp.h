#pragma once

#include "cpu_state.h"
#include "mem.h"

// One-instruction interpreter for code the JIT is still compiling (bcore's async compile): the GDB Blackfin
// simulator's decoder/executor (ext/bfin_sim) via interp_glue.c, on a copy of the CPU state. Built only with
// OP1EMU_INTERP (that links GPL code).
class Interpreter {
public:
    static bool Available();
    // Executes up to maxSteps instructions from cpu->pc, stopping after a change of flow or an MMR access.
    // Returns how many ran (cpu->packets too). 0: nothing changed, the instruction needs the CEC, the MMU,
    // an exception or IDLE; run the block through the JIT instead.
    static unsigned Run(CpuState* cpu, Memory* mem, bool userMode, unsigned maxSteps);
    // Plain memory the interpreter may access directly (guest base, size, host pointer); see Emulator::PlainRange.
    struct Range { uint32_t base, size; uint8_t* host; };
    static void SetDirectRanges(const Range* ranges, int count);
    // Called for every data access and instruction of the following Run()s (nullptr: off); see memmodel.h.
    using TraceFn = void (*)(void* ctx, uint32_t addr, uint32_t len, int kind);
    static void SetTrace(TraceFn fn, void* ctx);
};
