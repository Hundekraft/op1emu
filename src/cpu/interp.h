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
};
