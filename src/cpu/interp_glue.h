/* C interface to the bfin_sim interpreter glue (interp_glue.c). */
#ifndef INTERP_GLUE_H
#define INTERP_GLUE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct interp_glue_mem {
  void *ctx;
  uint8_t (*read8) (void *ctx, uint32_t addr);
  uint16_t (*read16) (void *ctx, uint32_t addr);
  uint32_t (*read32) (void *ctx, uint32_t addr);
  void (*write8) (void *ctx, uint32_t addr, uint8_t v);
  void (*write16) (void *ctx, uint32_t addr, uint16_t v);
  void (*write32) (void *ctx, uint32_t addr, uint32_t v);
  /* Guest addresses below raw_limit are plain memory at raw_base + addr (may be 0 = none). */
  uint8_t *raw_base;
  uint32_t raw_limit;
};

/* The registers one instruction can read or change; ASTAT as the architectural 32-bit word. */
struct interp_glue_regs {
  uint32_t dpregs[16], iregs[4], mregs[4], bregs[4], lregs[4];
  uint32_t ax[2], aw[2], lt[2], lc[2], lb[2];
  uint32_t rets, reti, retx, retn, rete, pc, usp, ksp, seqstat, syscfg, astat;
  uint32_t emudat[2];
  uint32_t cycles[3];   /* CYCLES, CYCLES2, CYCLES2 shadow */
};

enum { INTERP_GLUE_OK = 0, INTERP_GLUE_BAIL = 1 };

/* Executes up to max_steps instructions from regs->pc, stopping after an instruction that changes the flow
   (jump, branch, hardware-loop back edge) or touches an MMR (>= 0xFFC00000), and before one that needs
   something only the JIT path handles (CEC, MMU, exceptions, IDLE). Returns the number executed; regs hold
   the state after the last one (unchanged when 0). Not reentrant. */
unsigned interp_glue_run (struct interp_glue_regs *regs, const struct interp_glue_mem *mem, int user_mode,
                          unsigned max_steps);

#ifdef __cplusplus
}
#endif

#endif
