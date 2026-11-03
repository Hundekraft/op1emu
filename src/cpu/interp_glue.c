/* Glue between the GDB Blackfin simulator's instruction interpreter (ext/bfin_sim, GPL) and op1emu.
 * Runs one instruction on a private copy of the CPU state. Memory goes through callbacks into the
 * emulator's memory map. Anything that needs the core event controller or the MMU (exceptions, RAISE,
 * CLI/STI, RTI/RTX/RTN/RTE, RETI push/pop, user-mode checks, misaligned access) aborts the step with
 * longjmp before it changes anything we keep; the caller then runs the block through the JIT instead. */
#include <setjmp.h>
#include <string.h>

#include "sim-main.h"
#include "bfin-sim.h"
#include "dv-bfin_cec.h"
#include "dv-bfin_mmu.h"

#include "interp_glue.h"

static jmp_buf bail_env;
static int bailing;
static const struct interp_glue_mem *glue_mem;
static int glue_user_mode;

static void bail (void)
{
  if (bailing)
    _longjmp (bail_env, 1);
}

/* ---- Simulator callbacks ---- */

static int mmr_touched;

unsigned
sim_core_read_buffer (SIM_DESC sd, sim_cpu *cpu, unsigned map, void *buffer, address_word addr, unsigned len)
{
  (void) sd; (void) cpu;
  if (glue_mem->trace && map != exec_map)
    glue_mem->trace (glue_mem->tctx, addr, len, 0);
  for (int r = 0; r < glue_mem->n_ranges; r++)
    if (addr - glue_mem->ranges[r].lo < glue_mem->ranges[r].size
        && len <= glue_mem->ranges[r].size - (addr - glue_mem->ranges[r].lo))
      {
        memcpy (buffer, glue_mem->ranges[r].host + (addr - glue_mem->ranges[r].lo), len);
        return len;
      }
  if (addr >= 0xFFC00000u)
    mmr_touched = 1;
  switch (len)
    {
    case 1: { uint8_t v = glue_mem->read8 (glue_mem->ctx, addr); memcpy (buffer, &v, 1); return 1; }
    case 2: { uint16_t v = glue_mem->read16 (glue_mem->ctx, addr); memcpy (buffer, &v, 2); return 2; }
    case 4: { uint32_t v = glue_mem->read32 (glue_mem->ctx, addr); memcpy (buffer, &v, 4); return 4; }
    default:
      for (unsigned i = 0; i < len; i++)
        ((uint8_t *) buffer)[i] = glue_mem->read8 (glue_mem->ctx, addr + i);
      return len;
    }
}

unsigned
sim_core_write_buffer (SIM_DESC sd, sim_cpu *cpu, unsigned map, const void *buffer, address_word addr, unsigned len)
{
  (void) sd; (void) cpu; (void) map;
  if (glue_mem->trace)
    glue_mem->trace (glue_mem->tctx, addr, len, 1);
  for (int r = 0; r < glue_mem->n_ranges; r++)
    if (addr - glue_mem->ranges[r].lo < glue_mem->ranges[r].size
        && len <= glue_mem->ranges[r].size - (addr - glue_mem->ranges[r].lo))
      {
        memcpy (glue_mem->ranges[r].host + (addr - glue_mem->ranges[r].lo), buffer, len);
        return len;
      }
  if (addr >= 0xFFC00000u)
    mmr_touched = 1;
  switch (len)
    {
    case 1: { uint8_t v; memcpy (&v, buffer, 1); glue_mem->write8 (glue_mem->ctx, addr, v); return 1; }
    case 2: { uint16_t v; memcpy (&v, buffer, 2); glue_mem->write16 (glue_mem->ctx, addr, v); return 2; }
    case 4: { uint32_t v; memcpy (&v, buffer, 4); glue_mem->write32 (glue_mem->ctx, addr, v); return 4; }
    default:
      for (unsigned i = 0; i < len; i++)
        glue_mem->write8 (glue_mem->ctx, addr + i, ((const uint8_t *) buffer)[i]);
      return len;
    }
}

/* ---- CEC / MMU: not ours to handle here ---- */

void cec_exception (SIM_CPU *cpu, int excp) { (void) cpu; (void) excp; bail (); }
void cec_hwerr (SIM_CPU *cpu, int hwerr) { (void) cpu; (void) hwerr; bail (); }
void cec_latch (SIM_CPU *cpu, int ivg) { (void) cpu; (void) ivg; bail (); }
bu32 cec_cli (SIM_CPU *cpu) { (void) cpu; bail (); return 0; }
void cec_sti (SIM_CPU *cpu, bu32 ints) { (void) cpu; (void) ints; bail (); }
void cec_push_reti (SIM_CPU *cpu) { (void) cpu; bail (); }
void cec_pop_reti (SIM_CPU *cpu) { (void) cpu; bail (); }
void cec_return (SIM_CPU *cpu, int ivg) { (void) cpu; (void) ivg; bail (); }
bool cec_is_user_mode (SIM_CPU *cpu) { (void) cpu; return glue_user_mode != 0; }
void cec_require_supervisor (SIM_CPU *cpu) { (void) cpu; if (glue_user_mode) bail (); }

void mmu_check_addr (SIM_CPU *cpu, bu32 addr, bool write, bool inst, int size)
{
  if (size > 1 && (addr & (size - 1)))
    bail ();
  if (!inst && glue_mem->data_check)
    {
      uint32_t flags = (write ? 1u << 16 : 0) | (!glue_user_mode ? 1u << 17 : 0)
        | (BFIN_CPU_STATE.multi_pc == PCREG + 6 ? 1u << 18 : 0);
      if (glue_mem->data_check (glue_mem->ctx, PCREG, addr, size, flags))
        bail ();
    }
}
void mmu_check_cache_addr (SIM_CPU *cpu, bu32 addr, bool write, bool inst)
{
  (void) cpu; (void) addr; (void) write; (void) inst;
}
void mmu_process_fault (SIM_CPU *cpu, bu32 addr, bool write, bool inst, bool unaligned, bool miss)
{
  (void) cpu; (void) addr; (void) write; (void) inst; (void) unaligned; (void) miss;
  bail ();
}
void bfin_trace_queue (SIM_CPU *cpu, bu32 src_pc, bu32 dst_pc, int hwloop)
{
  (void) cpu; (void) src_pc; (void) dst_pc; (void) hwloop;
}

/* ---- Step ---- */

static struct sim_state sim_sd;
static struct _sim_cpu sim_cpu_obj;
static struct bfin_cpu_state sim_st;

static unsigned
insn_length (bu16 iw0)
{
  if ((iw0 & 0xc000) != 0xc000)
    return 2;
  if ((iw0 & 0x0800) && (iw0 & 0xe800) != 0xe800)   /* multi-issue, not LINK */
    return 8;
  return 4;
}

unsigned
interp_glue_run (struct interp_glue_regs *regs, const struct interp_glue_mem *mem, int user_mode,
                 unsigned max_steps)
{
  static struct bfin_cpu_state saved;
  SIM_CPU *cpu = &sim_cpu_obj;
  unsigned steps = 0;
  int i;

  glue_mem = mem;
  glue_user_mode = user_mode;
  sim_sd.cpu = cpu;
  sim_sd.open_kind = SIM_OPEN_STANDALONE;
  sim_sd.environment = OPERATING_ENVIRONMENT;
  sim_cpu_obj.state = &sim_sd;
  sim_cpu_obj.arch_data = &sim_st;
  memset (&sim_st, 0, sizeof sim_st);
  memcpy (sim_st.dpregs, regs->dpregs, sizeof sim_st.dpregs);
  memcpy (sim_st.iregs, regs->iregs, sizeof sim_st.iregs);
  memcpy (sim_st.mregs, regs->mregs, sizeof sim_st.mregs);
  memcpy (sim_st.bregs, regs->bregs, sizeof sim_st.bregs);
  memcpy (sim_st.lregs, regs->lregs, sizeof sim_st.lregs);
  memcpy (sim_st.ax, regs->ax, sizeof sim_st.ax);
  memcpy (sim_st.aw, regs->aw, sizeof sim_st.aw);
  memcpy (sim_st.lt, regs->lt, sizeof sim_st.lt);
  memcpy (sim_st.lc, regs->lc, sizeof sim_st.lc);
  memcpy (sim_st.lb, regs->lb, sizeof sim_st.lb);
  memcpy (sim_st.emudat, regs->emudat, sizeof sim_st.emudat);
  memcpy (sim_st.cycles, regs->cycles, sizeof sim_st.cycles);
  sim_st.rets = regs->rets; sim_st.reti = regs->reti; sim_st.retx = regs->retx;
  sim_st.retn = regs->retn; sim_st.rete = regs->rete; sim_st.pc = regs->pc;
  sim_st.usp = regs->usp; sim_st.ksp = regs->ksp; sim_st.seqstat = regs->seqstat; sim_st.syscfg = regs->syscfg;
  {
    bu32 a = regs->astat;
    SET_ASTAT (a);
  }

  while (steps < max_steps)
    {
      bu32 pc = sim_st.pc, insn_len;
      bu16 iw0;
      int flow;

      sim_core_read_buffer (&sim_sd, cpu, exec_map, &iw0, pc, 2);
      /* IDLE needs op1emu's idle handling (wait for an interrupt): leave it to the JIT. */
      if (iw0 == 0x0020)
        break;
      if (glue_mem->trace)
        glue_mem->trace (glue_mem->tctx, pc, insn_length (iw0), 2);
      saved = sim_st;
      mmr_touched = 0;
      if (_setjmp (bail_env))
        {
          bailing = 0;
          sim_st = saved;   /* undo the partly executed instruction */
          break;
        }
      bailing = 1;
      sim_st.did_jump = false;
      insn_len = interp_insn_bfin (cpu, pc);
      bailing = 0;

      /* As GDB's step_once: advance (hardware loops included) unless the instruction jumped, then count
         the loop down when the instruction was a loop's last. */
      if (!sim_st.did_jump)
        sim_st.pc = hwloop_get_next_pc (cpu, pc, insn_len);
      for (i = 1; i >= 0; --i)
        if (sim_st.lc[i] && pc == sim_st.lb[i])
          {
            sim_st.lc[i]--;
            if (sim_st.lc[i])
              break;
          }
      ++steps;
      flow = sim_st.pc != pc + insn_length (iw0);
      if (flow || mmr_touched)
        break;
    }
  if (!steps)
    return 0;

  memcpy (regs->dpregs, sim_st.dpregs, sizeof sim_st.dpregs);
  memcpy (regs->iregs, sim_st.iregs, sizeof sim_st.iregs);
  memcpy (regs->mregs, sim_st.mregs, sizeof sim_st.mregs);
  memcpy (regs->bregs, sim_st.bregs, sizeof sim_st.bregs);
  memcpy (regs->lregs, sim_st.lregs, sizeof sim_st.lregs);
  memcpy (regs->ax, sim_st.ax, sizeof sim_st.ax);
  memcpy (regs->aw, sim_st.aw, sizeof sim_st.aw);
  memcpy (regs->lt, sim_st.lt, sizeof sim_st.lt);
  memcpy (regs->lc, sim_st.lc, sizeof sim_st.lc);
  memcpy (regs->lb, sim_st.lb, sizeof sim_st.lb);
  memcpy (regs->emudat, sim_st.emudat, sizeof sim_st.emudat);
  memcpy (regs->cycles, sim_st.cycles, sizeof sim_st.cycles);
  regs->rets = sim_st.rets; regs->reti = sim_st.reti; regs->retx = sim_st.retx;
  regs->retn = sim_st.retn; regs->rete = sim_st.rete; regs->pc = sim_st.pc;
  regs->usp = sim_st.usp; regs->ksp = sim_st.ksp; regs->seqstat = sim_st.seqstat; regs->syscfg = sim_st.syscfg;
  {
    SIM_CPU *c = cpu;
    (void) c;
    regs->astat = ASTAT;
  }
  return steps;
}
