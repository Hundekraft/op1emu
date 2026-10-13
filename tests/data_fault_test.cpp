#include "cpu/emu.h"
#include "cpu/bcore_memory.h"
#include "cpu/interp.h"
#include "core.h"
#include "mmr.h"
#include "../ext/bcore/src/cec.h"
#include "../ext/bcore/src/evt.h"
#include <cstdio>
#include <cstdlib>
static void require(bool v,const char* m){if(!v){fprintf(stderr,"FAIL: %s\n",m);exit(1);}}
int main(){
    Emulator emu;
    MemoryDevice ram("RAM",0,0x10000),mmu("MMU",0xFFE00000,0x2000);
    ram.BindFastMem(nullptr);mmu.BindFastMem(nullptr);emu.BindDevice(&ram);emu.BindDevice(&mmu);
    EmulatorMemory mem(emu);CpuState cpu{};Core core(&cpu,&mem);require(core.init(2),"init");
    cec_init();evt_init();evt_write(EVT_BASE+12,0x300);cec_return_rti(&cpu);
    emu.MemoryWrite16(0x100,0x9000); // R0 = [P0++]
    emu.MemoryWrite16(0x102,0x2000);
    cpu.pc=0x100;cpu.dpregs[0]=0x1234;cpu.dpregs[8]=0x201;
    if(Interpreter::Available()){
        require(Interpreter::Run(&cpu,&mem,true,1)==0,"interpreter yields fault to JIT");
        require(cpu.pc==0x100 && cpu.dpregs[0]==0x1234 && cpu.dpregs[8]==0x201,"interpreter rollback");
    }
    require(core.run(0x100),"faulting load");
    require(cpu.pc==0x300 && cpu.retx==0x100 && (cpu.seqstat&63)==0x24,"exception dispatch");
    require(emu.MemoryRead32(0xFFE0000C)==0x201 && emu.MemoryRead32(0xFFE00008)==0,"data fault MMRs");
    require(emu.MemoryRead32(0xFFE0100C)==0x100 && emu.MemoryRead32(0xFFE01008)==0,"instruction fault MMRs");
    // An actual guest handler reads the MMR through JIT, then reaches a loop.
    emu.MemoryWrite16(0x300,0x9000);emu.MemoryWrite16(0x302,0x2000);
    cpu.dpregs[8]=0xFFE0000C;require(core.run(cpu.pc),"guest handler");
    require(cpu.dpregs[0]==0x201,"guest can inspect fault address");
    // Diagnostic model: enabled data CPLBs, page boundaries, permissions and overlap.
    setenv("OP1EMU_STRICT_MEMORY","1",1);EmulatorMemory strict(emu);
    require(strict.strict_data_checks(),"strict mode enabled");
    strict.write32(0xFFE00004,2);
    strict.write32(0xFFE00100,0);strict.write32(0xFFE00200,1|4|8|16); // 1 KiB, user RW
    require(strict.check_data_access(0x100,0x3fc,4,0)==0,"last word in page");
    require(strict.check_data_access(0x100,0x400,4,0)==0x26,"page miss");
    require(emu.MemoryRead32(0xFFE00008)==(1u<<19),"miss status");
    strict.write32(0xFFE00200,1|4); // user read only
    require(strict.check_data_access(0x100,0x200,4,1u<<16)==0x23,"user write protection");
    require(emu.MemoryRead32(0xFFE00008)==((1u<<16)|1),"matched entry in status");
    require(strict.check_data_access(0x100,0x200,4,(1u<<16)|(1u<<17))==0x23,"supervisor write protection");
    strict.write32(0xFFE00104,0);strict.write32(0xFFE00204,1|4);
    require(strict.check_data_access(0x100,0x200,4,0)==0x27,"overlapping CPLBs");
    require((emu.MemoryRead32(0xFFE00008)&0xffff)==3,"both matching CPLBs");
    strict.write32(0xFFE00204,0);strict.write32(0xFFE00200,1|16|(1u<<12));
    require(strict.check_data_access(0x100,0x200,4,(1u<<16)|(1u<<17))==0x23,"clean writeback page");
    strict.write32(0xFFE00200,1|16|(1u<<12)|(1u<<7));
    require(strict.check_data_access(0x100,0x200,4,(1u<<16)|(1u<<17))==0,"dirty writeback page");
    require(strict.check_data_access(0x100,0xFFE00004,4,0)==0x23,"user MMR denied");
    require(strict.check_data_access(0x100,0xFFE00004,4,(1u<<17)|(1u<<18))==0x23,"DAG1 MMR denied");
    require(strict.check_data_access(0x100,0xFFE00004,4,1u<<17)==0,"supervisor MMR allowed");
    strict.write32(0xFFE00004,0);
    require(strict.check_data_access(0x100,0x45534e4c,4,1u<<17)==UINT32_MAX,"unmapped pointer stops diagnostically");
    CpuState scpu{};Core strictCore(&scpu,&strict);require(strictCore.init(2),"strict JIT init");
    cec_init();evt_init();evt_write(EVT_BASE+12,0x300);cec_return_rti(&scpu);
    scpu.pc=0x100;scpu.dpregs[8]=0x45534e4c;
    require(Interpreter::Run(&scpu,&strict,true,1)==0,"strict mode never bypassed by interpreter");
    require(strictCore.run(0x100) && scpu.halted && scpu.did_jump && scpu.pc==0x100,"JIT diagnostic stops before invalid load");
    cec_init();evt_init();evt_write(EVT_BASE+12,0x300);cec_return_rti(&scpu);
    scpu.halted=false;scpu.pc=0x100;scpu.dpregs[8]=0x200;scpu.dpregs[0]=0x87654321;
    strict.write32(0xFFE00004,2);strict.write32(0xFFE00200,0);
    const uint32_t beforeStatus=emu.MemoryRead32(0xFFE00008);
    const uint32_t beforeAddress=emu.MemoryRead32(0xFFE0000C);
    require(Interpreter::Run(&scpu,&strict,true,1)==0,"interpreter rejects CPLB miss");
    require(emu.MemoryRead32(0xFFE00008)==beforeStatus && emu.MemoryRead32(0xFFE0000C)==beforeAddress,"interpreter probe leaves fault MMRs unchanged");
    require(strictCore.run(0x100) && (scpu.seqstat&63)==0x26 && scpu.pc==0x300,"CPLB miss before inline RAM access");
    require(scpu.dpregs[0]==0x87654321 && scpu.dpregs[8]==0x200,"protection fault preserves destination and pointer");
    strict.write32(0xFFE00200,1|4|8|16);
    scpu.pc=0x100;scpu.dpregs[8]=0x200;strict.write32(0x200,0xdeadbeef);
    if(Interpreter::Available()) {
        require(Interpreter::Run(&scpu,&strict,true,1)==1,"strict interpreter permits valid load");
        require(scpu.dpregs[0]==0xdeadbeef && scpu.dpregs[8]==0x204,"strict interpreted load result");
    }
    unsetenv("OP1EMU_STRICT_MEMORY");
    puts("PASS: interpreter fallback, EVT3, fault MMRs, data CPLB checks and invalid-pointer stop");
}
