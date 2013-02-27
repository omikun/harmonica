#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

#include <chdl/gateops.h>
#include <chdl/bvec-basic-op.h>
#include <chdl/adder.h>
#include <chdl/mult.h>
#include <chdl/divider.h>
#include <chdl/shifter.h>
#include <chdl/mux.h>
#include <chdl/enc.h>
#include <chdl/llmem.h>
#include <chdl/memory.h>

#include <chdl/opt.h>
#include <chdl/tap.h>
#include <chdl/sim.h>
#include <chdl/netlist.h>

#include "pipeline.h"
#include "funcunit.h"

const unsigned LOG2WIDTH(5), WIDTH(1<<LOG2WIDTH), REGS(8),
               LOG2ROMSZ(7), RAMSZ(256), IIDBITS(6);

#define DEBUG
//#define WITH_FPU  // Floating point
//#define WITH_FXPU // Fixed point
//#define WITH_DIV
//#define WITH_MUL
static const unsigned FPU_E(8), FPU_M(23);

#ifdef DEBUG
#define DBGTAP(x) do {TAP(x); } while(0)
#else
#define DBGTAP(x) do {} while(0)
#endif

using namespace std;
using namespace chdl;

#include "harpinst.h"
#include "fpu.h"
#include "regfile.h"

// This is the basic design for the branch prediction module. Given the current
// fetch PC, this unit has to determine the next PC at which to fetch. When
// jumps are taken and resolved, the "jmpPc" and "takenJmp" signals are
// asserted.
template <unsigned LAT, unsigned N>
  bvec<N> BranchPredict
    (node flushOut, bvec<N> pc, node stall, bvec<N> jmpPc, node takenJmp)
{
  flushOut = takenJmp;
  return Mux(takenJmp, Mux(stall, pc + Lit<N>(N/8), pc), jmpPc);
  //return Mux(stall, Mux(takenJmp, pc + Lit<N>(N/8), jmpPc), pc);
}

template <unsigned N> bvec<N> InstructionMemory(bvec<N> addr) {
  // Stall every tenth cycle, simulating, e.g., icache misses.
  bvec<4> stallctr;
  stallctr = Reg(Mux(GetStall(0), 
                      Mux(stallctr == Lit<4>(9),
                          stallctr + Lit<4>(1), Lit<4>(0)),
                      stallctr));
  PipelineBubble(0, stallctr == Lit<4>(9));
  DBGTAP(stallctr);

  bvec<LOG2ROMSZ-2> a(addr[range<2, LOG2ROMSZ-1>()]);
  return PipelineReg(0, LLRom<LOG2ROMSZ-2, N>(a, "rom.hex"));
}

template<unsigned N, unsigned R> struct harmonica {
  harmonica() {}
  ~harmonica() {
    for (size_t i = 0; i < funcUnits.size(); ++i) delete funcUnits[i];
  }

  void addFuncUnit(FuncUnit<N, R> *fu) { funcUnits.push_back(fu); }

  void generate() {
    // // // Fetch  Unit // // //
    bvec<IIDBITS> iid;
    bvec<N> pc, jmpPc;
    node takenJmp, validInst(GetValid(0)), brMispred;
    iid = Reg(Mux(GetStall(0), iid + Lit<IIDBITS>(1), iid));
    pc = Reg(BranchPredict<2>(brMispred, pc, GetStall(0) || !validInst, jmpPc, takenJmp));
    DBGTAP(pc);
    DBGTAP(iid);
    DBGTAP(validInst);
    DBGTAP(brMispred);


    bvec<N> ir(InstructionMemory(pc));
    DBGTAP(ir);

    // Fetch->Decode pipeline regs
    node validInst_d(PipelineReg(1, validInst)); DBGTAP(validInst_d);
    bvec<IIDBITS> iid_d(PipelineReg(1, iid));    DBGTAP(iid_d);
    bvec<N> pc_d(pc);

    // // // Decoder // // //
    harpinst<N, CLOG2(R), CLOG2(R)> inst(ir);
    node wrmem_d(inst.is_store());

    // // // Registers/Scheduling // // //
    // Predicate register file
    node predvalue, px, p0value, p0valid, p1value, p1valid, p_wb_val, p_wb,
         predvalid;
    bvec<CLOG2(R)> p_wb_idx;
    bvec<IIDBITS> p_wb_iid, p_wb_curiid;
    vec<3, rdport<CLOG2(R), 1>> prf_rd;
    prf_rd[0] = rdport<CLOG2(R), 1>(inst.get_psrc0(), bvec<1>(p0value));
    prf_rd[1] = rdport<CLOG2(R), 1>(inst.get_psrc1(), bvec<1>(p1value));
    prf_rd[2] = rdport<CLOG2(R), 1>(inst.get_pred(), bvec<1>(predvalue));
    wrport<CLOG2(R), 1> prf_wr(p_wb_idx, bvec<1>(p_wb_val), p_wb);
    Regfile(prf_rd, prf_wr, "p");

    px = validInst_d && (!inst.has_pred() || predvalue);

    DBGTAP(predvalue);
    DBGTAP(predvalid);
    DBGTAP(px);    

    // Predicate valid bits
    node wrpred_d(px && inst.has_pdst() && !GetStall(1));
    vec<3, rdport<CLOG2(R), 1> > pvb_rd;
    pvb_rd[0] = rdport<CLOG2(R), 1>(inst.get_psrc0(), bvec<1>(p0valid));
    pvb_rd[1] = rdport<CLOG2(R), 1>(inst.get_psrc1(), bvec<1>(p1valid));
    pvb_rd[2] = rdport<CLOG2(R), 1>(inst.get_pred(), bvec<1>(predvalid));
    Bitfile(pvb_rd, p_wb_idx, p_wb, inst.get_pdst(), wrpred_d, "p");
    PipelineBubble(2, inst.has_psrc0() && !p0valid);
    PipelineBubble(2, inst.has_psrc1() && !p1valid);
    PipelineBubble(2, inst.has_pred() && !predvalid);

    // Predicate writer IID bits
    vec<1, rdport<CLOG2(R), IIDBITS> > piid_rd;
    piid_rd[0] = rdport<CLOG2(R), IIDBITS>(p_wb_idx, p_wb_curiid);
    wrport<CLOG2(R), IIDBITS> piid_wr(inst.get_pdst(), iid_d, wrpred_d);
    Regfile(piid_rd, piid_wr, "piid");

    // GP register file
    bvec<N> r0value, r1value, r2value, r_wb_val;
    node r0valid, r1valid, r2valid, r_wb;
    bvec<CLOG2(R)> r_wb_idx;
    bvec<IIDBITS> r_wb_iid, r_wb_curiid;
    vec<3, rdport<CLOG2(R), N>> rf_rd;
    rf_rd[0] = rdport<CLOG2(R), N>(inst.get_rsrc0(), r0value);
    rf_rd[1] = rdport<CLOG2(R), N>(inst.get_rsrc1(), r1value);
    rf_rd[2] = rdport<CLOG2(R), N>(inst.get_rsrc2(), r2value);
    wrport<CLOG2(R), N> rf_wr(r_wb_idx, r_wb_val, r_wb);
    Regfile(rf_rd, rf_wr);

    DBGTAP(r0valid); DBGTAP(r1valid); DBGTAP(r2valid);
    DBGTAP(r0value); DBGTAP(r1value); DBGTAP(r2value);
    DBGTAP(r_wb); DBGTAP(r_wb_iid); DBGTAP(r_wb_curiid);
    DBGTAP(r_wb_idx); DBGTAP(r_wb_val);
    DBGTAP(p_wb); DBGTAP(p_wb_iid); DBGTAP(p_wb_curiid);
    DBGTAP(p_wb_idx); DBGTAP(p_wb_val);

    // GPR valid bits
    node wrreg_d(px && inst.has_rdst() && !GetStall(1));
    vec<3, rdport<CLOG2(R), 1> > rvb_rd;
    rvb_rd[0] = rdport<CLOG2(R), 1>(inst.get_rsrc0(), bvec<1>(r0valid));
    rvb_rd[1] = rdport<CLOG2(R), 1>(inst.get_rsrc1(), bvec<1>(r1valid));
    rvb_rd[2] = rdport<CLOG2(R), 1>(inst.get_rsrc2(), bvec<1>(r2valid));
    Bitfile(rvb_rd, r_wb_idx, r_wb, inst.get_rdst(), wrreg_d);
    PipelineBubble(2, inst.has_rsrc0() && !r0valid);
    PipelineBubble(2, inst.has_rsrc1() && !r1valid);
    PipelineBubble(2, inst.has_rsrc2() && !r2valid);

    DBGTAP(wrreg_d);
    DBGTAP(inst.get_rdst());

    // GPR writer IID bits
    vec<1, rdport<CLOG2(R), IIDBITS> > riid_rd;
    riid_rd[0] = rdport<CLOG2(R), IIDBITS>(r_wb_idx, r_wb_curiid);
    wrport<CLOG2(R), IIDBITS> riid_wr(inst.get_rdst(), iid_d, wrreg_d);
    Regfile(riid_rd, riid_wr, "riid");

    vec<8, fuInput<N, R>> fuin;
    bvec<8> fustall;
    for (unsigned i = 0; i < 8; ++i) {
      fuin[i].pc = PipelineReg(2, PipelineReg(1, pc + Lit<N>(N/8)));
      fuin[i].r0 = PipelineReg(2, r0value);
      fuin[i].r1 = PipelineReg(2, r1value);
      fuin[i].r2 = PipelineReg(2, r2value);
      fuin[i].imm = PipelineReg(2, inst.get_imm());
      fuin[i].p0 = PipelineReg(2, p0value);
      fuin[i].p1 = PipelineReg(2, p1value);
      fuin[i].hasimm = PipelineReg(2, inst.has_imm());
      fuin[i].iid = PipelineReg(2, iid_d);
      fuin[i].op = PipelineReg(2, inst.get_opcode());
      fuin[i].didx = PipelineReg(2, inst.get_rdst());
      fuin[i].pdest = PipelineReg(2, inst.has_pdst());
      fuin[i].stall = fustall[i];
    }

    DBGTAP(fuin[0].r0);
    DBGTAP(fuin[0].r1);
    DBGTAP(fuin[0].r2);
    DBGTAP(fuin[0].imm);
    DBGTAP(fuin[0].p0);
    DBGTAP(fuin[0].p1);
    DBGTAP(fuin[0].iid);
    DBGTAP(fuin[0].didx);
    DBGTAP(fuin[0].op);
    DBGTAP(fustall);

    // Determine taken jump
    takenJmp = !GetStall(1) && px && inst.is_jmp();
    jmpPc = Mux(inst.has_imm(), r0value, pc_d + inst.get_imm());
    PipelineFlush(1, brMispred);

    DBGTAP(takenJmp);

    // // // Functional Units // // //
    // Decide at the decode stage which functional unit is producing the result.
    bvec<64> opdec(Decoder(inst.get_opcode(), px));
    vector<node> fu_sel;
    for (unsigned i = 0; i < funcUnits.size(); ++i) {
      vector<node> sel_nodes;
      vector<unsigned> opcodes(funcUnits[i]->get_opcodes());
      for (unsigned j = 0; j < opcodes.size(); ++j)
        sel_nodes.push_back(opdec[opcodes[j]]);
      fu_sel.push_back(OrN(sel_nodes) && (wrreg_d || wrpred_d || wrmem_d));
      DBGTAP(fu_sel[i]);
    }

    // Attach the inputs and outputs
    vec<8, fuOutput<N, R>> fuout;
    bvec<8> fu_issue, fu_notready;
    for (unsigned i = 0; i < funcUnits.size(); ++i) {
      fu_issue[i] = PipelineReg(2, fu_sel[i]);
      fuout[i] = funcUnits[i]->generate(fuin[i], fu_issue[i]);
      fu_notready[i] = !funcUnits[i]->ready() && fu_issue[i];
    }
    PipelineStall(2, OrN(fu_notready));
    DBGTAP(fuout[0].out);
    DBGTAP(fuout[0].valid);
    DBGTAP(fuout[0].iid);
    DBGTAP(fuout[0].didx);
    DBGTAP(fuout[1].out);
    DBGTAP(fuout[1].valid);
    DBGTAP(fuout[1].iid);
    DBGTAP(fuout[1].didx);
    DBGTAP(fu_notready);

    // Writeback
    bvec<8> fu_valid, fu_pdest;
    for (unsigned i = 0; i < 8; ++i) {
      fu_valid[i] = fuout[i].valid;
      fu_pdest[i] = fuout[i].pdest;
    }
    node try_r_wb(OrN(fu_valid & ~fu_pdest)),
         try_p_wb(OrN(fu_valid & fu_pdest));
    bvec<3> fu_r_out_sel(Log2(fu_valid & ~fu_pdest)),
            fu_p_out_sel(Log2(fu_valid & fu_pdest));
    bvec<8> other_val_r(~(Lit<8>(1)<<fu_r_out_sel) & (fu_valid&~fu_pdest)),
            other_val_p(~(Lit<8>(1)<<fu_p_out_sel) & (fu_valid&fu_pdest));
    node collision_r(OrN(other_val_r)), collision_p(OrN(other_val_p));
    DBGTAP(other_val_r); DBGTAP(collision_r);
    DBGTAP(other_val_p); DBGTAP(collision_p);

    fustall = other_val_r | other_val_p;

    vec<8, bvec<N> > fu_outputs;
    vec<8, bvec<LOG2(R)> > fu_didx;
    vec<8, bvec<IIDBITS> > fu_iid;
    for (unsigned i = 0; i < funcUnits.size(); ++i) {
      fu_outputs[i] = fuout[i].out;
      fu_didx[i] = fuout[i].didx;
      fu_iid[i] = fuout[i].iid;
    }
    r_wb_val = Mux(fu_r_out_sel, fu_outputs);
    r_wb_idx = Mux(fu_r_out_sel, fu_didx);
    r_wb_iid = Mux(fu_r_out_sel, fu_iid);

    p_wb_val = Mux(fu_p_out_sel, fu_outputs)[0];
    p_wb_idx = Mux(fu_p_out_sel, fu_didx);
    p_wb_iid = Mux(fu_p_out_sel, fu_iid);

    r_wb = try_r_wb && r_wb_iid == r_wb_curiid;
    p_wb = try_p_wb && p_wb_iid == p_wb_curiid;
    DBGTAP(try_p_wb); DBGTAP(fu_pdest); DBGTAP(fu_valid);

    // Generate hazard unit/pipeline regs.
    genPipelineRegs();
  }  


  vector<FuncUnit<N, R>*> funcUnits;
};

int main() {
  harmonica<WIDTH, REGS> pipeline;

  pipeline.addFuncUnit(new BasicAlu<WIDTH, REGS>());
  pipeline.addFuncUnit(new PredLu<WIDTH, REGS>());
  pipeline.addFuncUnit(new SramLsu<WIDTH, REGS, RAMSZ>());

  pipeline.generate();

  optimize();

  // Do the simulation
  ofstream wave_file("harmonica.vcd");
  run(wave_file, 2000);

  // Print the netlist
  ofstream netlist_file("harmonica.nand");
  print_netlist(netlist_file);
}
