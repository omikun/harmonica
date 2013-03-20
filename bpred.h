#ifndef __BPRED_H
#define __BPRED_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <chdl/shifter.h>

// This is the basic design for the branch prediction module. Given the current
// fetch PC, this unit has to determine the next PC at which to fetch. When
// jumps are taken and resolved, the "jmpPc" and "takenJmp" signals are
// asserted.

static const unsigned BR_CNT(2), BR_SIZE(32);

template <unsigned N> struct branchDelay {
  branchDelay() {}
  branchDelay(node taken, bvec<N> targetPc, bvec<CLOG2(BR_SIZE)> btb_idx, bvec<CLOG2(BR_SIZE)> bpt_idx):
    taken(taken), targetPc(targetPc), btb_idx(btb_idx), bpt_idx(bpt_idx) {}

  // targetPc that is read from the BTB
  // It is compared with jmpPc to determine if the targetPc was correct.
  // If not, flushOut is generated.
  node taken;
  bvec<N> targetPc;
  // Keep previous btb_rd_idx, and bpt_rd_idx to update the BTB and BPT,
  // after the jmpPc is resolved.
  bvec<CLOG2(BR_SIZE)> btb_idx, bpt_idx;
};

template <unsigned LAT, unsigned N>
  bvec<N> BranchPredict
    (node flushOut, bvec<N> pc, node stall, bvec<N> jmpPc, node takenJmp)
{
  // Reset signal to initialize states
  node reset = Reg(Lit(1)); // 01111.... pulse
  node reset_ = !reset; // 10000... pulse

  // Branch delay queue
  vec<LAT,branchDelay<N> > bdq;
  // Branch target buffer
  vec<BR_SIZE,bvec<N> > btb;
  // Global history register - currently as long as CLOG2(BR_SIZE)
  bvec<CLOG2(BR_SIZE)> ghr;
  // Branch pattern table
  vec<BR_SIZE,bvec<BR_CNT> > bpt;
  // Counters
  bvec<N> cnt_pred_taken, cnt_actual_taken;

  // Part of pc used to index tables
  // Branch tables are indexed by lowest CLOG2(BR_SIZE) bits (i.e., 5 bits for 32-entry table), 
  // after taking out the least N/2 bits (i.e., 4 bits for 32-bit inst) that are useless.
  bvec<CLOG2(BR_SIZE)> pc_idx = pc[range<N/2,N/2+CLOG2(BR_SIZE)-1>()];

  /***** BTB and BPT indices *****/
  // Read indices are based on the current pc.
  // Write indices are the pc of <LAT> cycles ago,
  // which means BTB and BPT are not speculatively update, but only after jmps are resolved.

  // BTB read index - current pc
  // Even if flushOut happens, just read BTB entry - Pipeline may handle it.
  bvec<CLOG2(BR_SIZE)> btb_rd_idx = pc_idx;
  // Demux'd BTB read signal
  bvec<BR_SIZE> btb_rd_sig = Decoder(btb_rd_idx,
                                     Mux(/*Stall?*/stall,
                                         /*NS*/Lit(1),
                                         /*S*/Lit(0)));
  // BTB write index - pc of <LAT> cycles ago
  // Even if flushOut happens, BTB must be updated
  bvec<CLOG2(BR_SIZE)> btb_wr_idx = bdq[LAT-1].btb_idx;
  // Demux'd BTB write signal
  bvec<BR_SIZE> btb_wr_sig = Decoder(btb_wr_idx,
                                     Mux(/*Stall?*/stall,
                                         /*NS*/Lit(1),
                                         /*S*/Lit(0)));

  // BPT read index - XOR(pc,ghr);
  bvec<CLOG2(BR_SIZE)> bpt_rd_idx = Xor(pc_idx,
                                        ghr[range<0,CLOG2(BR_SIZE)-1>()]);
  // Demux'd BPT read signal
  bvec<BR_SIZE> bpt_rd_sig = Decoder(bpt_rd_idx,
                                     Mux(/*Stall?*/stall,
                                         /*NS*/Lit(1),
                                         /*S*/Lit(0)));
  // BPT write index - xor(pc of <LAT> cycles ago, ghr or <LAT> cycles ago)
  bvec<CLOG2(BR_SIZE)> bpt_wr_idx = bdq[LAT-1].bpt_idx;
  // Demux'd BPT write signal
  bvec<BR_SIZE> bpt_wr_sig = Decoder(bpt_wr_idx,
                                     Mux(/*Stall?*/stall,
                                         /*NS*/Lit(1),
                                         /*S*/Lit(0)));

  /***** Read operations *****/
  // Target PC in the BTB.
  bvec<N> targetPc = Mux(btb_rd_idx,btb);
  // nextPc = pc + 4
  bvec<N> nextPc = pc+Lit<N>(N/8);
  // The MSB of counter indicates whether it's a taken branch.
  node taken =  Mux(bpt_rd_idx,bpt)[CLOG2(BR_CNT)-1];

  /***** Write operations *****/
  flushOut = Reg(Mux(/*Stall?*/stall,
                     // flushOut is generated if predicted taken for !takenJmp
                     // or wrong targetPc for jmpPc.
                     /*NS*/Mux(/*Taken?*/takenJmp,
                               /*NT*/bdq[LAT-1].taken,
                               /*T*/bdq[LAT-1].targetPc!=jmpPc),
                     /*S*/Lit(0)));

  for(int i = 0; i < BR_SIZE; i++)
  {
    // For a taken branch, update the BTB.
    btb[i] = Wreg<N>(btb_wr_sig[i],
                                   Mux(/*Taken?*/takenJmp,
                                       /*NT*/btb[i],
                                       /*T*/jmpPc));
    // Update branch pattern table based on takenJmp result.
    bpt[i] = Wreg<BR_CNT>(Mux(/*reset?*/reset_,
                              /*NR*/bpt_wr_sig[i],
                              /*R*/Lit(1)),
                                                  Mux(/*!reset?*/reset_,
                                                      /*NR*/Mux(/*Taken?*/takenJmp,
                                                                // Not-taken jmp decreases a saturating counter
                                                                /*NT*/Mux(/*!00?*/OrN(bpt[i]),
                                                                          /*00*/bpt[i],
                                                                          /*!00*/bpt[i]-bvec<BR_CNT>(Lit(1))), 
                                                                // Taken jmp increases a saturating counter
                                                                /*T*/Mux(/*11?*/AndN(bpt[i]),
                                                                         /*!11*/bpt[i]+bvec<BR_CNT>(Lit(1)),
                                                                         /*11*/bpt[i])),
                                                      // BPT counters are initialized to 1 (weakly not taken).
                                                      /*R*/bvec<BR_CNT>(Lit((1<<(BR_CNT/2))-1))));
  }

  // Shift branch delay queue (always).
  for(int i = 1; i < LAT; i++)
  {
    bdq[i] = bdq[i-1];
  }
  // New branch delay queue entry with targetPc, BTB index, and BPT index
  bdq[0] = branchDelay<N>(taken,targetPc,btb_rd_idx,bpt_rd_idx);

  // Shift global history register (only when !stall).
  ghr = Wreg<CLOG2(BR_SIZE)>(!stall,Cat(/*ghr[1:MSB]*/ghr[range<0,CLOG2(BR_SIZE)-2>()],
                                        /*ghr[0]*/takenJmp));

  /***** Count operations *****/
  cnt_pred_taken = Wreg<N>(!stall,
                           Mux(/*Taken?*/taken, 
                               /*NT*/cnt_pred_taken,
                               /*T*/cnt_pred_taken+bvec<N>(Lit(1))));
  cnt_actual_taken = Wreg<N>(!stall,
                             Mux(/*Taken?*/takenJmp,
                                 /*NT*/cnt_actual_taken,
                                 /*T*/cnt_actual_taken+bvec<N>(Lit(1))));

  return Mux(/*Taken*/taken,
             /*NT*/nextPc,
             /*T*/targetPc);
}


#endif
