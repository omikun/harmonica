#ifndef __PIPELINE_H
#define __PIPELINE_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

#include <chdl/gateops.h>
#include <chdl/bvec-basic-op.h>

#include <chdl/adder.h>
#include <chdl/shifter.h>
#include <chdl/mux.h>
#include <chdl/enc.h>
#include <chdl/llmem.h>
#include <chdl/memory.h>

#include <chdl/opt.h>
#include <chdl/tap.h>
#include <chdl/sim.h>
#include <chdl/netlist.h>

//#include "hazunit.h"


    // // // // // // // // // // // // // // // // // // // // // // // // //
// // // TODO - These should probably find their way into CHDL headers  // // //
  // // // // // // // // // // // // // // // // // // // // // // // // //

chdl::node OrN(std::vector<chdl::node> &v);
chdl::node AndN(std::vector<chdl::node> &v);
chdl::node operator==(std::vector<chdl::node> a, std::vector<chdl::node> b);

    // // // // // // // // // // // // // // // // // // // // // // // // //
// // // // // // // // // // // // // // // // // // // // // // // // // // //
  // // // // // // // // // // // // // // // // // // // // // // // // //

// One bit of a pipeline register                                               
struct pregbit {
  pregbit(chdl::node d, chdl::node q): d(d), q(q) {}
  chdl::node d, q;
};

// Entire pipeline register                                                     
struct preg {
  std::vector<chdl::node> stall, flush, bubble;
  std::vector<pregbit> bits;
  chdl::node anystall, flushorbubble;
};

extern std::map<unsigned, preg> pregs;

chdl::node PipelineReg(unsigned n, chdl::node d);
void PipelineStall(unsigned n, chdl::node stall);
void PipelineBubble(unsigned n, chdl::node stall);
chdl::node GetStall(unsigned n);
chdl::node GetValid(unsigned n);
void PipelineFlush(unsigned n, chdl::node flush);
void genPipelineRegs();

template <unsigned N> chdl::bvec<N> PipelineReg(unsigned n, chdl::bvec<N> d) {
  chdl::bvec<N> q;
  for (unsigned i = 0; i < N; ++i) q[i] = PipelineReg(n, d[i]);
  return q;
}

#endif
