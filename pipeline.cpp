#include "pipeline.h"

using namespace std;
using namespace chdl;

std::map<unsigned, preg> pregs;

    // // // // // // // // // // // // // // // // // // // // // // // // //
// // // TODO - These should probably find their way into CHDL headers  // // //
  // // // // // // // // // // // // // // // // // // // // // // // // //

template <typename FUNC>
  std::vector<chdl::node> ReduceInternal(std::vector<chdl::node> &v, FUNC f)
{
  if (v.size() == 1) return v;

  vector<node> v2;
  for (unsigned i = 0; i < v.size(); i += 2) {
    if (i+1 == v.size()) v2.push_back(v[i]);
    else                 v2.push_back(f(v[i], v[i+1]));
  }

  return ReduceInternal<FUNC>(v2, f);
}

// Overload this function for variable-sized vector<node>s instead of bvecs.
chdl::node OrN(std::vector<chdl::node> &v) {
  if (v.size() == 0) return chdl::Lit(0);
  return ReduceInternal<node (*)(node, node)>(v, Or)[0];
}

chdl::node AndN(std::vector<chdl::node> &v) {
  if (v.size() == 0) return chdl::Lit(1);
  return ReduceInternal<node (*)(node, node)>(v, And)[0];
}

chdl::node operator==(std::vector<chdl::node> a, std::vector<chdl::node> b) {
  if (a.size() != b.size()) return chdl::Lit(0);

  std::vector<chdl::node> eq;
  for (unsigned i = 0; i < a.size(); ++i) eq.push_back(a[i] == b[i]);

  return AndN(eq);
}

  // // // // // // // // // // // // // // // // // // // // // // // // //
// // // // // // // // // // // // // // // // // // // // // // // // // // //
  // // // // // // // // // // // // // // // // // // // // // // // // //

chdl::node PipelineReg(unsigned n, chdl::node d) {
  chdl::node q;
  pregs[n].bits.push_back(pregbit(d, q));
  return q;
}

void PipelineStall(unsigned n, chdl::node stall) {
  for (unsigned i = 0; i <= n; ++i) pregs[i].stall.push_back(stall);
}

void PipelineBubble(unsigned n, chdl::node stall) {
  for (unsigned i = 0; i < n; ++i) pregs[i].stall.push_back(stall);
  pregs[n].bubble.push_back(stall);
}

chdl::node GetStall(unsigned n) { return pregs[n].anystall; }
chdl::node GetValid(unsigned n) { return !pregs[n].flushorbubble; }

void PipelineFlush(unsigned n, chdl::node flush) {
  for (unsigned i = 0; i <= n; ++i) pregs[i].flush.push_back(flush);
}

void genPipelineRegs() {
  // Create the pipeline registers                                              
  for (auto it = pregs.begin(); it != pregs.end(); ++it) {
    preg &pr(it->second);
    pr.anystall = OrN(pr.stall);
    pr.flushorbubble = OrN(pr.flush) || OrN(pr.bubble) && !(pr.anystall);
    for (unsigned i = 0; i < pr.bits.size(); ++i) {
      pr.bits[i].q = Wreg(!pr.anystall,
                          Mux(pr.flushorbubble, pr.bits[i].d, Lit(0)));
    }
      { ostringstream oss; oss << "stall" << it->first;
        tap(oss.str(), pr.anystall); }
      { ostringstream oss; oss << "flushorbubble" << it->first;
        tap(oss.str(), pr.flushorbubble); }

  }
}
