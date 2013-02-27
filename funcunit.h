#ifndef __FUNCUNIT_H
#define __FUNCUNIT_H

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

static const unsigned IDLEN(6);

template <unsigned N, unsigned R> struct fuInput {
  chdl::bvec<N> r0, r1, r2, imm;
  chdl::node p0, p1, hasimm, stall, pdest;
  chdl::bvec<6> op;
  chdl::bvec<IDLEN> iid;
  chdl::bvec<CLOG2(R)> didx;
};

template <unsigned N, unsigned R> struct fuOutput {
  chdl::bvec<N> out;
  chdl::bvec<IDLEN> iid;
  chdl::node valid, pdest;
  chdl::bvec<CLOG2(R)> didx;
};

template <unsigned N, unsigned R> class FuncUnit {
 public:
  FuncUnit() {}

  virtual std::vector<unsigned> get_opcodes() = 0;

  virtual fuOutput<N, R> generate(fuInput<N, R> in, chdl::node valid) = 0;
  virtual chdl::node ready() { return chdl::Lit(1); } // Output is ready.
};

// Functional unit with 1-cycle latency supporting all common arithmetic/logic
// instructions.
template <unsigned N, unsigned R> class BasicAlu : public FuncUnit<N, R> {
 public:
  std::vector<unsigned> get_opcodes() {
    std::vector<unsigned> ops;

    for (unsigned i = 0x05; i <= 0x0b; ++i) ops.push_back(i); // 0x05 - 0x0b
    for (unsigned i = 0x0f; i <= 0x15; ++i) ops.push_back(i); // 0x0f - 0x15
    for (unsigned i = 0x19; i <= 0x1a; ++i) ops.push_back(i); // 0x19 - 0x1a
    ops.push_back(0x25);

    return ops;
  }

  virtual fuOutput<N, R> generate(fuInput<N, R> in, chdl::node valid) {
    using namespace std;
    using namespace chdl;

    fuOutput<N, R> o;

    bvec<N> a(in.r0), b(Mux(in.hasimm, in.r1, in.imm));

    bvec<N> sum(Adder(a, Mux(in.op[0], b, ~b), in.op[0]));

    vec<64, bvec<N>> mux_in;
    mux_in[0x05] = -a;
    mux_in[0x06] = ~a;
    mux_in[0x07] = a & b;
    mux_in[0x08] = a | b;
    mux_in[0x09] = a ^ b;
    mux_in[0x0a] = sum;
    mux_in[0x0b] = sum;
    mux_in[0x0f] = a << Zext<CLOG2(N)>(b);
    mux_in[0x10] = a >> Zext<CLOG2(N)>(b);
    mux_in[0x11] = mux_in[0x07];
    mux_in[0x12] = mux_in[0x08];
    mux_in[0x13] = mux_in[0x09];
    mux_in[0x14] = sum;
    mux_in[0x15] = sum;
    mux_in[0x19] = mux_in[0x0f];
    mux_in[0x1a] = mux_in[0x10];
    mux_in[0x25] = b;

    node w(!in.stall);
    o.out = Wreg(w, Mux(in.op, mux_in));
    o.valid = Wreg(w, valid);
    o.iid = Wreg(w, in.iid);
    o.didx = Wreg(w, in.didx);
    o.pdest = Wreg(w, in.pdest);
  
    isReady = w;

    return o;
  }

  virtual chdl::node ready() { return isReady; }
 private:

  chdl::node isReady;
};

// Predicate logic unit. All of the predicate/predicate and register/predicate
// instructions.
template <unsigned N, unsigned R> class PredLu : public FuncUnit<N, R> {
 public:
  std::vector<unsigned> get_opcodes() {
    std::vector<unsigned> ops;

    for (unsigned i = 0x26; i <= 0x2c; ++i) ops.push_back(i); // 0x26 - 0x2c

    return ops;
  }

  virtual fuOutput<N, R> generate(fuInput<N, R> in, chdl::node valid) {
    using namespace std;
    using namespace chdl;

    fuOutput<N, R> o;

    bvec<N> r0(in.r0);
    node p0(in.p0), p1(in.p1);

    bvec<64> mux_in;
    mux_in[0x26] = OrN(r0);
    mux_in[0x27] = p0 && p1;
    mux_in[0x28] = p0 || p1;
    mux_in[0x29] = p0 != p1;
    mux_in[0x2a] = !p0;
    mux_in[0x2b] = r0[N-1];
    mux_in[0x2c] = !OrN(r0);

    node w(!in.stall);
    o.out = Zext<N>(bvec<1>(Wreg(w, Mux(in.op, mux_in))));
    o.valid = Wreg(w, valid);
    o.iid = Wreg(w, in.iid);
    o.didx = Wreg(w, in.didx);
    o.pdest = Wreg(w, in.pdest);
  
    isReady = w;

    return o;
  }

  chdl::node ready() { return isReady; }
 private:
  chdl::node isReady;
};

// Integrated SRAM load/store unit with no MMU
template <unsigned N, unsigned R, unsigned SIZE>
  class SramLsu : public FuncUnit<N, R>
{
 public:
  std::vector<unsigned> get_opcodes() {
    std::vector<unsigned> ops;

    ops.push_back(0x23);
    ops.push_back(0x24);

    return ops;
  }

  virtual fuOutput<N, R> generate(fuInput<N, R> in, chdl::node valid) {
    const unsigned L2WORDS(CLOG2(SIZE/(N/8)));

    using namespace std;
    using namespace chdl;

    fuOutput<N, R> o;

    bvec<6> op(in.op);
    bvec<N> r0(in.r0), r1(in.r1), imm(in.imm),
            addr(imm + Mux(op[0], in.r1, in.r0));
    bvec<L2WORDS> memaddr(Zext<L2WORDS>(addr[range<CLOG2(N/8), N-1>()]));
    bvec<CLOG2(N)> memshift(Lit<CLOG2(N)>(8) *
                              Zext<CLOG2(N)>(addr[range<0, CLOG2(N/8)-1>()]));

    bvec<N> sramout = Syncmem(memaddr, r0, valid && !op[0]);

    o.out = sramout >> memshift;
    o.valid = PipelineReg(3, valid && op[0]);
    o.iid = PipelineReg(3, in.iid);
    o.didx = PipelineReg(3, in.didx);
    o.pdest = PipelineReg(3, in.pdest);

    return o;
  }
 private:
};

#endif
