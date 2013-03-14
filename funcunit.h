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
#include <chdl/hierarchy.h>

static const unsigned IDLEN(6);

template <unsigned N, unsigned R, unsigned L> struct fuInput {
  chdl::bvec<N> imm, pc;
  chdl::vec<L, chdl::bvec<N>> r0, r1, r2;
  chdl::bvec<L> p0, p1;
  chdl::node hasimm, stall, pdest;

  chdl::bvec<6> op;
  chdl::bvec<IDLEN> iid;
  chdl::bvec<CLOG2(R)> didx;

  chdl::bvec<L> wb;
};

template <unsigned N, unsigned R, unsigned L> struct fuOutput {
  chdl::vec<L, chdl::bvec<N>> out;
  chdl::bvec<L> wb;
  chdl::bvec<IDLEN> iid;
  chdl::node valid, pdest;
  chdl::bvec<CLOG2(R)> didx;
};

template <unsigned N, unsigned R, unsigned L> class FuncUnit {
 public:
  FuncUnit() {}

  virtual std::vector<unsigned> get_opcodes() = 0;

  virtual fuOutput<N, R, L> generate(fuInput<N, R, L> in, chdl::node valid) = 0;
  virtual chdl::node ready() { return chdl::Lit(1); } // Output is ready.
};

// Functional unit with 1-cycle latency supporting all common arithmetic/logic
// instructions.
template <unsigned N, unsigned R, unsigned L>
  class BasicAlu : public FuncUnit<N, R, L>
{
 public:
  std::vector<unsigned> get_opcodes() {
    std::vector<unsigned> ops;

    for (unsigned i = 0x05; i <= 0x0b; ++i) ops.push_back(i); // 0x05 - 0x0b
    for (unsigned i = 0x0f; i <= 0x15; ++i) ops.push_back(i); // 0x0f - 0x15
    for (unsigned i = 0x19; i <= 0x1c; ++i) ops.push_back(i); // 0x19 - 0x1c
    ops.push_back(0x25);

    return ops;
  }

  virtual fuOutput<N, R, L> generate(fuInput<N, R, L> in, chdl::node valid) {
    using namespace std;
    using namespace chdl;

    hierarchy_enter("BasicAlu");

    tap("valid_alu", valid);
    tap("stall_alu", in.stall);

    fuOutput<N, R, L> o;
    node w(!in.stall);

    for (unsigned i = 0; i < L; ++i) {
      bvec<N> a(in.r0[i]), b(Mux(in.hasimm, in.r1[i], in.imm));

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
      mux_in[0x1b] = in.pc;
      mux_in[0x1c] = in.pc;
      mux_in[0x25] = b;

      o.out[i] = Wreg(w, Mux(in.op, mux_in));
    }

    o.valid = Wreg(w, valid);
    o.iid = Wreg(w, in.iid);
    o.didx = Wreg(w, in.didx);
    o.pdest = Wreg(w, in.pdest);
    o.wb = Wreg(w, in.wb);
  
    isReady = w;

    hierarchy_exit();

    return o;
  }

  virtual chdl::node ready() { return isReady; }
 private:

  chdl::node isReady;
};

// Predicate logic unit. All of the predicate/predicate and register/predicate
// instructions.
template <unsigned N, unsigned R, unsigned L>
 class PredLu : public FuncUnit<N, R, L>
{
 public:
  std::vector<unsigned> get_opcodes() {
    std::vector<unsigned> ops;

    for (unsigned i = 0x26; i <= 0x2c; ++i) ops.push_back(i); // 0x26 - 0x2c

    return ops;
  }

  virtual fuOutput<N, R, L> generate(fuInput<N, R, L> in, chdl::node valid) {
    using namespace std;
    using namespace chdl;

    hierarchy_enter("PredLu");

    fuOutput<N, R, L> o;

    node w(!in.stall);

    tap("valid_plu", valid);
    tap("stall_plu", in.stall);

    for (unsigned i = 0; i < L; ++i) {
      bvec<N> r0(in.r0[i]);
      node p0(in.p0[i]), p1(in.p1[i]);

      bvec<64> mux_in;
      mux_in[0x26] = OrN(r0);
      mux_in[0x27] = p0 && p1;
      mux_in[0x28] = p0 || p1;
      mux_in[0x29] = p0 != p1;
      mux_in[0x2a] = !p0;
      mux_in[0x2b] = r0[N-1];
      mux_in[0x2c] = !OrN(r0);

      o.out[i] = Zext<N>(bvec<1>(Wreg(w, Mux(in.op, mux_in))));
    }

    o.valid = Wreg(w, valid);
    o.iid = Wreg(w, in.iid);
    o.didx = Wreg(w, in.didx);
    o.pdest = Wreg(w, in.pdest);
    o.wb = Wreg(w, in.wb);
    isReady = w;

    hierarchy_exit();

    return o;
  }

  chdl::node ready() { return isReady; }
 private:
  chdl::node isReady;
};

// Integrated SRAM load/store unit with no MMU, per-lane RAM
template <unsigned N, unsigned R, unsigned L, unsigned SIZE>
  class SramLsu : public FuncUnit<N, R, L>
{
 public:
  std::vector<unsigned> get_opcodes() {
    std::vector<unsigned> ops;

    ops.push_back(0x23);
    ops.push_back(0x24);

    return ops;
  }

  virtual fuOutput<N, R, L> generate(fuInput<N, R, L> in, chdl::node valid) {
    const unsigned L2WORDS(CLOG2(SIZE/(N/8)));

    using namespace std;
    using namespace chdl;

    hierarchy_enter("SramLsu");

    fuOutput<N, R, L> o;

    node w(!in.stall);

    bvec<6> op(in.op);
    bvec<N> imm(in.imm);

    for (unsigned i = 0; i < L; ++i) {
      bvec<N> r0(in.r0[i]), r1(in.r1[i]), imm(in.imm),
              addr(imm + Mux(op[0], r1, r0));
      bvec<L2WORDS> memaddr(Zext<L2WORDS>(addr[range<CLOG2(N/8), N-1>()]));
      bvec<CLOG2(N)> memshift(Lit<CLOG2(N)>(8) *
                                Zext<CLOG2(N)>(addr[range<0, CLOG2(N/8)-1>()]));

      bvec<N> sramout = Syncmem(memaddr, r0, valid && !op[0]);

      o.out[i] = sramout >> memshift;
    }

    o.valid = Wreg(w, valid && op[0]);
    o.iid = Wreg(w, in.iid);
    o.didx = Wreg(w, in.didx);
    o.pdest = Wreg(w, in.pdest);
    o.wb = Wreg(w, in.wb);

    hierarchy_exit();

    return o;
  }
 private:
};

#endif
