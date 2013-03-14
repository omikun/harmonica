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
static const unsigned Q(8);
static const unsigned DELAY(12);

template <unsigned N, unsigned R> struct fuInput {
  chdl::bvec<N> r0, r1, r2, imm, pc;
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
    for (unsigned i = 0x19; i <= 0x1c; ++i) ops.push_back(i); // 0x19 - 0x1c
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
    mux_in[0x1b] = in.pc;
    mux_in[0x1c] = in.pc;
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
/*
   Load instruction:
   Cycle 1: insert request into loadqueue
   				Set pending flag, clear free flag
   				check for Load Store Forwarding (LSF)
					if LSF enabled, clear pending flag,
					set loaded flag, skip to cycle 4
				Bypass: push request to memory if ldq empty AND no LSF
					clear pending flag
   Cycle 2: Compete for memory if pending flag is set
   				Clear pending flag once sent to memory
   Cycle 3: Data returned from memory
   				Set loaded flag
				TODO: commit data if no other loaded data
					Clear loaded flag,
					Set free flag
   Cycle 4: Once loaded, compete for write back commit
   				If selected for commit, 
					Clear loaded flag
					Set free flag
   Store instruction:
   Cycle 1: insert request into store queue
				check for WAR hazard, set waiting flag
				and waitidx (queue id of dependent load)
   Cycle 2: if no WAR and at head of queue, compete for memory
   				if WAR, wait until pending flag of dependent
				load is cleared (1 cycle bubble for pending flag
				to propagate)

*/
template <unsigned N, unsigned R, unsigned SIZE>
  class SramLsu : public FuncUnit<N, R>
{
 private:
  chdl::node isReady;
 public:
  virtual chdl::node ready() { return isReady; }
  std::vector<unsigned> get_opcodes() {
    std::vector<unsigned> ops;

    ops.push_back(0x23);
    ops.push_back(0x24);

    return ops;
  }

  template <unsigned M> 
	  chdl::bvec <M> delay (chdl::bvec <M> in, unsigned cycles) {
	  if (cycles == 0) return in;
	  else return chdl::Reg<M>(delay(in, cycles-1));
  }
  chdl::node delay(chdl::node in, unsigned cycles) {
	  if (cycles == 0) return in;
	  else return chdl::Reg(delay(in, cycles-1));
  }

  virtual fuOutput<N, R> generate(fuInput<N, R> in, chdl::node valid) {
    const unsigned L2WORDS(CLOG2(SIZE/(N/8)));
 
  struct mshrld {
	  chdl::bvec<CLOG2(R)> didx;
	  chdl::bvec<IDLEN> iid;
	  chdl::bvec<L2WORDS> memaddr;
	  chdl::bvec<N> data;
	  chdl::node free;
	  chdl::node loaded;
	  chdl::node pending;
  };
 
  struct mshrst {
	  chdl::bvec<CLOG2(R)> didx;
  	  chdl::bvec<IDLEN> iid;
	  chdl::bvec<L2WORDS> memaddr;
	  chdl::bvec<N> data;
	  chdl::node committed;
	  chdl::node waiting;
		chdl::bvec<CLOG2(Q)> waitidx;
	  chdl::node valid;
  };

    using namespace std;
    using namespace chdl;

    fuOutput<N, R> o;

    bvec<6> op(in.op);
    bvec<N> r0(in.r0), r1(in.r1), imm(in.imm),
            addr(imm + Mux(op[0], in.r1, in.r0));
    bvec<L2WORDS> memaddr(Zext<L2WORDS>(addr[range<CLOG2(N/8), N-1>()]));
    bvec<CLOG2(N)> memshift(Lit<CLOG2(N)>(8) *
                              Zext<CLOG2(N)>(addr[range<0, CLOG2(N/8)-1>()]));

	vec<Q, mshrld> ldq;
	vec<Q, mshrst > stq;
	
	vec<Q, bvec<N> > stqData;
	bvec<Q> ldqFree;
	for(int i=0; i<Q; i++)
	{
		stqData[i] = stq[i].data;
		ldqFree[i] = ldq[i].free;
	}
	chdl::node ldqFreeFlag = OrN(ldqFree);

	//write to empty MSHR
	////////////////////////////////////////////////
	// load queue
	chdl::node resetReg;
	chdl::node reset = !resetReg;
	bvec<Q> ldqDone;
	resetReg = Reg(Lit(1));
	bvec<CLOG2(Q)> freeldqidx= Log2(ldqFree);
	chdl::node ldqAvailable = OrN(ldqFree);
	chdl::node ldqEmpty = AndN(ldqFree);
	chdl::node stqEnable = valid && !op[0] && isReady;
	chdl::node ldqEnable = valid && op[0] && isReady;
	bvec<Q> ldqInsert = Decoder(freeldqidx, ldqEnable);
	bvec<Q> ldqPending;
	bvec<Q> returnSelect = Decoder(returnqid, memvalid); //have to move returnqid up here
	for(int i=0; i<Q; ++i)
	{
		ldq[i].free = Wreg(reset || (ldqInsert[i]) || ldqDone[i] || returnSelect[i], reset || ldqDone[i] || (returnSelect[i] && !readyToCommit) );	//ldqDone[i] should always be 0 on MSHR insert, how to set w/o ldqDone and readyToCommit?
		ldq[i].memaddr= Wreg<L2WORDS>(ldqInsert[i], memaddr);
		ldq[i].iid = Wreg(ldqInsert[i], in.iid);
		ldq[i].didx = Wreg(ldqInsert[i], in.didx);

		ostringstream oss;
		oss << "ldq" << i << ".memaddr";
		tap(oss.str(), ldq[i].memaddr);
	}

	TAP(reset); TAP(resetReg);
	TAP(ldqAvailable); TAP(ldqEnable);
	TAP(ldqInsert);
	TAP(valid); TAP(op);
	TAP(ldqDone);
	TAP(memaddr);


	// store queue 
	bvec<Q> stqWaiting, stqValid;
	chdl::node sendldreq=Lit(1), sendstreq(0);
	bvec<CLOG2(Q)+1> stqSize;
	bvec<CLOG2(Q)> head, tail;
	chdl::node stqReq; 			//only when memory commit arbitration decides
	chdl::node dependentLd;
	tail = Wreg(stqReq && Mux(tail, stqValid), tail+Lit<3>(1) );	//only on successful commit to memory
	head = Wreg(stqEnable, head+Lit<3>(1) ); //only on valid st requests 
	chdl::node validStReq = stqReq && Mux(tail, stqValid);
	stqSize = Wreg( Xor(stqEnable, validStReq), Mux(validStReq, stqSize+Lit<CLOG2(Q)+1>(1), stqSize+Lit<CLOG2(Q)+1>(0xF)) ); // take stall into account?

	TAP(stqSize); TAP(tail); TAP(head);
	TAP(stqValid); TAP(stqEnable); TAP(stqReq);
	TAP(validStReq);
	//LSF and WAR hazard detection
	//find matching addr in store queue
	chdl::bvec<Q> WARaddrMatch, LSFaddrMatch, ldqMatch, stqMatch;
	bvec<N> stqLSFData = Mux(Enc(LSFaddrMatch), stqData);
	TAP(stqLSFData); TAP(ldqFree);
	
	bvec<L2WORDS> ldqMemaddrOut; 
	for(int i=0; i<Q; ++i)
	{
		WARaddrMatch[i] = memaddr == ldq[i].memaddr;	
		ldqMatch[i] = ldq[i].pending && WARaddrMatch[i];
		
		LSFaddrMatch[i] = (memaddr == stq[i].memaddr);
		stqMatch[i] = LSFaddrMatch[i] && stq[i].valid;	
	}
	bvec<CLOG2(Q)> dependentLdidx = Enc(ldqMatch);
	dependentLd = OrN(WARaddrMatch) && ldqMemaddrOut != memaddr;
	bvec<Q> stqInsert= Decoder(head, stqEnable);

	chdl::node enableLSF = OrN(stqMatch);
	//bvec<CLOG2(Q)> lsfLoadidx = Enc(LSFaddrMatch);	//what happens if more than 1 matching store?
	bvec<Q> ldqLSF;
	bvec<Q> stqTailSelect = Decoder(tail, sendstreq);
	bvec<Q> clearWait;
	for(int i=0; i<Q; ++i)
	{
		ldqLSF[i] = stqMatch[i] && enableLSF;

		stq[i].iid = Wreg(stqInsert[i], in.iid);
		stq[i].memaddr = Wreg<L2WORDS>(stqInsert[i], memaddr);
		stq[i].data = Wreg<N>(stqInsert[i], r0);
		clearWait[i] = Mux(stq[i].waitidx, ldqPending);
		stq[i].waiting = Wreg(stqInsert[i] || !clearWait[i], dependentLd && stqInsert[i]); 
		stq[i].waitidx = Wreg(stqInsert[i], dependentLdidx);
		stq[i].valid = Wreg(stqInsert[i] || (stqTailSelect[i]), stqInsert[i]);//and what clears it?
	}

	TAP(LSFaddrMatch); TAP(stqMatch);
	TAP(WARaddrMatch); TAP(ldqMatch);
//*/

	//send req to memory
	////////////////////////////////////////////////
	
	chdl::node memStall = Lit(0);
	vec<Q, bvec<CLOG2(R)> > stqdidx, ldqdidx;
	vec<Q, bvec<IDLEN> > stqiid, ldqiid;
	vec< Q, bvec<L2WORDS> > stqMemaddr, ldqMemaddr;
	
	bvec<CLOG2(Q)> pendingldqidx = Log2(ldqPending);
	chdl::node ldqPendingFlag = OrN(ldqPending);
	bvec<Q> ldqReq = Decoder(pendingldqidx, ldqPendingFlag);

	for(int i=0; i<Q; ++i)
	{
		ldqPending[i] = ldq[i].pending;
		ldq[i].pending = Wreg( (ldqInsert[i] ) || (ldqReq[i] && sendldreq && !memStall), (ldqInsert[i] && !enableLSF && !ldqEmpty) );

		ldqMemaddr[i] = ldq[i].memaddr;
		ldqiid[i] = ldq[i].iid;
		ldqdidx[i] = ldq[i].didx;

		stqMemaddr[i] = stq[i].memaddr;
		stqData[i] = stq[i].data;
		stqiid[i] = stq[i].iid;
		stqdidx[i] = stq[i].didx;
		stqWaiting[i] = stq[i].waiting;
		stqValid[i] = stq[i].valid;
	}
	
	//loading data (from memory or st forwarding)
	////////////////////////////////////////////////
	
	//always send ld req unless store queue is full, but not if head store depends on a pending ld
	chdl::node WARexists = Mux(tail, stqWaiting);
	sendldreq = !( ( (stqSize == Lit<CLOG2(Q)+1>(Q)) && !WARexists) || ldqEmpty);
	sendstreq = !sendldreq;

	ldqMemaddrOut = Mux(ldqEmpty && !enableLSF, Mux(pendingldqidx, ldqMemaddr), memaddr);
	bvec<L2WORDS> stqMemaddrOut = Mux(tail, stqMemaddr);
	bvec<L2WORDS> memaddrOut = Mux(sendldreq, stqMemaddrOut, ldqMemaddrOut);
	chdl::node memValidOut = ldqPendingFlag || Mux(tail, stqValid);

	TAP(ldqPending);
	TAP(memaddrOut);
	TAP(sendldreq); TAP(sendstreq);
	TAP(stqWaiting);
	
	//TESTING PURPOSE ONLY; MUST CHANGE WHEN INTERFACING WITH MEMORY
	//bvec<N> sramout = Syncmem(memaddr, r0, valid && !op[0]);
//*/
	bvec<N> sramout = Syncmem(memaddrOut, Mux(tail, stqData), memValidOut, "data.hex");
	bvec<N> memDataIn = delay(sramout, DELAY);
	bvec<CLOG2(Q)> returnqidx = delay(pendingldqidx, DELAY);
	chdl::node memvalid = delay(ldqPendingFlag, DELAY);
	//memStall = sramout[8];

	bvec<Q> ldqReturn = Decoder(returnqidx, memvalid);
	chdl::node readyToCommit = OrN(ldqDone);
	
	for(int i=0; i<Q; ++i)
	{	
		ldq[i].loaded = Wreg( ldqInsert[i] || ldqDone[i] || (ldqReturn[i] && memvalid), (ldqReturn[i] && memvalid && readyToCommit) || (ldqInsert[i] && enableLSF) );
		ldq[i].data = Wreg<N>( (ldqReturn[i] && memvalid) || ldqInsert[i], Mux(ldqInsert[i], memDataIn, stqLSFData) );	//loads from store at insertion regardless of LSF is enabled, is this a problem?
	}

	TAP(ldqLSF);
	TAP(pendingldqidx); TAP(returnqidx);
	TAP(memvalid); TAP(ldqPendingFlag);
	TAP(ldqReturn);
	//commit 
	///////////////////////////////////////////////////
	bvec<Q> ldqLoaded;
	vec<Q, bvec<N> > ldqData;
	for(int i=0; i<Q; ++i)
	{
		ldqLoaded[i] = ldq[i].loaded;
		ldqData[i] = ldq[i].data;
	}

	chdl::node regStall = Reg(in.stall);	
	bvec<CLOG2(Q)> loadedldqidx = Log2(ldqLoaded);
	chdl::node ldqLoadedFlag = OrN(ldqLoaded);
	bvec<Q> ldqCommit = Decoder(loadedldqidx, ldqLoadedFlag);

	for(int i=0; i<Q; ++i)
	{
		ldqDone[i] = ldqCommit[i] && !regStall;
	}

	isReady = ldqAvailable && stqSize != Lit<CLOG2(Q)+1>(Q); // && stqAvailable or stqSize > 0
	TAP(isReady);

#if 1
    o.out = Mux(readyToCommit, memDataIn, Mux(loadedldqidx, ldqData));
	o.valid = readyToCommit || memvalid;
	o.iid = Mux(readyToCommit, Mux(returnqidx, ldqiid), Mux(loadedldqidx, ldqiid) );
	o.didx = Mux(readyToCommit, Mux(returnqidx, ldqdidx), Mux(loadedldqidx, ldqdidx) );
#else
    o.out = sramout >> memshift;
    o.valid = PipelineReg(3, valid && op[0]);
    o.iid = PipelineReg(3, in.iid);
    o.didx = PipelineReg(3, in.didx);
#endif 
	o.pdest = PipelineReg(3, in.pdest);

	//TAP(ldqData);
	TAP(o.out);
	TAP(o.iid);
	TAP(o.didx);
	TAP(ldqLoadedFlag);
	TAP(ldqLoaded);
	TAP(regStall);
	TAP(ldqCommit);
	return o;
  }
};

#if 0
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

#endif
