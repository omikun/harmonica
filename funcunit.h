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
#include <chdl/input.h>

#define PROTOTYPE 1
#define blah

#ifdef DEBUG
#define DBGTAP(x) do {TAP(x); } while(0)
#else
#define DBGTAP(x) do {} while(0)
#endif

static const unsigned IDLEN(6);
static const unsigned Q(4);
static const unsigned DELAY(5);

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
 
    using namespace std;
    using namespace chdl;

	struct mshrld {
		bvec<CLOG2(R)> didx;
		bvec<IDLEN> iid;
		bvec<L2WORDS> memaddr;
		bvec<N> data;
		node free;
		node loaded;
		node pending;
	};

	struct mshrst {
		bvec<CLOG2(R)> didx;
		bvec<IDLEN> iid;
		bvec<L2WORDS> memaddr;
		bvec<N> data;
		node committed;
		node waiting;
		bvec<CLOG2(Q)> waitidx;
		node valid;
	};

	node resetReg;
	node reset = !resetReg;
	resetReg = Reg(Lit(1));

	vec<Q, mshrld> ldq;
	vec<Q, mshrst > stq;
    fuOutput<N, R> o;
    bvec<6> op(in.op);
    bvec<N> r0(in.r0), r1(in.r1), imm(in.imm),
            addr(imm + Mux(op[0], in.r1, in.r0));
    bvec<L2WORDS> memaddr(Zext<L2WORDS>(addr[range<CLOG2(N/8), N-1>()]));
    bvec<CLOG2(N)> memshift(Lit<CLOG2(N)>(8) *
                              Zext<CLOG2(N)>(addr[range<0, CLOG2(N/8)-1>()]));
	
	//insert to empty MSHR
	////////////////////////////////////////////////
	// load queue
	node memvalid;					//data from mem is valid 
	node readyToCommit; 			//have loaded ldq entry for commit
	node stqEnable, ldqEnable;  	//write enable for insertion
	bvec<CLOG2(Q)> freeldqidx, returnqidx; //qid of data from mem
	bvec<Q> ldqFree, ldqDone, ldqInsert, ldqPending, returnSelect, ldqLoaded;
	vec<Q, bvec<N> > ldqData;
	vec<Q, bvec<CLOG2(R)> > ldqdidx;
	vec<Q, bvec<IDLEN> > ldqiid;
    vec<Q, bvec<L2WORDS> > ldqMemaddr;
	bvec<L2WORDS> ldqMemaddrOut; 

	stqEnable = valid && !op[0] && isReady;
	ldqEnable = valid && op[0] && isReady;
	freeldqidx= Log2(ldqFree);
	node ldqAvailable = OrN(ldqFree);
	node ldqEmpty = AndN(ldqFree);
	ldqInsert = Decoder(freeldqidx, ldqEnable);
	returnSelect = Decoder(returnqidx, memvalid); 
	for(int i=0; i<Q; ++i)
	{
		ldqFree[i] = Wreg(reset || (ldqInsert[i]) || ldqDone[i] || returnSelect[i], 
						reset || ldqDone[i] || (returnSelect[i] && !readyToCommit) );
		ldqMemaddr[i] = Wreg<L2WORDS>(ldqInsert[i], memaddr);
		ldqiid[i] = Wreg(ldqInsert[i], in.iid);
		ldqdidx[i] = Wreg(ldqInsert[i], in.didx);

		ostringstream oss;
		oss << "ldq" << i << ".memaddr";
		tap(oss.str(), ldq[i].memaddr);
	}

	DBGTAP(reset); DBGTAP(resetReg);
	DBGTAP(ldqAvailable); DBGTAP(ldqEnable);
	DBGTAP(ldqInsert);
	DBGTAP(valid); DBGTAP(op);
	DBGTAP(ldqDone);
	DBGTAP(ldqFree);
	DBGTAP(memaddr);


	// store queue 
	//////////////////////////////////////////////////////////
	node dependentLd;
	node sendldreq, sendstreq;
	node validStqReq; 		//output to memory arbitration select signal
	bvec<Q> stqInsert, stqWaiting, stqValid, clearWait, stqTailSelect;
	vec<Q, bvec<N> > stqData;
	vec<Q, bvec<IDLEN> > stqiid;
	vec<Q, bvec<CLOG2(R)> > stqdidx;
	vec<Q, bvec<CLOG2(Q)> > stqWaitidx;
	vec< Q, bvec<L2WORDS> > stqMemaddr;
	bvec<CLOG2(Q)+1> stqSize;
	bvec<CLOG2(Q)> head, tail;

	//head/tail pointers into stq
	validStqReq = sendstreq && Mux(tail, stqValid);
	tail = Wreg(validStqReq, tail+Lit<CLOG2(Q)>(1) ); 
	head = Wreg(stqEnable, head+Lit<CLOG2(Q)>(1) ); 
	stqSize = Wreg( Xor(stqEnable, validStqReq), 
			Mux(validStqReq, stqSize+Lit<CLOG2(Q)+1>(1), 
			stqSize+Lit<CLOG2(Q)+1>(0xF)) ); // take stall into account?

	DBGTAP(stqSize); DBGTAP(tail); DBGTAP(head);
	DBGTAP(stqValid); DBGTAP(stqEnable); 
	DBGTAP(validStqReq);
	node stqValidReady = (Mux(tail, stqValid) );
	DBGTAP(stqValidReady);

	//Load Store Forwarding (LSF) and WAR hazard detection
	node enableLSF;
	bvec<Q> WARaddrMatch, LSFaddrMatch, ldqMatch, stqMatch;
	bvec<N> stqLSFData = Mux(Enc(LSFaddrMatch), stqData);
	
	for(int i=0; i<Q; ++i)
	{
		WARaddrMatch[i] = memaddr == ldq[i].memaddr;	
		ldqMatch[i] = ldq[i].pending && WARaddrMatch[i];
		
		LSFaddrMatch[i] = (memaddr == stq[i].memaddr);
		stqMatch[i] = LSFaddrMatch[i] && stq[i].valid;	
	}
	//WAR detection
	bvec<CLOG2(Q)> dependentLdidx = Enc(ldqMatch);
	dependentLd = OrN(WARaddrMatch) && ldqMemaddrOut != memaddr;
	stqInsert = Decoder(head, stqEnable);
	//LSF detection
	enableLSF = OrN(stqMatch);
	stqLSFData = Mux(Enc(LSFaddrMatch), stqData);

	for(int i=0; i<Q; ++i)
	{
		stqMemaddr[i] = Wreg<L2WORDS>(stqInsert[i], memaddr);
		stqData[i] = Wreg<N>(stqInsert[i], r0);
		stqiid[i] = Wreg(stqInsert[i], in.iid);
		clearWait[i] = Mux(stqWaitidx[i], ldqPending);
		stqWaiting[i] = Wreg(stqInsert[i] || !clearWait[i], dependentLd && stqInsert[i]); 
		stqWaitidx[i] = Wreg<CLOG2(Q)>(stqInsert[i], dependentLdidx);
		stqValid[i] = Wreg(stqInsert[i] || (stqTailSelect[i]), stqInsert[i]);//what clears it?

		TAP(stqData[i]);
	}

	DBGTAP(LSFaddrMatch); DBGTAP(stqMatch);
	DBGTAP(WARaddrMatch); DBGTAP(ldqMatch);
	DBGTAP(stqLSFData); 
	DBGTAP(stqTailSelect); DBGTAP(stqInsert);

	//send req to memory
	////////////////////////////////////////////////
	node memStall = Lit(0);
	node ldqPendingFlag = OrN(ldqPending);
	bvec<CLOG2(Q)> pendingldqidx = Mux(ldqPendingFlag, freeldqidx, Log2(ldqPending) );
	bvec<Q> ldqReq = Decoder(pendingldqidx, ldqPendingFlag);

	for(int i=0; i<Q; ++i)
	{
		//ldqPending[i] = 1 when: on insert 
		//ldqPending[i] = 0 when: sent to memory, bypass, or memStall
		// bypass occurs when ldqEmpty or no ldqPending
		ldqPending[i] = Wreg( (ldqInsert[i] ) || (ldqReq[i] && sendldreq && !memStall), (ldqInsert[i] && !enableLSF && !ldqPendingFlag && memStall) );
	}
	
	//loading data (from memory or st forwarding)
	////////////////////////////////////////////////
	node WARexists = Mux(tail, stqWaiting);
	sendldreq = (ldqEnable || !( ( (stqSize == Lit<CLOG2(Q)+1>(Q)) && !WARexists) || !ldqPendingFlag) ) ;//TODO should this be included? && !memStall;
	sendstreq = !sendldreq && !memStall;//should memstall be taken in to account?

	stqTailSelect = Decoder(tail, sendstreq);

	ldqMemaddrOut = Mux(!ldqPendingFlag && !enableLSF, Mux(pendingldqidx, ldqMemaddr), memaddr);
	bvec<L2WORDS> stqMemaddrOut = Mux(tail, stqMemaddr);
	bvec<L2WORDS> memaddrOut = Mux(sendldreq, stqMemaddrOut, ldqMemaddrOut);
	node memValidOut = ldqEnable || ldqPendingFlag || Mux(tail, stqValid); 
	node memWrite = Mux(tail, stqValid) && sendstreq;

	DBGTAP(ldqPending);
	DBGTAP(memaddrOut);
	DBGTAP(sendldreq); DBGTAP(sendstreq);
	DBGTAP(stqWaiting);
	
	//TESTING PURPOSE ONLY; MUST CHANGE WHEN INTERFACING WITH MEMORY
	//bvec<N> sramout = Syncmem(memaddr, r0, valid && !op[0]);
//*/
//TODO: must add latch to memory output
	bvec<N> sramout, memDataIn;
	bvec<Q> ldqReturn;	
#if PROTOTYPE
	sramout = Syncmem(memaddrOut, Mux(tail, stqData), memValidOut);
	memDataIn = delay(sramout, DELAY-1);
	returnqidx = delay(pendingldqidx, DELAY);
	memvalid = delay(memValidOut, DELAY);
	//memStall = sramout[8];

	ldqReturn = Decoder(returnqidx, memvalid);
#else
	sramout = Input<N>("memDataIn"); 
	returnqidx = Input<CLOG2(Q)>("memqidIn"); 
	memvalid = Input("memValidIn");
	TAP(memaddrOut); TAP(memWrite); TAP(memValidOut); TAP(pendingldqidx); 
#endif

	readyToCommit = OrN(ldqDone);
	
	for(int i=0; i<Q; ++i)
	{	
		ldqLoaded = Wreg( ldqInsert[i] || ldqDone[i] || (ldqReturn[i] && memvalid), (ldqReturn[i] && memvalid && readyToCommit) || (ldqInsert[i] && enableLSF) );
		ldqData = Wreg<N>( (ldqReturn[i] && memvalid) || ldqInsert[i], Mux(ldqInsert[i], memDataIn, stqLSFData) );	//loads from store at insertion regardless of LSF is enabled, is this a problem?
	}

	DBGTAP(memDataIn); DBGTAP(sramout);
	DBGTAP(pendingldqidx); DBGTAP(returnqidx);
	DBGTAP(memvalid); 
	DBGTAP(memValidOut); 
	DBGTAP(ldqPendingFlag);
	DBGTAP(ldqReturn);
	//commit 
	///////////////////////////////////////////////////
	for(int i=0; i<Q; ++i)
	{
		ldqLoaded[i] = ldq[i].loaded;
		ldqData[i] = ldq[i].data;
	}

	node regStall = Reg(in.stall);	
	bvec<CLOG2(Q)> loadedldqidx = Log2(ldqLoaded);
	node ldqLoadedFlag = OrN(ldqLoaded);
	bvec<Q> ldqCommit = Decoder(loadedldqidx, ldqLoadedFlag);

	for(int i=0; i<Q; ++i)
	{
		ldqDone[i] = ldqCommit[i] && !regStall;
	}

	isReady = ldqAvailable && stqSize != Lit<CLOG2(Q)+1>(Q); // && stqAvailable or stqSize > 0
	DBGTAP(isReady);

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

	//DBGTAP(ldqData);
	DBGTAP(o.out);
	DBGTAP(o.iid);
	DBGTAP(o.didx);
	DBGTAP(ldqLoadedFlag);
	DBGTAP(ldqLoaded);
	DBGTAP(regStall);
	DBGTAP(ldqCommit);
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

