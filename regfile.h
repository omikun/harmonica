// Types describing read and write ports
template <unsigned M, unsigned N, unsigned L> struct rdport {
  rdport() {}
  rdport(bvec<M> a, vec<L, bvec<N>> q): a(a), q(q) {}
  bvec<M> a;
  vec<L, bvec<N>> q;
};

template <unsigned M, unsigned N, unsigned L> struct wrport {
  wrport() {}
  wrport(bvec<M> a, vec<L, bvec<N>> d, bvec<L> we): a(a), d(d), we(we) {}
  bvec<M> a;
  vec<L, bvec<N>> d;
  bvec<L> we;
};

// 2^M-entry N-bit R-read-port, 1-write-port register file
template <unsigned M, unsigned N, unsigned R, unsigned L>
  void Regfile(vec<R, rdport<M, N, L>> r, wrport<M, N, L> w, string prefix = "")
{
  HIERARCHY_ENTER();

  const unsigned long SIZE(1<<M);

  bvec<SIZE> wrsig(Decoder(w.a, OrN(w.we)));

  vec<L, vec<SIZE, bvec<N>>> regs;
  for (unsigned j = 0; j < L; ++j) {
    for (unsigned i = 0; i < SIZE; ++i) {
      unsigned long initialval(i == 0 ? j : 0);
      regs[j][i] = Wreg(wrsig[i] && w.we[j], w.d[j], initialval);
      #ifdef DEBUG
      ostringstream oss;
      oss << prefix << "reg" << j << i;
      tap(oss.str(), regs[j][i]);
      #endif
    }
  }

  for (unsigned j = 0; j < L; ++j)
    for (unsigned i = 0; i < R; ++i)
      r[i].q[j] = Mux(r[i].a, regs[j]);

  HIERARCHY_EXIT();
}

// Valid bit file (2^M entry, 1 set port, 1 clear port, R-read-port)
template <unsigned M, unsigned R>
  void Bitfile(vec<R, rdport<M, 1, 1>> r, 
               bvec<M> set_idx, node set, bvec<M> clear_idx, node clear,
               string prefix = "")

{
  HIERARCHY_ENTER();

  const unsigned long SIZE(1<<M);
  bvec<SIZE> setsig(Decoder(set_idx, set)), clearsig(Decoder(clear_idx, clear));
  tap("setsig", setsig);
  tap("clearsig", clearsig);

  vec<SIZE, bvec<1>> bits;
  for (unsigned i = 0; i < SIZE; ++i) {
    bits[i] =
      Wreg(setsig[i]||clearsig[i], bvec<1>(setsig[i] && !clearsig[i]), 1);
    #ifdef DEBUG
    ostringstream oss;
    oss << prefix << "vreg" << i;
    tap(oss.str(), bits[i]);
    #endif
  }

  for (unsigned i = 0; i < R; ++i)
    r[i].q[0] = Mux(r[i].a, bits);

  HIERARCHY_EXIT();
}
