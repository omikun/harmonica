// Types describing read and write ports
template <unsigned M, unsigned N> struct rdport {
  rdport() {}
  rdport(bvec<M> a, bvec<N> q): a(a), q(q) {}
  bvec<M> a;
  bvec<N> q;
};

template <unsigned M, unsigned N> struct wrport {
  wrport() {}
  wrport(bvec<M> a, bvec<N> d, node we): a(a), d(d), we(we) {}
  bvec<M> a;
  bvec<N> d;
  node we;
};

// 2^M-entry N-bit R-read-port, 1-write-port register file
template <unsigned M, unsigned N, unsigned R>
  void Regfile(vec<R, rdport<M, N>> r, wrport<M, N> w, string prefix = "")
{
  const unsigned long SIZE(1<<M);

  bvec<SIZE> wrsig(Decoder(w.a, w.we));

  vec<SIZE, bvec<N>> regs;
  for (unsigned i = 0; i < SIZE; ++i) {
    regs[i] = Wreg(wrsig[i], w.d);
    #ifdef DEBUG
    ostringstream oss;
    oss << prefix << "reg" << i;
    tap<N, bvec>(oss.str(), regs[i]);
    #endif
  }

  for (unsigned i = 0; i < R; ++i)
    r[i].q = Mux(r[i].a, regs);
}

// Valid bit file (2^M entry, 1 set port, 1 clear port, R-read-port)
template <unsigned M, unsigned R>
  void Bitfile(vec<R, rdport<M, 1>> r, 
               bvec<M> set_idx, node set, bvec<M> clear_idx, node clear,
               string prefix = "")

{
  const unsigned long SIZE(1<<M);
  bvec<SIZE> setsig(Decoder(set_idx, set)), clearsig(Decoder(clear_idx, clear));
  tap<SIZE, bvec>("setsig", setsig);
  tap<SIZE, bvec>("clearsig", clearsig);

  vec<SIZE, bvec<1>> bits;
  for (unsigned i = 0; i < SIZE; ++i) {
    bits[i] = Wreg(setsig[i]||clearsig[i], bvec<1>(setsig[i] && !clearsig[i]));
    #ifdef DEBUG
    ostringstream oss;
    oss << prefix << "vreg" << i;
    tap<1, bvec>(oss.str(), bits[i]);
    #endif
  }

  for (unsigned i = 0; i < R; ++i)
    r[i].q = Mux(r[i].a, bits);
}
