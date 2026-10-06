import os
EO = int(os.environ.get("EO", "0x18002A6D0"), 16)
RB = int(os.environ.get("RB", "0x1801FB7D0"), 16)
WHICH = int(os.environ.get("WHICH", "1"))
ON = [0]
def eo_ent():
    ON[0] += 1
    if ON[0] != WHICH: return None
    rbp.enabled = True
    out("EOPT start")
    return 1
def eo_ex(st):
    rbp.enabled = False
    out("EOPT end")
probe_call(EO, eo_ent, eo_ex)
def rb_ent():
    return (arg(0), arg(2), arg(3), arg(4))
def rb_ex(st):
    rb, cost, res, jac = st
    cf = u64(rb)
    vt = u64(cf) - base + 0x180000000
    nres = i32(cf + 32)
    np_ = (u64(cf + 16) - u64(cf + 8)) // 4
    pbs = u64(rb + 16)
    params = []
    for i in range(np_):
        pb = u64(pbs + 8 * i)
        sz = i32(pb + 8)
        st_ = u64(pb + 24)
        params.append(fmt(f64(st_, sz) if sz > 1 else [f64(st_)]))
    ok = reg("rax") & 0xff
    r = []
    if res:
        r = f64(res, nres) if nres > 1 else [f64(res)]
    extra = (" meas=%s sqi=%s" % (fmt(f64(cf + 0x40, 2)), fmt(f64(cf + 0xA0, 4)))) if nres == 2 else ""
    out("RB vt=%x n=%d ok=%d jac=%d cost=%.17g r=%s p=%s%s" % (vt, nres, ok, 1 if jac else 0, f64(cost), fmt(r), "|".join(params), extra))
rbp = probe_call(RB, rb_ent, rb_ex)
rbp.enabled = False
