import os
EO = int(os.environ.get("EO", "0x18002A6D0"), 16)
FN = int(os.environ.get("FN", "0x18000E0A0"), 16)
ON = [0, 0]
def eo_ent():
    ON[1] += 1
    if ON[1] > 1: return None
    ON[0] = 1
    return 1
def eo_ex(st):
    ON[0] = 0
probe_call(EO, eo_ent, eo_ex)
N = [0]
def ent():
    if not ON[0]: return None
    N[0] += 1
    if N[0] > 20: return None
    return (arg(0), arg(1), arg(2), arg(3))
def ex(st):
    this, p, r, j = st
    pp = [u64(p + 8 * i) for i in range(3)]
    out("RE p=%s|%s|%s r=%s meas=%s sqrtinfo=%s" % (fmt(f64(pp[0], 7)), fmt(f64(pp[1], 3)), fmt(f64(pp[2], 7)), fmt(f64(r, 2)), fmt(f64(this + 0x40 - 0x10, 2)), ""))
probe_call(FN, ent, ex)
