import os
FN = int(os.environ.get("FN", "0x180110490"), 16)
N = [0]
def ent():
    N[0] += 1
    if N[0] > 40: return False
    this = arg(0)
    ih = u64(this + 520)
    nf = u64(this + 296)
    out("II %d id=%d static=%d wbias=%s abias=%s stats=%s saved=%d,%s" % (N[0], i32(nf + 0xFC), mem(nf + 0xE4, 1)[0], fmt(f64(ih + 232, 3)), fmt(f64(ih + 208, 3)),
        fmt([u64(this + 2824 + 24 * k) for k in range(3)] + [f64(this + 2824 + 24 * k + 8) for k in range(3)] + [f64(this + 2824 + 24 * k + 16) for k in range(3)]), mem(this + 1648, 1)[0], fmt(f64(this + 1656, 3))))
    return False
probe(FN, ent)
