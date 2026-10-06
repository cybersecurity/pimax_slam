import os
FN = int(os.environ.get("FN", "0x180110490"), 16)
N = [0]
def deque(d):
    mp, ms, off, sz = u64(d + 8), u64(d + 16), u64(d + 24), u64(d + 32)
    return [u64(mp + 8 * ((off + i) & (ms - 1))) for i in range(sz)]
def ent():
    N[0] += 1
    if N[0] > 2: return False
    this = arg(0)
    nf = u64(this + 296)
    for nm, d in (("win", this + 2784), ("bundle", nf + 0x18)):
        els = deque(d)
        out("II %d %s n=%d" % (N[0], nm, len(els)))
        for e in els:
            out("  %.9f %s %s" % (f64(e), fmt(f32(e + 8, 3)), fmt(f32(e + 20, 3))))
    return False
probe(FN, ent)
