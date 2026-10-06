import os
FN = int(os.environ.get("FN", "0x18002A6D0"), 16)
N = [0]
def ent():
    N[0] += 1
    if N[0] > int(os.environ.get("LIMIT", "3")): return None
    this = arg(0)
    out("EOPT in its=%d verbose=%d stat=%d lm=%d" % (arg(1) & 0xffffffff, arg(2) & 0xff, arg(3) & 0xff, u64(this + 0x110)))
    return this
def ex(this):
    m = u64(this + 0x190)
    s = m + 0x1F0
    out("EOPT out lm=%d init=%.17g final=%.17g fixed=%.17g succ=%d unsucc=%d term=%d" % (u64(this + 0x110), f64(s + 40), f64(s + 48), f64(s + 56), i32(s + 88), i32(s + 92), i32(s + 4)))
probe_call(FN, ent, ex)
