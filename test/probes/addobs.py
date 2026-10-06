import os
FN = int(os.environ.get("FN", "0x180010C20"), 16)
N = [0]
def ent():
    N[0] += 1
    if N[0] > 12000: return None
    fr = u64(arg(1)); kp = arg(2)
    lb = u64(fr + 680)
    pt = u64(lb + 16 * kp)
    tid = i32(u64(fr + 0x2C0) + 4 * kp)
    return "AO %d tid=%d lvl=%d Tfw=%s pos=%s" % (N[0], tid, i32(u64(fr + 0x270) + 4 * kp), fmt(f64(fr + 64, 7)), fmt(f32(pt + 4, 3)))
def ex(s):
    rb = reg("rax")
    cf = u64(rb) if rb else 0
    out(s + (" meas=%s" % fmt(f64(cf + 0x40, 2)) if cf else " null"))
probe_call(FN, ent, ex)
