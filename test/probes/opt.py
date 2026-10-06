import os
FN = int(os.environ.get("FN", "0x180013680"), 16)
N = [0]
def bundle(fb, tag):
    out("%s id=%d Tinit=%s v=%s bg=%s ba=%s kf=%d" % (tag, i32(fb + 0xFC), fmt(f64(fb + 0x50, 7)), fmt(f64(fb + 0x90, 3)), fmt(f64(fb + 0xA8, 3)), fmt(f64(fb + 0xC0, 3)), mem(fb + 0xF8, 1)[0]))
    b, e = vec_range(fb)
    for k in range((e - b) // 16):
        fr = u64(b + 16 * k)
        out("  f%d T_f_w=%s" % (k, fmt(f64(fr + 64, 7))))
def ent():
    N[0] += 1
    if N[0] > 4: return None
    fb = u64(arg(1))
    bundle(fb, "OPT in")
    return fb
def ex(fb):
    bundle(fb, "OPT out")
probe_call(FN, ent, ex)
