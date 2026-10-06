import os
def via():
    m = arg(1)
    ba = arg(5)
    out("VIA n=%d ba=%s" % (u64(m + 8), fmt(f64(ba, 3))))
    for v in msvc_map_nodes(m):
        f = v + 16
        out(" t=%.17g R=%s T=%s" % (f64(v), fmt(f64(f + 24, 9)), fmt(f64(f + 96, 3))))
        pi = u64(f + 120)
        if pi:
            b, e = vec_range(pi + 10504)
            J = f64(pi + 256, 225)
            out("  n=%d sum_dt=%.17g dp=%s dq=%s dv=%s ba=%s bg=%s acc0=%s gyr0=%s" % ((e - b) // 8, f64(pi + 10416), fmt(f64(pi + 10424, 3)),
                fmt(f64(pi + 10448, 4)), fmt(f64(pi + 10480, 3)), fmt(f64(pi + 208, 3)), fmt(f64(pi + 232, 3)), fmt(f64(pi+160,3)), fmt(f64(pi+184,3))))
            out("  J_R_BG=%s J_P_BA=%s" % (fmt([J[c * 15 + r] for r in range(3, 6) for c in range(12, 15)]), fmt([J[c * 15 + r] for r in range(0, 3) for c in range(9, 12)])))
            if b != e:
                dts = f64(b, (e - b) // 8)
                out("  dts=%s" % fmt(dts[:5] + dts[-3:]))
    return False
probe(int(os.environ.get("FN", "0x180157240"), 16), via)  # rebuilt: FN=$(python3 tools/sym.py VisualIMUAlignment | cut -d" " -f1)
