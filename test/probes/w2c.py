import os
AO = int(os.environ.get("AO", "0x180010C20"), 16)
W2C = int(os.environ.get("W2C", "0x180095450"), 16)
P3 = int(os.environ.get("P3", "0x180161BA0"), 16)
N = [0]; ON = [0]
LO, HI = int(os.environ.get("LO", "6221")), int(os.environ.get("HI", "6221"))
def ao_ent():
    N[0] += 1
    if LO <= N[0] <= HI:
        ON[0] = 1; w.enabled = True; p.enabled = True
        out("AO %d" % N[0])
        return 1
    return None
def ao_ex(st):
    ON[0] = 0; w.enabled = False; p.enabled = False
probe_call(AO, ao_ent, ao_ex)
def w_ent():
    return (arg(1), arg(2), f64(arg(2), 3))
def w_ex(st):
    o, x, xv = st
    out(" w2c in=%s out=%s" % (fmt(xv), fmt(f64(o, 2))))
w = probe_call(W2C, w_ent, w_ex); w.enabled = False
def p_ent():
    pt = u64(arg(2))   # Eigen::Ref: data pointer first
    return (arg(3), f64(pt, 3))
def p_ex(st):
    o, xv = st
    out(" p3 in=%s out=%s" % (fmt(xv), fmt(f64(o, 2))))
p = probe_call(P3, p_ent, p_ex); p.enabled = False
