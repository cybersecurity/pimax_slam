import idautils, idc, ida_funcs, ida_hexrays, ida_auto, ida_frame, json, os, ida_pro

OUT = os.path.dirname(os.path.abspath(__file__))
ida_auto.auto_wait()
ida_hexrays.init_hexrays_plugin()

strs = {s.ea: str(s) for s in idautils.Strings()}


def decomp(f):
    hf = ida_hexrays.hexrays_failure_t()
    try:
        c = ida_hexrays.decompile_func(f, hf, 0)
    except Exception as e:
        return None, "EXC %s" % e
    return c, hf.desc()


info = []
fails = []
skip = set()
if os.path.exists(os.path.join(OUT, "skip.txt")):
    skip = {int(l, 16) for l in open(os.path.join(OUT, "skip.txt")) if l.strip()}
done = set()
if os.path.exists(os.path.join(OUT, "done.txt")):
    done = {int(l, 16) for l in open(os.path.join(OUT, "done.txt")) if l.strip()}
prog = open(os.path.join(OUT, "progress.txt"), "w")
dn = open(os.path.join(OUT, "done.txt"), "a")
with open(os.path.join(OUT, "all.c"), "a") as fo:
    for ea in list(idautils.Functions()):
        f = ida_funcs.get_func(ea)
        name = idc.get_func_name(ea)
        refs, callees = set(), set()
        for i in idautils.FuncItems(ea):
            for x in idautils.DataRefsFrom(i):
                if x in strs:
                    refs.add(strs[x][:160])
            for x in idautils.CodeRefsFrom(i, 0):
                g = ida_funcs.get_func(x)
                if g and g.start_ea != ea:
                    callees.add(g.start_ea)
        callers = sorted({ida_funcs.get_func(x.frm).start_ea for x in idautils.XrefsTo(ea) if ida_funcs.get_func(x.frm)})
        lib = bool(f.flags & ida_funcs.FUNC_LIB)
        info.append({"ea": ea, "name": name, "size": f.end_ea - ea, "lib": lib,
                     "strings": sorted(refs), "callees": sorted(callees), "callers": callers})
        if lib or ea in done:
            continue
        if ea in skip:
            fo.write("//==== %x %s size=%d\n// DECOMPILER CRASH - skipped\n\n" % (ea, name, f.end_ea - ea))
            continue
        prog.seek(0); prog.write("%x\n" % ea); prog.flush(); os.fsync(prog.fileno())
        c, err = decomp(f)
        if not c:
            end = f.end_ea
            ida_frame.del_frame(f)
            ida_funcs.del_func(ea)
            ida_funcs.add_func(ea, end)
            ida_auto.auto_wait()
            f = ida_funcs.get_func(ea)
            c, err = decomp(f) if f else (None, "readd failed")
        if not c:
            fails.append((ea, err))
        txt = str(c) if c else "// DECOMPILE FAILED: %s" % err
        fo.write("//==== %x %s size=%d\n%s\n\n" % (ea, name, (f.end_ea - ea) if f else 0, txt))
        fo.flush(); dn.write("%x\n" % ea); dn.flush()

json.dump(info, open(os.path.join(OUT, "funcs.json"), "w"))
with open(os.path.join(OUT, "fails.txt"), "w") as ff:
    for ea, err in fails:
        ff.write("%x %s\n" % (ea, err))
with open(os.path.join(OUT, "strings.txt"), "w") as fs:
    for ea, s in sorted(strs.items()):
        fs.write("%x %r\n" % (ea, s))
ida_pro.qexit(0)
