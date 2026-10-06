import idautils, idc, ida_bytes, ida_funcs, ida_auto, ida_name, os, ida_pro, json
# Dumps every MSVC vtable (??_7...) with its slots, plus all named data/functions (names.json).
OUT = os.path.dirname(os.path.abspath(__file__))
ida_auto.auto_wait()
text = idaapi_seg = None
vt = []
names = []
for ea, name in idautils.Names():
    names.append((ea, name, idc.demangle_name(name, idc.get_inf_attr(idc.INF_SHORT_DN)) or ""))
    if not name.startswith("??_7"):
        continue
    dn = idc.demangle_name(name, idc.get_inf_attr(idc.INF_SHORT_DN)) or name
    slots = []
    p = ea
    while True:
        q = ida_bytes.get_qword(p)
        f = ida_funcs.get_func(q)
        if not f or f.start_ea != q:
            break
        slots.append((q, idc.get_func_name(q)))
        p += 8
        if p != ea and ida_name.get_name(p):
            break
    vt.append({"ea": ea, "name": dn, "slots": slots})
json.dump(vt, open(os.path.join(OUT, "vtables.json"), "w"), indent=0)
json.dump(names, open(os.path.join(OUT, "names.json"), "w"))
ida_pro.qexit(0)
