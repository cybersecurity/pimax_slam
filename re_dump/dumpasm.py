import idautils, idc, ida_funcs, ida_auto, os, ida_pro
OUT = os.path.dirname(os.path.abspath(__file__))
ida_auto.auto_wait()
with open(os.path.join(OUT, "all.asm"), "w") as fo:
    for ea in idautils.Functions():
        f = ida_funcs.get_func(ea)
        fo.write("//==== %x %s size=%d\n" % (ea, idc.get_func_name(ea), f.end_ea - ea))
        for i in idautils.FuncItems(ea):
            cmt = idc.get_cmt(i, 0) or ""
            fo.write("%x  %s%s\n" % (i, idc.generate_disasm_line(i, 0), ("  ; " + cmt) if cmt else ""))
        fo.write("\n")
ida_pro.qexit(0)
