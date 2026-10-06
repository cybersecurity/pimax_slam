#!/usr/bin/env python3
"""Reverse-engineering helper for pimax_slam.pi.dll (reads headless IDA dumps + the DLL file).

  ida_dump.py fn   <addr|name> [--asm]   pseudocode (or disassembly) of one function
  ida_dump.py list <lo> <hi>             functions in [lo, hi): addr size lines name | first string
  ida_dump.py xref <addr>                callers / callees of a function (with sizes and string tags)
  ida_dump.py str  <regex>               functions referencing strings matching regex
  ida_dump.py rd   <va> <count> <fmt>    read data from the DLL image; fmt: f32 f64 i32 u32 u64 str bytes
  ida_dump.py find <regex>               functions whose pseudocode matches regex

Reads re_dump/{all.c,all.asm,funcs.json} (see README to regenerate them) and, for `rd`, the original
DLL from $ORIG_DLL, $RUNTIME/pimax_slam.pi.dll or re_dump/orig.dll.
"""
import json, os, re, struct, sys
DUMP = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "re_dump")
DLL = os.environ.get("ORIG_DLL") or (os.path.join(os.environ["RUNTIME"], "pimax_slam.pi.dll") if "RUNTIME" in os.environ
       else os.path.join(DUMP, "orig.dll"))

def load(kind):
    txt = open(DUMP + ("/all.asm" if kind == "asm" else "/all.c")).read()
    parts = re.split(r'^(//==== [0-9a-f]+ \S+ size=\d+\n)', txt, flags=re.M)
    out = {}
    for i in range(1, len(parts), 2):
        out[int(parts[i].split()[1], 16)] = parts[i] + parts[i + 1]
    return out

def info():
    return {f["ea"]: f for f in json.load(open(DUMP + "/funcs.json"))}

def resolve(a, inf):
    try:
        return int(a, 16)
    except ValueError:
        for ea, f in inf.items():
            if f["name"] == a:
                return ea
    sys.exit("unknown function " + a)

def tag(f):
    s = [x for x in f["strings"] if len(x) > 3]
    return (" | " + s[0][:80]) if s else ""

def main():
    cmd, args = sys.argv[1], sys.argv[2:]
    inf = info()
    if cmd == "fn":
        ea = resolve(args[0], inf)
        d = load("asm" if "--asm" in args else "c")
        print(d.get(ea, "not dumped (library function?)"))
    elif cmd == "list":
        lo, hi = int(args[0], 16), int(args[1], 16)
        d = load("c")
        for ea in sorted(inf):
            if lo <= ea < hi:
                f = inf[ea]
                n = d.get(ea, "").count("\n")
                print("%x %6d %5d %s%s%s" % (ea, f["size"], n, "L " if f["lib"] else "", f["name"][:60], tag(f)))
    elif cmd == "xref":
        ea = resolve(args[0], inf)
        f = inf[ea]
        print("callers:")
        for c in f["callers"]:
            g = inf.get(c)
            if g: print("  %x %6d %s%s" % (c, g["size"], g["name"], tag(g)))
        print("callees:")
        for c in f["callees"]:
            g = inf.get(c)
            if g: print("  %x %6d %s%s" % (c, g["size"], g["name"], tag(g)))
        print("strings:")
        for s in f["strings"]: print("  " + repr(s))
    elif cmd == "str":
        rx = re.compile(args[0])
        for ea in sorted(inf):
            for s in inf[ea]["strings"]:
                if rx.search(s):
                    print("%x %s  %r" % (ea, inf[ea]["name"], s)); break
    elif cmd == "find":
        rx = re.compile(args[0]); d = load("c")
        for ea in sorted(d):
            if rx.search(d[ea]): print("%x %s" % (ea, inf.get(ea, {}).get("name", "?")))
    elif cmd == "rd":
        import pefile
        pe = pefile.PE(DLL, fast_load=True)
        va, n, fmt = int(args[0], 16), int(args[1]), args[2]
        rva = va - pe.OPTIONAL_HEADER.ImageBase
        if fmt == "str":
            data = pe.get_data(rva, 4096); print(repr(data.split(b"\0")[0].decode("latin1"))); return
        sz = {"f32": 4, "f64": 8, "i32": 4, "u32": 4, "u64": 8, "bytes": 1}[fmt]
        data = pe.get_data(rva, n * sz)
        if fmt == "bytes": print(data.hex()); return
        code = {"f32": "f", "f64": "d", "i32": "i", "u32": "I", "u64": "Q"}[fmt]
        for i in range(n):
            v = struct.unpack_from("<" + code, data, i * sz)[0]
            print("%x %s" % (va + i * sz, repr(v) if "f" in fmt else ("%d (0x%x)" % (v, v & 0xffffffffffffffff))))
    else:
        print(__doc__)

main()
