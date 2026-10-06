# Helpers for test/gdb_probe.sh probe scripts (exec'd inside gdb's python).
import struct
import gdb

_inf = gdb.selected_inferior()

def rel(va):            # image VA (0x180...) -> runtime address
    return va - 0x180000000 + base

def mem(addr, n):
    return bytes(_inf.read_memory(addr, n))

def u64(a): return struct.unpack("<Q", mem(a, 8))[0]
def i32(a): return struct.unpack("<i", mem(a, 4))[0]
def f64(a, n=1):
    v = struct.unpack("<%dd" % n, mem(a, 8 * n))
    return v[0] if n == 1 else list(v)
def f32(a, n=1):
    v = struct.unpack("<%df" % n, mem(a, 4 * n))
    return v[0] if n == 1 else list(v)
def reg(r): return int(gdb.parse_and_eval("$" + r)) & 0xFFFFFFFFFFFFFFFF
def arg(i):             # integer/pointer argument i (0-based) at function entry (MS x64)
    if i < 4: return reg(["rcx", "rdx", "r8", "r9"][i])
    return u64(reg("rsp") + 8 * (i + 1))
def xmm(i): return struct.unpack("<2d", bytes(int(b) & 0xff for b in gdb.parse_and_eval("$xmm%d.v16_int8" % i)))[0]

def msvc_map_nodes(m):
    """Value addresses (node + 32, i.e. key) of an MSVC std::map/set in order. m = address of the map."""
    head = u64(m)
    out = []
    def walk(n):
        if mem(n + 25, 1)[0]: return     # _Isnil
        walk(u64(n)); out.append(n + 32); walk(u64(n + 16))
    walk(u64(head + 8))
    return out

def vec_range(v):       # MSVC std::vector: _Myfirst, _Mylast
    return u64(v), u64(v + 8)

def fmt(xs): return ",".join("%.17g" % x for x in xs)

def out(s):
    OUT.write(s + "\n"); OUT.flush()

class _BP(gdb.Breakpoint):
    def __init__(self, addr, cb):
        super().__init__("*0x%x" % addr, internal=True)
        self.cb = cb
    def stop(self):
        try:
            return bool(self.cb())
        except Exception as e:
            out("probe error: %r" % e)
            return False

def probe(va, cb):
    return _BP(rel(va), cb)

class _RetBP(gdb.Breakpoint):
    """One breakpoint per return address, shared by all pending calls (keyed by the caller's rsp)."""
    def __init__(self, addr):
        super().__init__("*0x%x" % addr, internal=True)
        self.calls = {}
    def stop(self):
        cb = self.calls.pop(reg("rsp"), None)
        if cb is not None:
            try:
                cb()
            except Exception as e:
                out("probe error: %r" % e)
        if not self.calls:
            self.enabled = False
        return False

_ret_bps = {}

def probe_call(va, on_entry, on_exit):
    """Breakpoint at function entry `va`; on_entry() returns a state object (or None to skip);
    on_exit(state) runs when the function returns (rax/xmm0 hold the return value)."""
    def cb():
        st = on_entry()
        if st is not None:
            ret = u64(reg("rsp"))
            bp = _ret_bps.get(ret)
            if bp is None:
                bp = _ret_bps[ret] = _RetBP(ret)
            bp.calls[reg("rsp") + 8] = (lambda st=st: on_exit(st))
            bp.enabled = True
        return False
    return _BP(rel(va), cb)

def vecxd(p):           # Eigen::VectorXd / MatrixXd column data at object p (data, rows[, cols])
    d, n = u64(p), u64(p + 8)
    return f64(d, n) if n > 1 else ([f64(d)] if n == 1 else [])
