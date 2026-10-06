#!/usr/bin/env python3
"""Classify every function of pimax_slam.pi.dll as library or project code.

Signals (strongest first): IDA/FLIRT/export names, vtable membership (re_dump/vtables.json), source-path and
library-specific strings, then call-graph and address-neighbourhood propagation (MSVC lays out each object
file contiguously, so unlabelled functions between two functions of the same component belong to it).

  classify.py                 writes notes/classification.tsv  (ea size component evidence name)
  classify.py summary         per-component counts / bytes and address ranges
"""
import json, os, re, sys, collections

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DUMP = os.path.join(ROOT, "re_dump")

# (component, regex on name/demangled name)
NAME_RULES = [
    ("lib:ceres", r"ceres|Ceres"),
    ("lib:boost", r"boost|portable_binary_[io]archive"),
    ("lib:glog", r"google@@|glog|LogMessage|LogDestination"),
    ("lib:gflags", r"gflags|FlagRegisterer|CommandLineFlag"),
    ("lib:glew", r"glew|GLEW"),
    ("lib:tinyxml2", r"tinyxml2"),
    ("lib:dbow2", r"DBoW2|FORB"),
    ("lib:pangolin", r"pangolin"),
    ("lib:concrt", r"Concurrency"),
    ("lib:crt", r"^_+|^j_|^\?\?_[GE]|_Init_thread|__security|_CxxThrow|memcpy|memset|memmove|^str|^w?cs|_RTC|__scrt|__vcrt|_onexit|atexit|^std::|^\?\?[01]?.*std@@|@std@@"),
    ("lib:eigen", r"Eigen"),
    ("lib:opencv", r"@cv@@|cv::"),
]
# (component, regex on referenced string)
STR_RULES = [
    ("lib:ceres", r"thirdparty[\\/]ceres|Check failed|ceres|Ceres|Jacobian|jacobian|residual|Residual|Schur|LBFGS|dogleg|Dogleg|line search|Line search|Levenberg|trust region|Trust region|preconditioner|Preconditioner|linear solver|SPARSE_|DENSE_|ITERATIVE_SCHUR|CGNR|SUITE_SPARSE|EIGEN_SPARSE|parameter block|ParameterBlock|LocalParameterization|Manifold|Solver::Options|Termination|termination|minimizer|Minimizer"),
    ("lib:glog", r"glog-0\.5|Log file|log_dir|logtostderr|Check failure stack|InitGoogleLogging|minloglevel|stderrthreshold|alsologtostderr|email|Mailer|logbuf|tombstone|vmodule|symbolize"),
    ("lib:gflags", r"gflags-2\.2|flagfile|fromenv|tryfromenv|undefok|helpshort|helpxml|helpmatch|helpon|helppackage|tab_completion|ERROR: unknown command line flag|ERROR: illegal value"),
    ("lib:boost", r"[Bb]oost|serialization::archive|class version|stream error|output stream error|invalid signature|unsupported version|pointer conflict|incompatible native format|array size too short|unregistered class|multiple code instantiation|code instantiated in more than one module|class name too long|portable_binary"),
    ("lib:glew", r"^GL_|^WGL_|^GLX_|^gl[A-Z]|^wgl[A-Z]|^glX"),
    ("lib:jpeg", r"JPEG|quantiz|Huffman|JFIF|marker 0x|Bogus|Quantization table|thumbnail|Arithmetic table|EMS|XMS|Backing store|Virtual array|Scan script|Premature end of input|DCT|sampling factors"),
    ("lib:zlib", r"deflate|inflate|Jean-loup|incorrect header check|invalid distance|need dictionary|buffer error|data error|stream end|file error|insufficient memory|incompatible version"),
    ("lib:tinyxml2", r"XML_|XMLDocument|tinyxml|Error=%s ErrorID|<!--|<!DOCTYPE|Element nesting"),
    ("lib:dbow2", r"Vocabulary::|DBoW2|L1-norm|L2-norm|Chi-square|KL-divergence|Bhattacharyya|Dot product|TF-IDF|^TF$|^IDF$|BINARY"),
    ("lib:pangolin", r"[Pp]angolin|GlFont|GlSl|framebuffer"),
    ("lib:crt", r"bad allocation|bad array new length|bad cast|Unknown exception|string too long|vector too long|invalid string position|map/set too long|deque<T> too long|list too long|unordered_map/set too long|invalid hash bucket|invalid unordered_map<K, T> key|invalid map<K, T> key|broken promise|future already retrieved|promise already satisfied|no state|resource deadlock|operation not permitted|ios_base::|bad function call|invalid vector subscript|invalid deque|regex_error"),
    ("project", r"pimax_slam[\\/]beta111[\\/]src|^INFO=|ReLoc|ReLocalize|LoopClosing|PlatMap|platMap|FrameProcessor|DepthFilter|StereoInit|ImuProcessor|Headset|Pimax|camera[0-3] empty|seeds|Seeds|keyframe|KeyFrame|Frame-Id|frame bundle|Frame Bundle|landmark|edgelet|Backend:|imu initial|IMU|gravity|Gravity|reloc|Reloc|PIMAX_|Prior position|map version|LoadSavePlatMap|device_calibration|Fisheye|stereo|Stereo|Outlier rejection|motion prior|Marginaliz|Mesh|mesh|polygon|ground|Ground|tracking|Tracking|shutter"),
]
EIGEN_ASSERT = r"basalt-headers[\\/]thirdparty[\\/]eigen|eigen_assert|EIGEN_|Eigen::|m_isInitialized|rows\(\)|cols\(\)"


def load():
    funcs = json.load(open(os.path.join(DUMP, "funcs.json")))
    vts = json.load(open(os.path.join(DUMP, "vtables.json"))) if os.path.exists(os.path.join(DUMP, "vtables.json")) else []
    names = json.load(open(os.path.join(DUMP, "names.json"))) if os.path.exists(os.path.join(DUMP, "names.json")) else []
    return funcs, vts, names


def vt_component(cls):
    if re.search(r"ceres", cls): return "lib:ceres"
    if re.search(r"boost|portable_binary", cls): return "lib:boost"
    if re.search(r"google|glog", cls): return "lib:glog"
    if re.search(r"tinyxml2", cls): return "lib:tinyxml2"
    if re.search(r"DBoW2", cls): return "lib:dbow2"
    if re.search(r"pangolin", cls): return "lib:pangolin"
    if re.search(r"Concurrency", cls): return "lib:concrt"
    if re.search(r"totem|pimax|ThreeDof|vk::|BEBLID|common::", cls) and "std::" not in cls.split("<")[0]: return "project"
    if re.search(r"^std::|^class std|^struct std|type_info|exception", cls): return "lib:crt"
    if re.search(r"cv::", cls): return "lib:opencv"
    return None


def classify():
    funcs, vts, names = load()
    by = {f["ea"]: f for f in funcs}
    comp, why = {}, {}
    dem = {ea: (dn or n) for ea, n, dn in names}

    def setc(ea, c, w):
        if ea in by and ea not in comp:
            comp[ea], why[ea] = c, w

    # 1. names (only real names, not sub_XXXX)
    for f in funcs:
        n = f["name"]
        if n.startswith("sub_") or n.startswith("nullsub") or n.startswith("unknown_libname"):
            continue
        d = dem.get(f["ea"], n)
        if re.search(r"Headset", d):
            setc(f["ea"], "project", "name"); continue
        for c, rx in NAME_RULES:
            if re.search(rx, n) or re.search(rx, d):
                # templates over project types are project-instantiated library code; keep as lib
                setc(f["ea"], c, "name"); break
        if f.get("lib"):
            setc(f["ea"], "lib:crt", "flirt")
    # 2. vtables
    for v in vts:
        c = vt_component(v["name"])
        if not c:
            continue
        for ea, _ in v["slots"]:
            setc(ea, c if c != "project" else "project", "vtable " + v["name"][:60])
    # 3. strings
    for f in funcs:
        if f["ea"] in comp:
            continue
        hits = collections.Counter()
        for s in f["strings"]:
            for c, rx in STR_RULES:
                if re.search(rx, s):
                    hits[c] += 1; break
        if hits:
            c = hits.most_common(1)[0][0]
            setc(f["ea"], c, "str")
    # 4. neighbourhood fill: an unlabelled run between two labelled functions of the same component
    eas = sorted(by)
    labelled = [ea for ea in eas if ea in comp]
    changed = True
    it = 0
    while changed and it < 4:
        changed = False; it += 1
        idx = {ea: i for i, ea in enumerate(eas)}
        prev = None
        run = []
        for ea in eas:
            if ea in comp:
                if prev is not None and run and comp[prev] == comp[ea]:
                    for r in run:
                        comp[r], why[r] = comp[ea], "between"
                        changed = True
                prev, run = ea, []
            else:
                run.append(ea)
        # call-graph: unlabelled function whose callers are all one component
        for ea in eas:
            if ea in comp:
                continue
            cs = {comp.get(c) for c in by[ea]["callers"]} - {None}
            if len(cs) == 1:
                comp[ea], why[ea] = cs.pop(), "callers"
                changed = True
    for ea in eas:
        comp.setdefault(ea, "?")
        why.setdefault(ea, "")
    return funcs, comp, why


def main():
    funcs, comp, why = classify()
    out = os.path.join(ROOT, "notes", "classification.tsv")
    with open(out, "w") as fo:
        for f in sorted(funcs, key=lambda f: f["ea"]):
            fo.write("%x\t%d\t%s\t%s\t%s\n" % (f["ea"], f["size"], comp[f["ea"]], why[f["ea"]], f["name"][:100]))
    if len(sys.argv) > 1 and sys.argv[1] == "summary":
        cnt, byt = collections.Counter(), collections.Counter()
        for f in funcs:
            cnt[comp[f["ea"]]] += 1; byt[comp[f["ea"]]] += f["size"]
        for c in sorted(cnt, key=lambda c: -byt[c]):
            print("%-14s %5d funcs %8d bytes" % (c, cnt[c], byt[c]))
        # ranges
        print("\nruns (component, first, last, count):")
        run = None
        for f in sorted(funcs, key=lambda f: f["ea"]):
            c = comp[f["ea"]]
            if run and run[0] == c:
                run[2] = f["ea"]; run[3] += 1
            else:
                if run and run[3] >= 8: print("  %-12s %x-%x %d" % tuple(run))
                run = [c, f["ea"], f["ea"], 1]
        if run and run[3] >= 8: print("  %-12s %x-%x %d" % tuple(run))


if __name__ == "__main__":
    main()
