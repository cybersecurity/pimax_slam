#!/usr/bin/env python3
"""Parser for the pimax_slam PlatMap files (chunk c15_platmap, see notes/c15_platmap.md).

The relocalization map is written by PlatMap::save (0x180183ec0) with a NATIVE
boost::archive::binary_oarchive (boost 1.74, MSVC x64) to
    <map_path><map_tag><map_name>          e.g. ".../platMap_orborb_K8L4.bin"
and can alternatively be read by PlatMap::load_bin (0x180183aa0) from a PORTABLE
archive (boost 1.74 example portable_binary_oarchive), e.g. "...bin.pba".
The YAML side-car index ("platMap_orborb_K8L4.yaml", PlatMap::saveIndex 0x180184590)
can be dumped with --index.

Both archive flavours carry the same object stream (schema is fixed, boost archives are
not self-describing):

    header                    (archive flavour specific)
    std::string  "PlatMap"
    PlatMap                   -> std::vector<std::vector<KeyFrame*>>  (kf_list_)

Usage:
    parse_platmap.py FILE [--json OUT.json] [--max-kf N] [--quiet]
    parse_platmap.py --index FILE.yaml

On a file that is not a boost serialization archive (for instance the controller LED
database ../slam/pimax_database.bin, which is a *cereal* vote map of
LedObjectPoseEstimator, not a PlatMap) the tool says so and exits with status 2.
"""
import argparse
import json
import re
import struct
import sys

SIGNATURE = b"serialization::archive"

# Types that carry boost class information (tracking byte + version) the first time an
# object of that type is (de)serialized.  std::vector<int> does not (collection of a
# primitive -> object_serializable).
CLASS_INFO = {
    "PlatMap", "vector<vector<KeyFrame*>>", "vector<KeyFrame*>", "KeyFrame",
    "QuatTransformation", "Vector3d", "vector<Point2f>", "Point2f", "vector<Mat>", "Mat",
    "BowVector", "map<uint,double>", "pair<const uint,double>", "vector<Point3f>", "Point3f",
}


class ArchiveError(Exception):
    pass


class Reader:
    def __init__(self, data, portable):
        self.d = data
        self.p = 0
        self.portable = portable
        self.big_endian = False
        self.class_state = {}      # type name -> (tracking, version)
        self.objects = []          # tracked objects (object id -> python object)
        self.keyframe_cid = None

    # ---- raw ---------------------------------------------------------------------------
    def raw(self, n):
        if self.p + n > len(self.d):
            raise ArchiveError("unexpected end of file at 0x%x (need %d bytes)" % (self.p, n))
        b = self.d[self.p:self.p + n]
        self.p += n
        return b

    def u8(self):
        return self.raw(1)[0]

    def f32(self):
        return struct.unpack("<f", self.raw(4))[0]

    def f64(self):
        return struct.unpack("<d", self.raw(8))[0]

    # ---- integers ------------------------------------------------------------------------
    def vint(self, maxsize):
        """portable_binary_iarchive::load_impl (boost 1.74 example)."""
        size = struct.unpack("<b", self.raw(1))[0]
        if size == 0:
            return 0
        neg = size < 0
        if neg:
            size = -size
        if size > maxsize:
            raise ArchiveError("portable integer of %d bytes > %d at 0x%x" % (size, maxsize, self.p - 1))
        b = self.raw(size)
        v = int.from_bytes(b, "big" if self.big_endian else "little", signed=False)
        return -v if neg else v

    def integer(self, nbytes, signed=True):
        if self.portable:
            return self.vint(nbytes)
        b = self.raw(nbytes)
        return int.from_bytes(b, "little", signed=signed)

    def i32(self):
        return self.integer(4)

    def u32(self):
        return self.integer(4, signed=False)

    def i64(self):
        return self.integer(8)

    def u64(self):
        return self.integer(8, signed=False)

    def boolean(self):
        if self.portable:
            return bool(self.vint(1))
        return bool(self.u8())

    # boost bookkeeping types
    def tracking(self):
        return self.boolean()

    def version(self):
        return self.integer(4, signed=False)          # version_type: uint32

    def class_id(self):
        return self.integer(2)                         # class_id_type: int16

    def object_id(self):
        return self.integer(4, signed=False)           # object_id_type: uint32

    def collection_size(self):
        return self.integer(8, signed=False)           # collection_size_type: size_t

    def item_version(self):
        return self.integer(4, signed=False)           # item_version_type: uint32

    def string(self):
        n = self.integer(8, signed=False)
        if n > len(self.d):
            raise ArchiveError("bogus string length %d at 0x%x" % (n, self.p))
        return self.raw(n).decode("latin-1")

    # ---- header --------------------------------------------------------------------------
    def header(self):
        info = {"flavour": "portable" if self.portable else "native-binary"}
        sig = self.string()
        if sig != SIGNATURE.decode():
            raise ArchiveError("bad signature %r" % sig)
        if self.portable:
            info["library_version"] = self.vint(2)
            flags = self.u8()                         # m_flags >> 8
            info["flags"] = flags
            self.big_endian = bool((flags << 8) & 0x4000)
        else:
            info["library_version"] = int.from_bytes(self.raw(2), "little")
            sizes = self.raw(4)
            info["sizeof(int,long,float,double)"] = list(sizes)
            endian = struct.unpack("<i", self.raw(4))[0]
            if endian != 1:
                raise ArchiveError("endian check int is %d" % endian)
            if sizes[0] != 4 or sizes[2] != 4 or sizes[3] != 8:
                raise ArchiveError("unexpected primitive sizes %s" % list(sizes))
        return info

    # ---- objects -------------------------------------------------------------------------
    def preamble(self, tname):
        """basic_iarchive_impl::load_preamble for a by-value object."""
        if tname not in self.class_state:
            if tname in CLASS_INFO:
                self.class_state[tname] = (self.tracking(), self.version())
            else:
                self.class_state[tname] = (False, 0)
        return self.class_state[tname]

    def obj(self, tname, fn):
        tracking, version = self.preamble(tname)
        if tracking:
            oid = self.object_id()
            if oid < len(self.objects):
                return {"$ref": oid}
            self.objects.append(None)
            val = fn()
            self.objects[oid] = val
            return val
        return fn()

    def collection(self, read_item, with_item_version=True):
        n = self.collection_size()
        if n > len(self.d):
            raise ArchiveError("bogus collection size %d at 0x%x" % (n, self.p))
        if with_item_version:
            self.item_version()
        return [read_item() for _ in range(n)]

    # concrete types ----------------------------------------------------------------------
    def point2f(self):
        return self.obj("Point2f", lambda: [self.f32(), self.f32()])

    def point3f(self):
        return self.obj("Point3f", lambda: [self.f32(), self.f32(), self.f32()])

    def vec_point2f(self):
        return self.obj("vector<Point2f>", lambda: self.collection(self.point2f))

    def vec_point3f(self):
        return self.obj("vector<Point3f>", lambda: self.collection(self.point3f))

    def vec_int(self):
        def body():
            if self.portable:
                return self.collection(self.i32)
            # native binary_oarchive: array optimisation -> count + raw int32 array
            n = self.collection_size()
            if n * 4 > len(self.d) - self.p:
                raise ArchiveError("bogus vector<int> size %d at 0x%x" % (n, self.p))
            return list(struct.unpack("<%di" % n, self.raw(4 * n)))
        return self.obj("vector<int>", body)

    def mat(self):
        def body():
            cols = self.i32()
            rows = self.i32()
            typ = self.i32()
            continuous = self.boolean()
            depth = typ & 7
            cn = ((typ >> 3) & 511) + 1
            elem = [1, 1, 2, 2, 4, 4, 8, 2][depth] * cn
            size = rows * cols * elem
            if size < 0 or size > len(self.d) - self.p:
                raise ArchiveError("bogus cv::Mat %dx%d type %d at 0x%x" % (rows, cols, typ, self.p))
            data = self.raw(size)
            return {"rows": rows, "cols": cols, "type": typ, "continuous": continuous,
                    "data": data.hex()}
        return self.obj("Mat", body)

    def vec_mat(self):
        return self.obj("vector<Mat>", lambda: self.collection(self.mat))

    def vector3d(self):
        def body():
            rows = self.i64()
            cols = self.i64()
            if rows * cols != 3:
                raise ArchiveError("Vector3d with %dx%d at 0x%x" % (rows, cols, self.p))
            return [self.f64() for _ in range(rows * cols)]
        return self.obj("Vector3d", body)

    def quat_transformation(self):
        def body():
            w, x, y, z = self.f64(), self.f64(), self.f64(), self.f64()
            return {"q_wxyz": [w, x, y, z], "p": self.vector3d()}
        return self.obj("QuatTransformation", body)

    def bow_vector(self):
        def pair():
            return self.obj("pair<const uint,double>", lambda: [self.u32(), self.f64()])

        def map_body():
            return self.collection(pair)

        return self.obj("BowVector", lambda: self.obj("map<uint,double>", map_body))

    def keyframe_data(self):
        kf = {}
        kf["map_id"] = self.i32()
        kf["NframeID"] = self.i32()
        kf["frame_id"] = self.i32()
        kf["cam_id"] = self.i32()
        kf["timestamp"] = self.f64()
        kf["T_w_c"] = self.quat_transformation()
        kf["bow_keypoints"] = self.vec_point2f()
        kf["bow_features"] = self.vec_mat()
        kf["bow_node_ids"] = self.vec_int()
        kf["vec_bow"] = self.bow_vector()
        kf["svo_features"] = self.vec_mat()
        kf["svo_keypoints"] = self.vec_point2f()
        kf["svo_node_ids"] = self.vec_int()
        kf["svo_landmarks_cam"] = self.vec_point3f()
        kf["num_bow_features"] = self.u64()
        return kf

    def keyframe_ptr(self):
        """basic_iarchive_impl::load_pointer for KeyFrame* (non-polymorphic)."""
        cid = self.class_id()
        if cid == -1:
            return None
        if self.keyframe_cid is None:
            self.keyframe_cid = cid
            # class preamble (class_info: tracking + version) follows the first class id
            self.class_state["KeyFrame"] = (self.tracking(), self.version())
        elif cid != self.keyframe_cid:
            raise ArchiveError("unexpected class id %d (KeyFrame is %d) at 0x%x"
                               % (cid, self.keyframe_cid, self.p - 2))
        tracking, _ = self.class_state["KeyFrame"]
        if tracking:
            oid = self.object_id()
            if oid < len(self.objects):
                return {"$ref": oid}
            if oid != len(self.objects):
                raise ArchiveError("object id %d out of sequence at 0x%x" % (oid, self.p))
            self.objects.append(None)
            kf = self.keyframe_data()
            kf["$oid"] = oid
            self.objects[oid] = kf
            return kf
        return self.keyframe_data()

    def platmap(self):
        def kf_group():
            return self.obj("vector<KeyFrame*>", lambda: self.collection(self.keyframe_ptr))

        def kf_list():
            return self.obj("vector<vector<KeyFrame*>>", lambda: self.collection(kf_group))

        return self.obj("PlatMap", kf_list)


def detect(data):
    if len(data) >= 30 and data[:8] == struct.pack("<Q", len(SIGNATURE)) and data[8:30] == SIGNATURE:
        return False   # native
    if len(data) >= 24 and data[0] == 1 and data[1] == len(SIGNATURE) and data[2:24] == SIGNATURE:
        return True    # portable
    return None


def parse(data):
    portable = detect(data)
    if portable is None:
        raise ArchiveError("not a boost serialization archive (first bytes %s)" % data[:16].hex())
    r = Reader(data, portable)
    hdr = r.header()
    tag = r.string()
    if tag != "PlatMap":
        raise ArchiveError("unexpected tag string %r" % tag)
    groups = r.platmap()
    if r.p != len(data):
        hdr["trailing_bytes"] = len(data) - r.p
    return hdr, groups, r


def summarize(hdr, groups, r, max_kf, out=sys.stdout):
    print("archive: %s, boost library version %s" % (hdr["flavour"], hdr["library_version"]), file=out)
    for k, v in hdr.items():
        if k not in ("flavour", "library_version"):
            print("  %s: %s" % (k, v), file=out)
    print("tag: PlatMap", file=out)
    print("class info:", {k: v for k, v in r.class_state.items()}, file=out)
    nkf = sum(1 for g in groups for kf in g if isinstance(kf, dict) and "$ref" not in kf)
    print("kf groups: %d, keyframes: %d (KeyFrame class id %s)" % (len(groups), nkf, r.keyframe_cid), file=out)
    maps = {}
    for g in groups:
        for kf in g:
            if kf and "map_id" in kf:
                maps.setdefault(kf["map_id"], []).append(kf)
    for mid, kfs in sorted(maps.items()):
        ts = [k["timestamp"] for k in kfs]
        print("  map_id %d: %d keyframes, timestamp %.6f .. %.6f" % (mid, len(kfs), min(ts), max(ts)), file=out)
    shown = 0
    for gi, g in enumerate(groups):
        for kf in g:
            if shown >= max_kf:
                return
            if kf is None:
                print("  [%d] null" % gi, file=out)
                continue
            if "$ref" in kf:
                print("  [%d] -> object %d" % (gi, kf["$ref"]), file=out)
                continue
            shown += 1
            print("  [%d] oid %s map %d NframeID %d frame_id %d cam %d t=%.6f q=%s p=%s"
                  % (gi, kf.get("$oid"), kf["map_id"], kf["NframeID"], kf["frame_id"], kf["cam_id"],
                     kf["timestamp"], ["%.4f" % v for v in kf["T_w_c"]["q_wxyz"]],
                     ["%.4f" % v for v in kf["T_w_c"]["p"]]), file=out)
            print("       bow: %d kps, %d desc, %d node ids, bowvec %d words | svo: %d desc, %d kps, "
                  "%d node ids, %d landmarks | num_bow_features %d"
                  % (len(kf["bow_keypoints"]), len(kf["bow_features"]), len(kf["bow_node_ids"]),
                     len(kf["vec_bow"]), len(kf["svo_features"]), len(kf["svo_keypoints"]),
                     len(kf["svo_node_ids"]), len(kf["svo_landmarks_cam"]), kf["num_bow_features"]),
                  file=out)


def parse_index(text):
    """cv::FileStorage YAML written by PlatMap::saveIndex."""
    ver = re.search(r"map_version_:\s*\"?([^\"\n]*)\"?", text)
    entries = []
    for m in re.finditer(r"\{([^}]*)\}", text, re.S):
        e = {}
        for k, v in re.findall(r"(\w+):\s*([-+0-9.eE]+)", m.group(1)):
            e[k] = float(v) if k == "newest_timestamp_" else int(v)
        entries.append(e)
    if not entries:   # block style
        for blk in re.split(r"\n\s*-\s*", text)[1:]:
            e = {}
            for k, v in re.findall(r"(\w+):\s*([-+0-9.eE]+)", blk):
                e[k] = float(v) if k == "newest_timestamp_" else int(v)
            if e:
                entries.append(e)
    return (ver.group(1) if ver else None), entries


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("--json", help="write the full decoded object tree as JSON")
    ap.add_argument("--max-kf", type=int, default=8, help="keyframes to print (default 8)")
    ap.add_argument("--index", action="store_true", help="FILE is the YAML index")
    a = ap.parse_args()

    if a.index:
        ver, entries = parse_index(open(a.file, encoding="latin-1").read())
        print("map_version_: %s (loader requires \"1.0.0\", at most 3 entries)" % ver)
        for e in entries:
            print("  ", e)
        return 0

    data = open(a.file, "rb").read()
    try:
        hdr, groups, r = parse(data)
    except ArchiveError as e:
        print("%s: %s" % (a.file, e))
        if detect(data) is None and len(data) >= 8:
            n = struct.unpack("<Q", data[:8])[0]
            print("  hint: first u64 = %d; a cereal BinaryArchive container would start with its "
                  "element count (e.g. LedObjectPoseEstimator's pimax_database.bin vote map)." % n)
        return 2
    summarize(hdr, groups, r, a.max_kf)
    if a.json:
        with open(a.json, "w") as f:
            json.dump({"header": hdr, "kf_list": groups}, f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
