#!/usr/bin/env python3
"""Make a deterministic test copy of the original pimax_slam.pi.dll.

Two things make the original non-deterministic run to run (besides thread timing, which the replayer
controls with its settle time and cv::setNumThreads(1)):

1. Wall-clock solver budget.
Estimator::optimize (0x18002A6D0) sets ceres::Solver::Options::max_solver_time_in_seconds = 0.025 before
every solve (`mov rcx, 3F9999999999999Ah` at 0x18002A763), so the number of Ceres iterations - and from
then on every pose - depends on how fast the machine is. For deterministic A/B runs both DLLs must run
without that budget: this script patches the immediate to 1e9 s, and the rebuilt DLL does the same when
configured with -DPIMAX_SLAM_TEST_DETERMINISTIC=ON.
2. std::random_device. StereoTriangulation::compute seeds the std::mt19937 of its two std::shuffle calls
   with std::random_device (0x180148065, 0x1801480EC). Both go through the import thunk
   std::_Random_device at 0x180395D04 (`jmp [__imp_...]`, 6 bytes), which is replaced by
   `mov eax, 5489; ret`. The test build of the rebuilt DLL seeds with the same constant.

Optionally (--log-level N) the initial level of the printf logger (g_logger.m_level, .data 0x18046A004,
shipped value 3) is changed so that LOGI/LOGD lines are written too; the rebuilt test build takes
-DPIMAX_SLAM_TEST_LOG_LEVEL=N.

  make_deterministic_orig.py [--log-level N] <pimax_slam.pi.dll> <out.dll>
"""
import struct
import sys

import pefile

VA = 0x18002A763
OLD = bytes.fromhex("48b9") + struct.pack("<d", 0.025)
NEW = bytes.fromhex("48b9") + struct.pack("<d", 1e9)

args = sys.argv[1:]
log_level = None
if args[0] == "--log-level":
    log_level = int(args[1]); args = args[2:]
src, dst = args[0], args[1]
pe = pefile.PE(src)
off = pe.get_offset_from_rva(VA - pe.OPTIONAL_HEADER.ImageBase)
data = bytearray(open(src, "rb").read())
if data[off:off + len(OLD)] != OLD:
    sys.exit("unexpected bytes at 0x%x: %s" % (VA, data[off:off + len(OLD)].hex()))
data[off:off + len(NEW)] = NEW
RD_VA = 0x180395D04
rd_off = pe.get_offset_from_rva(RD_VA - pe.OPTIONAL_HEADER.ImageBase)
if data[rd_off:rd_off + 2] != b"\xff\x25":
    sys.exit("unexpected bytes at 0x%x" % RD_VA)
data[rd_off:rd_off + 6] = b"\xb8" + struct.pack("<I", 5489) + b"\xc3"
if log_level is not None:
    lv_off = pe.get_offset_from_rva(0x18046A004 - pe.OPTIONAL_HEADER.ImageBase)
    if struct.unpack_from("<i", data, lv_off)[0] != 3:
        sys.exit("unexpected logger level value")
    struct.pack_into("<i", data, lv_off, log_level)
    print("logger level 3 -> %d" % log_level)
open(dst, "wb").write(data)
print("patched max_solver_time_in_seconds 0.025 -> 1e9 at 0x%x, std::_Random_device -> 5489 at 0x%x" % (VA, RD_VA))
