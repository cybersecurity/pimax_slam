/* Variant of replay.c for gdb probing (test/gdb_probe.sh): with PS_BREAK set it executes int3 right after
 * LoadLibrary so that the debugger can read the DLL base (rbx) and plant breakpoints. Generated from replay.c. */
/* Replays a trace written by gen_scenario.py into a pimax_slam.pi.dll (original or rebuilt) through
 * its C API and records every HeadsetPutHmdImuData output.
 *
 *   replay.exe <dll> <device_calibration.xml> <out_dir> <voc.dbow> <loc_mode> <trace.bin> <out.bin>
 *
 * out.bin: "PSLO" u32 version=1, then for every 'I' record: u64 tsNs, i32 ret, 104-byte HeadsetPose
 * (pre-filled with 0xCD so untouched bytes are visible); for every 'F' record a marker
 * u64 0xFFFFFFFFFFFFFFFF, i32 ret of HeadsetPutCameraImage, u32 queued images, HeadsetGroundState (28 B),
 * i32 ret of HeadsetGetGroundState, u8 loc state, i32 ret of HeadsetGetLocModeState.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>

#define PIMAX_SLAM_NO_EXPORTS
#include "../include/pimax_slam.h"

typedef int (*InitFn)(const char*, const char*, const char*, const char*, uint8_t);
typedef int (*PutImageFn)(const HeadsetImage*, const HeadsetImage*, const HeadsetImage*, const HeadsetImage*);
typedef int (*PutImuFn)(const HeadsetImuData*, HeadsetPose*);
typedef int (*GroundFn)(HeadsetGroundState*);
typedef int (*LocFn)(uint8_t*);
typedef int (*EnableFn)(bool);
typedef int (*ModeFn)(uint8_t);
typedef unsigned (*LeftFn)(void);
typedef int (*ReleaseFn)(void);
typedef const char* (*VersionFn)(void);

static void rd(FILE* f, void* p, size_t n) {
    if (fread(p, 1, n, f) != n) {
        fprintf(stderr, "truncated trace\n");
        exit(3);
    }
}

int main(int argc, char** argv) {
    if (argc < 8) {
        fprintf(stderr, "usage: replay dll calib.xml out_dir voc.dbow loc_mode trace.bin out.bin\n");
        return 2;
    }
    HMODULE m = LoadLibraryA(argv[1]);
    if (getenv("PS_BREAK")) { printf("base %p\n", (void*)m); fflush(stdout); __debugbreak(); }
    if (!m) {
        fprintf(stderr, "LoadLibrary(%s) failed: %lu\n", argv[1], GetLastError());
        return 2;
    }
    InitFn init = (InitFn)GetProcAddress(m, "HeadsetInitialImpl");
    PutImageFn putImage = (PutImageFn)GetProcAddress(m, "HeadsetPutCameraImage");
    PutImuFn putImu = (PutImuFn)GetProcAddress(m, "HeadsetPutHmdImuData");
    GroundFn ground = (GroundFn)GetProcAddress(m, "HeadsetGetGroundState");
    LocFn loc = (LocFn)GetProcAddress(m, "HeadsetGetLocModeState");
    EnableFn enable = (EnableFn)GetProcAddress(m, "HeadsetSetDeviceEnable");
    ModeFn mode = (ModeFn)GetProcAddress(m, "HeadsetSetTrackingMode");
    LeftFn left = (LeftFn)GetProcAddress(m, "HeadsetLeftImageNumber");
    ReleaseFn release = (ReleaseFn)GetProcAddress(m, "HeadsetReleaseImpl");
    VersionFn version = (VersionFn)GetProcAddress(m, "HeadsetGetVersion");
    if (!init || !putImage || !putImu || !ground || !loc || !enable || !mode || !left || !release || !version) {
        fprintf(stderr, "missing export\n");
        return 2;
    }
    printf("version %s\n", version());
    /* PS_CV_THREADS=n: cv::setNumThreads(n) in the shared opencv_core453.dll before init. The edgelet
     * non-max suppression writes a shared vector from cv::parallel_for_ rows, so A/B runs are only
     * reproducible with n=1. */
    const char* cvth = getenv("PS_CV_THREADS");
    if (cvth) {
        HMODULE cvcore = GetModuleHandleA("opencv_core453.dll");
        void (*setNumThreads)(int) = cvcore ? (void (*)(int))GetProcAddress(cvcore, "?setNumThreads@cv@@YAXH@Z") : NULL;
        if (setNumThreads) {
            setNumThreads(atoi(cvth));
            printf("cv::setNumThreads(%d)\n", atoi(cvth));
        }
    }

    FILE* tf = fopen(argv[6], "rb");
    FILE* of = fopen(argv[7], "wb");
    if (!tf || !of) {
        fprintf(stderr, "cannot open trace/output\n");
        return 2;
    }
    char magic[4];
    uint32_t hdr[3];
    rd(tf, magic, 4);
    rd(tf, hdr, 12);
    if (memcmp(magic, "PSLT", 4) != 0 || hdr[0] != 1) {
        fprintf(stderr, "bad trace\n");
        return 2;
    }
    const int W = (int)hdr[1], H = (int)hdr[2];
    uint8_t* pix[4];
    for (int i = 0; i < 4; ++i)
        pix[i] = (uint8_t*)malloc((size_t)W * H);
    fwrite("PSLO", 1, 4, of);
    uint32_t over = 1;
    fwrite(&over, 4, 1, of);

    int rc = init(argv[2], argv[3], argv[4], "", (uint8_t)atoi(argv[5]));
    printf("HeadsetInitialImpl -> %d\n", rc);
    fflush(stdout);

    long nImu = 0, nFrames = 0, nPose = 0;
    int type;
    while ((type = fgetc(tf)) != EOF) {
        if (type == 'I') {
            HeadsetImuData imu;
            memset(&imu, 0, sizeof(imu));
            rd(tf, &imu.timestamp, 8);
            rd(tf, imu.acc, 12);
            rd(tf, imu.gyr, 12);
            rd(tf, &imu.reserved32, 8);
            HeadsetPose pose;
            memset(&pose, 0xCD, sizeof(pose));
            int32_t ret = putImu(&imu, &pose);
            fwrite(&imu.timestamp, 8, 1, of);
            fwrite(&ret, 4, 1, of);
            fwrite(&pose, 1, sizeof(pose), of);
            ++nImu;
            nPose += ret == 1;
        } else if (type == 'F') {
            uint64_t ts;
            uint32_t shutter, gain;
            rd(tf, &ts, 8);
            rd(tf, &shutter, 4);
            rd(tf, &gain, 4);
            HeadsetImage im[4];
            for (int i = 0; i < 4; ++i) {
                rd(tf, pix[i], (size_t)W * H);
                memset(&im[i], 0, sizeof(im[i]));
                im[i].timestamp = ts;
                im[i].shutter_speed_ns = shutter;
                im[i].gain = gain;
                im[i].width = W;
                im[i].height = H;
                im[i].stride = W;
                im[i].data = pix[i];
            }
            int32_t ret = putImage(&im[0], &im[1], &im[2], &im[3]);
            /* wait until the vision thread has taken the bundle off its queue (the following 'W'
             * record then gives it time to finish processing) */
            for (int i = 0; i < 500 && left() > 0; ++i)
                Sleep(1);
            uint64_t marker = ~0ull;
            uint32_t queued = left();
            HeadsetGroundState g;
            memset(&g, 0xCD, sizeof(g));
            int32_t gret = ground(&g);
            uint8_t ls = 0xCD;
            int32_t lret = loc(&ls);
            fwrite(&marker, 8, 1, of);
            fwrite(&ret, 4, 1, of);
            fwrite(&queued, 4, 1, of);
            fwrite(&g, 1, sizeof(g), of);
            fwrite(&gret, 4, 1, of);
            fwrite(&ls, 1, 1, of);
            fwrite(&lret, 4, 1, of);
            ++nFrames;
        } else if (type == 'M') {
            int32_t md;
            rd(tf, &md, 4);
            mode((uint8_t)md);
        } else if (type == 'E') {
            uint8_t e;
            rd(tf, &e, 1);
            enable(e != 0);
        } else if (type == 'W') {
            uint32_t ms;
            rd(tf, &ms, 4);
            Sleep(ms);
        } else {
            fprintf(stderr, "bad record type %d\n", type);
            return 3;
        }
    }
    /* let the queue drain before tearing down */
    for (int i = 0; i < 200 && left() > 0; ++i)
        Sleep(20);
    printf("frames %ld imu %ld poses %ld\n", nFrames, nImu, nPose);
    printf("HeadsetReleaseImpl -> %d\n", release());
    fclose(of);
    fclose(tf);
    return 0;
}
