/*
 * pimax_slam.h -- public C interface of pimax_slam.pi.dll (Pimax headset inside-out tracking,
 * version "Pimax_SLAM_2.0.0.1"), reconstructed from the binary (chunk c14_interface_api).
 *
 * Consumer: pi_server.exe loads the DLL ("\pimax_slam.pi.dll" next to "\slam\voc_GEN_8X4.dbow" /
 * "pimax_database.bin") and resolves the functions BY NAME with GetProcAddress
 * ("LoadSlamLibrary load failed, some method miss match." if one is missing). pi_server resolves
 * nine of the ten exports; HeadsetLeftImageNumber is exported but not used by pi_server.
 *
 * Evidence for every record below (offsets verified in the disassembly):
 *   HeadsetImage     : HeadsetPutCameraImage 0x180162530 -- cv::Mat(rows=+24, cols=+20, CV_8UC1,
 *                      data=+32, step=(size_t)(int)+28).clone(); +8 and +12 are collected into two
 *                      uint32 vectors; frame timestamp = +0 of the 4th header.
 *                      NOTE: the sibling LedObjectPoseEstimator.dll reads cols/rows/step at +16/+20/+24;
 *                      this DLL reads +20/+24/+28 (+16 is never read here).
 *   HeadsetImuData   : HeadsetPutHmdImuData 0x180162D70 / SLAMManager::PutImu 0x180168640
 *                      (ts +0 as uint64 ns, acc +8, gyr +20, +32 copied but unused).
 *   HeadsetPose      : thread_local result of SLAMManager::PutImu (104 bytes copied to *pose);
 *                      same layout as ControllerPose of LedObjectPoseEstimator.dll.
 *   HeadsetGroundState: SLAMManager::GetGroundState 0x180167C60 (copied from FrameProcessor
 *                      +1416..+1443 under FrameProcessor mutex +1336).
 *
 * Global state: one std::unique_ptr<pimax::totem::SLAMManager> (0xB20 bytes). Every function
 * returns -1 (or 0 / nullptr where noted) while no system exists.
 */
#ifndef PIMAX_SLAM_H
#define PIMAX_SLAM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One tracking-camera image header (40 bytes). The DLL copies the pixels (cv::Mat::clone).
 * Every image must be 640x480 8-bit grey, otherwise the whole call fails. */
typedef struct HeadsetImage {
    uint64_t timestamp;          /* +0  ns; only the 4th header's value is used as the frame time */
    uint32_t shutter_speed_ns;   /* +8  exposure; only camera 3 is checked: < 100000 -> warning
                                  *     "-------- image shutter_speed_ns is %u, too short!" (frame
                                  *     is still used). Forwarded per camera to the frontend. */
    uint32_t gain;               /* +12 forwarded per camera to the frontend (second uint32 vector);
                                  *     TODO(verify): meaning (gain / exposure index) */
    uint32_t reserved16;         /* +16 never read */
    int32_t  width;              /* +20 cols; must be 640 */
    int32_t  height;             /* +24 rows; must be 480 */
    int32_t  stride;             /* +28 bytes per row (sign-extended to size_t) */
    const void* data;            /* +32 CV_8UC1 pixels */
} HeadsetImage;

/* One HMD IMU sample (40 bytes). */
typedef struct HeadsetImuData {
    uint64_t timestamp;          /* +0  ns (internally t = timestamp*1e-9 - offset, offset 0.0065 s
                                  *     if the measured IMU rate is < 900 Hz else 0.0035 s) */
    float    acc[3];             /* +8  m/s^2; samples with |acc| < 0.001 are rejected */
    float    gyr[3];             /* +20 rad/s */
    uint64_t reserved32;         /* +32 copied, never used */
} HeadsetImuData;

/* Pose output of HeadsetPutHmdImuData (97 bytes used, sizeof 104). */
typedef struct HeadsetPose {
    float    qx, qy, qz, qw;            /* +0  orientation */
    float    position[3];               /* +16 */
    float    angular_velocity[3];       /* +28 */
    float    velocity[3];               /* +40 */
    float    angular_acceleration[3];   /* +52 (always 0) */
    float    linear_acceleration[3];    /* +64 */
    uint64_t timestamp;                 /* +80 ns (IMU time after the offset correction) */
    uint8_t  tracking_state;            /* +88 6DoF: FrameProcessor state byte +445;
                                         *     IMU-only (kDof3 mode): 4 */
    uint8_t  tracking_flag;             /* +89 FrameProcessor state byte +444 (0 in kDof3 mode) */
    float    confidence;                /* +92 6DoF: state float +440; 3DoF fallback while 6DoF
                                         *     is invalid: 3DoF-tracker ready flag (0/1);
                                         *     kDof3 mode: 1.0f */
    bool     is_6dof;                   /* +96 */
} HeadsetPose;

/* Ground estimate of HeadsetGetGroundState (28 bytes). */
typedef struct HeadsetGroundState {
    bool  valid;                        /* +0 */
    float point0[3];                    /* +4  TODO(verify): two ground points in world frame; the */
    float point1[3];                    /* +16 frontend invalidates when |point0.z - point1.z| > 3 */
} HeadsetGroundState;

/* HeadsetSetTrackingMode values (SLAMManager +1729). */
enum HeadsetTrackingMode {
    kHeadsetDof3 = 0,   /* any value != 1: IMU-only; switching to it STOPS the vision thread for good */
    kHeadsetDof6 = 1    /* default */
};

#ifndef PIMAX_SLAM_NO_EXPORTS
/* 0x180162390. Returns 0 on success, -1 if a system already exists or calibration_path cannot be
 * opened with std::ifstream ("calibration_directory not exist").
 *   calibration_path : device calibration (opened as a file for the existence check, then given to
 *                      the device_calibration.xml parser as "calibration_directory")
 *   output_directory : log directory (6DOF_<Y_m_d_H_M_S>.txt, keeps 5) and map directory
 *   vocabulary_path  : DBoW2 vocabulary (voc_GEN_8X4.dbow)
 *   unused           : 4th argument, forwarded to the SLAMManager ctor and never read there;
 *                      TODO(verify): probably the map/database path
 *   loc_mode         : FrameProcessor::loc_mode_ (0 kNormal / 1 kLabMap / 2 kLabLoc, see loop closing) */
int HeadsetInitialImpl(const char* calibration_path, const char* output_directory,
                       const char* vocabulary_path, const char* unused, uint8_t loc_mode);
/* 0x180162530. Skips the first 5 calls of the process (returns -1). Returns 0 when queued,
 * -1 when no system / an image is empty or not 640x480. While the device is disabled
 * SLAMManager::PutImages drops the bundle silently (the return value is still 0). */
int HeadsetPutCameraImage(const HeadsetImage* cam0, const HeadsetImage* cam1,
                          const HeadsetImage* cam2, const HeadsetImage* cam3);
/* 0x180162D70. Returns 1 when *pose was written, 0 when the sample was dropped (rate detection
 * warm-up, timestamp not newer than the previous sample, |acc| < 0.001), -1 when no system or
 * disabled. */
int HeadsetPutHmdImuData(const HeadsetImuData* imu, HeadsetPose* pose);
/* 0x180162340. 1 = written, 0 = kDof3 mode (zeroed), -1 = no system / disabled (zeroed). */
int HeadsetGetGroundState(HeadsetGroundState* ground);
/* 0x180162360. *state = FrameProcessor+1528 (relocalised in loc_mode 2). 1/0/-1 as above. */
int HeadsetGetLocModeState(uint8_t* state);
/* 0x180162E30. Always 0 (or -1 without system). Disabling sets the vision thread's quit flag. */
int HeadsetSetDeviceEnable(bool enable);
/* 0x180162E60. Always 0 (or -1 without system). */
int HeadsetSetTrackingMode(uint8_t mode);
/* 0x180162510. Number of queued, not yet processed image bundles (0 without system). */
unsigned int HeadsetLeftImageNumber(void);
/* 0x180162DF0. Always 0. */
int HeadsetReleaseImpl(void);
/* 0x180162380. "Pimax_SLAM_2.0.0.1" */
const char* HeadsetGetVersion(void);
#endif

#ifdef __cplusplus
}  /* extern "C" */

static_assert(sizeof(HeadsetImage) == 40, "HeadsetImage size");
static_assert(offsetof(HeadsetImage, shutter_speed_ns) == 8, "");
static_assert(offsetof(HeadsetImage, gain) == 12, "");
static_assert(offsetof(HeadsetImage, width) == 20, "");
static_assert(offsetof(HeadsetImage, height) == 24, "");
static_assert(offsetof(HeadsetImage, stride) == 28, "");
static_assert(offsetof(HeadsetImage, data) == 32, "");
static_assert(sizeof(HeadsetImuData) == 40, "HeadsetImuData size");
static_assert(offsetof(HeadsetImuData, acc) == 8, "");
static_assert(offsetof(HeadsetImuData, gyr) == 20, "");
static_assert(sizeof(HeadsetPose) == 104, "HeadsetPose size");
static_assert(offsetof(HeadsetPose, position) == 16, "");
static_assert(offsetof(HeadsetPose, angular_velocity) == 28, "");
static_assert(offsetof(HeadsetPose, velocity) == 40, "");
static_assert(offsetof(HeadsetPose, angular_acceleration) == 52, "");
static_assert(offsetof(HeadsetPose, linear_acceleration) == 64, "");
static_assert(offsetof(HeadsetPose, timestamp) == 80, "");
static_assert(offsetof(HeadsetPose, tracking_state) == 88, "");
static_assert(offsetof(HeadsetPose, tracking_flag) == 89, "");
static_assert(offsetof(HeadsetPose, confidence) == 92, "");
static_assert(offsetof(HeadsetPose, is_6dof) == 96, "");
static_assert(sizeof(HeadsetGroundState) == 28, "HeadsetGroundState size");
static_assert(offsetof(HeadsetGroundState, point0) == 4, "");
static_assert(offsetof(HeadsetGroundState, point1) == 16, "");
#endif

#endif /* PIMAX_SLAM_H */
