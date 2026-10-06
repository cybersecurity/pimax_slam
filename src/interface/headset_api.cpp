// pimax_slam.pi.dll -- src/interface/headset_api.cpp  (draft c14_interface_api/interface/headset_api.cpp)
//
// The 10 extern "C" exports of pimax_slam.pi.dll (0x180162340 .. 0x180162E8F, translation unit #2
// of chunk c14_interface_api; c00: TU41, the static unique_ptr 0x18047EDF0 is its only global with
// an atexit entry).  Exported by name through src/pimax_slam.def.
// TODO(verify): original file name (the TU has no __FILE__ string; its string pool is
// "Pimax_SLAM_2.0.0.1", "camera0..3 empty image", the shutter warning, "calibration_directory not exist").
// Public records: include/pimax_slam.h.
#include <cstdint>
#include <fstream>
#include <memory>
#include <vector>

#include <opencv2/core.hpp>

#include "common/logger.h"
#include "interface/slam_manager.h"
#include "pimax_slam.h"

// off_18046A1D8: a (non-const, constant-initialised) data pointer to the version string
// (0x1803B8300); HeadsetGetVersion loads it.  External linkage so that the load is kept.
// TODO(verify) name.
const char* g_pimax_slam_version = "Pimax_SLAM_2.0.0.1";

namespace {

// "Block" in the IDA database (0x18046A1E0 region). std::unique_ptr: HeadsetInitialImpl uses
// reset(new ...) (SLAMManager has EIGEN_MAKE_ALIGNED_OPERATOR_NEW -> malloc(0xB20) + alignment
// assert, ~SLAMManager + free), HeadsetReleaseImpl uses reset().
std::unique_ptr<pimax::totem::SLAMManager> g_system;

// dword_18047EDF8: counts every HeadsetPutCameraImage call made while a system exists
// (never reset, also not by HeadsetReleaseImpl / a new HeadsetInitialImpl).
int g_put_image_calls = 0;

}  // namespace

extern "C" {

// 0x180162340  (upstream: new)
int HeadsetGetGroundState(HeadsetGroundState* ground)
{
    if (!g_system)
        return -1;
    return g_system->GetGroundState(ground);   // 0x180167C60
}

// 0x180162360  (upstream: new)
int HeadsetGetLocModeState(uint8_t* state)
{
    if (!g_system)
        return -1;
    return g_system->GetLocModeState(state);   // 0x18016B860
}

// 0x180162380  (upstream: new)
const char* HeadsetGetVersion()
{
    return g_pimax_slam_version;
}

// 0x180162390  (upstream: new)
// The calibration path is opened with std::ifstream(path, ios::in, _SH_DENYNO) purely as an
// existence check (binary tests rdstate() != 0, i.e. !good()); the stream stays open until return.
// Unlike LedObjectPoseEstimator, the logger is initialised inside the SLAMManager ctor, and a second
// call while a system exists returns -1 before anything else.
int HeadsetInitialImpl(const char* calibration_path, const char* output_directory,
                       const char* vocabulary_path, const char* unused, uint8_t loc_mode)
{
    if (g_system)
        return -1;
    int ret;
    std::ifstream calib(calibration_path);
    if (!calib.good()) {
        LOGE("calibration_directory not exist\n");
        ret = -1;
    } else {
        g_system.reset(new pimax::totem::SLAMManager(calibration_path, output_directory,
                                                     vocabulary_path, unused, loc_mode));  // 0x1801663F0
        ret = 0;
    }
    return ret;
}

// 0x180162510  (upstream: new)
// Reads image_queue_.size() (deque _Mysize at +1168, truncated to 32 bit) without locking.
unsigned int HeadsetLeftImageNumber()
{
    if (!g_system)
        return 0;
    return static_cast<unsigned int>(g_system->image_queue_.size());
}

// 0x180162530  (upstream: new)
// Each header is copied, wrapped in cv::Mat(rows=+24, cols=+20, CV_8UC1, data=+32, step=(int)+28)
// and cloned. An image must be non-empty AND exactly 480x640, else "cameraN empty image" (WARN) and
// -1. Only camera 3's shutter value is checked (< 100000 -> WARN, frame still used). The bundle
// timestamp is the 4th header's +0. The two uint32 vectors are passed BY VALUE (copies made
// right-to-left: gains first).
int HeadsetPutCameraImage(const HeadsetImage* cam0, const HeadsetImage* cam1,
                          const HeadsetImage* cam2, const HeadsetImage* cam3)
{
    if (!g_system)
        return -1;
    if (++g_put_image_calls < 6)
        return -1;

    int ret;
    std::vector<cv::Mat> images;
    std::vector<uint32_t> shutters;
    std::vector<uint32_t> gains;
    images.reserve(4);
    shutters.reserve(4);
    gains.reserve(4);
    cv::Mat image;
    HeadsetImage header;

    header = *cam0;
    image = cv::Mat(header.height, header.width, CV_8UC1, const_cast<void*>(header.data),
                    (size_t)header.stride).clone();
    if (!image.empty() && image.rows == 480 && image.cols == 640) {
        images.push_back(image);
        shutters.push_back(header.shutter_speed_ns);
        gains.push_back(header.gain);

        header = *cam1;
        image = cv::Mat(header.height, header.width, CV_8UC1, const_cast<void*>(header.data),
                        (size_t)header.stride).clone();
        if (!image.empty() && image.rows == 480 && image.cols == 640) {
            images.push_back(image);
            shutters.push_back(header.shutter_speed_ns);
            gains.push_back(header.gain);

            header = *cam2;
            image = cv::Mat(header.height, header.width, CV_8UC1, const_cast<void*>(header.data),
                            (size_t)header.stride).clone();
            if (!image.empty() && image.rows == 480 && image.cols == 640) {
                images.push_back(image);
                shutters.push_back(header.shutter_speed_ns);
                gains.push_back(header.gain);

                header = *cam3;
                image = cv::Mat(header.height, header.width, CV_8UC1, const_cast<void*>(header.data),
                                (size_t)header.stride).clone();
                if (!image.empty() && image.rows == 480 && image.cols == 640) {
                    images.push_back(image);
                    const uint32_t shutter_speed_ns = header.shutter_speed_ns;
                    shutters.push_back(header.shutter_speed_ns);
                    gains.push_back(header.gain);
                    if (shutter_speed_ns < 100000)
                        LOGW("-------- image shutter_speed_ns is %u, too short! \n", shutter_speed_ns);
                    g_system->PutImages(images, shutters, gains, header.timestamp);  // 0x1801680D0
                    ret = 0;
                } else {
                    LOGW("camera3 empty image\n");
                    ret = -1;
                }
            } else {
                LOGW("camera2 empty image\n");
                ret = -1;
            }
        } else {
            LOGW("camera1 empty image\n");
            ret = -1;
        }
    } else {
        LOGW("camera0 empty image\n");
        ret = -1;
    }
    return ret;
}

// 0x180162D70  (upstream: new)
// The 40-byte record is copied and passed by value (hidden pointer to the copy).
int HeadsetPutHmdImuData(const HeadsetImuData* imu, HeadsetPose* pose)
{
    if (!g_system)
        return -1;
    return g_system->PutImu(*imu, pose);   // 0x180168640
}

// 0x180162DF0  (upstream: new)
int HeadsetReleaseImpl()
{
    g_system.reset();   // ~SLAMManager 0x180167320 + free
    return 0;
}

// 0x180162E30  (upstream: new)
int HeadsetSetDeviceEnable(bool enable)
{
    if (!g_system)
        return -1;
    g_system->SetDeviceEnable(enable);   // 0x1801691D0 (takes const bool&)
    return 0;
}

// 0x180162E60  (upstream: new)
int HeadsetSetTrackingMode(uint8_t mode)
{
    if (!g_system)
        return -1;
    g_system->SetTrackingMode(mode);   // 0x18016A220 (takes const uint8_t&)
    return 0;
}

}  // extern "C"
