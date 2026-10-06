// pimax_slam.pi.dll -- src/interface/slam_manager.h  (from draft c14)
//
// The 0xB20-byte headset tracking system behind the Headset* C API
// (translation unit #3 of chunk c14_interface_api, code 0x180162E90 .. 0x18016F0EF).
//
// TODO(verify): class and file name. No RTTI/vtable. The only naming hints are the processing
// thread's description "SLAMManager_Thread" (0x18016A460) and the svo_ros SvoInterface heritage
// (setImuPrior -> "Set initial orientation from accelerometer measurements.", "Could not align
// gravity!"). The string pool of the TU starts at 0x1803B83E0 ("a_position"...).
//
// Layout (sizeof == 0xB20 == 2848, allocated with malloc via EIGEN_MAKE_ALIGNED_OPERATOR_NEW):
//   offset  type                                  member                       evidence
//   +0      bool                                  smooth_initialized_          PredictPose 0x18016B940, PutImu
//   +16     PoseState (448)                       smooth_state_                PredictPose (copy target)
//   +464    bool                                  q_filter_initialized_        PredictPose, PutImu
//   +480    Eigen::Quaterniond                    q_offset_                    ctor identity, PredictPose
//   +512    Eigen::Quaterniond                    q_last_                      ctor identity, PredictPose
//   +544    bool                                  pos_filter_initialized_      PredictPose, PutImu
//   +552    Eigen::Vector3d                       pos_offset_                  ctor zero
//   +576    Eigen::Vector3d                       pos_last_                    ctor zero
//   +600    bool                                  angular_velocity_fit_        ctor false, never set -> dead code
//   +608    std::deque<OrientationSample>         q_history_                   48-byte elements
//   +648    int                                   imu_rate_mode_               0 unknown, 1 >= 900 Hz, 2 < 900 Hz
//   +656    double                                imu_time_offset_ = 0.007     PutImu
//   +664    double                                first_imu_t_ = -1.0          PutImu
//   +672    size_t                                imu_count_ = 0               PutImu
//   +680    Eigen::Vector3d                       un_gyr_                      Propagate 0x18016D1D0 (scratch)
//   +704    Eigen::Vector3d                       un_acc_                      Propagate
//   +728    (8 bytes padding)
//   +736    Eigen::Quaterniond                    delta_q_                     Propagate
//   +768    Eigen::Quaterniond                    result_delta_q_              Propagate
//   +800    Eigen::Matrix3d                       delta_R0_                    Propagate
//   +872    Eigen::Matrix3d                       delta_R1_                    Propagate
//   +944    Eigen::Vector3d                       gyr_0_, +968 acc_0_, +992 gyr_1_, +1016 acc_1_
//   +1040   std::shared_ptr<FrameProcessor>       frame_processor_
//   +1056   std::shared_ptr<vk::cameras::NCamera> ncam_
//   +1072   std::shared_ptr<ImuProcessor>         imu_processor_
//   +1088   std::shared_ptr<CeresBackendInterface> backend_
//   +1104   std::thread                           process_thread_              ProcessLoop 0x18016A460
//   +1120   std::thread                           unused_thread_               only checked in the dtor
//   +1136   std::deque<ImageBundle>               image_queue_                 80-byte elements
//   +1176   std::vector<ImuMeasurement>           imu_vec_                     32-byte elements
//   +1200   HeadsetPose (104)                     last_6dof_pose_
//   +1304   bool                                  no_6dof_pose_yet_ = true
//   +1312   Eigen::Quaternionf                    q_3dof_to_6dof_ = Identity
//   +1328   std::mutex                            image_mutex_
//   +1408   std::condition_variable               image_cv_
//   +1480   bool                                  quit_ = false
//   +1488   std::mutex                            quit_mutex_
//   +1568   std::mutex                            wait_mutex_
//   +1648   std::mutex                            enable_mutex_
//   +1728   bool                                  enabled_ = true
//   +1729   uint8_t                               tracking_mode_ = 1 (kDof6)
//   +1736   double                                cam_imu_delta_ = 0           written by the calibration parser (time delta)
//   +1744   std::string                           device_sn_                   deviceUID from device_calibration.xml
//   +1776   int                                   device_type_ = -1            written by the calibration parser
//   +1780   int                                   frame_count_ = 0
//   +1792   PoseState                             last_low_pose_
//   +2240   PoseState                             predicted_state_
//   +2688   ImuMeasurement                        last_imu_
//   +2720   std::mutex                            ground_mutex_
//   +2800   Eigen::Quaternionf                    ground_q_ = Identity
//   +2816   bool                                  ground_valid_ = false
//   +2824   double                                ground_t_ = 0
//   +2832   std::unique_ptr<ThreeDof::ThreeDofTracker> three_dof_              0x2A0 bytes, ctor 0x1801A6AC0
//   +2840   std::unique_ptr<DequeHolder>          unknown_2840_                dtor 0x1801A3E00 (A1 name)
#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdDeque>
#include <opencv2/core.hpp>

#include "pimax_slam.h"
#include "common/camera_fwd.h"
#include "common/imu_calibration.h"              // ImuMeasurement (32-byte float sample)
#include "frontend/frame_processor_base.h"       // PoseState
#include "sensor_fusion/deque_holder.h"          // pimax::DequeHolder (= c14 "Unknown2840")
#include "sensor_fusion/three_dof_tracker.h"     // ThreeDof::ThreeDofTracker / State / ImuSample

namespace pimax {

// ThreeDof::State / ImuSample / ThreeDofTracker come from sensor_fusion/three_dof_tracker.h (A1):
// State is 80 bytes (16-aligned), ImuSample is {acc +0, gyr +12, t +24} (c14 had t first), and
// c14's "Config" (tracker +208) is the tracker's last ImuSample.  SetBias(bg, ba) 0x1801A7A50,
// Propagate 0x1801A7540.

namespace totem {

class FrameProcessor;
class ImuProcessor;
class CeresBackendInterface;

// ImuMeasurement: common/imu_calibration.h (members timestamp_ / angular_velocity_ /
// linear_acceleration_; c14 spelled them without the trailing underscore).
// PoseState: frontend/frame_processor_base.h (448 B, ctor 0x1800E2B10, operator= 0x180167790).
// c14 -> final field names: T_odom_imu -> T_world_imu, T_world_odom -> T_map_world,
// tracking_flag -> status, reset -> backend_static (c09's evidence: copy of
// CeresBackendInterface+160); the others are unchanged.

// 80 bytes (deque element, operator new(0x50)).  Implicit dtor 0x180167260 (3 vectors).
struct ImageBundle {
    uint64_t timestamp;                 // +0  ns
    std::vector<cv::Mat> images;        // +8
    std::vector<uint32_t> exposures;    // +32 (HeadsetImage +8, shutter_speed_ns)
    std::vector<uint32_t> gains;        // +56 (HeadsetImage +12)
    ImageBundle(uint64_t t, const std::vector<cv::Mat>& imgs, const std::vector<uint32_t>& exp,
                const std::vector<uint32_t>& gain)
        : timestamp(t), images(imgs), exposures(exp), gains(gain) {}
};

// 48 bytes (deque element of q_history_; _Allocate<16>(0x30)).
struct alignas(16) OrientationSample {
    double t;                  // +0
    Eigen::Quaterniond q;      // +16
    OrientationSample(double t_, const Eigen::Quaterniond& q_) : t(t_), q(q_) {}
};

class SLAMManager {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 0x1801663F0
    SLAMManager(const char* calibration_directory, const char* output_directory,
                const char* vocabulary_path, const char* unused, uint8_t loc_mode);
    // 0x180167320
    ~SLAMManager();

    int GetGroundState(HeadsetGroundState* ground);                       // 0x180167C60
    int GetLocModeState(uint8_t* state);                                  // 0x18016B860
    void PutImages(const std::vector<cv::Mat>& images, std::vector<uint32_t> exposures,
                   std::vector<uint32_t> gains, uint64_t timestamp);      // 0x1801680D0
    int PutImu(HeadsetImuData imu, HeadsetPose* pose);                    // 0x180168640
    void SetDeviceEnable(const bool& enable);                             // 0x1801691D0
    void SetTrackingMode(const uint8_t& mode);                            // 0x18016A220

    // members ------------------------------------------------------------------------------
    bool smooth_initialized_ = false;                                     // +0
    PoseState smooth_state_;                                              // +16
    bool q_filter_initialized_ = false;                                   // +464
    Eigen::Quaterniond q_offset_ = Eigen::Quaterniond::Identity();        // +480
    Eigen::Quaterniond q_last_ = Eigen::Quaterniond::Identity();          // +512
    bool pos_filter_initialized_ = false;                                 // +544
    Eigen::Vector3d pos_offset_ = Eigen::Vector3d::Zero();                // +552
    Eigen::Vector3d pos_last_ = Eigen::Vector3d::Zero();                  // +576
    bool angular_velocity_fit_ = false;                                   // +600
    std::deque<OrientationSample> q_history_;                             // +608
    int imu_rate_mode_ = 0;                                               // +648
    double imu_time_offset_ = 0.007;                                      // +656
    double first_imu_t_ = -1.0;                                           // +664
    size_t imu_count_ = 0;                                                // +672
    Eigen::Vector3d un_gyr_;                                              // +680
    Eigen::Vector3d un_acc_;                                              // +704
    Eigen::Quaterniond delta_q_;                                          // +736
    Eigen::Quaterniond result_delta_q_;                                   // +768
    Eigen::Matrix3d delta_R0_;                                            // +800
    Eigen::Matrix3d delta_R1_;                                            // +872
    Eigen::Vector3d gyr_0_;                                               // +944
    Eigen::Vector3d acc_0_;                                               // +968
    Eigen::Vector3d gyr_1_;                                               // +992
    Eigen::Vector3d acc_1_;                                               // +1016
    std::shared_ptr<FrameProcessor> frame_processor_;                     // +1040
    std::shared_ptr<vk::cameras::NCamera> ncam_;                          // +1056
    std::shared_ptr<ImuProcessor> imu_processor_;                         // +1072
    std::shared_ptr<CeresBackendInterface> backend_;                      // +1088
    std::thread process_thread_;                                          // +1104
    std::thread unused_thread_;                                           // +1120
    std::deque<ImageBundle> image_queue_;                                 // +1136
    std::vector<ImuMeasurement> imu_vec_;                                 // +1176
    HeadsetPose last_6dof_pose_;                                          // +1200 (not initialised)
    bool no_6dof_pose_yet_ = true;                                        // +1304
    Eigen::Quaternionf q_3dof_to_6dof_ = Eigen::Quaternionf::Identity();  // +1312
    std::mutex image_mutex_;                                              // +1328
    std::condition_variable image_cv_;                                    // +1408
    bool quit_ = false;                                                   // +1480
    std::mutex quit_mutex_;                                               // +1488
    std::mutex wait_mutex_;                                               // +1568
    std::mutex enable_mutex_;                                             // +1648
    bool enabled_ = true;                                                 // +1728
    uint8_t tracking_mode_ = 1;                                           // +1729
    double cam_imu_delta_ = 0.0;                                          // +1736 (parser output, TODO(verify) meaning)
    std::string device_sn_;                                               // +1744 (deviceUID)
    int device_type_ = -1;                                                // +1776 (1 Crystal Light, 2 Crystal Super, ...)
    int frame_count_ = 0;                                                 // +1780
    PoseState last_low_pose_;                                             // +1792
    PoseState predicted_state_;                                           // +2240
    ImuMeasurement last_imu_{0.0, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero()};  // +2688
    std::mutex ground_mutex_;                                             // +2720
    Eigen::Quaternionf ground_q_ = Eigen::Quaternionf::Identity();        // +2800
    bool ground_valid_ = false;                                           // +2816
    double ground_t_ = 0.0;                                               // +2824
    std::unique_ptr<ThreeDof::ThreeDofTracker> three_dof_;                // +2832
    std::unique_ptr<DequeHolder> unknown_2840_;                           // +2840 (dtor 0x1801A3E00)

    static void layout_check();

private:
    void Stop();                                                          // 0x18016A280
    void ClearImageQueue();                                               // 0x18016A3B0
    void ProcessLoop();                                                   // 0x18016A460
    bool PopImageBundle(std::vector<cv::Mat>& images, std::vector<uint32_t>& exposures,
                        std::vector<uint32_t>& gains, uint64_t& timestamp,
                        int& left);                                       // 0x180167DB0 (lambda body)
    bool SetImuPrior(const int64_t& timestamp_ns);                        // 0x18016EC80
    void UpdateThreeDof(ThreeDof::State& out, const ImuMeasurement& m);   // 0x18016B210
    HeadsetPose PredictPose(PoseState& low_pose, std::vector<ImuMeasurement>& imus,
                            bool& updated, PoseState& state);             // 0x18016B940
    void Propagate(const std::vector<ImuMeasurement>& imus, PoseState& state,
                   const double& t_start, const double& t_end);           // 0x18016D1D0
};

static_assert(sizeof(ImageBundle) == 80, "");
static_assert(sizeof(OrientationSample) == 48, "");

inline void SLAMManager::layout_check()
{
    static_assert(sizeof(SLAMManager) == 0xB20, "sizeof(SLAMManager)");
    static_assert(offsetof(SLAMManager, smooth_state_) == 16, "");
    static_assert(offsetof(SLAMManager, q_filter_initialized_) == 464, "");
    static_assert(offsetof(SLAMManager, q_offset_) == 480, "");
    static_assert(offsetof(SLAMManager, q_last_) == 512, "");
    static_assert(offsetof(SLAMManager, pos_filter_initialized_) == 544, "");
    static_assert(offsetof(SLAMManager, pos_offset_) == 552, "");
    static_assert(offsetof(SLAMManager, pos_last_) == 576, "");
    static_assert(offsetof(SLAMManager, angular_velocity_fit_) == 600, "");
    static_assert(offsetof(SLAMManager, q_history_) == 608, "");
    static_assert(offsetof(SLAMManager, imu_rate_mode_) == 648, "");
    static_assert(offsetof(SLAMManager, imu_time_offset_) == 656, "");
    static_assert(offsetof(SLAMManager, first_imu_t_) == 664, "");
    static_assert(offsetof(SLAMManager, imu_count_) == 672, "");
    static_assert(offsetof(SLAMManager, un_gyr_) == 680, "");
    static_assert(offsetof(SLAMManager, un_acc_) == 704, "");
    static_assert(offsetof(SLAMManager, delta_q_) == 736, "");
    static_assert(offsetof(SLAMManager, result_delta_q_) == 768, "");
    static_assert(offsetof(SLAMManager, delta_R0_) == 800, "");
    static_assert(offsetof(SLAMManager, delta_R1_) == 872, "");
    static_assert(offsetof(SLAMManager, gyr_0_) == 944, "");
    static_assert(offsetof(SLAMManager, acc_1_) == 1016, "");
    static_assert(offsetof(SLAMManager, frame_processor_) == 1040, "");
    static_assert(offsetof(SLAMManager, ncam_) == 1056, "");
    static_assert(offsetof(SLAMManager, imu_processor_) == 1072, "");
    static_assert(offsetof(SLAMManager, backend_) == 1088, "");
    static_assert(offsetof(SLAMManager, process_thread_) == 1104, "");
    static_assert(offsetof(SLAMManager, unused_thread_) == 1120, "");
    static_assert(offsetof(SLAMManager, image_queue_) == 1136, "");
    static_assert(offsetof(SLAMManager, imu_vec_) == 1176, "");
    static_assert(offsetof(SLAMManager, last_6dof_pose_) == 1200, "");
    static_assert(offsetof(SLAMManager, no_6dof_pose_yet_) == 1304, "");
    static_assert(offsetof(SLAMManager, q_3dof_to_6dof_) == 1312, "");
    static_assert(offsetof(SLAMManager, image_mutex_) == 1328, "");
    static_assert(offsetof(SLAMManager, image_cv_) == 1408, "");
    static_assert(offsetof(SLAMManager, quit_) == 1480, "");
    static_assert(offsetof(SLAMManager, quit_mutex_) == 1488, "");
    static_assert(offsetof(SLAMManager, wait_mutex_) == 1568, "");
    static_assert(offsetof(SLAMManager, enable_mutex_) == 1648, "");
    static_assert(offsetof(SLAMManager, enabled_) == 1728, "");
    static_assert(offsetof(SLAMManager, tracking_mode_) == 1729, "");
    static_assert(offsetof(SLAMManager, cam_imu_delta_) == 1736, "");
    static_assert(offsetof(SLAMManager, device_sn_) == 1744, "");
    static_assert(offsetof(SLAMManager, device_type_) == 1776, "");
    static_assert(offsetof(SLAMManager, frame_count_) == 1780, "");
    static_assert(offsetof(SLAMManager, last_low_pose_) == 1792, "");
    static_assert(offsetof(SLAMManager, predicted_state_) == 2240, "");
    static_assert(offsetof(SLAMManager, last_imu_) == 2688, "");
    static_assert(offsetof(SLAMManager, ground_mutex_) == 2720, "");
    static_assert(offsetof(SLAMManager, ground_q_) == 2800, "");
    static_assert(offsetof(SLAMManager, ground_valid_) == 2816, "");
    static_assert(offsetof(SLAMManager, ground_t_) == 2824, "");
    static_assert(offsetof(SLAMManager, three_dof_) == 2832, "");
    static_assert(offsetof(SLAMManager, unknown_2840_) == 2840, "");
}

}  // namespace totem
}  // namespace pimax
