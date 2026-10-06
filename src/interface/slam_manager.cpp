// pimax_slam.pi.dll -- src/interface/slam_manager.cpp  (draft c14_interface_api/interface/slam_manager.cpp)
//
// Headset tracking system behind the Headset* C API (translation unit #3 of chunk
// c14_interface_api, code 0x180162E90 .. 0x18016F0EF).
// TODO(verify): original class / file name, see slam_manager.h.
// Integration (B4): the 3-DoF types come from sensor_fusion/three_dof_tracker.h (A1) -- c14's local
// State/ImuSample/Config are gone (Config == the tracker's last ImuSample, ImuSample is
// {acc, gyr, t}); PoseState / FrameProcessorBase member names are A2's (see the comments at the
// use sites); Logger::Init 0x1801692E0 lives in common/logger.cpp (A1).
//
// Heritage: svo_ros SvoInterface (ctor wiring of the factories, setImuPrior, image loop), heavily
// rewritten by Pimax: C-API driven queues instead of ROS callbacks, a dedicated image thread,
// IMU-rate high-frequency pose prediction (Propagate / PredictPose) with output smoothing, and a
// 3DoF fallback (ThreeDof tracker) when vision is not valid or the app selects kDof3.
//
// Logger: LOGD/LOGI/LOGW/LOGE = pimax::g_logger at 0x18046A000 (0x18000C120 / 0x18000F500 /
// 0x18000F6A0 / 0x18000C2C0).
#include "interface/slam_manager.h"

#include <windows.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include <Eigen/QR>

#include "ceres_backend/ceres_backend_interface.hpp"
#include "common/frame.h"
#include "common/logger.h"
#include "direct/depth_filter.h"
#include "frontend/frame_processor.h"          // FrameProcessor
#include "frontend/imu_processor.h"            // ImuProcessor
#include "interface/ceres_backend_factory.h"   // ceres_backend_factory::makeBackend 0x180158F70
#include "interface/svo_factory.h"             // factory::* (TU #1 of this chunk)
#include "loop_closing/loop_closing.h"         // LoopClosing (complete type for shared_ptr assignment)
#include "pimax_slam.h"                        // HeadsetGetVersion (interface/headset_api.cpp)
#include "sensor_fusion/deque_holder.h"        // DequeHolder (c14 "Unknown2840")
#include "sensor_fusion/three_dof_tracker.h"   // ThreeDof::ThreeDofTracker

namespace pimax {
namespace totem {

namespace {

// 0x18016EED0  (upstream: new)
// Sets the Win32 thread description if Kernel32 exports SetThreadDescription (looked up once,
// function-local static). UTF-8 -> UTF-16 via MultiByteToWideChar(CP_UTF8).
void SetThreadName(const std::string& name)
{
    if (name.empty())
        return;
    using SetThreadDescriptionFn = HRESULT(WINAPI*)(HANDLE, PCWSTR);
    static SetThreadDescriptionFn set_thread_description = []() -> SetThreadDescriptionFn {
        HMODULE kernel32 = GetModuleHandleW(L"Kernel32.dll");
        return kernel32 ? reinterpret_cast<SetThreadDescriptionFn>(
                              GetProcAddress(kernel32, "SetThreadDescription"))
                        : nullptr;
    }();
    if (!set_thread_description)
        return;
    std::wstring wide;
    if (!name.empty()) {
        const int size = MultiByteToWideChar(CP_UTF8, 0, name.data(), (int)name.size(), nullptr, 0);
        if (size > 0) {
            std::wstring tmp(size, L'\0');   // 0x1801661E0
            MultiByteToWideChar(CP_UTF8, 0, name.data(), (int)name.size(), &tmp[0], size);
            wide = std::move(tmp);
        }
    }
    set_thread_description(GetCurrentThread(), wide.c_str());
}

// Part of 0x18016D1D0 (float precision on purpose: the argument is converted with cvtpd2ps,
// the series runs in single precision and the result is widened again).
inline float Sinc(float x)
{
    if (std::fabs(x) > 1e-6f)
        return std::sin(x) / x;   // sinf
    const float x2 = x * x;
    return 1.0f - x2 * (1.0f / 6.0f) + x2 * x2 * (1.0f / 120.0f) - x2 * x2 * x2 * (1.0f / 5040.0f);
}

}  // namespace

// 0x1801663F0  (upstream: SvoInterface::SvoInterface, heavily modified)
// `unused` (4th C-API argument) is never read. Logger output goes to <output_directory>.
SLAMManager::SLAMManager(const char* calibration_directory, const char* output_directory,
                         const char* vocabulary_path, const char* unused, uint8_t loc_mode)
{
    (void)unused;
    g_logger.Init(std::string(output_directory));   // 0x1801692E0
    LOGW("Headset tracking version %s \n", HeadsetGetVersion());
    LOGW("output_directory %s\n", output_directory);
    LOGW("input vocabulary full path %s\n", vocabulary_path);
    LOGW("input calibration_directory full path %s\n", calibration_directory);

    // 0x18015E250: parses device_calibration.xml, detects the device type, builds the camera rig,
    // the options and the FrameProcessor (strings passed by value).
    frame_processor_ = factory::makeFrameProcessor(std::string(calibration_directory),
                                                   std::string(output_directory),
                                                   &cam_imu_delta_, &device_sn_, &device_type_,
                                                   loc_mode);
    ncam_ = frame_processor_->cams_;                               // FrameProcessor +280
    imu_processor_ = factory::getImuProcessor(frame_processor_->imu_params_);   // 0x18015CBD0, +536 by value
    frame_processor_->imu_handler_ = imu_processor_;              // FrameProcessor +520
    LOGW("frame_processor_->loc_mode_ %d\n", loc_mode);
    frame_processor_->loc_mode_ = loc_mode;                       // FrameProcessor +464
    backend_ = ceres_backend_factory::makeBackend(ncam_);         // 0x180158F70
    frame_processor_->setBundleAdjuster(backend_);                // 0x1801274A0
    backend_->setImu(imu_processor_);                             // 0x180015A80
    frame_processor_->lc_ = factory::getLoopClosingModule(vocabulary_path, output_directory, ncam_,
                                                          device_sn_, loc_mode);  // 0x18015CEE0, +1624
    process_thread_ = std::thread(&SLAMManager::ProcessLoop, this);
    three_dof_ = std::unique_ptr<ThreeDof::ThreeDofTracker>(new ThreeDof::ThreeDofTracker());  // 0x1801A6AC0
    three_dof_->SetBias(frame_processor_->imu_params_.wBias,
                        frame_processor_->imu_params_.aBias);   // 0x1801A7A50 (fp +536 / +548 -> tracker +100 / +112)
    unknown_2840_ = std::unique_ptr<DequeHolder>();   // move-assigned from an empty temporary
    imu_vec_.clear();
}

// 0x180167320  (upstream: SvoInterface::~SvoInterface, modified)
// Member destruction follows (reverse declaration order). Destroying a joinable std::thread calls
// terminate(): process_thread_ is always joined by Stop() first; unused_thread_ never runs.
SLAMManager::~SLAMManager()
{
    Stop();
    g_logger.Close();   // inline: if (m_file.is_open()) m_file.close();
}

// 0x18016A280  (upstream: new)
// Also used by SetTrackingMode(kDof3). Without a running thread only the flag is set.
void SLAMManager::Stop()
{
    {
        std::unique_lock<std::mutex> lock(quit_mutex_);
        quit_ = true;
        lock.unlock();
    }
    if (process_thread_.joinable()) {
        frame_processor_->depth_filter_->stopThread();   // 0x1800A2730, FrameProcessor +504
        if (backend_)
            backend_->reset();   // 0x1800149F0: body only logs LOGI("Backend: Reset\n") (c14 "quitThread")
        LOGI("quit backend thread\n");
        image_cv_.notify_all();
        process_thread_.join();
        LOGI("thread_ join\n");
    }
}

// 0x18016A3B0  (upstream: new)
void SLAMManager::ClearImageQueue()
{
    std::unique_lock<std::mutex> lock(image_mutex_);
    while (!image_queue_.empty())
        image_queue_.pop_front();
    lock.unlock();
}

// 0x180167C60  (upstream: new)
int SLAMManager::GetGroundState(HeadsetGroundState* ground)
{
    ground->valid = false;
    ground->point0[0] = 0.0f;
    ground->point0[1] = 0.0f;
    ground->point0[2] = 0.0f;
    ground->point1[0] = 0.0f;
    ground->point1[1] = 0.0f;
    ground->point1[2] = 0.0f;
    bool enabled;
    {
        std::lock_guard<std::mutex> lock(enable_mutex_);
        enabled = enabled_;
    }
    if (!enabled)
        return -1;
    if (tracking_mode_ != 1)
        return 0;
    std::unique_lock<std::mutex> lock(frame_processor_->plane_mutex_);    // FrameProcessor +1336
    ground->valid = frame_processor_->plane_valid_;                       // +1416
    ground->point0[0] = frame_processor_->plane_imu_pos_[0];              // +1420
    ground->point0[1] = frame_processor_->plane_imu_pos_[1];
    ground->point0[2] = frame_processor_->plane_imu_pos_[2];
    ground->point1[0] = frame_processor_->plane_center_[0];               // +1432
    ground->point1[1] = frame_processor_->plane_center_[1];
    ground->point1[2] = frame_processor_->plane_center_[2];
    lock.unlock();
    return 1;
}

// 0x18016B860  (upstream: new)
int SLAMManager::GetLocModeState(uint8_t* state)
{
    *state = 0;
    bool enabled;
    {
        std::lock_guard<std::mutex> lock(enable_mutex_);
        enabled = enabled_;
    }
    if (!enabled)
        return -1;
    if (tracking_mode_ != 1)
        return 0;
    std::unique_lock<std::mutex> lock(frame_processor_->loc_mutex_);         // FrameProcessor +1448
    *state = frame_processor_->loc_state_;                                  // +1528
    lock.unlock();
    return 1;
}

// 0x1801680D0  (upstream: new; replaces SvoInterface::stereoCallback queueing)
// exposures / gains are by-value parameters (destroyed by the callee).
void SLAMManager::PutImages(const std::vector<cv::Mat>& images, std::vector<uint32_t> exposures,
                            std::vector<uint32_t> gains, uint64_t timestamp)
{
    bool enabled;
    {
        std::lock_guard<std::mutex> lock(enable_mutex_);
        enabled = enabled_;
    }
    if (!enabled)
        return;
    std::unique_lock<std::mutex> lock(image_mutex_);
    std::vector<cv::Mat> bundle_images;
    std::vector<uint32_t> bundle_exposures;
    std::vector<uint32_t> bundle_gains;
    bundle_images = images;
    bundle_exposures = exposures;
    bundle_gains = gains;
    image_queue_.emplace_back(timestamp, bundle_images, bundle_exposures, bundle_gains);
    lock.unlock();
    image_cv_.notify_all();
}

// 0x180167DB0  (upstream: new) -- body of the wait predicate in ProcessLoop.
// Called with wait_mutex_ held; takes image_mutex_ itself.
bool SLAMManager::PopImageBundle(std::vector<cv::Mat>& images, std::vector<uint32_t>& exposures,
                                 std::vector<uint32_t>& gains, uint64_t& timestamp, int& left)
{
    images.clear();
    std::unique_lock<std::mutex> lock(image_mutex_);
    if (image_queue_.empty()) {
        lock.unlock();
        return false;
    }
    ImageBundle bundle = std::move(image_queue_.front());
    image_queue_.pop_front();
    if (!image_queue_.empty()) {
        if (image_queue_.size() <= 5)
            LOGI("image leave size %d\n", image_queue_.size());
        else
            LOGW("image leave size %d\n", image_queue_.size());
    }
    left = static_cast<int>(image_queue_.size());
    lock.unlock();
    images.swap(bundle.images);
    exposures.swap(bundle.exposures);
    gains.swap(bundle.gains);
    timestamp = bundle.timestamp;
    return true;
}

// 0x18016EC80  (upstream: SvoInterface::setImuPrior, modified)
// FrameProcessor +2980 (set_reset_, c14 need_reset_) plays the role of !hasStarted() (set by the
// reset logic and at thread start).
// The incremental-prior branch has no log line in this build; set_initial_attitude_from_gravity_
// is gone.
bool SLAMManager::SetImuPrior(const int64_t& timestamp_ns)
{
    if (imu_processor_ && frame_processor_->set_reset_) {
        Eigen::Quaterniond R_imu_world;
        if (!imu_processor_->getInitialAttitude(timestamp_ns * 1e-9, R_imu_world))   // 0x18012A2B0
            return false;
        LOGI("Set initial orientation from accelerometer measurements.\n");
        frame_processor_->setRotationPrior(R_imu_world);   // 0x180128210
    } else if (imu_processor_ && frame_processor_->last_frames_) {   // getLastFrames()
        Eigen::Quaterniond R_lastimu_newimu;
        if (imu_processor_->getRelativeRotationPrior(                        // 0x18012AB60
                frame_processor_->last_frames_->getMinTimestampNanoseconds() * 1e-9,
                timestamp_ns * 1e-9, false, R_lastimu_newimu))
            frame_processor_->setRotationIncrementPrior(R_lastimu_newimu);   // 0x1801280A0
    }
    return true;
}

// 0x18016A460  (upstream: SvoInterface::stereoLoop/stereoCallback, rewritten)
// Thread entry (std::thread(&SLAMManager::ProcessLoop, this)). wait_mutex_ stays locked for the
// whole loop (only released inside wait_for); quit_mutex_ is released while a bundle is handled.
void SLAMManager::ProcessLoop()
{
    HANDLE thread = GetCurrentThread();
    SetThreadPriority(thread, THREAD_PRIORITY_TIME_CRITICAL);
    SetThreadPriorityBoost(thread, TRUE);
    SetProcessPriorityBoost(GetCurrentProcess(), TRUE);
    SetProcessWorkingSetSize(GetCurrentProcess(), (SIZE_T)-1, (SIZE_T)-1);
    SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED);
    SetThreadAffinityMask(thread, 0x3C);
    SetThreadName(std::string("SLAMManager_Thread"));

    std::unique_lock<std::mutex> quit_lock(quit_mutex_);
    std::unique_lock<std::mutex> lock(wait_mutex_);
    frame_processor_->set_reset_ = true;    // FrameProcessor +2980
    while (!quit_) {
        quit_lock.unlock();
        std::vector<cv::Mat> images;
        std::vector<uint32_t> exposures;
        std::vector<uint32_t> gains;
        uint64_t timestamp;
        int left = 0;
        if (!image_cv_.wait_for(lock, std::chrono::milliseconds(2), [&] {
                return PopImageBundle(images, exposures, gains, timestamp, left);
            })) {
            quit_lock.lock();
            continue;
        }
        ++frame_count_;
        if (device_type_ == 2 && frame_count_ < 25 && !frame_processor_->loc_mode_) {
            quit_lock.lock();
            LOGW("super jump first 25 frames, %d\n", frame_count_);
            continue;
        }
        if (!imu_processor_->waitTill(timestamp * 1e-9, 0.033)) {   // 0x18012B140
            quit_lock.lock();
            LOGW("not get imu data\n");
            continue;
        }
        if (frame_processor_->imu_not_initialized_ && !SetImuPrior(timestamp)) {   // FrameProcessor +3456
            quit_lock.lock();
            LOGW("Could not align gravity!\n");
            continue;
        }
        ImuMeasurements imu_measurements;
        if (!imu_processor_->getMeasurementsContainingEdges(timestamp * 1e-9, imu_measurements,
                                                             true)   // 0x18012A950
            || imu_measurements.size() < 20) {
            ClearImageQueue();
            quit_lock.lock();
            LOGW("no enough imu data\n");
            continue;
        }
        const auto start = std::chrono::system_clock::now();
        Eigen::Quaternionf ground_q = Eigen::Quaternionf::Identity();
        bool ground_valid = false;
        double ground_t = 0.0;
        if (frame_processor_->imu_not_initialized_) {
            std::lock_guard<std::mutex> ground_lock(ground_mutex_);
            ground_q = ground_q_;
            ground_valid = ground_valid_;
            ground_t = ground_t_;
        }
        frame_processor_->addImageBundle(images, exposures, gains, timestamp, imu_measurements, left,
                                         ground_q, ground_valid, ground_t);   // 0x1800FB650
        if (frame_processor_->set_reset_)
            ClearImageQueue();
        const auto end = std::chrono::system_clock::now();
        LOGI("image process time %.2f ms\n",
             std::chrono::duration<double>(end - start).count() * 1000.0);
        quit_lock.lock();
        Sleep(0);
    }
}

// 0x1801691D0  (upstream: new)
// QUIRK: disabling sets quit_ (the image thread leaves its loop for good); enabling only clears
// the flag again, it does not restart the thread.
void SLAMManager::SetDeviceEnable(const bool& enable)
{
    {
        std::lock_guard<std::mutex> lock(enable_mutex_);
        if (enabled_ == enable)
            return;
        enabled_ = enable;
    }
    if (!enable) {
        std::unique_lock<std::mutex> lock(quit_mutex_);
        quit_ = true;
        lock.unlock();
    } else {
        std::unique_lock<std::mutex> lock(quit_mutex_);
        quit_ = false;
        lock.unlock();
    }
}

// 0x18016A220  (upstream: new). No locking. QUIRK: kDof3 stops the image thread permanently.
void SLAMManager::SetTrackingMode(const uint8_t& mode)
{
    if (mode == tracking_mode_)
        return;
    tracking_mode_ = mode;
    if (mode == 1) {
        LOGI("SetTrackingMode kDof6\n");
    } else {
        LOGI("SetTrackingMode kDof3\n");
        Stop();
    }
}

// 0x18016B210  (upstream: new)
// Runs the IMU-only tracker on one sample. The tracker's state (+128) and last sample (+208; c14
// "config") are read without taking its mutex.  The sample is built as {acc (+20 of the
// measurement), gyr (+8), t} (A1 verified the stack layout).
void SLAMManager::UpdateThreeDof(ThreeDof::State& out, const ImuMeasurement& m)
{
    ThreeDof::State state;                  // 0x1801662D0
    std::vector<ThreeDof::ImuSample> imus;
    state = three_dof_->state_;
    ThreeDof::ImuSample last_imu = three_dof_->last_imu_;
    ThreeDof::ImuSample sample{m.linear_acceleration_, m.angular_velocity_, m.timestamp_};
    imus.push_back(sample);
    three_dof_->Propagate(last_imu, imus, state, &out);   // 0x1801A7540
}

// 0x180168640  (upstream: SvoInterface::imuCallback, rewritten)
// Returns 1 when *out was written, 0 when the sample was dropped, -1 when disabled.
int SLAMManager::PutImu(HeadsetImuData imu, HeadsetPose* out)
{
    bool enabled;
    {
        std::lock_guard<std::mutex> lock(enable_mutex_);
        enabled = enabled_;
    }
    if (!enabled)
        return -1;

    // IMU rate detection over the first 10 samples: < 900 Hz -> offset 6.5 ms, else 3.5 ms.
    if (imu_rate_mode_ == 0) {
        const double t = imu.timestamp * 1e-9;
        if (first_imu_t_ < 0.0) {
            first_imu_t_ = t;
            imu_count_ = 1;
            return 0;
        }
        const size_t n = imu_count_++;
        if (imu_count_ < 10)
            return 0;
        const double dt = t - first_imu_t_;
        if (dt <= 0.0) {
            first_imu_t_ = t;
            imu_count_ = 1;
            return 0;
        }
        imu_rate_mode_ = n / dt < 900.0 ? 2 : 1;
        imu_time_offset_ = n / dt < 900.0 ? 0.0065 : 0.0035;
    }

    const Eigen::Vector3f acc(imu.acc[0], imu.acc[1], imu.acc[2]);
    const ImuMeasurement m(imu.timestamp * 1e-9 - imu_time_offset_,
                           Eigen::Vector3f(imu.gyr[0], imu.gyr[1], imu.gyr[2]), acc);
    if (last_imu_.timestamp_ >= m.timestamp_) {
        LOGW("Dropping imu data: timestamp older than last frame.\n");
        return 0;
    }
    if (std::fabs(acc.norm()) < 0.001) {
        LOGE("error imu data, acc norm < 0.001\n");
        return 0;
    }

    if (imu_vec_.size() > 3000) {
        imu_vec_.erase(imu_vec_.begin(), imu_vec_.begin() + imu_vec_.size() / 2);   // 0x18016B7E0
        imu_processor_->limitMeasurementsSize();   // 0x18012B090 (c14 "trimMeasurements"): halves its deque when >= 3000
        static int s_overflow_count = 0;      // dword_18047EE1C
        ++s_overflow_count;
        if (s_overflow_count % 10 == 0)
            LOGW("imu_vec size > 3000\n");
    }

    ThreeDof::State three_dof_state;   // 0x1801662D0
    bool updated = false;
    static thread_local HeadsetPose pose;   // TLS +784 (zero-initialised)

    if (tracking_mode_ == 1) {
        imu_processor_->addImuMeasurement(m);   // 0x180129F40
        imu_vec_.push_back(m);
        PoseState low_pose;   // 0x1800E2B10
        {
            std::lock_guard<std::mutex> lock(frame_processor_->output_mutex_);  // FrameProcessor +696
            low_pose = frame_processor_->output_;                               // FrameProcessor +784
            if (low_pose.timestamp > last_low_pose_.timestamp) {
                updated = true;
                last_low_pose_ = low_pose;
            }
            if (low_pose.confidence == 1.0f && std::fabs(m.timestamp_ - low_pose.timestamp) > 4.0) {
                if (updated)
                    LOGE("m.t - low_pose.t > 1s\n");
                low_pose.confidence = 0.0f;
            }
        }
        if (low_pose.confidence == 1.0f) {
            // 6DoF output.
            no_6dof_pose_yet_ = false;
            if (!low_pose.tracking_state)
                smooth_initialized_ = false;
            pose = PredictPose(low_pose, imu_vec_, updated, predicted_state_);
            UpdateThreeDof(three_dof_state, m);
            Eigen::Quaternionf q6(pose.qw, pose.qx, pose.qy, pose.qz);
            q6.normalize();
            q_3dof_to_6dof_ = (q6 * three_dof_state.q.inverse()).normalized();
            last_6dof_pose_ = pose;
        } else {
            // Vision not valid: 3DoF orientation aligned to the last 6DoF orientation.
            smooth_initialized_ = false;
            q_filter_initialized_ = false;
            q_history_.clear();
            UpdateThreeDof(three_dof_state, m);
            const Eigen::Quaternionf q = q_3dof_to_6dof_ * three_dof_state.q;
            pos_filter_initialized_ = false;
            pos_offset_.setZero();
            if (three_dof_->ready_) {
                std::lock_guard<std::mutex> lock(ground_mutex_);
                ground_q_ = q;
                ground_valid_ = true;
                ground_t_ = m.timestamp_;
            }
            pose.qx = q.x();
            pose.qy = q.y();
            pose.qz = q.z();
            pose.qw = q.w();
            if (no_6dof_pose_yet_) {
                const Eigen::Isometry3d T = low_pose.T_map_world * low_pose.T_world_imu;   // 0x1801239E0
                pose.position[0] = (float)T.translation()[0];
                pose.position[1] = (float)T.translation()[1];
                pose.position[2] = (float)T.translation()[2];
            } else {
                pose.position[0] = last_6dof_pose_.position[0];
                pose.position[1] = last_6dof_pose_.position[1];
                pose.position[2] = last_6dof_pose_.position[2];
            }
            pose.angular_velocity[0] = m.angular_velocity_[0];
            pose.angular_velocity[1] = m.angular_velocity_[1];
            pose.angular_velocity[2] = m.angular_velocity_[2];
            pose.velocity[0] = pose.velocity[1] = pose.velocity[2] = 0.0f;
            pose.angular_acceleration[0] = pose.angular_acceleration[1] = pose.angular_acceleration[2] = 0.0f;
            pose.linear_acceleration[0] = pose.linear_acceleration[1] = pose.linear_acceleration[2] = 0.0f;
            pose.timestamp = static_cast<uint64_t>(m.timestamp_ * 1000000000.0);
            pose.confidence = static_cast<float>(three_dof_->ready_);
            pose.tracking_state = low_pose.tracking_state;
            pose.tracking_flag = low_pose.status;
            pose.is_6dof = false;
        }
    } else {
        // kDof3 mode: raw 3DoF tracker orientation.
        UpdateThreeDof(three_dof_state, m);
        pose.qx = three_dof_state.q.x();
        pose.qy = three_dof_state.q.y();
        pose.qz = three_dof_state.q.z();
        pose.qw = three_dof_state.q.w();
        pose.position[0] = pose.position[1] = pose.position[2] = 0.0f;
        pose.angular_velocity[0] = m.angular_velocity_[0];
        pose.angular_velocity[1] = m.angular_velocity_[1];
        pose.angular_velocity[2] = m.angular_velocity_[2];
        pose.velocity[0] = pose.velocity[1] = pose.velocity[2] = 0.0f;
        pose.angular_acceleration[0] = pose.angular_acceleration[1] = pose.angular_acceleration[2] = 0.0f;
        pose.linear_acceleration[0] = pose.linear_acceleration[1] = pose.linear_acceleration[2] = 0.0f;
        pose.timestamp = static_cast<uint64_t>(m.timestamp_ * 1000000000.0);
        pose.confidence = 1.0f;
        pose.tracking_state = 4;
        pose.tracking_flag = 0;
        pose.is_6dof = false;
    }
    last_imu_ = m;
    *out = pose;
    return 1;
}

// 0x18016B940  (upstream: new)
// High-rate 6DoF output: forward-propagates the latest vision state with the buffered IMU samples
// (at most 99 ms past the vision timestamp), blends position/velocity 90/10 with the previous
// smoothed state, and hides jumps at vision updates with decaying offsets (position: *0.99 per
// IMU sample; orientation: slerp(0.99) towards identity per sample).
HeadsetPose SLAMManager::PredictPose(PoseState& low_pose, std::vector<ImuMeasurement>& imus,
                                     bool& updated, PoseState& state)
{
    HeadsetPose out;
    if (updated) {
        imus.erase(std::remove_if(imus.begin(), imus.end(),
                                  [&low_pose](const ImuMeasurement& m) {
                                      return low_pose.timestamp > m.timestamp_;
                                  }),
                   imus.end());
    }

    PoseState local = low_pose;
    if (low_pose.backend_static) {   // c14 "reset"
        local.timestamp = imus.back().timestamp_;
        local.velocity.setConstant(0.0);
        local.angular_velocity.setConstant(0.0);
        local.linear_acceleration.setConstant(0.0);
        local.unknown_392.setConstant(0.0);
        state = local;
    } else if (updated) {
        if (local.confidence > 0.0f && imus.size() > 1) {
            Propagate(imus, local, local.timestamp,
                      std::min(local.timestamp + 0.099, imus.back().timestamp_));
            if (smooth_initialized_) {
                local.T_world_imu.translation() = 0.9 * local.T_world_imu.translation()
                                                 + (1.0 - 0.9) * smooth_state_.T_world_imu.translation();
                local.velocity = 0.9 * local.velocity + (1.0 - 0.9) * smooth_state_.velocity;
                smooth_state_ = local;
            } else {
                smooth_state_ = local;
                smooth_initialized_ = true;
            }
        }
        state = local;
    } else {
        if (local.confidence > 0.0f && imus.size() > 1) {
            Propagate(imus, state, state.timestamp,
                      std::min(local.timestamp + 0.099, imus.back().timestamp_));
            if (smooth_initialized_) {
                state.T_world_imu.translation() = 0.9 * state.T_world_imu.translation()
                                                 + (1.0 - 0.9) * smooth_state_.T_world_imu.translation();
                state.velocity = 0.9 * state.velocity + (1.0 - 0.9) * smooth_state_.velocity;
                smooth_state_ = state;
            }
        }
        local = state;
    }

    // Position with decaying jump offset.
    const Eigen::Isometry3d T_world_imu = local.T_map_world * local.T_world_imu;   // 0x1801239E0
    const Eigen::Vector3d t = T_world_imu.translation();
    if (pos_filter_initialized_) {
        if (updated)
            pos_offset_ = pos_last_ - t;
        pos_offset_ = pos_offset_ * 0.99;
    } else {
        pos_offset_.setConstant(0.0);
        pos_last_ = t;
        pos_filter_initialized_ = true;
    }
    pos_last_ = t + pos_offset_;
    out.position[0] = (float)pos_last_[0];
    out.position[1] = (float)pos_last_[1];
    out.position[2] = (float)pos_last_[2];

    // Orientation with decaying jump offset.
    Eigen::Quaterniond q(T_world_imu.linear());   // 0x1800B5D60
    q.normalize();
    if (q_filter_initialized_) {
        if (updated)
            q_offset_ = (q_last_ * q.inverse()).normalized();
        q_offset_ = Eigen::Quaterniond::Identity().slerp(0.99, q_offset_).normalized();   // 0x180166050
    } else {
        q_offset_ = Eigen::Quaterniond::Identity();
        q_last_ = q;
        q_filter_initialized_ = true;
    }
    Eigen::Quaterniond q_out = (q_offset_ * q).normalized();
    q_last_ = q_out;

    // Angular-velocity extrapolation from a 20 ms window of output orientations (quadratic fit,
    // ColPivHouseholderQR). DEAD CODE in this build: angular_velocity_fit_ (+600) is never set.
    if (angular_velocity_fit_) {
        q_history_.emplace_back(local.timestamp, q_out);
        while (q_history_.size() > 4 && local.timestamp - 0.02 > q_history_.front().t)
            q_history_.pop_front();
        while (q_history_.size() > 32)
            q_history_.pop_front();
        const size_t n = q_history_.size();
        if (n >= 4) {
            const Eigen::Quaterniond q_ref = q_out;
            Eigen::MatrixXd A(n, 3);
            Eigen::MatrixXd B(n, 3);
            for (size_t i = 0; i < n; ++i) {
                const double dt = q_history_[i].t - local.timestamp;
                A(i, 0) = 1.0;
                A(i, 1) = dt;
                A(i, 2) = dt * dt;
                Eigen::Quaterniond dq = q_ref.conjugate() * q_history_[i].q;
                if (dq.w() < 0.0)
                    dq.coeffs() *= -1.0;
                const Eigen::AngleAxisd aa(dq);   // 0x180163150
                B.row(i) = (aa.angle() * aa.axis()).transpose();
            }
            const Eigen::MatrixXd X = A.colPivHouseholderQr().solve(B);   // 0x180162E90 / 0x18016EAD0
            const Eigen::Vector3d omega = X.row(1).transpose();
            const double angle = omega.norm();
            if (angle > 1e-9) {
                const Eigen::Vector3d axis = omega / angle;
                Eigen::Quaterniond dq;
                dq.w() = std::cos(angle * 0.5);
                dq.vec() = axis * std::sin(angle * 0.5);
                q_out = (q_ref * dq).normalized();
                q_last_ = q_out;
                q_history_.back().q = q_out;
            }
        }
    } else if (!q_history_.empty()) {
        q_history_.clear();
    }

    out.qw = (float)q_out.w();
    out.qx = (float)q_out.x();
    out.qy = (float)q_out.y();
    out.qz = (float)q_out.z();
    const Eigen::Vector3d velocity = local.T_map_world.linear() * local.velocity;
    out.velocity[0] = (float)velocity[0];
    out.velocity[1] = (float)velocity[1];
    out.velocity[2] = (float)velocity[2];
    const Eigen::Vector3d acceleration = local.T_map_world.linear() * local.linear_acceleration;
    out.linear_acceleration[0] = (float)acceleration[0];
    out.linear_acceleration[1] = (float)acceleration[1];
    out.linear_acceleration[2] = (float)acceleration[2];
    out.angular_velocity[0] = (float)local.angular_velocity[0];
    out.angular_velocity[1] = (float)local.angular_velocity[1];
    out.angular_velocity[2] = (float)local.angular_velocity[2];
    out.angular_acceleration[0] = 0.0f;
    out.angular_acceleration[1] = 0.0f;
    out.angular_acceleration[2] = 0.0f;
    out.tracking_state = low_pose.tracking_state;   // NOTE: from low_pose, not from `local`
    out.tracking_flag = low_pose.status;
    out.timestamp = static_cast<uint64_t>(local.timestamp * 1000000000.0);
    out.confidence = local.confidence;
    out.is_6dof = true;
    if (!low_pose.tracking_state)
        out.is_6dof = false;
    return out;
}

// 0x18016D1D0  (upstream: new; VINS-Mono-style mid-point integration, Pimax variant)
// Propagates `s` from t_start to t_end with the buffered IMU samples. Differences to VINS
// midPointIntegration: the acceleration term uses (R0 + R1) * 0.5*(a0 + a1) (not R0*a0 + R1*a1),
// the half-angle quaternion uses a single-precision sinc, the first step is re-interpolated to
// t_start and the last one to t_end. All per-step scratch lives in SLAMManager members (+680..+1039).
// QUIRK: the log passes two doubles to "%lu".
void SLAMManager::Propagate(const std::vector<ImuMeasurement>& imus, PoseState& s,
                            const double& t_start, const double& t_end)
{
    const Eigen::Vector3d g = s.gravity;
    if (imus.front().timestamp_ > t_end) {
        LOGW("front t %lu > t_end %lu\n", imus.front().timestamp_, t_end);
        return;
    }
    const Eigen::Vector3d p = s.T_world_imu.translation();
    const Eigen::Matrix3d R = s.T_world_imu.linear();
    const Eigen::Quaterniond q0(R);   // 0x180020AF0
    double sum_dt = 0.0;
    Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d delta_v = Eigen::Vector3d::Zero();
    Eigen::Vector3d delta_p = Eigen::Vector3d::Zero();
    bool first = false;
    Eigen::Vector3d un_acc_last = imus.back().linear_acceleration_.cast<double>() - s.acc_bias;
    double t0 = t_start;
    for (size_t i = 1; i < imus.size(); ++i) {
        gyr_0_ = imus[i - 1].angular_velocity_.cast<double>() - s.gyr_bias;
        acc_0_ = imus[i - 1].linear_acceleration_.cast<double>() - s.acc_bias;
        gyr_1_ = imus[i].angular_velocity_.cast<double>() - s.gyr_bias;
        acc_1_ = imus[i].linear_acceleration_.cast<double>() - s.acc_bias;
        double t1 = imus[i].timestamp_;
        double dt = t1 - t0;
        if (t1 > t_end) {
            t1 = t_end;
            dt = t_end - t0;
            const double r = dt / (imus[i].timestamp_ - imus[i - 1].timestamp_);
            gyr_1_ = (1.0 - r) * gyr_0_ + r * gyr_1_;
            acc_1_ = (1.0 - r) * acc_0_ + r * acc_1_;
        }
        if (1e-6 > dt)
            continue;
        sum_dt += dt;
        if (!first) {
            first = true;
            const double r = dt / (t1 - imus[i - 1].timestamp_);
            gyr_0_ = r * gyr_0_ + (1.0 - r) * gyr_1_;
            acc_0_ = r * acc_0_ + (1.0 - r) * acc_1_;
        }
        un_gyr_ = 0.5 * (gyr_0_ + gyr_1_);
        const double half_theta = un_gyr_.norm() * 0.5 * dt;
        const float sinc = Sinc(static_cast<float>(half_theta));
        const double c = std::cos(half_theta);
        delta_q_.vec() = (sinc * (dt * 0.5)) * un_gyr_;
        delta_q_.w() = c;
        result_delta_q_ = (delta_q * delta_q_).normalized();
        delta_R0_ = delta_q.toRotationMatrix();
        delta_R1_ = result_delta_q_.toRotationMatrix();
        un_acc_ = 0.5 * (acc_0_ + acc_1_);
        const Eigen::Vector3d delta_v_new = delta_v + (delta_R0_ + delta_R1_) * (dt * 0.5) * un_acc_;
        const Eigen::Vector3d dp = dt * delta_v + (delta_R0_ + delta_R1_) * (dt * 0.25 * dt) * un_acc_;
        delta_p += dp;
        s.angular_velocity = gyr_1_;
        un_acc_last = un_acc_;
        delta_q = result_delta_q_;
        delta_v = delta_v_new;
        t0 = t1;
        if (t1 == t_end)
            break;
    }
    const double half_T2 = sum_dt * 0.5 * sum_dt;
    s.T_world_imu.translation() = p + s.velocity * sum_dt + R * delta_p - half_T2 * g;
    s.T_world_imu.linear() = (q0 * delta_q).normalized().toRotationMatrix();
    s.velocity += R * delta_v - sum_dt * g;
    s.linear_acceleration = s.T_world_imu.linear() * un_acc_last - g;
    s.timestamp = t_end;
}

}  // namespace totem
}  // namespace pimax
