// pimax_slam.pi.dll -- src/interface/svo_factory.h  (from draft c14)
//
// Factory / configuration functions (translation unit #1 of chunk
// c14_interface_api, code 0x180159E10 .. 0x18016233F).
// TODO(verify): original file name. The TU follows ceres_backend_factory.cpp and contains the
// svo_ros svo_factory.cpp descendants (options loaders with all ROS params replaced by literals),
// the device_calibration.xml parser and the Fisheye62 camera instantiations. String pool
// 0x1803B7AB0 .. 0x1803B82A8.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <cstdint>

#include <Eigen/Core>

#include "common/camera_fwd.h"

namespace pimax {
namespace totem {

class FrameProcessor;
class ImuProcessor;
class LoopClosing;
struct BaseOptions;
struct DepthFilterOptions;
struct DetectorOptions;
struct InitializationOptions;
struct ReprojectorOptions;
struct StereoTriangulationOptions;
struct FeatureTrackerOptions;
struct LoopClosureOptions;   // 656 bytes, ctor 0x180159E10, dtor 0x18015A4F0 (layout: notes)

// ImuParams (96 bytes, FrameProcessorBase+536) is declared in frontend/frame_processor_base.h.
struct ImuParams;

namespace factory {

// 0x18015AEC0
std::vector<Eigen::Vector3d> FibonacciSphere(double radius, int n);
// 0x18015B0E0
void SplitPath(const std::string& full_path, std::string& dir, std::string& file);

BaseOptions loadBaseOptions(bool forward_default, std::string trace_dir);   // 0x18015D6E0
DepthFilterOptions loadDepthFilterOptions();                                 // 0x18015DA90
DetectorOptions loadDetectorOptions();                                       // 0x18015DAD0
InitializationOptions loadInitializationOptions();                           // 0x18015DB40
LoopClosureOptions loadLoopClosureOptions(const char* vocabulary_path,
                                          const char* output_directory);     // 0x18015DB90
ReprojectorOptions loadReprojectorOptions();                                 // 0x18015E0B0
StereoTriangulationOptions loadStereoOptions();                              // 0x18015E130
FeatureTrackerOptions loadTrackerOptions();                                  // 0x18015E170

// 0x18015CBD0
std::shared_ptr<ImuProcessor> getImuProcessor(ImuParams params);
// 0x18015CEE0
std::shared_ptr<LoopClosing> getLoopClosingModule(const char* vocabulary_path,
                                                  const char* output_directory,
                                                  const std::shared_ptr<vk::cameras::NCamera>& cam,
                                                  std::string device_sn, uint8_t loc_mode);
// 0x1801621A0
void setInitialPose(FrameProcessor& vo);
// 0x18015E250 -- device_calibration.xml parser + makeStereo.
std::shared_ptr<FrameProcessor> makeFrameProcessor(std::string calibration_file,
                                                   std::string output_directory,
                                                   double* cam_imu_delta, std::string* device_sn,
                                                   int* device_type, const uint8_t& loc_mode);

}  // namespace factory
}  // namespace totem
}  // namespace pimax
