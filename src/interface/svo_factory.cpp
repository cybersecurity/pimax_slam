// pimax_slam.pi.dll -- src/interface/svo_factory.cpp  (draft c14_interface_api/interface/svo_factory.cpp)
//
// Translation unit #1 of chunk c14_interface_api (code 0x180159E10 .. 0x18016233F, string pool
// 0x1803B7AB0 .. 0x1803B82A8).  Implicit members emitted here: LoopClosureOptions default ctor
// 0x180159E10 / dtor 0x18015A4F0 (default member initialisers in loop_closing/loop_closing.h),
// ImuParams::operator= 0x18015A8A0, the CameraGeometry<Pinhole<Equidistant>> (Fisheye62) virtuals
// (third_party/vikit).
// TODO(verify): original file name. Descends from svo_ros/src/svo_factory.cpp (all vk::param
// lookups replaced by literals), plus the Pimax device_calibration.xml parser (tinyxml2).
//
// The *Options structs are defined by the chunks that own them (frontend/direct/loop_closing);
// the values written here are listed with their byte offsets in notes/c14_interface_api.md.
// Pimax structs differ from upstream in places (float thresholds, extra fields); offsets are
// given where the upstream member name is uncertain.
#include "interface/svo_factory.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <tinyxml2.h>

#include <vikit/cameras/camera_geometry.h>
#include <vikit/cameras/equidistant_distortion.h>   // Pimax Fisheye62 version (thirdparty/vikit)
#include <vikit/cameras/ncamera.h>
#include <vikit/cameras/pinhole_projection.h>

#include "common/logger.h"
#include "common/transformation.h"      // Transformation = kindr QuatTransformation, Quaternion = Eigen
#include "frontend/frame_processor.h"
#include "frontend/frame_processor_base.h"   // BaseOptions, ImuParams
#include "frontend/imu_processor.h"
#include "frontend/initialization.h"
#include "frontend/reprojector.h"
#include "frontend/stereo_triangulation.h"
#include "direct/depth_filter.h"
#include "direct/feature_detection_types.h"
#include "tracker/feature_tracking_types.h"
#include "loop_closing/loop_closing.h"
#include "loop_closing/map_alignment.h"

namespace pimax {
namespace totem {
namespace factory {

using Camera = vk::cameras::CameraGeometry<
    vk::cameras::PinholeProjection<vk::cameras::EquidistantDistortion>>;

// 0x18015AEC0  (upstream: new)
// Fibonacci sphere: n points on a sphere of the given radius (golden-angle spiral).
std::vector<Eigen::Vector3d> FibonacciSphere(double radius, int n)
{
    std::vector<Eigen::Vector3d> points;
    if (n)
        points.reserve(n);
    const double golden_angle = (3.0 - std::sqrt(5.0)) * M_PI;
    for (int i = 0; i < n; ++i) {
        const double y = (1.0 - (i + i) / (n - 1.0)) * radius;   // (double)i + (double)i
        const double r = std::sqrt(radius * radius - y * y);
        const double theta = i * golden_angle;
        const double x = std::cos(theta) * r;
        const double z = std::sin(theta) * r;
        points.emplace_back(x, y, z);   // 0x180159250
    }
    return points;
}

// 0x18015B0E0  (upstream: new)
void SplitPath(const std::string& full_path, std::string& dir, std::string& file)
{
    char drive[_MAX_DRIVE];
    char directory[_MAX_DIR];
    char fname[_MAX_FNAME];
    char ext[_MAX_EXT];
    _splitpath(full_path.c_str(), drive, directory, fname, ext);
    dir = std::string(drive) + std::string(directory);
    file = std::string(fname) + std::string(ext);
}

// 0x18015D6E0  (upstream: svo::factory::loadBaseOptions, modified: literal values)
// Field offsets: BaseOptions (size >= 250), see notes. The kfselect_criterion string is built and
// never compared (the ROS lookup was removed): FORWARD is set unconditionally.
BaseOptions loadBaseOptions(bool forward_default, std::string trace_dir)
{
    BaseOptions o;                       // Pimax defaults: max_n_kfs 200, quality_max_fts_drop 150,
                                         // use_imu true, ... (see notes)
    o.trace_dir = trace_dir;             // +168
    o.quality_min_fts = 50;              // +200
    o.quality_max_fts_drop = 100;        // +208
    o.relocalization_max_trials = 5;     // +216
    o.poseoptim_prior_lambda = 0.5;      // +152
    o.poseoptim_using_unit_sphere = true;   // +160
    o.img_align_prior_lambda_rot = 0.5;  // +120
    o.img_align_prior_lambda_trans = 0.0;   // +128
    o.structure_optimization_max_pts = 20;  // +164
    o.init_map_scale = 1.5;              // +80
    std::string default_kf_criterion = forward_default ? "FORWARD" : "DOWNLOOKING";
    o.kfselect_criterion = KeyframeCriterion::FORWARD;   // +8 (= 1)
    o.kfselect_min_dist = 0.12;          // +16
    o.kfselect_numkfs_upper_thresh = 140;   // +24
    o.kfselect_numkfs_lower_thresh = 60;    // +32
    o.kfselect_min_dist_metric = 0.1;    // +40
    o.kfselect_min_angle = 20.0;         // +48
    o.kfselect_min_disparity = 40.0;     // +64
    o.kfselect_min_num_frames_between_kfs = 2;   // +56
    o.kfselect_backend_max_time_sec = 3.0;       // +72
    o.img_align_max_level = 4;           // +96
    o.img_align_min_level = 2;           // +104
    o.img_align_robustification = false; // +112
    o.img_align_use_distortion_jacobian = false;   // +140
    o.img_align_est_illumination_gain = true;      // +141
    o.img_align_est_illumination_offset = true;    // +142
    o.poseoptim_thresh = 2.0;            // +144
    o.update_seeds_with_old_keyframes = true;      // +225
    o.use_async_reprojectors = true;     // +226
    o.trace_statistics = false;          // +227
    o.backend_scale_stable_thresh = 0.02;   // +232
    o.global_map_lc_timeout_sec_ = 2.0;  // +240
    o.grid_size = 20;                    // +248 (uint16) Pimax field (c14 "unknown_248")
    return o;
}

// 0x18015DA90  (upstream: svo::factory::loadDepthFilterOptions, modified)
DepthFilterOptions loadDepthFilterOptions()
{
    DepthFilterOptions o;
    o.seed_convergence_sigma2_thresh = 200.0f;       // +0 float (upstream double)
    o.mappoint_convergence_sigma2_thresh = 500.0f;   // +4 float
    o.use_inverse_depth = true;                      // +8
    o.max_search_level = 2;                          // +16
    o.verbose = false;                               // +24
    o.use_threaded_depthfilter = true;               // +25
    o.update_3d_point = true;                        // +26
    o.scan_epi_unit_sphere = true;                   // +27 (upstream default false)
    o.max_n_seeds_per_frame = 256;                   // +32
    o.max_map_seeds_per_frame = 200;                 // +40
    o.affine_est_offset = true;                      // +48
    o.affine_est_gain = false;                       // +49
    o.extra_map_points = false;                      // +50
    return o;
}

// 0x18015DAD0  (upstream: svo::factory::loadDetectorOptions, modified)
// Pimax DetectorOptions (80 bytes). +20 (4 bytes) is not written. TODO(verify) member names.
DetectorOptions loadDetectorOptions()
{
    DetectorOptions o;
    o.cell_size = 20;              // +0
    o.max_level = 2;               // +8
    o.min_level = 0;               // +12
    o.border = 8;                  // +16
    o.threshold_primary = 20.0;    // +24
    o.threshold_secondary = 10.0;  // +32
    o.disable_edgelets = false;    // +40 (byte; c14 "unknown_40")
    o.threshold_edgelet = 200.0;   // +48 (c14 "unknown_48")
    o.sampling_level = 0;          // +56 } one qword store of 0 in the binary
    o.level = 0;                   // +60 } (c14 "unknown_56")
    o.sec_grid_fineness = 1;       // +64
    o.threshold_shitomasi = 50.0;  // +72
    return o;
}

// 0x18015DB40  (upstream: svo::factory::loadInitializationOptions, modified; 64 bytes)
InitializationOptions loadInitializationOptions()
{
    InitializationOptions o;
    o.init_type = static_cast<InitializerType>(0);   // +0
    o.init_min_disparity = 30.0;           // +8
    o.init_disparity_pivot_ratio = 0.5;    // +16
    o.init_min_features = 45;              // +24
    o.init_min_features_factor = 2.0;      // +32
    o.init_min_tracked = 50;               // +40
    o.init_min_inliers = 70;               // +48
    o.reproj_error_thresh = 2.0;           // +56
    return o;
}

// 0x18015DB90  (upstream: svo::factory::loadLoopClosureOptions, modified)
// The vocabulary file name must be exactly "voc_GEN_8X4.dbow", otherwise loop closing is
// disabled. The map directory is <output_directory>/.
LoopClosureOptions loadLoopClosureOptions(const char* vocabulary_path, const char* output_directory)
{
    std::string full_path(vocabulary_path);
    std::string voc_name;
    std::string voc_path;
    SplitPath(full_path, voc_path, voc_name);
    LOGW("voc_path %s\n", voc_path.c_str());
    LOGW("voc_name %s\n", voc_name.c_str());
    std::string map_path = "/home/cc/tmp/vocabularies/";
    std::string image_log_base_path = "/home/cc/tmp/img_dir/";
    map_path = output_directory;
    map_path += "/";

    LoopClosureOptions o;   // 0x180159E10
    o.runlc = true;
    o.voc_name = "voc_GEN_8X4.dbow";
    if (o.voc_name != voc_name) {
        o.runlc = false;
        LOGE("input voc_name is not correct!\n");
    }
    o.voc_path = voc_path;
    o.alpha = 1.0;
    o.beta = 1.0;
    o.ignored_past_frames = 15;
    o.scale_ret_app = "None";
    o.bowthresh = 0.65;
    o.gv_3d_inlier_thresh = 0.4;
    o.min_num_3d = 10;
    o.orb_dist_thresh = 48;
    o.gv_2d_match_thresh = 0.1;
    o.use_opengv = false;
    o.enable_image_logging = false;
    o.image_log_base_path = image_log_base_path;
    o.proximity_dist_ratio = 0.01;
    o.proximity_offset = 0.2;
    o.global_map_type = "BuiltInPoseGraph";
    o.force_correction_dist_thresh_meter = 0.01;
    o.map_path = map_path;   // +512
    return o;
}

// 0x18015E0B0  (upstream: svo::factory::loadReprojectorOptions, modified; 96 bytes)
ReprojectorOptions loadReprojectorOptions()
{
    ReprojectorOptions o;
    o.max_n_features_per_frame = 100;       // +0
    o.max_map_features_per_frame = 100;     // +8
    o.cell_size = 100;                      // +16 TODO(verify)
    // A2 reconciliation: projectMapInFrame reads +32 as max_n_kfs, so the Pimax extra size_t is
    // +24 (c14 had max_n_kfs = 20 at +24 and unknown_32 = 19).
    o.pimax_unknown_24 = 20;                // +24 Pimax field (size_t)
    o.max_n_kfs = 19;                       // +32
    o.reproject_unconverged_seeds = true;   // +40
    o.max_unconverged_seeds_ratio = -1.0;   // +48
    o.min_required_features = 0;            // +56
    o.seed_sigma2_thresh = 200.0f;          // +64 float
    o.remove_unconstrained_points = true;   // +68
    o.affine_est_offset = true;             // +69
    o.affine_est_gain = false;              // +70
    o.max_fixed_landmarks = 50;             // +72
    o.max_n_global_kfs = 20;                // +80
    o.fixed_lm_grid_size = 50.0;            // +88
    return o;
}

// 0x18015E130  (upstream: svo::factory::loadStereoOptions, modified)
StereoTriangulationOptions loadStereoOptions()
{
    StereoTriangulationOptions o;
    o.triangulate_n_features = 120;   // +0
    o.mean_depth_inv = 0.3;           // +8
    o.min_depth_inv = 1.0;            // +16
    o.max_depth_inv = 0.05;           // +24
    return o;
}

// 0x18015E170  (upstream: svo::factory::loadTrackerOptions; values == upstream defaults)
FeatureTrackerOptions loadTrackerOptions()
{
    FeatureTrackerOptions o;
    o.klt_max_level = 4;
    o.klt_min_level = 0;
    o.klt_patch_sizes = {16, 16, 16, 8, 8};
    o.klt_max_iter = 30;
    o.klt_min_update_squared = 0.001;
    o.klt_template_is_first_observation = true;
    o.min_tracks_to_detect_new_features = 50;
    o.reset_before_detection = true;
    return o;
}

// 0x18015CBD0  (upstream: svo::factory::getImuHandler, modified)
// ImuParams passed by value. aBias goes to ImuProcessor +208 (acc_bias_) and wBias to +232
// (omega_bias_) -- consistent with the ImuProcessor layout of frontend/imu_processor.h (c10), so
// c14's "possible swap" TODO is resolved: no swap.
std::shared_ptr<ImuProcessor> getImuProcessor(ImuParams params)
{
    ImuCalibration imu_calib;
    imu_calib.delay_imu_cam = 0.0;
    imu_calib.max_imu_delta_t = 0.01;
    imu_calib.gyro_noise_density = 0.004;
    imu_calib.acc_noise_density = 0.02;
    imu_calib.imu_integration_sigma = 0.0;
    imu_calib.gyro_bias_random_walk_sigma = 0.0004;
    imu_calib.acc_bias_random_walk_sigma = 0.002;
    imu_calib.gravity_magnitude = 9.80667;
    imu_calib.omega_coriolis = Eigen::Vector3d::Zero();
    imu_calib.saturation_accel_max = 156.8;
    imu_calib.saturation_omega_max = 35.0;
    imu_calib.imu_rate = 1000.0;
    ImuInitialization imu_init;
    imu_init.velocity = Eigen::Vector3d::Zero();
    imu_init.omega_bias = Eigen::Vector3d::Zero();
    imu_init.acc_bias = Eigen::Vector3d::Zero();
    imu_init.velocity_sigma = 2.0;
    imu_init.omega_bias_sigma = 0.01;
    imu_init.acc_bias_sigma = 0.1;
    std::shared_ptr<ImuProcessor> imu_processor =
        std::make_shared<ImuProcessor>(imu_calib, imu_init);   // ctor 0x180129D60, 0x2A8 bytes
    imu_processor->acc_bias_ = params.aBias.cast<double>();     // ImuProcessor +208
    imu_processor->omega_bias_ = params.wBias.cast<double>();   // ImuProcessor +232
    return imu_processor;
}

// 0x18015CEE0  (upstream: svo::factory::getLoopClosingModule, modified)
std::shared_ptr<LoopClosing> getLoopClosingModule(const char* vocabulary_path,
                                                  const char* output_directory,
                                                  const std::shared_ptr<vk::cameras::NCamera>& cam,
                                                  std::string device_sn, uint8_t loc_mode)
{
    LoopClosureOptions lc_options = loadLoopClosureOptions(vocabulary_path, output_directory);
    if (!lc_options.runlc || !cam)
        return nullptr;
    std::stringstream ss;
    ss << lc_options.voc_path << lc_options.voc_name;
    std::string voc_file = ss.str();
    std::ifstream voc_stream(voc_file);
    const bool voc_ok = voc_stream.good();
    voc_stream.close();
    if (!voc_ok) {
        LOGE("Could not open voc file %s\n", voc_file.c_str());
        return nullptr;
    }
    std::shared_ptr<LoopClosing> lc =
        std::make_shared<LoopClosing>(lc_options, cam, device_sn,
                                      static_cast<LoopClosingMode>(loc_mode));   // 0x18017F730, 0xC10 bytes
    MapAlignmentOptions map_alignment_options;
    map_alignment_options.ransac3d_inlier_percent = 40.0;
    map_alignment_options.ransac3d_min_pts = 8;
    lc->map_alignment_se3_ = std::make_shared<MapAlignmentSE3>(map_alignment_options);   // 0x1801975B0
    return lc;
}

// 0x1801621A0  (upstream: svo::factory::setInitialPose, literal identity)
void setInitialPose(FrameProcessor& vo)
{
    // Quaternion = Eigen::Quaterniond; the (Eigen quaternion, position) kindr ctor runs the
    // checking RotationQuaternion ctor 0x1800089C0 as in the binary.
    Transformation T_world_imuinit(Quaternion(1.0, 0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0));
    vo.T_world_imuinit = T_world_imuinit;   // upstream inline setInitialImuPose(): FrameProcessorBase +400
}

// 0x18015E250  (upstream: new -- replaces loadCameraFromYaml/getImuHandler config + makeStereo)
// Parses device_calibration.xml (path = calibration_file), detects the device from deviceUID,
// reads the IMU state (SFConfig/Stateinit), builds 4 Fisheye62 cameras with FOV masks and the rig
// extrinsics, and creates the FrameProcessor. Both strings are by value.
std::shared_ptr<FrameProcessor> makeFrameProcessor(std::string calibration_file,
                                                   std::string output_directory,
                                                   double* cam_imu_delta, std::string* device_sn,
                                                   int* device_type, const uint8_t& loc_mode)
{
    LOGI("Parsing device_calibration.xml\n");
    tinyxml2::XMLDocument doc(true, tinyxml2::PRESERVE_WHITESPACE);   // 0x1801B0190
    doc.LoadFile(calibration_file.c_str());                            // 0x1801B1CC0
    ImuParams imu_params;                                              // 0x1800E1ED0
    std::vector<const char*> names = {"Calibration", "Rig"};
    tinyxml2::XMLElement* root = doc.FirstChildElement();
    *device_sn = std::string(root->Attribute("deviceUID"));
    LOGW("device sn: %s\n", device_sn->c_str());
    if (device_sn->substr(0, 3) == "P90" || device_sn->substr(0, 4) == "P330") {
        *device_type = 1;
        LOGW("This device is Pimax Crystal Light\n");
    } else if (device_sn->substr(0, 3) == "P40") {
        LOGW("This device is Pimax Crystal Super\n");
        *device_type = 2;
    } else if (device_sn->substr(0, 3) == "P51") {
        LOGW("This device is Pimax Crystal Dream Air\n");
        *device_type = 3;
    } else if (device_sn->substr(0, 3) == "P61") {
        LOGW("This device is Pimax Crystal Dream Air SE\n");
        *device_type = 4;
    } else {
        LOGW("This device is unknown device, can not support successfully\n");
    }

    tinyxml2::XMLElement* camera = root->FirstChildElement("Camera");
    Eigen::Isometry3d T_rig;   // last row (0,0,0,1) from the Transform ctor
    Eigen::Isometry3d T_bc;
    std::vector<Eigen::MatrixXd> unused_matrices;   // never used (destroyed at the end)
    std::vector<Eigen::VectorXd> unused_vectors;    // never used
    tinyxml2::XMLElement* sf_config = root->FirstChildElement("SFConfig");
    std::vector<float> ombc;
    std::vector<float> tbc;
    cv::Mat_<float> rvec(1, 3);
    cv::Mat R_cv = cv::Mat::zeros(3, 3, CV_64F);

    std::istringstream ombc_stream(std::string(sf_config->FirstChildElement("Stateinit")->Attribute("ombc")));
    ombc = std::vector<float>(std::istream_iterator<float>(ombc_stream), std::istream_iterator<float>());
    std::istringstream tbc_stream(std::string(sf_config->FirstChildElement("Stateinit")->Attribute("tbc")));
    tbc = std::vector<float>(std::istream_iterator<float>(tbc_stream), std::istream_iterator<float>());

    std::vector<float> a_bias;
    std::istringstream a_bias_stream(std::string(sf_config->FirstChildElement("Stateinit")->Attribute("aBias")));
    a_bias = std::vector<float>(std::istream_iterator<float>(a_bias_stream), std::istream_iterator<float>());
    imu_params.aBias << a_bias[0], a_bias[1], a_bias[2];

    std::vector<float> w_bias;
    std::istringstream w_bias_stream(std::string(sf_config->FirstChildElement("Stateinit")->Attribute("wBias")));
    w_bias = std::vector<float>(std::istream_iterator<float>(w_bias_stream), std::istream_iterator<float>());
    imu_params.wBias << w_bias[0], w_bias[1], w_bias[2];

    std::vector<float> ka;
    std::istringstream ka_stream(std::string(sf_config->FirstChildElement("Stateinit")->Attribute("ka")));
    ka = std::vector<float>(std::istream_iterator<float>(ka_stream), std::istream_iterator<float>());
    imu_params.ka << ka[0], ka[1], ka[2];

    std::vector<float> kg;
    std::istringstream kg_stream(std::string(sf_config->FirstChildElement("Stateinit")->Attribute("kg")));
    kg = std::vector<float>(std::istream_iterator<float>(kg_stream), std::istream_iterator<float>());
    imu_params.kg << kg[0], kg[1], kg[2];

    std::vector<float> na;
    std::istringstream na_stream(std::string(sf_config->FirstChildElement("Stateinit")->Attribute("na")));
    na = std::vector<float>(std::istream_iterator<float>(na_stream), std::istream_iterator<float>());
    imu_params.na << na[0], na[1], na[2];

    std::vector<float> ng;
    std::istringstream ng_stream(std::string(sf_config->FirstChildElement("Stateinit")->Attribute("ng")));
    ng = std::vector<float>(std::istream_iterator<float>(ng_stream), std::istream_iterator<float>());
    imu_params.ng << ng[0], ng[1], ng[2];

    std::vector<float> delta;
    std::istringstream delta_stream(std::string(sf_config->FirstChildElement("Stateinit")->Attribute("delta")));
    delta = std::vector<float>(std::istream_iterator<float>(delta_stream), std::istream_iterator<float>());
    *cam_imu_delta = delta[0];
    imu_params.delta = delta[0];

    // IMU <- reference camera extrinsics from Stateinit ombc (Rodrigues vector) / tbc.
    rvec << ombc[0], ombc[1], ombc[2];
    cv::Rodrigues(rvec, R_cv);
    Eigen::Matrix3d R_bc;
    cv::cv2eigen(R_cv, R_bc);   // 0x180159710
    Eigen::Vector3d t_bc;
    t_bc << tbc[0], tbc[1], tbc[2];
    T_bc.linear() = R_bc;
    T_bc.translation() = t_bc;

    std::vector<std::shared_ptr<vk::cameras::CameraGeometryBase>> cameras;
    vk::TransformationVector T_C_B;   // v309
    vk::TransformationVector T_B_C;   // v305
    const std::vector<Eigen::Vector3d> sphere = FibonacciSphere(10.0, 100000);
    for (int i = 0; i < 4; ++i) {
        std::cout << camera->Attribute("cam_name") << std::endl;
        for (size_t k = 0; k < 2; ++k) {
            if (k == 0) {
                tinyxml2::XMLElement* calib = camera->FirstChildElement(names[0]);
                std::vector<float> principal_point;
                std::istringstream pp_stream(std::string(calib->Attribute("principal_point")));
                std::copy(std::istream_iterator<float>(pp_stream), std::istream_iterator<float>(),
                          std::back_inserter(principal_point));
                std::vector<float> focal_length;
                std::istringstream fl_stream(std::string(calib->Attribute("focal_length")));
                std::copy(std::istream_iterator<float>(fl_stream), std::istream_iterator<float>(),
                          std::back_inserter(focal_length));
                std::vector<float> radial_distortion;
                std::istringstream rd_stream(std::string(calib->Attribute("radial_distortion")));
                std::copy(std::istream_iterator<float>(rd_stream), std::istream_iterator<float>(),
                          std::back_inserter(radial_distortion));
                vk::cameras::PinholeProjection<vk::cameras::EquidistantDistortion> projection(
                    focal_length[0], focal_length[1], principal_point[0], principal_point[1],
                    vk::cameras::EquidistantDistortion(radial_distortion[0], radial_distortion[1],
                                                       radial_distortion[2], radial_distortion[3],
                                                       radial_distortion[4], radial_distortion[5]));
                std::shared_ptr<Camera> cam = std::make_shared<Camera>(640, 480, projection);   // 0x1801B3C00
                cam->setLabel("camera " + std::to_string(cameras.size()));
                // FOV mask: every pixel hit by projecting the sphere points, dilated by 3 px.
                cv::Mat mask(480, 640, CV_8UC1, cv::Scalar(0));
                for (const Eigen::Vector3d& p : sphere) {
                    Eigen::Vector2d px;
                    cam->project3(p, &px, nullptr);
                    if (px[0] < 640.0 && px[0] > 0.0 && px[1] < 480.0 && px[1] > 0.0)
                        cv::circle(mask, cv::Point((int)px[0], (int)px[1]), 3, cv::Scalar(255.0), -1, 8, 0);
                }
                cam->setMask(mask);   // 0x1801B3F30
                cameras.push_back(cam);
            } else {
                tinyxml2::XMLElement* rig = camera->FirstChildElement(names[1]);
                std::vector<float> translation;
                std::vector<float> rotation;
                std::istringstream t_stream(std::string(rig->Attribute("translation")));
                translation = std::vector<float>(std::istream_iterator<float>(t_stream), std::istream_iterator<float>());
                std::istringstream r_stream(std::string(rig->Attribute("rowMajorRotationMat")));
                rotation = std::vector<float>(std::istream_iterator<float>(r_stream), std::istream_iterator<float>());
                Eigen::Vector3d t;
                t << translation[0], translation[1], translation[2];
                Eigen::Matrix3d R;
                R << rotation[0], rotation[1], rotation[2],
                     rotation[3], rotation[4], rotation[5],
                     rotation[6], rotation[7], rotation[8];
                T_rig.linear() = R;
                T_rig.translation() = t;
                const Eigen::Isometry3d T_b_c = T_bc * T_rig.inverse(Eigen::Isometry);   // 0x18015D410, 0x1801239E0
                const Eigen::Vector3d t_b_c = T_b_c.translation();
                // Transformation(const Eigen::Quaterniond&, const Vector3d&) 0x180022CE0 (checking
                // RotationQuaternion ctor 0x1800089C0); Eigen quaternion from matrix 0x1800B5D60.
                const Transformation T(Eigen::Quaterniond(T_b_c.linear()), t_b_c);
                T_C_B.push_back(T.inverse());   // 0x180013040
                T_B_C.push_back(T);
            }
        }
        camera = camera->NextSiblingElement("Camera");   // 0x1801B2190
    }

    std::shared_ptr<vk::cameras::NCamera> ncam =
        std::make_shared<vk::cameras::NCamera>(T_C_B, T_B_C, cameras, "PiMax");   // 0x180159A40
    if (ncam->numCameras() > 4)
        LOGW("Load more cameras not needed!\n");

    InitializationOptions init_options = loadInitializationOptions();
    init_options.init_type = static_cast<InitializerType>(0);   // upstream: kStereo
    ReprojectorOptions reprojector_options = loadReprojectorOptions();
    StereoTriangulationOptions stereo_options = loadStereoOptions();
    DetectorOptions detector_options = loadDetectorOptions();
    DepthFilterOptions depth_filter_options = loadDepthFilterOptions();
    std::shared_ptr<FrameProcessor> vo = std::make_shared<FrameProcessor>(   // 0x180159940
        loadBaseOptions(true, output_directory), depth_filter_options, detector_options,
        init_options, stereo_options, reprojector_options, loadTrackerOptions(), ncam,
        *device_sn, loc_mode);
    vo->imu_params_ = imu_params;   // +536 (0x18015A8A0)
    setInitialPose(*vo);
    return vo;
}

}  // namespace factory
}  // namespace totem
}  // namespace pimax
