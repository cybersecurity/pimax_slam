// pimax_slam.pi.dll -- src/ceres_backend/reprojection_error_impl.hpp (from draft c00; header-only bodies)
// Pimax variant of svo_ceres_backend/reprojection_error_impl.hpp
//
// upstream-modified (see notes/c00_globals.md "ReprojectionError"):
//   * Euclidean landmark (3 params) instead of homogeneous (4): the point is transformed with
//     rotation matrices only, no 4x4 T_CS/T_SW; the "behind camera" test is on p_C.z.
//   * project3 is called with a stack Matrix<2,3> instead of `new Matrix<2,3>`; it is NOT called at
//     all when p_C.z < 0 (then `kp` stays uninitialised -- quirk kept: residual uses garbage).
//   * Residual/error cached in mutable members error_ (+0x60) and weighted_error_ (+0x50).
//   * EvaluateWithMinimalJacobians weights with the SCALAR square_root_information_(0,0);
//     EvaluateMinimal (Pimax, vtable slot 9) weights with the full 2x2 matrix.
//   * point_constant_ is ignored (upstream zeroes J1 when set).
//   * All minimal jacobians are computed first (comma-initialised 3x6 [C, -C*skew(p)] blocks), then
//     copied/lifted.
#pragma once

#include "ceres_backend/reprojection_error.hpp"

#include "ceres_backend/matrix_operations.hpp"   // skewSymmetric (0x180016490)

namespace pimax {
namespace totem {
namespace ceres_backend {

// 0x180008F50 -- Construct with measurement and information matrix.
// (SizedCostFunction<2,7,3,7> base ctor + ErrorInterface::use_minimal_jacobians_ = false, member
// default-ctor alignment asserts, disabled_/point_constant_ = false; setInformation inlined.)
inline ReprojectionError::ReprojectionError(
    CameraConstPtr camera_geometry,
    const measurement_t& measurement, const covariance_t& information)
{
  setMeasurement(measurement);
  setInformation(information);
  setCameraGeometry(camera_geometry);
  weighted_error_.setZero();
  error_.setZero();
}

// 0x180015C30 (also inlined into the ctor) -- upstream-identical.
inline void ReprojectionError::setInformation(
    const covariance_t& information)
{
  information_ = information;
  covariance_ = information.inverse();
  // perform the Cholesky decomposition on order to obtain the correct error weighting
  Eigen::LLT<Eigen::Matrix2d> lltOfInformation(information_);
  square_root_information_ = lltOfInformation.matrixL().transpose();
}

// 0x18000C460 -- Pimax: dispatch on ErrorInterface::use_minimal_jacobians_ (both calls virtual).
inline bool ReprojectionError::Evaluate(double const* const * parameters,
                                        double* residuals,
                                        double** jacobians) const
{
  if (use_minimal_jacobians_)
  {
    return EvaluateMinimal(parameters, residuals, jacobians);
  }
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// 0x18000C490
inline bool ReprojectionError::EvaluateWithMinimalJacobians(
    double const* const * parameters, double* residuals, double** jacobians,
    double** jacobians_minimal) const
{
  if (disabled_)
  {
    residuals[0] = 0;
    residuals[1] = 0;
    if (jacobians_minimal != nullptr)
    {
      if (jacobians_minimal[0] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor> >
            J0_minimal_mapped(jacobians_minimal[0]);
        J0_minimal_mapped.setZero();
      }
      if (jacobians_minimal[1] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> >
            J1_minimal_mapped(jacobians_minimal[1]);
        J1_minimal_mapped.setZero();
      }
      if (jacobians_minimal[2] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor> >
            J2_minimal_mapped(jacobians_minimal[2]);
        J2_minimal_mapped.setZero();
      }
    }

    if (jacobians != nullptr)
    {
      if (jacobians[0] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> >
            J0(jacobians[0]);
        J0.setZero();
      }
      if (jacobians[1] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> >
            J1(jacobians[1]);
        J1.setZero();
      }
      if (jacobians[2] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> >
            J2(jacobians[2]);
        J2.setZero();
      }
    }
    return true;
  }

  // pose: world to sensor transformation
  Eigen::Map<const Eigen::Vector3d> t_WS_W(&parameters[0][0]);
  Eigen::Map<const Eigen::Quaterniond> q_WS(&parameters[0][3]);

  // the point in world coordinates (Euclidean)
  Eigen::Map<const Eigen::Vector3d> p_W(&parameters[1][0]);

  // the sensor to camera transformation
  Eigen::Map<const Eigen::Vector3d> t_SC_S(&parameters[2][0]);
  Eigen::Map<const Eigen::Quaterniond> q_SC(&parameters[2][3]);

  // transform the point into the camera (toRotationMatrix = COMDAT 0x180016600)
  Eigen::Matrix3d C_SW = q_WS.toRotationMatrix().transpose();
  Eigen::Matrix3d C_CS = q_SC.toRotationMatrix().transpose();
  Eigen::Vector3d p_W_rel = p_W - t_WS_W;
  Eigen::Vector3d p_S = C_SW * p_W_rel;
  Eigen::Vector3d p_S_rel = p_S - t_SC_S;
  Eigen::Vector3d p_C = C_CS * p_S_rel;

  // calculate the reprojection error
  measurement_t kp;                       // NOTE: stays uninitialised if not projected
  Eigen::Matrix<double, 2, 3> J_proj;
  bool projection_ok = true;
  if (jacobians != nullptr || jacobians_minimal != nullptr)
  {
    if (p_C[2] < 0.0
        || !camera_geometry_->project3(p_C, &kp, &J_proj).isKeypointVisible())
    {
      projection_ok = false;
    }
  }
  else
  {
    if (p_C[2] < 0.0
        || !camera_geometry_->project3(p_C, &kp, nullptr).isKeypointVisible())
    {
      projection_ok = false;
    }
  }

  measurement_t error = measurement_ - kp;
  error_ = error;

  // weight (scalar!):
  weighted_error_ = square_root_information_(0, 0) * error;

  // assign:
  residuals[0] = weighted_error_[0];
  residuals[1] = weighted_error_[1];

  // calculate jacobians, if required
  if (jacobians != nullptr || jacobians_minimal != nullptr)
  {
    // check validity:
    bool valid = true;
    if (p_C[2] < 0.2 || !projection_ok)
    {
      valid = false;
    }

    Eigen::Matrix<double, 2, 3> Jh_weighted;
    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J1_minimal;
    Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J0_minimal;
    Eigen::Matrix<double, 2, 6, Eigen::RowMajor> J2_minimal;
    // TODO(verify): the binary tests (jacobians || jacobians_minimal) a second time here
    // (redundant); kept to mirror the control flow.
    if (jacobians != nullptr || jacobians_minimal != nullptr)
    {
      Jh_weighted = square_root_information_(0, 0) * J_proj;
      if (valid)
      {
        Eigen::Matrix<double, 3, 6> J;
        J << C_SW, -C_SW * skewSymmetric(p_W_rel);
        J0_minimal = Jh_weighted * C_CS * J;

        Eigen::Matrix3d C_CW = C_CS * C_SW;
        J1_minimal = -Jh_weighted * C_CW;

        Eigen::Matrix<double, 3, 6> J_ext;
        J_ext << C_CS, -C_CS * skewSymmetric(p_S_rel);
        J2_minimal = Jh_weighted * J_ext;
      }
      else
      {
        J0_minimal.setZero();
        J1_minimal.setZero();
        J2_minimal.setZero();
      }
    }

    // if requested, provide minimal Jacobians
    if (jacobians_minimal != nullptr)
    {
      if (jacobians_minimal[0] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor> >
            J0_minimal_mapped(jacobians_minimal[0]);
        J0_minimal_mapped = J0_minimal;
      }
      if (jacobians_minimal[1] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> >
            J1_minimal_mapped(jacobians_minimal[1]);
        J1_minimal_mapped = J1_minimal;
      }
      if (jacobians_minimal[2] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor> >
            J2_minimal_mapped(jacobians_minimal[2]);
        J2_minimal_mapped = J2_minimal;
      }
    }

    if (jacobians != nullptr)
    {
      if (jacobians[0] != nullptr)
      {
        // pseudo inverse of the local parametrization Jacobian (0x18008CC60):
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseLocalParameterization::liftJacobian(parameters[0], J_lift.data());

        // hallucinate Jacobian w.r.t. state
        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> >
            J0(jacobians[0]);
        J0 = J0_minimal * J_lift;
      }
      if (jacobians[1] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> >
            J1(jacobians[1]);
        J1 = J1_minimal;
      }
      if (jacobians[2] != nullptr)
      {
        Eigen::Matrix<double, 6, 7, Eigen::RowMajor> J_lift;
        PoseLocalParameterization::liftJacobian(parameters[2], J_lift.data());

        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> > J2(jacobians[2]);
        J2 = J2_minimal * J_lift;
      }
    }
  }

  return true;
}

// 0x18000E0A0 -- Pimax-new (primary vtable slot 9): minimal jacobians only, 2x2 weighting.
inline bool ReprojectionError::EvaluateMinimal(
    double const* const * parameters, double* residuals,
    double** jacobians_minimal) const
{
  if (disabled_)
  {
    residuals[0] = 0;
    residuals[1] = 0;
    if (jacobians_minimal != nullptr)
    {
      if (jacobians_minimal[0] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor> >
            J0_minimal_mapped(jacobians_minimal[0]);
        J0_minimal_mapped.setZero();
      }
      if (jacobians_minimal[1] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> >
            J1_minimal_mapped(jacobians_minimal[1]);
        J1_minimal_mapped.setZero();
      }
      if (jacobians_minimal[2] != nullptr)
      {
        Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor> >
            J2_minimal_mapped(jacobians_minimal[2]);
        J2_minimal_mapped.setZero();
      }
    }
    return true;
  }

  Eigen::Map<const Eigen::Vector3d> t_WS_W(&parameters[0][0]);
  Eigen::Map<const Eigen::Quaterniond> q_WS(&parameters[0][3]);
  Eigen::Map<const Eigen::Vector3d> p_W(&parameters[1][0]);
  Eigen::Map<const Eigen::Vector3d> t_SC_S(&parameters[2][0]);
  Eigen::Map<const Eigen::Quaterniond> q_SC(&parameters[2][3]);

  Eigen::Matrix3d C_SW = q_WS.toRotationMatrix().transpose();
  Eigen::Matrix3d C_CS = q_SC.toRotationMatrix().transpose();
  Eigen::Vector3d p_W_rel = p_W - t_WS_W;
  Eigen::Vector3d p_S = C_SW * p_W_rel;
  Eigen::Vector3d p_S_rel = p_S - t_SC_S;
  Eigen::Vector3d p_C = C_CS * p_S_rel;

  measurement_t kp;                       // NOTE: stays uninitialised if not projected
  Eigen::Matrix<double, 2, 3> J_proj;
  bool projection_ok = true;
  if (jacobians_minimal != nullptr)
  {
    if (p_C[2] < 0.0
        || !camera_geometry_->project3(p_C, &kp, &J_proj).isKeypointVisible())
    {
      projection_ok = false;
    }
  }
  else
  {
    if (p_C[2] < 0.0
        || !camera_geometry_->project3(p_C, &kp, nullptr).isKeypointVisible())
    {
      projection_ok = false;
    }
  }

  measurement_t error = measurement_ - kp;
  error_ = error;
  measurement_t weighted_error = square_root_information_ * error;
  weighted_error_ = weighted_error;
  residuals[0] = weighted_error[0];
  residuals[1] = weighted_error[1];

  if (jacobians_minimal != nullptr)
  {
    bool valid = true;
    if (p_C[2] < 0.2 || !projection_ok)
    {
      valid = false;
    }

    Eigen::Matrix<double, 2, 3> Jh_weighted = square_root_information_ * J_proj;

    if (jacobians_minimal[0] != nullptr)
    {
      Eigen::Matrix<double, 3, 6> J;
      J << C_SW, -C_SW * skewSymmetric(p_W_rel);
      Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor> >
          J0_minimal(jacobians_minimal[0]);
      if (valid)
      {
        J0_minimal = Jh_weighted * C_CS * J;
      }
      else
      {
        J0_minimal.setZero();
      }
    }
    if (jacobians_minimal[1] != nullptr)
    {
      Eigen::Matrix3d C_CW = C_CS * C_SW;
      Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> >
          J1_minimal(jacobians_minimal[1]);
      if (valid)
      {
        J1_minimal = -Jh_weighted * C_CW;
      }
      else
      {
        J1_minimal.setZero();
      }
    }
    if (jacobians_minimal[2] != nullptr)
    {
      Eigen::Matrix<double, 3, 6> J_ext;
      J_ext << C_CS, -C_CS * skewSymmetric(p_S_rel);
      Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor> >
          J2_minimal(jacobians_minimal[2]);
      if (valid)
      {
        J2_minimal = Jh_weighted * J_ext;
      }
      else
      {
        J2_minimal.setZero();
      }
    }
  }
  return true;
}

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
