// third_party/vikit/include/vikit/cameras/equidistant_distortion.h (original tree:
// thirdparty/vikit/vikit_cameras/include/vikit/cameras/equidistant_distortion.h) -- Pimax-modified
// vikit EquidistantDistortion ("Fisheye62": Kannala-Brandt k1..k4 + Brown tangential p1, p2).
//
// Reconstructed from the template instantiations emitted in the factory TU (chunk c14):
//   0x18015B760 distort(const Eigen::Vector2d&)
//   0x18015B960 jacobian(const Eigen::Vector2d&)
//   0x18015BD50 undistort(double& x, double& y, double fx, double fy)   <- Pimax signature
//   0x180161900 print (inlined into CameraGeometry::printParameters)
// Object layout (sizeof 72, inside PinholeProjection at +56, i.e. CameraGeometry +208):
//   +0 k1_  +8 k2_  +16 k3_  +24 k4_  +32 p1_  +40 p2_  +48 kEps = 1e-8  +56 kPi = M_PI
//   +64 kRThresh = 1e-8          (xmmword_1803B8270 = {1e-8, 3.141592653589793}, then 1e-8)
// TODO(verify): names/meaning of the two extra constants at +48/+56 (never read by the code in
// chunk c14).
//
// Differences to upstream (reference/rpg_svo_pro_open/vikit/vikit_cameras/.../equidistant_distortion.h):
//   * two tangential parameters; distort() adds the Brown tangential term after the radial
//     scaling; jacobian() includes it;
//   * thetad polynomial evaluated in Horner form;
//   * undistort() is a Gauss-Newton solve (max 10 iterations, |step|^2 < 1e-16) on the full
//     model, takes the focal lengths and skips points within 4 px of the principal point
//     ((x*fx)^2 + (y*fy)^2 < 16 -> returned unchanged); on divergence x = y = -1000.
#pragma once

#include <cmath>
#include <cstddef>
#include <iostream>
#include <Eigen/Core>
#include <glog/logging.h>

namespace vk {
namespace cameras {

class EquidistantDistortion
{
public:
  EquidistantDistortion(const double k1, const double k2, const double k3, const double k4,
                        const double p1, const double p2)
    : k1_(k1), k2_(k2), k3_(k3), k4_(k4), p1_(p1), p2_(p2)
  {
  }

  ~EquidistantDistortion() = default;

  // 0x18015B760  (upstream: modified -- tangential term, Horner)
  inline Eigen::Vector2d distort(const Eigen::Vector2d& vector) const
  {
    const double r = vector.norm();
    if (r < kRThresh)
    {
      return vector;
    }
    const double theta = std::atan(r);
    const double thetad = thetad_from_theta(theta);
    const double scaling = thetad / r;
    const Eigen::Vector2d v = vector * scaling;
    const double r2 = v.squaredNorm();
    const double xy2 = v[0] * 2.0 * v[1];
    Eigen::Vector2d d;
    d[0] = (v[0] * 2.0 * v[0] + r2) * p1_ + p2_ * xy2;
    d[1] = (v[1] * 2.0 * v[1] + r2) * p2_ + xy2 * p1_;
    return d + v;
  }

  // 0x18015B960  (upstream: modified -- chain rule through the tangential term)
  inline Eigen::Matrix2d jacobian(const Eigen::Vector2d& uv) const
  {
    const double r = uv.norm();
    if (r < kRThresh)
    {
      return Eigen::Matrix2d::Identity();
    }
    const double theta = std::atan(r);
    const double theta2 = theta * theta;
    const double thetad =
        ((((k4_ * theta2 + k3_) * theta2 + k2_) * theta2 + k1_) * theta2 + 1.0) * theta;
    const double scaling = thetad / r;
    const double dscaling =
        ((((theta2 * 9.0 * k4_ + k3_ * 7.0) * theta2 + k2_ * 5.0) * theta2 + k1_ * 3.0) * theta2 + 1.0)
        * (1.0 / (r * r + 1.0)) * r - thetad;
    const double dscaling_dr = dscaling / (r * r);
    const double inv_r = 1.0 / r;
    const double dscaling_du = uv(0) * inv_r * dscaling_dr;
    const double dscaling_dv = uv(1) * inv_r * dscaling_dr;
    Eigen::Matrix2d jac_r;
    jac_r << uv(0) * dscaling_du + scaling, uv(0) * dscaling_dv,
             uv(1) * dscaling_du,           uv(1) * dscaling_dv + scaling;
    const Eigen::Vector2d v = scaling * uv;
    Eigen::Matrix2d jac_t;
    jac_t << v(0) * 6.0 * p1_ + p2_ * (2.0 * v(1)),  2.0 * v(1) * p1_ + p2_ * (2.0 * v(0)),
             2.0 * v(1) * p1_ + p2_ * (2.0 * v(0)),  v(1) * 6.0 * p2_ + (2.0 * v(0)) * p1_;
    return jac_t * jac_r + jac_r;
  }

  // 0x18015BD50  (upstream: rewritten)
  inline void undistort(double& x, double& y, double fx, double fy) const
  {
    const Eigen::Vector2d target(x, y);
    Eigen::Vector2d xu = target;
    if ((fy * y) * (fy * y) + (x * fx) * (x * fx) >= 16.0)
    {
      int iter = 0;
      while (true)
      {
        const double r2 = xu[0] * xu[0] + xu[1] * xu[1];
        const double r = std::sqrt(r2);
        const double theta = std::atan2(r, 1.0);
        const double theta2 = theta * theta;
        const double thetad =
            ((((theta2 * k4_ + k3_) * theta2 + k2_) * theta2 + k1_) * theta2 + 1.0) * theta;
        const Eigen::Vector2d xr = thetad * Eigen::Vector2d(xu[0] / r, xu[1] / r);
        const double xr2 = xr[0] * xr[0];
        const double yr2 = xr[1] * xr[1];
        const Eigen::Vector2d xd((xr2 * 3.0 + yr2) * p1_ + xr[0] + 2.0 * p2_ * xr[0] * xr[1],
                                 (yr2 * 3.0 + xr2) * p2_ + xr[1] + 2.0 * p1_ * xr[0] * xr[1]);
        const Eigen::Vector2d e = target - xd;

        // d(xd)/d(xr): identity + tangential part.
        Eigen::Matrix2d A;
        A << p1_ * 3.0, p2_,
             p2_,       p1_;
        Eigen::Matrix2d B;
        B << p2_, p1_,
             p1_, p2_ * 3.0;
        Eigen::Matrix2d J_t;
        J_t.col(0) = (A * 2.0) * xr;
        J_t.col(1) = (B * 2.0) * xr;
        J_t(0, 0) += 1.0;
        J_t(1, 1) += 1.0;

        // d(xr)/d(xu): radial part.
        const double dthetad =
            ((((theta2 * 9.0 * k4_ + k3_ * 7.0) * theta2 + k2_ * 5.0) * theta2 + k1_ * 3.0) * theta2 + 1.0);
        const double r_n = xu.norm();
        const Eigen::Vector2d dtheta_dxu = xu * (1.0 / (r_n * r_n + 1.0) / r_n);
        const double inv_r = 1.0 / r;
        const Eigen::Vector2d dir = inv_r * xu;
        const double s = thetad / r;
        const Eigen::Vector2d ds_dxu = (dthetad * inv_r) * dtheta_dxu - (thetad / r2) * dir;
        const Eigen::Matrix2d J_r = s * Eigen::Matrix2d::Identity() + xu * ds_dxu.transpose();

        const Eigen::Matrix2d J = J_t * J_r;
        const Eigen::Matrix2d JtJ = J.transpose() * J;
        const double det = JtJ(0, 0) * JtJ(1, 1) - JtJ(0, 1) * JtJ(1, 0);
        Eigen::Matrix2d adj;
        adj << JtJ(1, 1), -JtJ(1, 0),
               -JtJ(0, 1), JtJ(0, 0);
        const Eigen::Matrix2d inv = adj * (1.0 / det);
        const Eigen::Vector2d step = inv * J.transpose() * e;
        xu += step;
        if (step.squaredNorm() < 1e-16)
          break;
        if (++iter >= 10)
        {
          x = -1000.0;
          y = -1000.0;
          return;
        }
      }
      x = xu[0];
      y = xu[1];
    }
  }

  // inlined into 0x180161900
  inline void print(std::ostream& out) const
  {
    out << "  Distortion: Fisheye62(" << k1_ << ", " << k2_ << ", " << k3_ << ", " << k4_ << ", "
        << p1_ << ", " << p2_ << ")" << std::endl;
  }

  // returns distortion parameters as vector [k1 k2 k3 k4 p1 p2]  (0x18015CA90 via CameraGeometry)
  inline Eigen::VectorXd getDistortionParameters() const
  {
    Eigen::VectorXd distortion(6);
    distortion(0) = k1_;
    distortion(1) = k2_;
    distortion(2) = k3_;
    distortion(3) = k4_;
    distortion(4) = p1_;
    distortion(5) = p2_;
    return distortion;
  }

  double k1_ = 0;  // +0
  double k2_ = 0;  // +8
  double k3_ = 0;  // +16
  double k4_ = 0;  // +24
  double p1_ = 0;  // +32 tangential
  double p2_ = 0;  // +40 tangential

private:
  inline double thetad_from_theta(const double theta) const
  {
    const double theta2 = theta * theta;
    return ((((theta2 * k4_ + k3_) * theta2 + k2_) * theta2 + k1_) * theta2 + 1.0) * theta;
  }

  const double kEps = 1e-8;        // +48 TODO(verify) name; unused here
  const double kPi = M_PI;         // +56 TODO(verify) name; unused here
  const double kRThresh = 1e-8;    // +64

  friend struct EquidistantDistortionLayout;
};

struct EquidistantDistortionLayout {
  static_assert(sizeof(EquidistantDistortion) == 72, "sizeof(EquidistantDistortion)");
  static_assert(offsetof(EquidistantDistortion, p2_) == 40, "EquidistantDistortion::p2_");
  static_assert(offsetof(EquidistantDistortion, kRThresh) == 64, "EquidistantDistortion::kRThresh");
};

}  // namespace cameras
}  // namespace vk

// ---------------------------------------------------------------------------------------------
// Other CameraGeometry<PinholeProjection<EquidistantDistortion>> members instantiated in the
// factory TU (vtable 0x1803B7C78, slots: dtor, ?, backProject3, project3, printParameters,
// errorMultiplier, getIntrinsicParameters, getDistortionParameters, getAngleError):
//   0x18015ACB0 scalar deleting dtor          upstream-identical
//   0x18015B690 backProject3                  modified, see below
//   0x180161BA0 project3 (-> 0x180161C40 PinholeProjection::project3)   upstream-identical
//   0x180161900 printParameters               upstream-identical except the distortion print
//   0x18015B750 errorMultiplier = |fx_|       upstream-identical
//   0x18015CE00 getIntrinsicParameters (fx, fy, cx, cy)   upstream-identical
//   0x18015CA90 getDistortionParameters (6 values)        modified (size)
//   0x18015CA20 getAngleError                 upstream-identical
// PinholeProjection layout (CameraGeometry +152): +0 cam_type_ (int, 0 = kPinhole), +8 fx_,
// +16 fy_, +24 fx_inv_, +32 fy_inv_, +40 cx_, +48 cy_, +56 distortion_.
//
// 0x18015B690 PinholeProjection<EquidistantDistortion>::backProject3 (Pimax):
//   double x = (keypoint[0]-cx_)*fx_inv_;
//   double y = (keypoint[1]-cy_)*fy_inv_;
//   if (x == 0.0 && y == 0.0) { (*out)[0] = 0.0; (*out)[1] = 0.0; }
//   else { distortion_.undistort(x, y, fx_, fy_); (*out)[0] = x; (*out)[1] = y; }
//   (*out)[2] = 1.0;
//   return true;
