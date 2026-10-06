// feature_alignment.cpp -- Pimax fork of svo_direct/src/feature_alignment.cpp
// Chunk c06_common_depthfilter, object range 0x1800A3900 .. 0x1800AA150.
// Everything else in that range is Eigen/STL template code instantiated by these two
// functions (JacobiSVD<Matrix3f>, LDLT<Matrix4f>, Matrix3f inverse/determinant,
// std::vector<Eigen::Vector2f>::_Emplace_reallocate) -- see the notes.

#include "direct/feature_alignment.h"

#include <cmath>
#include <limits>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace pimax {
namespace totem {
namespace feature_alignment {

// Pimax-new file statics used by align1D only.  They are NOT constants in the binary
// (plain .data ints, re-read on every use), so they are kept as mutable statics.
// TODO(verify) names / whether they are function-local statics of align1D.
static int g_align1d_halfpatch_size = 4;          // 0x18046A158
static int g_align1d_patch_size = 8;              // 0x18046A15C
static float g_align1d_weights[64];               // 0x18047DC40 (8x8 Gaussian weights)
static bool g_align1d_weights_initialized = false;// 0x18047DD40 (plain flag, not thread safe)

//------------------------------------------------------------------------------
// 0x1800A63E0
bool align1D(
    const cv::Mat& cur_img,
    const Eigen::Ref<GradientVector>& dir,                  // direction in which the patch is allowed to move
    uint8_t* ref_patch_with_border,
    uint8_t* ref_patch,
    const int n_iter,
    const bool affine_est_offset,
    const bool affine_est_gain,
    Keypoint* cur_px_estimate,
    double* h_inv)
{
  // Gaussian spatial weights, sigma = patch_size / 3, centred on the patch.
  if(!g_align1d_weights_initialized)
  {
    const float center = g_align1d_patch_size * 0.5f - 0.5f;
    const float sigma = g_align1d_patch_size / 3.0f;
    for(int y = 0; y < g_align1d_patch_size; ++y)
    {
      for(int x = 0; x < g_align1d_patch_size; ++x)
      {
        const float dx = static_cast<float>(x) - center;
        const float dy = static_cast<float>(y) - center;
        g_align1d_weights[y * g_align1d_patch_size + x] =
            std::exp(-(dx * dx + dy * dy) / (2 * sigma * sigma));
      }
    }
    g_align1d_weights_initialized = true;
  }

  bool converged = false;
  double u = cur_px_estimate->x();
  double v = cur_px_estimate->y();

  // We optimize feature position and two affine parameters.
  // Compute derivative of template and prepare inverse compositional.
  float ref_patch_dv[64];
  Eigen::Matrix3f H = Eigen::Matrix3f::Zero(3, 3);

  // Compute gradient (Pimax: 3x3 Sobel / 8) and weighted hessian.
  const int ref_step = g_align1d_patch_size + 2;
  float* it_dv = ref_patch_dv;
  for(int y = 0; y < g_align1d_patch_size; ++y)
  {
    uint8_t* it = ref_patch_with_border + (y+1)*ref_step + 1;
    for(int x = 0; x < g_align1d_patch_size; ++x, ++it, ++it_dv)
    {
      // quirk: the x kernel pairs the corners diagonally (BR-TL, BL-TR) instead of
      // (TR-TL, BR-BL); reproduced as found.
      const float dx = (static_cast<float>(it[ref_step+1] - it[-ref_step-1])
                        + 2.0f * static_cast<float>(it[1] - it[-1])
                        + static_cast<float>(it[ref_step-1] - it[-ref_step+1])) * 0.125f;
      const float dy = (static_cast<float>(it[ref_step+1] - it[-ref_step+1])
                        + 2.0f * static_cast<float>(it[ref_step] - it[-ref_step])
                        + static_cast<float>(it[ref_step-1] - it[-ref_step-1])) * 0.125f;
      Eigen::Vector3f J;
      J[0] = 0.5f * (dir(0) * dx + dir(1) * dy);

      // If not using the affine compensation, set the jacobian be zero.
      J[1] = affine_est_offset? 1.0f : 0.0f;
      J[2] = affine_est_gain? -1.0f*it[0] : 0.0f;

      *it_dv = J[0];
      const float w = g_align1d_weights[x + g_align1d_patch_size * y];
      H += J * w * J.transpose();
    }
  }

  // Pimax: regularization.
  H(0, 0) += 0.00001f;
  H(1, 1) += 0.00001f;
  H(2, 2) += 0.00001f;
  // If not use affine compensation, force update to be zero by
  // * setting the affine parameter block in H to identity
  // * setting the residual block to zero (see below)
  if(!affine_est_offset)
  {
    H(1, 1) = 1.0;
  }
  if(!affine_est_gain)
  {
    H(2, 2) = 1.0;
  }

  // Pimax: pseudo-inverse for (near) singular H.  Note: signed determinant test.
  Eigen::Matrix3f Hinv;
  if(H.determinant() < 1e-10f)
  {
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3f sv = svd.singularValues();
    for(int i = 0; i < 3; ++i)
    {
      if(sv(i) > 1e-6f)
        sv(i) = 1.0f / sv(i);
      else
        sv(i) = 0;
    }
    Hinv = svd.matrixV() * sv.asDiagonal() * svd.matrixU().transpose();
  }
  else
  {
    Hinv = H.inverse();
  }

  if(h_inv)
    *h_inv = 1.0/H(0,0)*g_align1d_patch_size*g_align1d_patch_size;

  // Pimax: affine parameters and position are double.
  double mean_diff = 0;
  double alpha = 1.0;
  double chi2 = std::numeric_limits<double>::max();
  int n_chi2_converged = 0;

  const int cur_step = cur_img.step.p[0];
  const int max_u = cur_img.cols - g_align1d_halfpatch_size;
  const int max_v = cur_img.rows - g_align1d_halfpatch_size;
  for(int iter = 0; iter<n_iter; ++iter)
  {
    int u_r = std::floor(u);
    int v_r = std::floor(v);
    // Pimax: clamp into the image instead of giving up.
    if(u_r < g_align1d_halfpatch_size + 1
       || v_r < g_align1d_halfpatch_size + 1
       || u_r >= max_u - 1
       || v_r >= max_v - 1)
    {
      const double lo = g_align1d_halfpatch_size + 1;
      u = std::max(lo, std::min(u, static_cast<double>(max_u - 2)));
      v = std::max(lo, std::min(v, static_cast<double>(max_v - 2)));
      u_r = std::floor(u);
      v_r = std::floor(v);
    }

    if(std::isnan(u) || std::isnan(v)) // TODO very rarely this can happen, maybe H is singular? should not be at corner.. check
      return false;

    // compute interpolation weights
    const double subpix_x = u-u_r;
    const double subpix_y = v-v_r;
    const double wTL = (1.0-subpix_x)*(1.0-subpix_y);
    const double wTR = subpix_x * (1.0-subpix_y);
    const double wBL = (1.0-subpix_x)*subpix_y;
    const double wBR = subpix_x * subpix_y;

    // loop through search_patch, interpolate
    uint8_t* it_ref = ref_patch;
    float* it_ref_dv = ref_patch_dv;
    double new_chi2 = 0.0;
    Eigen::Vector3f Jres = Eigen::Vector3f::Zero();
    for(int y=0; y<g_align1d_patch_size; ++y)
    {
      uint8_t* it = (uint8_t*) cur_img.data +
          (v_r+y-g_align1d_halfpatch_size)*cur_step + u_r-g_align1d_halfpatch_size;
      for(int x=0; x<g_align1d_patch_size; ++x, ++it, ++it_ref, ++it_ref_dv)
      {
        float w = g_align1d_weights[y * g_align1d_patch_size + x];
        const double cur_intensity =
            wTL*it[0] + wTR*it[1] + wBL*it[cur_step] + wBR*it[cur_step+1];
        const double res = cur_intensity - alpha*(*it_ref) + mean_diff;
        // Pimax: Huber-like down-weighting after the first three iterations.
        if(iter > 2)
        {
          const float abs_res = std::fabs(static_cast<float>(res));
          w *= (abs_res <= 5.0f) ? 1.0f : 5.0f / abs_res;
        }
        Jres[0] -= w*res*(*it_ref_dv);
        new_chi2 += w*res*res;

        // If affine compensation is used,
        // set Jres with respect to affine parameters.
        if(affine_est_offset)
        {
          Jres[1] -= w*res;
        }
        if(affine_est_gain)
        {
          Jres[2] -= (-1)*w*res*(*it_ref);
        }
      }
    }

    // Pimax: chi2 based termination.
    if(iter > 0)
    {
      if(std::fabs(new_chi2 - chi2) / (chi2 + 1e-10) < 1e-4)
      {
        if(++n_chi2_converged >= 2)
        {
          converged = true;
          break;
        }
      }
      else
      {
        n_chi2_converged = 0;
      }
      if(new_chi2 > chi2 * 1.5 && iter > 3)
        break;
    }
    chi2 = new_chi2;

    const Eigen::Vector3f update = Hinv * Jres;
    u += update[0]*dir[0];
    v += update[0]*dir[1];
    mean_diff += update[1];
    alpha += update[2];

    if(affine_est_gain)
      alpha = std::max(0.5, std::min(2.0, alpha));
    if(affine_est_offset)
      mean_diff = std::max(-30.0, std::min(30.0, mean_diff));

    const double step = update[0]*update[0];
    if(step < 5e-7)
    {
      // Pimax: converged, but keeps iterating (flag is sticky).
      converged = true;
    }
    else if(step > 0.01f && iter > 0)
    {
      // Pimax: large step -> take back half of it.
      u -= dir[0] * (update[0] * 0.5);
      v -= dir[1] * (update[0] * 0.5);
      alpha -= update[2] * 0.5;
      mean_diff -= update[1] * 0.5;
    }
  }

  if(converged)
  {
    // Pimax: final sanity checks.
    const int ui = static_cast<int>(u + 0.5);
    const int vi = static_cast<int>(v + 0.5);
    if(ui < g_align1d_halfpatch_size || vi < g_align1d_halfpatch_size
       || ui >= max_u || vi >= max_v)
      converged = false;

    float sum = 0.0f;
    for(int i = 0; i < 64; ++i)
      sum += ref_patch[i];
    const float mean = sum / 64.0f;
    if(mean < 10.0f || mean > 245.0f)
      converged = false;
  }

  *cur_px_estimate << u, v;
  return converged;
}

//------------------------------------------------------------------------------
// 0x1800A84F0
bool align2D(
    const cv::Mat& cur_img,
    uint8_t* ref_patch_with_border,
    uint8_t* ref_patch,
    const int n_iter,
    const bool affine_est_offset,
    const bool affine_est_gain,
    Keypoint& cur_px_estimate,
    bool no_simd,
    std::vector<Eigen::Vector2f> *each_step)
{
  if(each_step) each_step->clear();

  const int halfpatch_size_ = 4;
  const int patch_size_ = 8;
  const int patch_area_ = 64;
  bool converged=false;

  // We optimize feature position and two affine parameters.
  // compute derivative of template and prepare inverse compositional
  float ref_patch_dx[patch_area_];
  float ref_patch_dy[patch_area_];
  Eigen::Matrix4f H; H.setZero();

  // compute gradient and hessian
  const int ref_step = patch_size_+2;
  float* it_dx = ref_patch_dx;
  float* it_dy = ref_patch_dy;
  for(int y=0; y<patch_size_; ++y)
  {
    uint8_t* it = ref_patch_with_border + (y+1)*ref_step + 1;
    for(int x=0; x<patch_size_; ++x, ++it, ++it_dx, ++it_dy)
    {
      Eigen::Vector4f J;
      // Pimax: float literals (upstream 0.5 / 1.0 / -1.0 in double)
      J[0] = 0.5f * (it[1] - it[-1]);
      J[1] = 0.5f * (it[ref_step] - it[-ref_step]);

      // If not using the affine compensation, force the jacobian to be zero.
      J[2] = affine_est_offset? 1.0f : 0.0f;
      J[3] = affine_est_gain? -1.0f*it[0]: 0.0f;

      *it_dx = J[0];
      *it_dy = J[1];
      H += J*J.transpose();
    }
  }
  // If not use affine compensation, force update to be zero by
  // * setting the affine parameter block in H to identity
  // * setting the residual block to zero (see below)
  if(!affine_est_offset)
  {
    H(2, 2) = 1.0;
  }
  if(!affine_est_gain)
  {
    H(3, 3) = 1.0;
  }
  float mean_diff = 0;
  float alpha = 1.0;

  // Compute pixel location in new image:
  float u = cur_px_estimate.x();
  float v = cur_px_estimate.y();

  if(each_step) each_step->push_back(Eigen::Vector2f(u, v));

  // termination condition
  const float min_update_squared = 0.03*0.03; // TODO I suppose this depends on the size of the image (ate)
  const int cur_step = cur_img.step.p[0];
  Eigen::Vector4f update; update.setZero();
  // Pimax: LDLT solve instead of the explicit inverse.
  const Eigen::LDLT<Eigen::Matrix4f> H_ldlt = H.ldlt();
  for(int iter = 0; iter<n_iter; ++iter)
  {
    int u_r = std::floor(u);
    int v_r = std::floor(v);
    if(u_r < halfpatch_size_
       || v_r < halfpatch_size_
       || u_r >= cur_img.cols-halfpatch_size_
       || v_r >= cur_img.rows-halfpatch_size_)
      break;

    if(std::isnan(u) || std::isnan(v)) // TODO very rarely this can happen, maybe H is singular? should not be at corner.. check
      return false;

    // compute interpolation weights (Pimax: float)
    float subpix_x = u-u_r;
    float subpix_y = v-v_r;
    float wTL = (1.0f-subpix_x)*(1.0f-subpix_y);
    float wTR = subpix_x * (1.0f-subpix_y);
    float wBL = (1.0f-subpix_x)*subpix_y;
    float wBR = subpix_x * subpix_y;

    // loop through search_patch, interpolate
    uint8_t* it_ref = ref_patch;
    float* it_ref_dx = ref_patch_dx;
    float* it_ref_dy = ref_patch_dy;
    Eigen::Vector4f Jres; Jres.setZero();
    for(int y=0; y<patch_size_; ++y)
    {
      uint8_t* it = (uint8_t*) cur_img.data + (v_r+y-halfpatch_size_)*cur_step + u_r-halfpatch_size_;
      for(int x=0; x<patch_size_; ++x, ++it, ++it_ref, ++it_ref_dx, ++it_ref_dy)
      {
        float search_pixel = wTL*it[0] + wTR*it[1] + wBL*it[cur_step] + wBR*it[cur_step+1];
        float res = search_pixel - alpha*(*it_ref) + mean_diff;
        Jres[0] -= res*(*it_ref_dx);
        Jres[1] -= res*(*it_ref_dy);

        // If affine compensation is used,
        // set Jres with respect to affine parameters.
        if(affine_est_offset)
        {
          Jres[2] -= res;
        }

        if(affine_est_gain)
        {
          Jres[3] -= (-1)*res*(*it_ref);
        }
      }
    }
    // If not use affine compensation, force update to be zero.
    if(!affine_est_offset)
    {
      Jres[2] = 0.0;
    }
    if(!affine_est_gain)
    {
      Jres[3] = 0.0;
    }

    update = H_ldlt.solve(Jres);
    u += update[0];
    v += update[1];
    mean_diff += update[2];
    alpha += update[3];

    if(each_step) each_step->push_back(Eigen::Vector2f(u, v));

    if(update[0]*update[0]+update[1]*update[1] < min_update_squared)
    {
      converged=true;
      break;
    }
  }

  cur_px_estimate << u, v;
  (void)no_simd;

  return converged;
}

} // namespace feature_alignment
} // namespace totem
} // namespace pimax
