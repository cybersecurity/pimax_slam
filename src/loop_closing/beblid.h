// pimax_slam.pi.dll -- src/loop_closing/beblid.h  (from draft c14)
//
// Project-local copy of opencv_contrib 4.5.3 xfeatures2d BEBLID
// (reference/opencv_contrib-4.5.3/modules/xfeatures2d/src/beblid.cpp), global namespace.
// Code 0x18016F2F0 .. 0x180170C6F (chunk c14_interface_api, after the headset-system TU).
//
// Differences to upstream:
//   * a single concrete class `BEBLID : public cv::Feature2D` (no BEBLID/BEBLID_Impl split),
//     created through std::make_shared (BEBLID::create 0x180170570, called by the LoopClosing ctor
//     as BEBLID::create(256, 0.75f));
//   * ctor takes (n_bits, scale_factor); any n_bits <= 256 selects the 256 table, otherwise the
//     512 table, then wl_params_ is resized to n_bits (so n_bits need not be 256/512);
//   * compute() has no empty-image / empty-keypoint checks and no colour conversion; it integrates
//     the input directly; the parallel body is wrapped in the project's ParallelLambdaWrapper
//     (vtable 0x1803B8E38) instead of cv::ParallelLoopBodyLambdaWrapper;
//   * rectifyABWL computes m02 of the unrotated case in double (0.5 literal);
//   * computeABWLResponse indexes the integral image with cols instead of at<int>(y, x);
//   * empty() returns false; getDefaultName() returns "BEBLID" + std::to_string(n_bits).
// The weak-learner tables (0x1803B9220: 512 x 24 bytes, 0x1803BC220: 256 x 24 bytes) are
// byte-identical to upstream beblid.p512.hpp / beblid.p256.hpp (checked value by value).
//
// Layout (sizeof 72; make_shared block 0x58):
//   +0 vfptr (0x1803B8A28)  +8 vbptr  +16 std::vector<ABWLParams> wl_params_  +40 float scale_factor_
//   +44 cv::Size patch_size_ (32, 32)  +60 vtordisp  +64 cv::Algorithm (vfptr 0x1803B8A70)
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

struct ABWLParams
{
    int x1, y1, x2, y2, boxRadius, th;
};

class BEBLID : public cv::Feature2D
{
public:
    // 0x18016F2F0
    BEBLID(int n_bits, float scale_factor);
    // 0x18016FC40 / 0x18016FCA0 (compiler-generated, vbase variants)
    ~BEBLID() override = default;

    // 0x180170570
    static std::shared_ptr<BEBLID> create(int n_bits, float scale_factor);

    // 0x180170610
    int descriptorSize() const override { return int(wl_params_.size() / 8); }
    // 0x18000FC80 (folded "return 0")
    int descriptorType() const override { return CV_8UC1; }
    // 0x18001A920 (folded "return 6")
    int defaultNorm() const override { return cv::NORM_HAMMING; }
    // 0x18017063C (vtordisp thunk to a folded "return false")
    bool empty() const override { return false; }
    // 0x180170654 -> 0x180170660
    cv::String getDefaultName() const override;

    // 0x18016FE50
    void compute(cv::InputArray image, std::vector<cv::KeyPoint>& keypoints,
                 cv::OutputArray descriptors) override;

private:
    std::vector<ABWLParams> wl_params_;   // +16
    float scale_factor_;                  // +40
    cv::Size patch_size_;                 // +44 (32, 32)
};

static_assert(sizeof(ABWLParams) == 24, "");
static_assert(sizeof(BEBLID) == 72, "sizeof(BEBLID) (make_shared block 0x58)");
