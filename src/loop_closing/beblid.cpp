// pimax_slam.pi.dll -- src/loop_closing/beblid.cpp  (draft c14_interface_api/loop_closing/beblid.cpp)
//
// Project-local BEBLID (see beblid.h for the differences to upstream
// opencv_contrib-4.5.3/modules/xfeatures2d/src/beblid.cpp).  Code 0x18016F2F0 .. 0x180170C6F.
// TODO(verify): file name (TU between the headset system and loop_closing.cpp; no __FILE__).
// beblid.p256.hpp / beblid.p512.hpp are verbatim copies of the upstream tables (0x1803BC220 /
// 0x1803B9220, byte-identical).
#include "loop_closing/beblid.h"

#include <cmath>
#include <functional>
#include <string>

#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>

// Project copy of cv::ParallelLoopBodyLambdaWrapper (global namespace; RTTI name
// "ParallelLambdaWrapper", vtable 0x1803B8E38).  Its two virtuals are identical to OpenCV's
// inline wrapper and were folded by /OPT:ICF with it (dtor 0x180093630, operator() 0x1800935C0,
// cv::ParallelLoopBodyLambdaWrapper vtable 0x1803B1570).  TODO(verify) original location/name of
// this helper (the classic pre-OpenCV-3.3 "parallel_for_ with a lambda" snippet).
class ParallelLambdaWrapper : public cv::ParallelLoopBody
{
private:
    std::function<void(const cv::Range&)> m_functor;

public:
    ParallelLambdaWrapper(std::function<void(const cv::Range&)> functor) : m_functor(functor) {}

    virtual void operator()(const cv::Range& range) const CV_OVERRIDE { m_functor(range); }
};

#define CV_ROUNDNUM(x) ((int)(x + 0.5f))
#define CV_DEGREES_TO_RADS 0.017453292519943295 // (M_PI / 180.0)
#define CV_BEBLID_EXTRA_RATIO_MARGIN 1.75f

// inlined into the parallel body 0x18016F540  (upstream-identical)
static inline bool isKeypointInTheBorder(const cv::KeyPoint &kp,
                                         const cv::Size &imgSize,
                                         const cv::Size &patchSize = {32, 32},
                                         float scaleFactor = 1)
{
    float s = scaleFactor * kp.size / (patchSize.width + patchSize.height);
    cv::Size2f border(patchSize.width * s * CV_BEBLID_EXTRA_RATIO_MARGIN,
                      patchSize.height * s * CV_BEBLID_EXTRA_RATIO_MARGIN);
    if (kp.pt.x < border.width || kp.pt.x + border.width >= imgSize.width)
        return true;
    if (kp.pt.y < border.height || kp.pt.y + border.height >= imgSize.height)
        return true;
    return false;
}

// 0x180170840  (upstream-modified: m02 of the angle == -1 case is computed in double)
static void rectifyABWL(const std::vector<ABWLParams> &wlPatchParams,
                        std::vector<ABWLParams> &wlImageParams,
                        const cv::KeyPoint &kp,
                        float scaleFactor = 1,
                        const cv::Size &patchSize = cv::Size(32, 32))
{
    float m00, m01, m02, m10, m11, m12;
    float s, cosine, sine;

    s = scaleFactor * kp.size / (0.5f * (patchSize.width + patchSize.height));
    wlImageParams.resize(wlPatchParams.size());   // 0x180170B80

    if (kp.angle == -1)
    {
        m00 = s;
        m01 = 0.0f;
        m02 = kp.pt.x - s * 0.5 * patchSize.width;   // double arithmetic, then narrowed
        m10 = 0.0f;
        m11 = s;
        m12 = kp.pt.y - s * 0.5f * patchSize.height;
    }
    else
    {
        cosine = (kp.angle >= 0) ? float(cos(kp.angle * CV_DEGREES_TO_RADS)) : 1.f;
        sine = (kp.angle >= 0) ? float(sin(kp.angle * CV_DEGREES_TO_RADS)) : 0.f;

        m00 = s * cosine;
        m01 = -s * sine;
        m02 = (-s * cosine + s * sine) * patchSize.width * 0.5f + kp.pt.x;
        m10 = s * sine;
        m11 = s * cosine;
        m12 = (-s * sine - s * cosine) * patchSize.height * 0.5f + kp.pt.y;
    }

    for (size_t i = 0; i < wlPatchParams.size(); i++)
    {
        wlImageParams[i].x1 = CV_ROUNDNUM(m00 * wlPatchParams[i].x1 + m01 * wlPatchParams[i].y1 + m02);
        wlImageParams[i].y1 = CV_ROUNDNUM(m10 * wlPatchParams[i].x1 + m11 * wlPatchParams[i].y1 + m12);
        wlImageParams[i].x2 = CV_ROUNDNUM(m00 * wlPatchParams[i].x2 + m01 * wlPatchParams[i].y2 + m02);
        wlImageParams[i].y2 = CV_ROUNDNUM(m10 * wlPatchParams[i].x2 + m11 * wlPatchParams[i].y2 + m12);
        wlImageParams[i].boxRadius = CV_ROUNDNUM(s * wlPatchParams[i].boxRadius);
    }
}

// 0x1801700F0  (upstream-modified: indexes data[y * cols + x] instead of at<int>(y, x))
static float computeABWLResponse(const ABWLParams &wlImageParams,
                                 const cv::Mat &integralImage)
{
    int frameWidth, frameHeight, box1x1, box1y1, box1x2, box1y2, box2x1, box2y1, box2x2, box2y2;
    int A, B, C, D;
    int box_area1, box_area2;
    float sum1, sum2, average1, average2;
    const int *integral = reinterpret_cast<const int *>(integralImage.data);

    frameWidth = integralImage.cols;
    frameHeight = integralImage.rows;

    box1x1 = wlImageParams.x1 - wlImageParams.boxRadius;
    if (box1x1 < 0)
        box1x1 = 0;
    else if (box1x1 >= frameWidth - 1)
        box1x1 = frameWidth - 2;
    box1y1 = wlImageParams.y1 - wlImageParams.boxRadius;
    if (box1y1 < 0)
        box1y1 = 0;
    else if (box1y1 >= frameHeight - 1)
        box1y1 = frameHeight - 2;
    box1x2 = wlImageParams.x1 + wlImageParams.boxRadius + 1;
    if (box1x2 <= 0)
        box1x2 = 1;
    else if (box1x2 >= frameWidth)
        box1x2 = frameWidth - 1;
    box1y2 = wlImageParams.y1 + wlImageParams.boxRadius + 1;
    if (box1y2 <= 0)
        box1y2 = 1;
    else if (box1y2 >= frameHeight)
        box1y2 = frameHeight - 1;

    box2x1 = wlImageParams.x2 - wlImageParams.boxRadius;
    if (box2x1 < 0)
        box2x1 = 0;
    else if (box2x1 >= frameWidth - 1)
        box2x1 = frameWidth - 2;
    box2y1 = wlImageParams.y2 - wlImageParams.boxRadius;
    if (box2y1 < 0)
        box2y1 = 0;
    else if (box2y1 >= frameHeight - 1)
        box2y1 = frameHeight - 2;
    box2x2 = wlImageParams.x2 + wlImageParams.boxRadius + 1;
    if (box2x2 <= 0)
        box2x2 = 1;
    else if (box2x2 >= frameWidth)
        box2x2 = frameWidth - 1;
    box2y2 = wlImageParams.y2 + wlImageParams.boxRadius + 1;
    if (box2y2 <= 0)
        box2y2 = 1;
    else if (box2y2 >= frameHeight)
        box2y2 = frameHeight - 1;

    A = integral[box1y1 * frameWidth + box1x1];
    B = integral[box1y1 * frameWidth + box1x2];
    C = integral[box1y2 * frameWidth + box1x1];
    D = integral[box1y2 * frameWidth + box1x2];
    sum1 = float(A + D - B - C);
    box_area1 = (box1y2 - box1y1) * (box1x2 - box1x1);
    average1 = sum1 / box_area1;

    A = integral[box2y1 * frameWidth + box2x1];
    B = integral[box2y1 * frameWidth + box2x2];
    C = integral[box2y2 * frameWidth + box2x1];
    D = integral[box2y2 * frameWidth + box2x2];
    sum2 = float(A + D - B - C);
    box_area2 = (box2y2 - box2y1) * (box2x2 - box2x1);
    average2 = sum2 / box_area2;

    return average1 - average2;
}

// 0x18016F2F0  (upstream-modified: BEBLID_Impl ctor; table chosen by n_bits <= 256, then resized)
BEBLID::BEBLID(int n_bits, float scale_factor)
    : scale_factor_(scale_factor), patch_size_(32, 32)
{
    #include "loop_closing/beblid.p512.hpp"   // upstream tables, byte-identical (0x1803B9220)
    #include "loop_closing/beblid.p256.hpp"   // (0x1803BC220)
    if (n_bits <= 256)
        wl_params_.assign(wl_params_256, wl_params_256 + sizeof(wl_params_256) / sizeof(wl_params_256[0]));
    else
        wl_params_.assign(wl_params_512, wl_params_512 + sizeof(wl_params_512) / sizeof(wl_params_512[0]));
    wl_params_.resize(n_bits);
}

// 0x180170570  (upstream: BEBLID::create via makePtr; here std::make_shared)
std::shared_ptr<BEBLID> BEBLID::create(int n_bits, float scale_factor)
{
    return std::make_shared<BEBLID>(n_bits, scale_factor);
}

// 0x180170660  (upstream: new)
cv::String BEBLID::getDefaultName() const
{
    return std::string("BEBLID") + std::to_string(wl_params_.size());
}

// 0x18016FE50 (+ parallel body 0x18016F540, std::function thunks 0x18016FDD0/0x18016FE30/0x18016FE40)
// (upstream-modified: no empty checks / gray conversion; computeBEBLID inlined)
void BEBLID::compute(cv::InputArray image, std::vector<cv::KeyPoint> &keypoints,
                     cv::OutputArray _descriptors)
{
    cv::Mat integralImg;
    cv::integral(image, integralImg);
    _descriptors.create((int)keypoints.size(), descriptorSize(), descriptorType());
    cv::Mat descriptors = _descriptors.getMat();

    const int *integralPtr = integralImg.ptr<int>();
    cv::Size frameSize(integralImg.cols - 1, integralImg.rows - 1);

    cv::parallel_for_(cv::Range(0, int(keypoints.size())), ParallelLambdaWrapper([&](const cv::Range &range)
    {
        ABWLParams *wl;
        float responseFun;
        int areaResponseFun, kpIdx;
        size_t wlIdx;
        int box1x1, box1y1, box1x2, box1y2, box2x1, box2y1, box2x2, box2y2, bit_idx, side;
        uchar byte = 0;
        std::vector<ABWLParams> imgWLParams(wl_params_.size());
        uchar *d = &descriptors.at<uchar>(range.start, 0);

        for (kpIdx = range.start; kpIdx < range.end; kpIdx++)
        {
            rectifyABWL(wl_params_, imgWLParams, keypoints[kpIdx], scale_factor_, patch_size_);

            if (isKeypointInTheBorder(keypoints[kpIdx], frameSize, patch_size_, scale_factor_))
            {
                for (wlIdx = 0; wlIdx < wl_params_.size(); wlIdx++) {
                    bit_idx = 7 - int(wlIdx % 8);
                    responseFun = computeABWLResponse(imgWLParams[wlIdx], integralImg);
                    byte |= (responseFun <= wl_params_[wlIdx].th) << bit_idx;
                    if (bit_idx == 0)
                    {
                        *d = byte;
                        byte = 0;
                        d++;
                    }
                }
            }
            else
            {
                wl = imgWLParams.data();
                for (wlIdx = 0; wlIdx < wl_params_.size(); wlIdx++)
                {
                    bit_idx = 7 - int(wlIdx % 8);
                    box1x1 = wl->x1 - wl->boxRadius;
                    box1y1 = (wl->y1 - wl->boxRadius) * integralImg.cols;
                    box1x2 = wl->x1 + wl->boxRadius + 1;
                    box1y2 = (wl->y1 + wl->boxRadius + 1) * integralImg.cols;
                    box2x1 = wl->x2 - wl->boxRadius;
                    box2y1 = (wl->y2 - wl->boxRadius) * integralImg.cols;
                    box2x2 = wl->x2 + wl->boxRadius + 1;
                    box2y2 = (wl->y2 + wl->boxRadius + 1) * integralImg.cols;
                    side = 1 + (wl->boxRadius << 1);

                    areaResponseFun = (integralPtr[box1y1 + box1x1]
                        + integralPtr[box1y2 + box1x2]
                        - integralPtr[box1y1 + box1x2]
                        - integralPtr[box1y2 + box1x1]
                        - integralPtr[box2y1 + box2x1]
                        - integralPtr[box2y2 + box2x2]
                        + integralPtr[box2y1 + box2x2]
                        + integralPtr[box2y2 + box2x1]);

                    byte |= (areaResponseFun <= (wl_params_[wlIdx].th * (side * side))) << bit_idx;
                    wl++;
                    if (bit_idx == 0)
                    {
                        *d = byte;
                        byte = 0;
                        d++;
                    }
                }
            }
        }
    }));
}
