/**
 * PIMAX addition to DBoW2: the descriptor stream reader of DBoW3's DescManip
 * (reference/DBow3/src/DescManip.{h,cpp}), needed by TemplatedVocabulary::fromStream.
 * Only fromStream exists in the image (0x1801B6D90, between ScoringObject.obj and the QuickLZ
 * object).  TODO(verify) the original class/file name (DBoW3 name kept).
 */
#ifndef __D_T_DESC_MANIP__
#define __D_T_DESC_MANIP__

#include <istream>
#include <opencv2/core.hpp>

namespace DBoW2 {

class DescManip
{
public:
  /// 0x1801B6D90: reads cols, rows, type (int32 each), then elemSize()*cols bytes (one row).
  static void fromStream(cv::Mat &m, std::istream &str);
};

} // namespace DBoW2

#endif
