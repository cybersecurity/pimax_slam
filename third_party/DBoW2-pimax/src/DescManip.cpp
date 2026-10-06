/**
 * PIMAX addition to DBoW2: DBoW3 DescManip::fromStream (reference/DBow3/src/DescManip.cpp),
 * binary 0x1801B6D90.  upstream-identical (DBoW3).
 */
#include "DBoW2/DescManip.h"

namespace DBoW2 {

// 0x1801B6D90
void DescManip::fromStream(cv::Mat &m,std::istream &str){
    int type,cols,rows;
    str.read((char*)&cols,sizeof(cols));
    str.read((char*)&rows,sizeof(rows));
    str.read((char*)&type,sizeof(type));
    m.create(rows,cols,type);
    str.read((char*)m.ptr<char>(0),m.elemSize()*m.cols);
}

} // namespace DBoW2
