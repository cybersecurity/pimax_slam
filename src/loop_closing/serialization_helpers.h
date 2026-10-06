// pimax_slam.pi.dll -- src/loop_closing/serialization_helpers.h
//
// The ONE place for the project's free boost::serialization functions for third-party types used
// by the PlatMap persistence (pimax_database.bin / platMap_*.bin / *.pba).  Reconciled from
// drafts c07 (loop_closing/serialization_part.cpp: cv::Mat, BowVector, Transformation save),
// c09 / c10 (loop_closing/serialization_helpers.h: Eigen, cv::Point*, Transformation load) and
// c15 (loop_closing/platmap.h, same bodies).  The KeyFrame-graph save/load and the member
// serialize() functions are in loop_closing/platmap.h.
//
// Instantiations (binary_[io]archive / portable_binary_[io]archive):
//   cv::Mat          save 0x1800DBFC0 / 0x1800DCFE0   load 0x1800DB6F0 / 0x1800DC670  (c07)
//   DBoW2::BowVector save 0x1800DBB10 / 0x1800DCAE0   load 0x1800DB230 / 0x1800DC190  (c07)
//   Transformation   save 0x1800DB8D0 / 0x1800DC8A0   load inlined 0x180116460 / 0x180117280
//   Eigen::Vector3d  inlined in (o|i)serializer 0x180125550 / 0x1801263B0 / 0x180116280 /
//                    0x180117040: ONE serialize() for both directions -- the save path also
//                    contains the (no-op) resize with its fixed-size assertion (c10).
//   cv::Point3f / cv::Point2f  inlined (c09 / c10).
// All have implementation level object_class_info and version 0.  TODO(verify) header name.
#pragma once

#include <map>
#include <vector>

#include <boost/serialization/array.hpp>
#include <boost/serialization/array_wrapper.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

#include <DBoW2/BowVector.h>
#include <Eigen/Core>
#include <kindr/minimal/quat-transformation.h>
#include <opencv2/core/core.hpp>

namespace boost {
namespace serialization {

// ---- Eigen::Matrix (only Vector3d is instantiated) --------------------------------------------
// rows/cols are 8-byte Eigen::Index; resize() is unconditional (asserts for fixed sizes).
template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows,
          int _MaxCols>
inline void serialize(Archive& ar,
                      Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& t,
                      const unsigned int /*file_version*/)
{
  Eigen::Index rows = t.rows();
  Eigen::Index cols = t.cols();
  ar & rows;
  ar & cols;
  t.resize(rows, cols);
  ar & boost::serialization::make_array(t.data(), rows * cols);
}

// ---- OpenCV points ---------------------------------------------------------------------------
template <class Archive>
inline void serialize(Archive& ar, cv::Point3f& p, const unsigned int /*version*/)
{
  ar & p.x;
  ar & p.y;
  ar & p.z;
}

template <class Archive>
inline void serialize(Archive& ar, cv::Point2f& p, const unsigned int /*version*/)
{
  ar & p.x;
  ar & p.y;
}

// ---- cv::Mat -----------------------------------------------------------------------------------
// The well-known "boost serialization of cv::Mat" snippet: cols, rows, type, continuous flag,
// then the data as one array or row by row.  Quirk: data_size is `unsigned int` (truncates > 4 GB).
template <class Archive>
void save(Archive& ar, const cv::Mat& m, const unsigned int /*version*/)
{
  int cols, rows, type;
  bool continuous;

  cols = m.cols;
  rows = m.rows;
  type = m.type();
  continuous = m.isContinuous();

  ar & cols & rows & type & continuous;

  if (continuous)
  {
    const unsigned int data_size = rows * cols * m.elemSize();
    ar & boost::serialization::make_array(m.ptr(), data_size);
  }
  else
  {
    const unsigned int row_size = cols * m.elemSize();
    for (int i = 0; i < rows; i++)
    {
      ar & boost::serialization::make_array(m.ptr(i), row_size);
    }
  }
}

template <class Archive>
void load(Archive& ar, cv::Mat& m, const unsigned int /*version*/)
{
  int cols, rows, type;
  bool continuous;

  ar & cols & rows & type & continuous;

  m.create(rows, cols, type);

  if (continuous)
  {
    const unsigned int data_size = rows * cols * m.elemSize();
    ar & boost::serialization::make_array(m.ptr(), data_size);
  }
  else
  {
    const unsigned int row_size = cols * m.elemSize();
    for (int i = 0; i < rows; i++)
    {
      ar & boost::serialization::make_array(m.ptr(i), row_size);
    }
  }
}

// ---- DBoW2::BowVector --------------------------------------------------------------------------
// Copied through a plain std::map<unsigned int, double> (its own class info in the archive).
// Quirk: load does not clear `bow` first.
template <class Archive>
void save(Archive& ar, const DBoW2::BowVector& bow, const unsigned int /*version*/)
{
  std::map<DBoW2::WordId, DBoW2::WordValue> m;
  for (DBoW2::BowVector::const_iterator it = bow.begin(); it != bow.end(); ++it)
    m[it->first] = it->second;
  ar & m;
}

template <class Archive>
void load(Archive& ar, DBoW2::BowVector& bow, const unsigned int /*version*/)
{
  std::map<DBoW2::WordId, DBoW2::WordValue> m;
  ar & m;
  for (std::map<DBoW2::WordId, DBoW2::WordValue>::const_iterator it = m.begin(); it != m.end();
       ++it)
    bow[it->first] = it->second;
}

// ---- kindr::minimal::QuatTransformationTemplate<double> ---------------------------------------
// On disk: w, x, y, z (doubles), then the position as a tracked Eigen::Vector3d object.
template <class Archive>
void save(Archive& ar, const kindr::minimal::QuatTransformationTemplate<double>& T,
          const unsigned int /*version*/)
{
  const Eigen::Quaterniond q = T.getRotation().toImplementation();
  const Eigen::Vector3d t = T.getPosition();
  double w = q.w();
  double x = q.x();
  double y = q.y();
  double z = q.z();
  ar & w;
  ar & x;
  ar & y;
  ar & z;
  ar & t;
}

template <class Archive>
void load(Archive& ar, kindr::minimal::QuatTransformationTemplate<double>& T,
          const unsigned int /*version*/)
{
  double w, x, y, z;
  Eigen::Vector3d position;
  ar & w;
  ar & x;
  ar & y;
  ar & z;
  ar & position;
  // RotationQuaternion(const Eigen::Quaterniond&) == 0x1800089C0 (checks |q|^2 within 1e-4)
  T = kindr::minimal::QuatTransformationTemplate<double>(
      kindr::minimal::RotationQuaternionTemplate<double>(Eigen::Quaterniond(w, x, y, z)),
      position);
}

}  // namespace serialization
}  // namespace boost

BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat)
BOOST_SERIALIZATION_SPLIT_FREE(DBoW2::BowVector)
BOOST_SERIALIZATION_SPLIT_FREE(kindr::minimal::QuatTransformationTemplate<double>)
