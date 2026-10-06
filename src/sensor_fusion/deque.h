// src/sensor_fusion/deque.h  (path from the _wassert strings:
//   "E:\code_codex\pimax_slam\beta111_5a7902_dll\src\sensor_fusion\deque.h")
//
// Identical to LedObjectPoseEstimator src/common/deque.h (same assert texts, same line
// numbers, same virtual slot order); only the path differs.  Only the
// Deque<Eigen::Vector3d> instantiation exists (vtable 0x1803BF310, 15 slots).  No constructor
// was emitted: the only owner (DequeHolder, SLAMManager+2840) is never constructed.
//
// Layout (Deque<Vector3d>, 32 bytes):
//   +0  vptr
//   +8  T*  data_       (released with free())
//   +16 int capacity_
//   +20 int head_       (index of the front element)
//   +24 int tail_       (one past the back element)
//   +28 int count_
//
// The line numbers of the asserts are forced with #line (assert -> _wassert(..., __LINE__)); they order the
// definitions in the header as:
//   push_back 111, push_front 124, pop_front 138, pop_back 152,
//   operator[] 168, from_back 178, operator[] const 189, from_back const 199.
// Virtual slot order (MSVC groups overloads) is kept below.  The tiny accessors
// (slots 9-11, 14) are COMDAT-folded with identical Ceres functions in this image.
#pragma once

#include <cassert>
#include <cstdlib>

#include <Eigen/Core>

namespace pimax {
namespace common {

template <typename T>
class Deque {
 public:
  // TODO(verify): constructor never emitted in the binary; reconstructed guess.
  explicit Deque(int capacity)
      : data_(static_cast<T*>(std::malloc(sizeof(T) * capacity))),
        capacity_(capacity), head_(0), tail_(0), count_(0) {}

  // slot 0: 0x1801A3ED0 (scalar deleting dtor)
  virtual ~Deque() { std::free(data_); }

  // slot 1: 0x1801A4200
  virtual void push_back(const T& v) {
#line 111
    assert(count_ < capacity_ && "ElemCount < Capacity");  // line 0x6F
    data_[tail_++] = v;
    ++count_;
    if (tail_ >= capacity_) tail_ -= capacity_;
  }

  // slot 2: 0x1801A4280
  virtual void push_front(const T& v) {
#line 124
    assert(count_ < capacity_ && "ElemCount < Capacity");  // line 0x7C
    if (--head_ < 0) head_ += capacity_;
    data_[head_] = v;
    ++count_;
  }

  // slot 3: 0x1801A4100
  virtual T pop_back() {
#line 152
    assert(count_ > 0 && "ElemCount > 0");  // line 0x98
    if (--tail_ < 0) tail_ += capacity_;
    T v = data_[tail_];
    --count_;
    return v;
  }

  // slot 4: 0x1801A4180
  virtual T pop_front() {
#line 138
    assert(count_ > 0 && "ElemCount > 0");  // line 0x8A
    T v = data_[head_++];
    --count_;
    if (head_ >= capacity_) head_ -= capacity_;
    return v;
  }

  // slot 5: 0x1801A3F80  (count-th element from the back; 0 == last)
  virtual T& from_back(int count) {
#line 178
    assert(count_ > count && "ElemCount > count");  // line 0xB2
    int idx = tail_ - count - 1;
    if (idx < 0) idx += capacity_;
    return data_[idx];
  }

  // slot 6: 0x1801A3FE0
  virtual const T& from_back(int count) const {
#line 199
    assert(count_ > count && "ElemCount > count");  // line 0xC7
    int idx = tail_ - count - 1;
    if (idx < 0) idx += capacity_;
    return data_[idx];
  }

  // slot 7: 0x1801A4040
  virtual T& operator[](int count) {
#line 168
    assert(count_ > count && "ElemCount > count");  // line 0xA8
    int idx = count + head_ - capacity_;
    if (count + head_ < capacity_) idx = count + head_;
    return data_[idx];
  }

  // slot 8: 0x1801A40A0
  virtual const T& operator[](int count) const {
#line 189
    assert(count_ > count && "ElemCount > count");  // line 0xBD
    int idx = count + head_ - capacity_;
    if (count + head_ < capacity_) idx = count + head_;
    return data_[idx];
  }

  // slot 9: 0x1801A3F50
  virtual int size() const { return count_; }
  // slot 10: 0x1801A3F40
  virtual int capacity() const { return capacity_; }
  // slot 11: 0x1801A3F20  (binary returns 0 in eax; TODO(verify) return type)
  virtual void clear() {
    head_ = 0;
    tail_ = 0;
    count_ = 0;
  }
  // slot 12: 0x1801A3F60
  virtual bool empty() const { return count_ == 0; }
  // slot 13: 0x1801A3F70
  virtual bool full() const { return count_ == capacity_; }
  // slot 14: 0x1801A3F30  TODO(verify): name; returns tail_ (+24)
  virtual int tail() const { return tail_; }

 protected:
  T* data_;
  int capacity_;
  int head_;
  int tail_;
  int count_;
};

static_assert(sizeof(Deque<Eigen::Vector3d>) == 32, "common::Deque<Vector3d> size");

}  // namespace common
}  // namespace pimax
