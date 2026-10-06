// src/sensor_fusion/vector.h (TODO(verify) path) -- identical to LedObjectPoseEstimator
// ctrl_three_dof/vector.h (Cardboard SDK util/vector.h), namespace pimax::ThreeDof.
// Dot/Cross are emitted out-of-line:
//   0x1801A5D70  double Dot(const Vector<3>&, const Vector<3>&)
//   0x1801A5DA0  double Dot(const Vector<4>&, const Vector<4>&)
//   0x1801A5D10  Vector<3> Cross(const Vector<3>&, const Vector<3>&)
// Length/Normalize/Normalized are inlined everywhere (std::sqrt with the
// "x < 0 ? call sqrt : sqrtsd" pattern).
#pragma once

#include <cmath>
#include <cstddef>

namespace pimax {
namespace ThreeDof {

template <int Dimension>
class Vector {
 public:
  Vector() {
    for (int i = 0; i < Dimension; ++i) elem_[i] = 0;
  }
  Vector(double e0, double e1, double e2) {
    static_assert(Dimension == 3, "3 args");
    elem_[0] = e0;
    elem_[1] = e1;
    elem_[2] = e2;
  }
  Vector(double e0, double e1, double e2, double e3) {
    static_assert(Dimension == 4, "4 args");
    elem_[0] = e0;
    elem_[1] = e1;
    elem_[2] = e2;
    elem_[3] = e3;
  }
  static Vector Zero() { return Vector(); }

  double& operator[](int i) { return elem_[i]; }
  const double& operator[](int i) const { return elem_[i]; }

  Vector& operator+=(const Vector& v) {
    for (int i = 0; i < Dimension; ++i) elem_[i] += v[i];
    return *this;
  }
  Vector& operator-=(const Vector& v) {
    for (int i = 0; i < Dimension; ++i) elem_[i] -= v[i];
    return *this;
  }
  Vector& operator*=(double s) {
    for (int i = 0; i < Dimension; ++i) elem_[i] *= s;
    return *this;
  }
  Vector& operator/=(double s) {
    for (int i = 0; i < Dimension; ++i) elem_[i] /= s;
    return *this;
  }
  friend Vector operator+(const Vector& a, const Vector& b) {
    Vector r = a;
    r += b;
    return r;
  }
  friend Vector operator-(const Vector& a, const Vector& b) {
    Vector r = a;
    r -= b;
    return r;
  }
  friend Vector operator*(const Vector& a, double s) {
    Vector r = a;
    r *= s;
    return r;
  }
  friend Vector operator/(const Vector& a, double s) {
    Vector r = a;
    r /= s;
    return r;
  }

 private:
  double elem_[Dimension];
};

typedef Vector<3> Vector3;
typedef Vector<4> Vector4;

// 0x1801A5D70 (Dimension 3) / 0x1801A5DA0 (Dimension 4)
template <int Dimension>
double Dot(const Vector<Dimension>& v0, const Vector<Dimension>& v1) {
  double result = v0[0] * v1[0];
  for (int i = 1; i < Dimension; ++i) result += v0[i] * v1[i];
  return result;
}

// 0x1801A5D10
inline Vector<3> Cross(const Vector<3>& a, const Vector<3>& b) {
  return Vector<3>(a[1] * b[2] - a[2] * b[1],
                   a[2] * b[0] - a[0] * b[2],
                   a[0] * b[1] - a[1] * b[0]);
}

template <int Dimension>
double Length(const Vector<Dimension>& v) {
  return std::sqrt(Dot(v, v));
}

template <int Dimension>
double Normalize(Vector<Dimension>* v) {
  const double len = Length(*v);
  if (len != 0.0) *v /= len;
  return len;
}

template <int Dimension>
Vector<Dimension> Normalized(const Vector<Dimension>& v) {
  Vector<Dimension> result = v;
  if (Normalize(&result))
    return result;
  else
    return Vector<Dimension>::Zero();
}

}  // namespace ThreeDof
}  // namespace pimax
