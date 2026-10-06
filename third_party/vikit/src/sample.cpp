// Upstream-identical copy of rpg_svo_pro_open/vikit/vikit_common/src/sample.cpp.
// Evidence it is linked: dynamic initialisers 0x180003110 (std::mt19937 default seed 5489 -> gen_int,
// state @0x18046A3E0) and 0x180003150 (std::ranlux24 default seed 19780503 -> gen_real, engine
// @0x18047F050, seeding helper 0x1801B5A80 = std::_Swc_base<...>::_Seed). The initialisers appear in
// decorated-name order (??__Egen_int < ??__Egen_real), not definition order. The Sample:: functions
// themselves are not in the image (unreferenced). TODO(verify) that gen_int is the engine used by
// 0x1801448C0 / 0x180147110 (they use the mt19937 @0x18046A3E0).
/*
 * sample.cpp
 *
 *  Created on: May 14, 2013
 *      Author: cforster
 */

#define _USE_MATH_DEFINES  // build fix for MSVC (M_PI); not semantically relevant
#include <cmath>
#include <vikit/sample.h>

namespace vk {

std::ranlux24 Sample::gen_real;
std::mt19937 Sample::gen_int;

void Sample::setTimeBasedSeed()
{
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  gen_real = std::ranlux24(seed);
  gen_int = std::mt19937(seed);
}

int Sample::uniform(int from, int to)
{
  std::uniform_int_distribution<int> distribution(from, to);
  return distribution(gen_int);
}

double Sample::uniform()
{
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  return distribution(gen_real);
}

double Sample::gaussian(double stddev)
{
  std::normal_distribution<double> distribution(0.0, stddev);
  return distribution(gen_real);
}

Eigen::Vector3d Sample::randomDirection3D()
{
  // equal-area projection according to:
  // https://math.stackexchange.com/questions/44689/how-to-find-a-random-axis-or-unit-vector-in-3d
  const double z = Sample::uniform()*2.0-1.0;
  const double t = Sample::uniform()*2.0*M_PI;
  const double r = std::sqrt(1.0 - z*z);
  const double x = r*std::cos(t);
  const double y = r*std::sin(t);
  return Eigen::Vector3d(x,y,z);
}

Eigen::Vector2d Sample::randomDirection2D()
{
  const double theta = Sample::uniform()*2.0*M_PI;
  return Eigen::Vector2d(std::cos(theta), std::sin(theta));
}

} // namespace vk
