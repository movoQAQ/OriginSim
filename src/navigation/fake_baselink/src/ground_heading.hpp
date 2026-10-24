#pragma once

#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>

namespace fake_baselink
{
inline double groundHeading(const tf2::Quaternion & rotation, double fallback)
{
  const tf2::Matrix3x3 matrix(rotation);
  return std::hypot(matrix[0][1], matrix[1][1]) > 0.1 ?
    std::atan2(-matrix[0][1], matrix[1][1]) : fallback;
}

struct Command {double x, y, yaw_rate;};

inline Command convertCommand(double x, double y, double yaw_rate,
  double angle_diff, double gain, double max_rate, bool align_to_motion)
{
  Command result{x * std::cos(angle_diff) + y * std::sin(angle_diff),
    -x * std::sin(angle_diff) + y * std::cos(angle_diff), yaw_rate};
  if (gain > 0.0 && std::abs(yaw_rate) < 1e-6 && std::hypot(x, y) > 0.005) {
    const double pi = std::acos(-1.0);
    const double error = align_to_motion ?
      std::remainder(std::atan2(y, x) - angle_diff, pi) :
      std::remainder(-angle_diff, 2.0 * pi);
    result.yaw_rate = std::clamp(gain * error, -max_rate, max_rate);
    if (align_to_motion && std::abs(error) > 0.25) result.x = result.y = 0.0;
  }
  return result;
}
}  // namespace fake_baselink
