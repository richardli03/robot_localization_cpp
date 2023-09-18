#include "angle_helpers.hpp"

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

std::array<float, 3>
euler_from_quaternion(geometry_msgs::msg::Quaternion quat) {
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  std::array<float, 3> rpy = {(float)roll, (float)pitch, (float)yaw};

  return rpy;
}

geometry_msgs::msg::Quaternion quaternion_from_euler(float roll, float pitch,
                                                     float yaw) {
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  geometry_msgs::msg::Quaternion quat;
  quat.x = cy * cp * sr - sy * sp * cr;
  quat.y = sy * cp * sr + cy * sp * cr;
  quat.z = sy * cp * cr - cy * sp * sr;
  quat.w = cy * cp * cr + sy * sp * sr;

  return quat;
}
