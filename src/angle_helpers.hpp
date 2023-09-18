#include "geometry_msgs/msg/quaternion.hpp"

/**
 * Convert a quaternion into euler angles (roll, pitch, yaw) roll is rotation
 * around x in radians (counterclockwise) pitch is rotation around y in radians
 * (counterclockwise) yaw is rotation around z in radians (counterclockwise)
 */
geometry_msgs::msg::Quaternion quaternion_from_euler(float roll, float pitch,
                                                     float yaw);
/**
 * Converts euler roll, pitch, yaw to quaternion (w in last place) quat = [x, y,
 * z, w]
 */
std::array<float, 3> euler_from_quaternion(geometry_msgs::msg::Quaternion quat);