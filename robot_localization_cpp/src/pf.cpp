#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <rclcpp/time.hpp>
#include <string>
#include <tuple>

#include "angle_helpers.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "helper_functions.hpp"
#include "occupancy_field.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
using std::placeholders::_1;

/**
 * Represents a hypothesis (particle) of the robot's pose consisting of x,y and
 * theta (yaw) Attributes: x: the x-coordinate of the hypothesis relative to the
 * map frame y: the y-coordinate of the hypothesis relative ot the map frame
 *      theta: the yaw of the hypothesis relative to the map frame
 *      w: the particle weight (the class does not ensure that particle weights
 * are normalized
 */
class Particle {
public:
  float w;
  float theta;
  float x;
  float y;

  /**
   * Construct a new Particle
   * x: the x-coordinate of the hypothesis relative to the map frame
   * y: the y-coordinate of the hypothesis relative ot the map frame
   * theta: the yaw of KeyboardInterruptthe hypothesis relative to the map
   * frame w: the particle weight (the class does not ensure that particle
   * weights are normalized
   */
  Particle(float w = 1.0, float theta = 0.0, float x = 0.0, float y = 0.0) {
    w = w;
    theta = theta;
    x = x;
    y = y;
  }

  /**
   * A helper function to convert a particle to a geometry_msgs/Pose message
   */
  geometry_msgs::msg::Pose as_pose() {
    geometry_msgs::msg::Pose pose = geometry_msgs::msg::Pose();
    pose.position.x = this->x;
    pose.position.y = this->y;
    pose.orientation = quaternion_from_euler(0, 0, this->theta);

    return pose;
  }
};

/**
 * The class that represents a Particle Filter ROS Node
 * Attributes list:
 *      base_frame: the name of the robot base coordinate frame (should be
 "base_footprint" for most robots)
 *      map_frame: the name of the map coordinate frame (should be "map" in most
 cases)
 *      odom_frame: the name of the odometry coordinate frame (should be "odom"
 in most cases)
 *      scan_topic: the name of the scan topic to listen to (should be "scan" in
 most cases)
 *      n_particles: the number of particles in the filter
 *      d_thresh: the amount of linear movement before triggering a filter
 update
 *      a_thresh: the amount of angular movement before triggering a filter
 update
 *      pose_listener: a subscriber that listens for new approximate pose
 estimates (i.e. generated through the rviz GUI)
 *      particle_pub: a publisher for the particle cloud
 *      last_scan_timestamp: this is used to keep track of the clock when using
 bags
 *      scan_to_process: the scan that our run_loop should process next
 *      occupancy_field: this helper class allows you to query the map for
 distance to closest obstacle
 *      ransform_helper: this helps with various transform operations
 (abstracting away the tf2 module)
 *      particle_cloud: a list of particles representing a probability
 distribution over robot poses
 *      current_odom_xy_theta: the pose of the robot in the odometry frame when
 the last filter update was performed. The pose is expressed as a list
 [x,y,theta] (where theta is the yaw) thread: this thread runs your main loop
*/
class ParticleFilter : public rclcpp::Node {
public:
  std::string base_frame;
  std::string map_frame;
  std::string odom_frame;
  std::string scan_topic;
  int n_particles;
  float d_thresh;
  float a_thresh;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub;
  std::optional<builtin_interfaces::msg::Time> last_scan_timestamp;
  std::optional<sensor_msgs::msg::LaserScan> scan_to_process;
  std::vector<float> current_odom_xy_theta;
  std::vector<Particle> particle_cloud;
  std::optional<geometry_msgs::msg::Pose> odom_pose;
  // OccupancyField occupancy_field;

  ParticleFilter() : Node("pf") {
    base_frame = "base_footprint"; // the frame of the robot base
    map_frame = "map";             // the name of the map coordinate frame
    odom_frame = "odom";           // the name of the odometry coordinate frame
    scan_topic = "scan"; // the topic where we will get laser scans from

    n_particles = 300; // the number of particles to use

    d_thresh = 0.2; // the amount of linear movement before performing an update
    a_thresh =
        M_PI / 6; // the amount of angular movement before performing an update

    // TODO: define additional constants if needed

    // pose_listener responds to selection of a new approximate robot
    // location (for instance using rviz)
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 10,
        std::bind(&ParticleFilter::update_initial_pose, this, _1));

    // publish the current particle cloud.  This enables viewing particles
    // in rviz.
    particle_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "particlecloud", 10);

    // laser_subscriber listens for data from the lidar
    this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 10, std::bind(&ParticleFilter::scan_received, this, _1));

    // this is used to keep track of the timestamps coming from bag files
    // knowing this information helps us set the timestamp of our map ->
    // odom transform correctly
    last_scan_timestamp.reset();
    // this is the current scan that our run_loop should process
    scan_to_process.reset();

    // your particle cloud will go here
    OccupancyField occupancy_field = OccupancyField(this);
    transform_helper_ = std::make_shared<TFHelper>(this);

    std::thread loop_thread(std::bind(&ParticleFilter::loop_wrapper, this));
    this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ParticleFilter::pub_latest_transform, this));
  }

private:
  /**
   * This function takes care of sending out the map to odom transform
   */
  void pub_latest_transform() {
    if (last_scan_timestamp.has_value()) {
      rclcpp::Time last_scan_time(last_scan_timestamp.value());
      rclcpp::Duration offset(0, 100000000);
      auto postdated_timestamp = last_scan_time + offset;
      transform_helper_->send_last_map_to_odom_transform(map_frame, odom_frame,
                                                         postdated_timestamp);
    }
  }

  /**
   * This function takes care of calling the run_loop function repeatedly.
   * We are using a separate thread to run the loop_wrapper to work around
   * issues with single threaded executors in ROS2
   */
  void loop_wrapper() {
    while (true) {
      this->run_loop();
      sleep(100);
    }
  }

  /**
   * This is the main run_loop of our particle filter.  It checks to see if
   * any scans are ready and to be processed and will call several helper
   * functions to complete the processing.
   * ou do not need to modify this function, but it is helpful to understand
   * it.
   */
  void run_loop() {
    if (!scan_to_process.has_value()) {
      return;
    }
    auto msg = scan_to_process.value();

    std::tuple<std::optional<geometry_msgs::msg::Pose>,
               std::optional<std::chrono::nanoseconds>>
        matching_odom_pose = transform_helper_->get_matching_odom_pose(
            odom_frame, base_frame, msg.header.stamp);
    auto new_pose = std::get<0>(matching_odom_pose);
    auto dt = std::get<1>(matching_odom_pose);
    if (!new_pose.has_value()) {
      // we were unable to get the pose of the robot corresponding to the
      // scan timestamp
      if (dt.has_value() && dt.value() < std::chrono::nanoseconds(0)) {
        //  we will never get this transform, since it is before our
        //  oldest one
        scan_to_process.reset();
      }
      return;
    }

    auto polar_coord = transform_helper_->convert_scan_to_polar_in_robot_frame(
        msg, base_frame);
    auto r = std::get<0>(polar_coord);
    auto theta = std::get<1>(polar_coord);
    // std::cout << std::format("r[0]={}, theta[0]={}",
    // std::get<0>(polar_coord)[0], std::get<1>(polar_coord)[0]);
    // clear the current scan so that we can process the next one
    scan_to_process.reset();

    odom_pose = new_pose;
    auto new_odom_xy_theta =
        transform_helper_->convert_pose_to_xy_theta(odom_pose.value());

    if (current_odom_xy_theta.size() == 0) {
      current_odom_xy_theta = new_odom_xy_theta;
    } else if (particle_cloud.size() == 0) {
      // now that we have all of the necessary transforms we can update
      // the particle cloud
      initialize_particle_cloud();
    } else if (moved_far_enough_to_update(new_odom_xy_theta)) {
      // we have moved far enough to do an update!
      update_particles_with_odom(); // update based on odometry
      update_particles_with_laser(r,
                                  theta); // update based on laser scan
      update_robot_pose();  // update robot's pose based on particles
      resample_particles(); // resample particles to focus on areas of
                            // high density
    }

    // publish particles (so things like rviz can see them)
    publish_particles(msg.header.stamp);
  }

  bool moved_far_enough_to_update(std::vector<float> new_odom_xy_theta) {
    return abs(new_odom_xy_theta[0] - current_odom_xy_theta[0] > d_thresh ||
               abs(new_odom_xy_theta[1] - current_odom_xy_theta[1]) >
                   d_thresh ||
               abs(new_odom_xy_theta[2] - current_odom_xy_theta[2]) > a_thresh);
  }

  /**
   * Update the estimate of the robot's pose given the updated particles.
   * There are two logical methods for this:
   * (1): compute the mean pose(2): compute the most likely pose (i.e. the
   * mode of the distribution)
   */
  void update_robot_pose() {
    // first make sure that the particle weights are normalized
    normalize_particles();

    // TODO: assign the latest pose into self.robot_pose as a
    // geometry_msgs.Pose object just to get started we will fix the robot's
    // pose to always be at the origin
    geometry_msgs::msg::Pose robot_pose;
    if (odom_pose.has_value()) {
      transform_helper_->fix_map_to_odom_transform(robot_pose,
                                                   odom_pose.value());
    } else {
      // TODO: print something
    }
  }

  /**
   * Update the particles using the newly given odometry pose.
   * The function computes the value delta which is a tuple (x,y,theta)
   * that indicates the change in position and angle between the odometry
   * when the particles were last updated and the current odometry.
   */
  void update_particles_with_odom() {
    auto new_odom_xy_theta =
        transform_helper_->convert_pose_to_xy_theta(odom_pose.value());

    // compute the change in x,y,theta since our last update
    if (current_odom_xy_theta.size() >= 0) {
      auto old_odom_xy_theta = current_odom_xy_theta;
      auto delta_x = new_odom_xy_theta[0] - current_odom_xy_theta[0];
      auto delta_y = new_odom_xy_theta[1] - current_odom_xy_theta[1];
      auto delta_theta = new_odom_xy_theta[2] - current_odom_xy_theta[2];
    } else {
      current_odom_xy_theta = new_odom_xy_theta;
      return;
    }

    // TODO: modify particles using delta
  }

  /**
   * Resample the particles according to the new particle weights.
   * The weights stored with each particle should define the probability that
   * a particular particle is selected in the resampling step.  You may want
   * to make use of the given helper function draw_random_sample in
   * helper_functions.py.
   */
  void resample_particles() {
    // make sure the distribution is normalized
    normalize_particles();
    // TODO: fill out the rest of the implementation
  }

  /**
   * Updates the particle weights in response to the scan data
   * r: the distance readings to obstacles
   * theta: the angle relative to the robot frame for each corresponding
   * reading
   */
  void update_particles_with_laser(std::vector<float> r,
                                   std::vector<float> theta) {
    // TODO: implement this
    (void)r;
    (void)theta;
  }

  /**
   * Callback function to handle re-initializing the particle filter based on
   * a pose estimate. These pose estimates could be generated by another ROS
   * Node or could come from the rviz GUI
   */
  void update_initial_pose(geometry_msgs::msg::PoseWithCovarianceStamped msg) {
    auto xy_theta = transform_helper_->convert_pose_to_xy_theta(msg.pose.pose);
    initialize_particle_cloud(xy_theta);
  }

  /**
   * Initialize the particle cloud.
   * Arguments
   *      xy_theta: a triple consisting of the mean x, y, and theta (yaw) to
   * initialize the particle cloud around.  If this input is omitted, the
   * odometry will be used
   */
  void initialize_particle_cloud(
      std::optional<std::vector<float>> xy_theta = std::nullopt) {
    if (!xy_theta.has_value()) {
      xy_theta = transform_helper_->convert_pose_to_xy_theta(odom_pose.value());
    }
    // TODO: create particles

    normalize_particles();
    update_robot_pose();
  }

  void normalize_particles() {
    // TODO: implement this
  }

  void publish_particles(rclcpp::Time timestamp) {
    geometry_msgs::msg::PoseArray poses;
    poses.header.stamp = timestamp;
    poses.header.frame_id = map_frame;

    for (unsigned int i = 0; i < particle_cloud.size(); i++) {
      poses.poses[i] = particle_cloud[i].as_pose();
    }

    // actually send the message so that we can view it in rviz
    particle_pub->publish(poses);
  }

  void scan_received(sensor_msgs::msg::LaserScan msg) {
    last_scan_timestamp = msg.header.stamp;
    /**
     * we throw away scans until we are done processing the previous scan
     * self.scan_to_process is set to None in the run_loop
     */
    if (!scan_to_process.has_value()) {
      scan_to_process = msg;
    }
  }

private:
  std::shared_ptr<TFHelper> transform_helper_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParticleFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
