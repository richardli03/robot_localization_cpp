#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <rclcpp/time.hpp>
#include <string>
#include <tuple>
#include <eigen3/Eigen/Dense>

#include "angle_helpers.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "helper_functions.hpp"
#include "occupancy_field.hpp"
#include "pf.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <random>
using std::placeholders::_1;

Particle::Particle(float w, float theta, float x, float y)
{
  this->w = w;
  this->theta = theta;
  this->x = x;
  this->y = y;
}

/**
 * A helper function to convert a particle to a geometry_msgs/Pose message
 */
geometry_msgs::msg::Pose Particle::as_pose()
{
  geometry_msgs::msg::Pose pose = geometry_msgs::msg::Pose();
  pose.position.x = this->x;
  pose.position.y = this->y;
  pose.position.z = 0.0;
  pose.orientation = quaternion_from_euler(0, 0, this->theta);

  return pose;
}

ParticleFilter::ParticleFilter() : Node("pf")
{
  base_frame = "base_footprint"; // the frame of the robot base
  map_frame = "map";             // the name of the map coordinate frame
  odom_frame = "odom";           // the name of the odometry coordinate frame
  scan_topic = "scan";           // the topic where we will get laser scans from

  n_particles = 500; // the number of particles to use

  d_thresh = 0.1; // the amount of linear movement before performing an update
  a_thresh =
      M_PI / 6; // the amount of angular movement before performing an update

  // TODO: define additional constants if needed

  // pose_listener responds to selection of a new approximate robot
  // location (for instance using rviz)
  auto sub1_opt = rclcpp::SubscriptionOptions();
  sub1_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  initial_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10,
      std::bind(&ParticleFilter::update_initial_pose, this, _1),
      sub1_opt);

  // publish the current particle cloud.  This enables viewing particles
  // in rviz.
  particle_pub = this->create_publisher<nav2_msgs::msg::ParticleCloud>(
      "particle_cloud", 10);

  auto sub2_opt = rclcpp::SubscriptionOptions();
  sub2_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // laser_subscriber listens for data from the lidar
  laserscan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic,
      10,
      std::bind(&ParticleFilter::scan_received, this, _1),
      sub2_opt);

  // this is used to keep track of the timestamps coming from bag files
  // knowing this information helps us set the timestamp of our map ->
  // odom transform correctly
  last_scan_timestamp.reset();
  // this is the current scan that our run_loop should process
  scan_to_process.reset();

  timer = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ParticleFilter::pub_latest_transform, this));
}

void ParticleFilter::pub_latest_transform()
{
  if (last_scan_timestamp.has_value())
  {
    rclcpp::Time last_scan_time(last_scan_timestamp.value());
    rclcpp::Duration offset(0, 100000000);
    auto postdated_timestamp = last_scan_time + offset;
    transform_helper_->send_last_map_to_odom_transform(map_frame, odom_frame,
                                                       postdated_timestamp);
  }
}

void ParticleFilter::run_loop()
{
  if (!scan_to_process.has_value())
  {
    return;
  }
  auto msg = scan_to_process.value();
  std::tuple<std::optional<geometry_msgs::msg::Pose>,
             std::optional<std::chrono::nanoseconds>>
      matching_odom_pose = transform_helper_->get_matching_odom_pose(
          odom_frame, base_frame, msg.header.stamp);
  auto new_pose = std::get<0>(matching_odom_pose);
  auto dt = std::get<1>(matching_odom_pose);
  if (!new_pose.has_value())
  {
    // we were unable to get the pose of the robot corresponding to the
    // scan timestamp
    if (dt.has_value() && dt.value() < std::chrono::nanoseconds(0))
    {
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
  // clear the current scan so that we can process the next one
  scan_to_process.reset();
  odom_pose = new_pose;
  auto new_odom_xy_theta =
      transform_helper_->convert_pose_to_xy_theta(odom_pose.value());
  if (current_odom_xy_theta.size() == 0)
  {
    current_odom_xy_theta = new_odom_xy_theta;
  }
  else if (particle_cloud.size() == 0)
  {
    // now that we have all of the necessary transforms we can update
    // the particle cloud
    initialize_particle_cloud();
  }
  else if (moved_far_enough_to_update(new_odom_xy_theta))
  {
    std::cout << current_odom_xy_theta[2] << "\n";
    std::cout << "NUM PARTICLES" << particle_cloud.size() << std::endl;
    // we have moved far enough to do an update!
    update_particles_with_odom(); // update based on odometry
    update_particles_with_laser(r,
                                theta); // update based on laser scan
    update_robot_pose();                // update robot's pose based on particles
    resample_particles();               // resample particles to focus on areas of
                                        // high density
  }

  // publish particles (so things like rviz can see them)
  publish_particles(msg.header.stamp);
}

bool ParticleFilter::moved_far_enough_to_update(std::vector<float> new_odom_xy_theta)
{
  return abs(new_odom_xy_theta[0] - current_odom_xy_theta[0]) > d_thresh ||
             abs(new_odom_xy_theta[1] - current_odom_xy_theta[1]) >
                 d_thresh ||
             abs(new_odom_xy_theta[2] - current_odom_xy_theta[2]) > a_thresh;
}

void ParticleFilter::update_robot_pose()
{
  // first make sure that the particle weights are normalized
  this->normalize_particles();

  // TODO: assign the latest pose into self.robot_pose as a
  // geometry_msgs.Pose object just to get started we will fix the robot's
  // pose to always be at the origin
  Particle best_particle = this->particle_cloud[0];
  for (unsigned int i = 0; i < this->particle_cloud.size(); i++) {
    if (this->particle_cloud[i].w > best_particle.w) {
      best_particle = this->particle_cloud[i];
    }
  }
  std::cout << best_particle.x << "," << best_particle.w << std::endl;
  auto robot_pose = best_particle.as_pose();
  this->transform_helper_->fix_map_to_odom_transform(robot_pose,
                                                     this->odom_pose.value());
}

void ParticleFilter::update_particles_with_odom()
{

  auto new_odom_xy_theta =
      transform_helper_->convert_pose_to_xy_theta(odom_pose.value());
  if (!current_odom_xy_theta.size() >= 3)
  {
    current_odom_xy_theta = new_odom_xy_theta;
    return;
  }  
  auto delta_x = new_odom_xy_theta[0] - current_odom_xy_theta[0];
  auto delta_y = new_odom_xy_theta[1] - current_odom_xy_theta[1];
  auto delta_theta = new_odom_xy_theta[2] - current_odom_xy_theta[2];
  current_odom_xy_theta = new_odom_xy_theta;

  auto delta_x_n = cos(current_odom_xy_theta[2])*delta_x + sin(current_odom_xy_theta[2])*delta_y;
  auto delta_y_n = -sin(current_odom_xy_theta[2])*delta_x + cos(current_odom_xy_theta[2])*delta_y;
  
  for (auto& particle: particle_cloud) {
    particle.x += cos(particle.theta)*delta_x_n - sin(particle.theta)*delta_y_n;
    particle.y += sin(particle.theta)*delta_x_n + cos(particle.theta)*delta_y_n;
    particle.theta += delta_theta;
  }
}

void ParticleFilter::resample_particles()
{
  // make sure the distribution is normalized
  normalize_particles();
  std::vector<float> probabilities;
  std::vector<unsigned int> choices;


  for (unsigned int i = 0; i < particle_cloud.size(); i++) {
    probabilities.push_back(particle_cloud[i].w);  
    choices.push_back(i);
  }
  // TODO: fill out the rest of the implementation
  auto results = draw_random_sample(choices, probabilities, n_particles);
  
  std::vector<Particle> new_particles = particle_cloud;
  particle_cloud.clear();
  
  for (unsigned int i = 0; i < choices.size(); i++) {
    particle_cloud.push_back(new_particles[results[i]]);
  }
  
  

  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> particle_noise(0, 1);
  for (unsigned int i = 0; i < particle_cloud.size(); i++) {
    particle_cloud[i].x += particle_noise(gen) * 0.1;
    particle_cloud[i].y += particle_noise(gen) * 0.1;
    particle_cloud[i].theta += particle_noise(gen) * 0.02;
  }

  normalize_particles();
}

void ParticleFilter::update_particles_with_laser(std::vector<float> r,
                                                 std::vector<float> theta)
{
  for (unsigned int j = 0; j < particle_cloud.size(); j++) {
    int sum = 0;
    double thresh = 0.1;
    double intermediate;
    auto laser_scan_map_frame = particle_cloud[j].transform_scan_to_map(r, theta);


    // std::cout << "before" << particle_cloud[j].w << std::endl;
    for (unsigned int i = 0; i < r.size(); i++) {
      intermediate = occupancy_field->get_closest_obstacle_distance(
                      laser_scan_map_frame(0, i), laser_scan_map_frame(1, i)), r.size();
      if (intermediate < thresh) {
        sum ++;
      }
    }
    particle_cloud[j].w = sum;
    // std::cout << "after" << particle_cloud[j].w << std::endl;
  }
}

void ParticleFilter::update_initial_pose(geometry_msgs::msg::PoseWithCovarianceStamped msg)
{
  auto xy_theta = transform_helper_->convert_pose_to_xy_theta(msg.pose.pose);
  initialize_particle_cloud(xy_theta);
}

void ParticleFilter::initialize_particle_cloud(
    std::optional<std::vector<float>> xy_theta)
{
  if (!xy_theta.has_value())
  {
    xy_theta = transform_helper_->convert_pose_to_xy_theta(odom_pose.value());
  }
  
  
  double mean = 0;
  double std = 0.1;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> x_distribution(xy_theta.value()[0], std);
  std::normal_distribution<double> y_distribution(xy_theta.value()[1], std);
  std::normal_distribution<double> theta_distribution(xy_theta.value()[2], 0.2); 

  particle_cloud = std::vector<Particle>();
  // TODO: create particles
  for (unsigned int i = 0; i < n_particles; i++){
    Particle gen_particle;
    gen_particle.x = x_distribution(gen);
    gen_particle.y = y_distribution(gen);
    gen_particle.theta = theta_distribution(gen);
    // gen_particle.x = 0.0;
    // gen_particle.y = 0.0;
    // gen_particle.theta = 0.0;
    gen_particle.w = 1; // so we can see that thing
    particle_cloud.push_back(gen_particle); 
  }
  

  normalize_particles();
  update_robot_pose();
}

void ParticleFilter::normalize_particles()
{
  // RICHARD: this should modify particles in place (i'm really not sure how c++ does it)
  float sum_of_weights = 0.0;
  for (auto& particle: particle_cloud){
    sum_of_weights += particle.w;
  }

  for (auto& particle: particle_cloud){
    particle.w = particle.w / sum_of_weights;
  }
}

void ParticleFilter::publish_particles(rclcpp::Time timestamp)
{
  nav2_msgs::msg::ParticleCloud msg;
  msg.header.stamp = timestamp;
  msg.header.frame_id = map_frame;

  for (unsigned int i = 0; i < particle_cloud.size(); i++)
  {
    nav2_msgs::msg::Particle converted;
    converted.weight = particle_cloud[i].w;
    converted.pose = particle_cloud[i].as_pose();
    msg.particles.push_back(converted);
  }

  // actually send the message so that we can view it in rviz
  particle_pub->publish(msg);
}

void ParticleFilter::scan_received(sensor_msgs::msg::LaserScan msg)
{
  last_scan_timestamp = msg.header.stamp;
  /**
   * we throw away scans until we are done processing the previous scan
   * self.scan_to_process is set to None in the run_loop
   */
  if (!scan_to_process.has_value())
  {
    scan_to_process = msg;
  }
  // call run_loop to see if we need to update our filter, this will prevent more scans from coming in
  run_loop();
}

void ParticleFilter::setup_helpers(std::shared_ptr<ParticleFilter> nodePtr)
{
  occupancy_field = std::make_shared<OccupancyField>(OccupancyField(nodePtr));
  std::cout << "done generating occupancy field" << std::endl;
  transform_helper_ = std::make_shared<TFHelper>(TFHelper(nodePtr));
  std::cout << "done generating TFHelper" << std::endl;
}

Eigen::Matrix3d Particle::xy_theta_to_pose() {
  Eigen::Matrix3d pose{{cos(this->theta), -sin(this->theta), this->x},
                       {sin(this->theta), cos(this->theta), this->y},
                       {0, 0, 1}};
  return pose;
}

Eigen::Matrix3Xd Particle::transform_scan_to_map(std::vector<float> r,
                                                   std::vector<float> theta)
{
  Eigen::Matrix3Xd lidar_scan(3, r.size());

  for (unsigned int i = 0; i < r.size(); i++) {
    lidar_scan(0, i) = r[i] * cos(theta[i]);
    lidar_scan(1, i) = r[i] * sin(theta[i]);
    lidar_scan(2, i) = 1;
  }

  return this->xy_theta_to_pose() * lidar_scan;
}

int main(int argc, char **argv)
{
  // this is useful to give time for the map server to get ready...
  // TODO: fix in some other way
  sleep(5);
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<ParticleFilter>();
  node->setup_helpers(node);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}


