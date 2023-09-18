#include "occupancy_field.hpp"

#include <nav_msgs/srv/get_map.hpp>

#include "knncpp.h"
#include "rclcpp/rclcpp.hpp"

typedef knncpp::Matrixi Matrixi;

OccupancyField::OccupancyField(std::shared_ptr<rclcpp::Node> node) {
  // grab the map from the map server
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client_service =
      node->create_client<nav_msgs::srv::GetMap>("map_server/map");
  std::chrono::duration<int> service_delay(1);
  while (!map_client_service->wait_for_service(service_delay)) {
    RCLCPP_DEBUG_STREAM(node->get_logger(),
                        "service not available. waiting again...");
  }
  auto future = map_client_service->async_send_request(
      std::make_shared<nav_msgs::srv::GetMap::Request>());
  rclcpp::spin_until_future_complete(node, future);
  auto map = future.get()->map;

  // The coordinates of each grid cell in the map
  Matrix x(2, map.info.width * map.info.height);

  // while we're at it let's count the number of occupied cells
  unsigned int total_occupied = 0;
  unsigned int curr = 0;
  unsigned int ind = 0;

  for (unsigned int i = 0; i < map.info.width; i++) {
    for (unsigned int j = 0; j < map.info.height; j++) {
      // occupancy grids are stored in row major order
      ind = i + j * map.info.width;
      if (map.data[ind] > 0) {
        total_occupied++;
      }
      x(0, curr) = (float)i;
      x(1, curr) = (float)j;
      curr++;
    }
  }

  // The coordinates of each occupied grid cell in the map
  Matrix occupied_coordinates(2, total_occupied);
  curr = 0;
  for (unsigned int i = 0; i < map.info.width; i++) {
    for (unsigned int j = 0; j < map.info.height; j++) {
      // occupancy grids are stored in row major order
      unsigned int ind = i + j * map.info.width;
      if (map.data[ind] > 0) {
        occupied_coordinates(0, curr) = (float)i;
        occupied_coordinates(1, curr) = (float)j;
        curr++;
      }
    }
  }
  std::cout << "created indices " << total_occupied << std::endl;
  RCLCPP_DEBUG_STREAM(node->get_logger(), "building kd tree");
  knncpp::KDTreeMinkowskiX<double, knncpp::EuclideanDistance<double>> kdtree(
      occupied_coordinates);
  kdtree.build();

  RCLCPP_DEBUG_STREAM(node->get_logger(), "finding neighbors");
  Matrixi indices;
  Matrix distances;
  kdtree.query(x, 1, indices, distances);
  std::cout << "query done" << std::endl;
  // find neighrbors
  RCLCPP_DEBUG_STREAM(node->get_logger(), "populating occupancy field");
  Matrix closest_occ(map.info.width, map.info.height);
  // float closest_occ[map.info.width][map.info.height];
  curr = 0;

  for (unsigned int i = 0; i < map.info.width; i++) {
    for (unsigned int j = 0; j < map.info.height; j++) {
      closest_occ(i, j) = distances(curr, 0) * map.info.resolution;
      curr++;
    }
  }

  // populate occupancy field
  RCLCPP_DEBUG_STREAM(node->get_logger(), "occupancy field ready");
};

std::array<unsigned int, 4> OccupancyField::get_obstacle_bounding_box() {
  unsigned int x_min = UINT8_MAX, x_max = 0, y_min = UINT8_MAX, y_max = 0;

  for (unsigned int i = 0; i < occupied_coordinates.cols(); i++) {
    if (occupied_coordinates(0, i) < x_min) {
      x_min = occupied_coordinates(0, i);
    }
    if (occupied_coordinates(0, i) > x_max) {
      x_max = occupied_coordinates(0, i);
    }
    if (occupied_coordinates(1, i) < y_min) {
      y_min = occupied_coordinates(0, i);
    }
    if (occupied_coordinates(1, i) > y_max) {
      y_max = occupied_coordinates(0, i);
    }
  }

  std::array<unsigned int, 4> ret{{x_min, x_max, y_min, y_max}};
  return ret;
}

float OccupancyField::get_closest_obstacle_distance(float x, float y) {
  float x_coord = int((x - map.info.origin.position.x) / map.info.resolution);
  float y_coord = int((y - map.info.origin.position.y) / map.info.resolution);

  bool is_valid = (x_coord >= 0) && (y_coord >= 0) &&
                  (x_coord < map.info.width) && (y_coord < map.info.height);
  if (is_valid) {
    // TODO: fix
    // float dist = closest_occ(x_coord, y_coord);
    float dist = 0.0;
    return dist;
  } else {
    return (float)UINT16_MAX;
  }
};
