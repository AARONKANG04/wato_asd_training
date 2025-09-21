#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <vector>
#include <cmath>
 
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
  robot::CostmapCore costmap_;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_;

  static constexpr double RESOLUTION = 0.1;
  static constexpr int MAP_WIDTH = 300; 
  static constexpr int MAP_HEIGHT = 300;
  static constexpr int ORIGIN_X = -15; 
  static constexpr int ORIGIN_Y = -15;
  static constexpr double INFLATION_RADIUS = 1.15;
  static constexpr int OBSTACLE_COST = 100;

  std::vector<std::vector<int>> costmap_data_;

  void initializeCostmap();
  void convertToGrid(double range, double angle, double robot_x, double robot_y, double robot_yaw, int &x, int &y);
  void markObstacle(int x_grid, int y_grid);
  void inflateObstacles();
  void publishCostmap();
};
 
#endif 
