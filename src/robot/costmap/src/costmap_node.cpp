#include <chrono>
#include <memory>

#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&CostmapNode::odomCallback, this, std::placeholders::_1)
  );

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  initializeCostmap();
}

void CostmapNode::initializeCostmap() {
  costmap_data_.resize(MAP_HEIGHT, std::vector<int>(MAP_WIDTH, 0)); // no detections yet so fill with zeros
}


void CostmapNode::convertToGrid(double range, double angle, double robot_x, double robot_y, double robot_yaw, int &x, int &y) {

  double x_robot = range * cos(angle);
  double y_robot = range * sin(angle);

  double x_world = robot_x + (x_robot * cos(robot_yaw) - y_robot * sin(robot_yaw));
  double y_world = robot_y + (x_robot * sin(robot_yaw) + y_robot * cos(robot_yaw));

  x = static_cast<int>((x_world - ORIGIN_X) / RESOLUTION);
  y = static_cast<int>((y_world - ORIGIN_Y) / RESOLUTION);
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
  if (x_grid >= 0 && x_grid < MAP_WIDTH && y_grid >= 0 && y_grid < MAP_HEIGHT) { // check bounds
    costmap_data_[y_grid][x_grid] = OBSTACLE_COST;
  }
}

void CostmapNode::inflateObstacles() {
  std::vector<std::vector<int>> inflated_map = costmap_data_;
  
  int inflation_cells = static_cast<int>(INFLATION_RADIUS / RESOLUTION);
  for (int y = 0; y < MAP_HEIGHT; y++) {
    for (int x = 0; x < MAP_WIDTH; x++) {
      if (costmap_data_[y][x] == OBSTACLE_COST) {
        for (int dy = -inflation_cells; dy <= inflation_cells; dy++) {
          for (int dx = -inflation_cells; dx <= inflation_cells; dx++) {
            int nx = x + dx;
            int ny = y + dy;

            if (nx < 0 || nx >= MAP_WIDTH || ny < 0 || ny >= MAP_HEIGHT) continue;

            double distance = sqrt(dx * dx + dy * dy) * RESOLUTION;
            if (distance <= INFLATION_RADIUS) {
              int cost = static_cast<int>(OBSTACLE_COST * (1.0 - (distance / INFLATION_RADIUS)));
              if (cost > inflated_map[ny][nx]) {
                inflated_map[ny][nx] = cost;
              }
            }
          }
        }
      }
    }
  }
  costmap_data_ = inflated_map;
}

void CostmapNode::publishCostmap() {
  nav_msgs::msg::OccupancyGrid msg;

  msg.header.stamp = this->now();
  msg.header.frame_id = "sim_world";

  msg.info.resolution = RESOLUTION;
  msg.info.width = MAP_WIDTH;
  msg.info.height = MAP_HEIGHT;
  msg.info.origin.position.x = ORIGIN_X;
  msg.info.origin.position.y = ORIGIN_Y;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data.resize(MAP_WIDTH * MAP_HEIGHT);
  for (int y = 0; y < MAP_HEIGHT; y++) {
    for (int x = 0; x < MAP_WIDTH; x++) {
      msg.data[y * MAP_WIDTH + x] = costmap_data_[y][x];
    }
  }

  costmap_pub_->publish(msg);
}

void CostmapNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  RCLCPP_INFO(this->get_logger(), "Received odom: Robot at (%.2f, %.2f)", x, y);
  latest_odom_ = msg;
}

double extractYaw(const geometry_msgs::msg::Quaternion& quat) {
  double t0 = +2.0 * (quat.w * quat.z + quat.x * quat.y);
  double t1 = +1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(t0, t1);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  initializeCostmap();
  
  if (!latest_odom_) {
    RCLCPP_WARN(this->get_logger(), "No odometry available for transformation, skipping laser update");
    return;
  }
  
  double robot_x = latest_odom_->pose.pose.position.x;
  double robot_y = latest_odom_->pose.pose.position.y;
  double robot_yaw = extractYaw(latest_odom_->pose.pose.orientation);
  
  for (size_t i = 0; i < scan->ranges.size(); i++) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    if (range < scan->range_max && range > scan->range_min) {
      int x_grid, y_grid;
      convertToGrid(range, angle, robot_x, robot_y, robot_yaw, x_grid, y_grid);
      markObstacle(x_grid, y_grid);
    }
  }
  inflateObstacles();
  publishCostmap();
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}