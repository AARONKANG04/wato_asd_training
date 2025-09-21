#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), last_x_(0), last_y_(0), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, 
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
  );
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, 
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
  );
  
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/sim_world", 10);

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&MapMemoryNode::updateMap, this)
  );

  initializeGlobalMap();
  publishInitialMap();

  RCLCPP_INFO(this->get_logger(), "Map Memory Node initialized.");
}

void MapMemoryNode::initializeGlobalMap() {
  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = 0.1;
  global_map_.info.width = 1000; 
  global_map_.info.height = 1000;
  global_map_.info.origin.position.x = -50.0; 
  global_map_.info.origin.position.y = -50.0;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;
  
  global_map_.data.resize(global_map_.info.width * global_map_.info.height, -1);
}

void MapMemoryNode::publishInitialMap() {
    global_map_.header.stamp = this->get_clock()->now();
    map_pub_->publish(global_map_);
    RCLCPP_INFO(this->get_logger(), "Published initial global map (%dx%d cells, %.2fm resolution)", 
                global_map_.info.width, global_map_.info.height, global_map_.info.resolution);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg; 
  costmap_updated_ = true;
  RCLCPP_DEBUG(this->get_logger(), "Received costmap update");
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  latest_odom_ = msg;
  
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double distance = std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));

  if (distance >= DISTANCE_THRESHOLD) { 
    last_x_ = x;
    last_y_ = y;
    should_update_map_ = true;
    RCLCPP_INFO(this->get_logger(), "Robot moved %.2f meters, triggering map update", distance);
  }
}

void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_updated_) {
    RCLCPP_INFO(this->get_logger(), "Updating global map");
    integrateCostmap();
    
    global_map_.header.stamp = this->get_clock()->now();
    map_pub_->publish(global_map_);
    
    should_update_map_ = false;
    costmap_updated_ = false;
  }
}

void MapMemoryNode::integrateCostmap() {
  if (global_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Global map not initialized, cannot integrate costmap");
    return;
  }

  if (!latest_odom_) {
    RCLCPP_WARN(this->get_logger(), "No odometry data for integration. Cannot know where the robot is.");
    return;
  }

  double robot_x = latest_odom_->pose.pose.position.x;
  double robot_y = latest_odom_->pose.pose.position.y;
  
  int global_width = global_map_.info.width;
  int global_height = global_map_.info.height;
  int costmap_width = latest_costmap_.info.width;
  int costmap_height = latest_costmap_.info.height;
  
  RCLCPP_DEBUG(this->get_logger(), "Integrating costmap at robot position (%.2f, %.2f)", robot_x, robot_y);
  
  for (int costmap_y = 0; costmap_y < costmap_height; ++costmap_y) {
    for (int costmap_x = 0; costmap_x < costmap_width; ++costmap_x) {
      
      double costmap_world_x = (costmap_x * latest_costmap_.info.resolution) + latest_costmap_.info.origin.position.x;
      double costmap_world_y = (costmap_y * latest_costmap_.info.resolution) + latest_costmap_.info.origin.position.y;
      
      double global_world_x = robot_x + costmap_world_x;
      double global_world_y = robot_y + costmap_world_y;
      
      int global_x = static_cast<int>((global_world_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
      int global_y = static_cast<int>((global_world_y - global_map_.info.origin.position.y) / global_map_.info.resolution);
      
      if (global_x >= 0 && global_x < global_width && global_y >= 0 && global_y < global_height) {
        
        int costmap_idx = costmap_y * costmap_width + costmap_x;
        int global_idx = global_y * global_width + global_x;
        
        int8_t costmap_value = latest_costmap_.data[costmap_idx];
        
        if (costmap_value != -1) {
          global_map_.data[global_idx] = costmap_value;
        }
      }
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}