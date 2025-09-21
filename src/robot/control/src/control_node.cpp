#include "control_node.hpp"
#include <chrono>

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
  
  RCLCPP_INFO(this->get_logger(), "Pure Pursuit Control Node initialized");
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = msg;
  RCLCPP_INFO(this->get_logger(), "Received new path with %zu waypoints", msg->poses.size());
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = msg;
}

void ControlNode::controlLoop() {
  if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }
  

  if (isGoalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping robot");
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }

  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    RCLCPP_WARN(this->get_logger(), "No valid lookahead point found");
    return;
  }

  auto cmd_vel = computeVelocity(*lookahead_point);
  
  cmd_vel_pub_->publish(cmd_vel);
  
  RCLCPP_DEBUG(this->get_logger(), "Published cmd_vel: linear=%.2f, angular=%.2f", 
               cmd_vel.linear.x, cmd_vel.angular.z);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  if (!current_path_ || current_path_->poses.empty()) {
    return std::nullopt;
  }
  
  double robot_x = robot_odom_->pose.pose.position.x;
  double robot_y = robot_odom_->pose.pose.position.y;
  
  int closest_idx = findClosestWaypoint();
  
  for (size_t i = closest_idx; i < current_path_->poses.size(); ++i) {
    double waypoint_x = current_path_->poses[i].pose.position.x;
    double waypoint_y = current_path_->poses[i].pose.position.y;
    
    geometry_msgs::msg::Point robot_pos;
    robot_pos.x = robot_x;
    robot_pos.y = robot_y;
    
    geometry_msgs::msg::Point waypoint_pos;
    waypoint_pos.x = waypoint_x;
    waypoint_pos.y = waypoint_y;
    
    double distance = computeDistance(robot_pos, waypoint_pos);
    
    if (distance >= 1.0) {
      return current_path_->poses[i];
    }
  }
  
  return current_path_->poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped& target) {
  geometry_msgs::msg::Twist cmd_vel;
  
  double robot_x = robot_odom_->pose.pose.position.x;
  double robot_y = robot_odom_->pose.pose.position.y;
  double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);
  
  double target_x = target.pose.position.x;
  double target_y = target.pose.position.y;
  
  double angle_to_target = atan2(target_y - robot_y, target_x - robot_x);
  
  double steering_angle = angle_to_target - robot_yaw;
  
  while (steering_angle > M_PI) steering_angle -= 2.0 * M_PI;
  while (steering_angle < -M_PI) steering_angle += 2.0 * M_PI;
  
  double angular_velocity = steering_angle * 2.0;
  
  if (angular_velocity > 1.0) angular_velocity = 1.0;
  if (angular_velocity < -1.0) angular_velocity = -1.0;
  
  double linear_velocity = 0.5;
  if (abs(steering_angle) > M_PI / 4) {
    linear_velocity *= 0.5;
  }
  
  cmd_vel.linear.x = linear_velocity;
  cmd_vel.angular.z = angular_velocity;
  
  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return sqrt(dx * dx + dy * dy);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion& quat) {
  double w = quat.w;
  double x = quat.x;
  double y = quat.y;
  double z = quat.z;
  
  return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

int ControlNode::findClosestWaypoint() {
  if (!current_path_ || current_path_->poses.empty()) {
    return 0;
  }
  
  double robot_x = robot_odom_->pose.pose.position.x;
  double robot_y = robot_odom_->pose.pose.position.y;
  
  int closest_idx = 0;
  double min_distance = std::numeric_limits<double>::max();
  
  for (size_t i = 0; i < current_path_->poses.size(); ++i) {
    geometry_msgs::msg::Point robot_pos;
    robot_pos.x = robot_x;
    robot_pos.y = robot_y;
    
    geometry_msgs::msg::Point waypoint_pos;
    waypoint_pos.x = current_path_->poses[i].pose.position.x;
    waypoint_pos.y = current_path_->poses[i].pose.position.y;
    
    double distance = computeDistance(robot_pos, waypoint_pos);
    
    if (distance < min_distance) {
      min_distance = distance;
      closest_idx = i;
    }
  }
  
  return closest_idx;
}

bool ControlNode::isGoalReached() {
  if (!current_path_ || current_path_->poses.empty()) {
    return false;
  }
  
  auto final_waypoint = current_path_->poses.back();
  
  geometry_msgs::msg::Point robot_pos;
  robot_pos.x = robot_odom_->pose.pose.position.x;
  robot_pos.y = robot_odom_->pose.pose.position.y;
  
  geometry_msgs::msg::Point goal_pos;
  goal_pos.x = final_waypoint.pose.position.x;
  goal_pos.y = final_waypoint.pose.position.y;
  
  double distance_to_goal = computeDistance(robot_pos, goal_pos);
  
  return distance_to_goal < 0.2;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}