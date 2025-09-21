#include "planner_node.hpp"
#include <chrono>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())), 
                             state_(State::WAITING_FOR_GOAL) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
  
  RCLCPP_INFO(this->get_logger(), "Planner Node initialized - Waiting for goal");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  map_received_ = true;
  
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    RCLCPP_INFO(this->get_logger(), "Map updated - Replanning path");
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  
  RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)", 
              goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached! Waiting for new goal");
      state_ = State::WAITING_FOR_GOAL;
    }
  }
}

bool PlannerNode::goalReached() {
  if (!goal_received_) return false;
  
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  
  return distance < GOAL_TOLERANCE;
}

void PlannerNode::planPath() {
  if (!goal_received_ || !map_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
    return;
  }
  
  CellIndex start = worldToGrid(robot_pose_.position.x, robot_pose_.position.y);
  CellIndex goal_cell = worldToGrid(goal_.point.x, goal_.point.y);
  
  if (!isValidCell(start) || !isValidCell(goal_cell)) {
    RCLCPP_ERROR(this->get_logger(), "Start or goal position is invalid!");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Planning path from (%d, %d) to (%d, %d)", 
              start.x, start.y, goal_cell.x, goal_cell.y);
  
  std::vector<CellIndex> path_cells = aStar(start, goal_cell);
  
  if (path_cells.empty()) {
    RCLCPP_WARN(this->get_logger(), "No path found to goal!");
    return;
  }
                                             
  nav_msgs::msg::Path path = pathFromCells(path_cells);
  path_pub_->publish(path);
  
  RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints", path.poses.size());
}

std::vector<CellIndex> PlannerNode::aStar(const CellIndex& start, const CellIndex& goal) {
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, bool, CellIndexHash> closed_set;
  
  g_score[start] = 0.0;
  double f_start = heuristic(start, goal);
  open_set.push(AStarNode(start, f_start));
  
  while (!open_set.empty()) {
    AStarNode current = open_set.top();
    open_set.pop();
    
    if (closed_set[current.index]) continue;
    closed_set[current.index] = true;
    
    if (current.index == goal) {
      std::vector<CellIndex> path;
      CellIndex curr = goal;
      
      while (curr != start) {
        path.push_back(curr);
        curr = came_from[curr];
      }
      path.push_back(start);
      
      std::reverse(path.begin(), path.end());
      return path;
    }
    
    std::vector<CellIndex> neighbors = getNeighbors(current.index);
    for (const CellIndex& neighbor : neighbors) {
      if (closed_set[neighbor] || !isValidCell(neighbor)) continue;
      
      int map_idx = neighbor.y * current_map_.info.width + neighbor.x;
      double map_cost = static_cast<double>(current_map_.data[map_idx]);
      double move_cost = (neighbor.x != current.index.x && neighbor.y != current.index.y) ? 1.414 : 1.0; // Diagonal vs. straight
      double tentative_g = g_score[current.index] + move_cost + map_cost;
      
      if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current.index;
        g_score[neighbor] = tentative_g;
        double f_score = tentative_g + heuristic(neighbor, goal);
        open_set.push(AStarNode(neighbor, f_score));
      }
    }
  }
  
  return std::vector<CellIndex>();
}

CellIndex PlannerNode::worldToGrid(double x, double y) {
  int grid_x = static_cast<int>((x - current_map_.info.origin.position.x) / current_map_.info.resolution);
  int grid_y = static_cast<int>((y - current_map_.info.origin.position.y) / current_map_.info.resolution);
  return CellIndex(grid_x, grid_y);
}

std::pair<double, double> PlannerNode::gridToWorld(const CellIndex& cell) {
  double x = cell.x * current_map_.info.resolution + current_map_.info.origin.position.x;
  double y = cell.y * current_map_.info.resolution + current_map_.info.origin.position.y;
  return std::make_pair(x, y);
}

bool PlannerNode::isValidCell(const CellIndex& cell) {
  if (cell.x < 0 || cell.x >= static_cast<int>(current_map_.info.width) ||
      cell.y < 0 || cell.y >= static_cast<int>(current_map_.info.height)) {
    return false;
  }
  
  int idx = cell.y * current_map_.info.width + cell.x;
  return current_map_.data[idx] < 50;
}

double PlannerNode::heuristic(const CellIndex& a, const CellIndex& b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

std::vector<CellIndex> PlannerNode::getNeighbors(const CellIndex& cell) {
  std::vector<CellIndex> neighbors;
  
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;
      neighbors.push_back(CellIndex(cell.x + dx, cell.y + dy));
    }
  }
  
  return neighbors;
}

nav_msgs::msg::Path PlannerNode::pathFromCells(const std::vector<CellIndex>& cells) {
  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "sim_world";
  
  for (const CellIndex& cell : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    
    auto world_pos = gridToWorld(cell);
    pose.pose.position.x = world_pos.first;
    pose.pose.position.y = world_pos.second;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    
    path.poses.push_back(pose);
  }
  
  return path;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}