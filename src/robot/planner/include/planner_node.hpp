#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>

#include "planner_core.hpp"

struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const {
    return (x != other.x || y != other.y);
  }
};


struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};


struct AStarNode {
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};


struct CompareF {
  bool operator()(const AStarNode &a, const AStarNode &b) {
    return a.f_score > b.f_score;
  }
};

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    robot::PlannerCore planner_;
    
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_;
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;
    
    bool goal_received_ = false;
    bool map_received_ = false;
    
    static constexpr double GOAL_TOLERANCE = 0.5;
    
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    
    void planPath();
    bool goalReached();
    std::vector<CellIndex> aStar(const CellIndex& start, const CellIndex& goal);
     
    CellIndex worldToGrid(double x, double y);
    std::pair<double, double> gridToWorld(const CellIndex& cell);
    bool isValidCell(const CellIndex& cell);
    double heuristic(const CellIndex& a, const CellIndex& b);
    std::vector<CellIndex> getNeighbors(const CellIndex& cell);
    nav_msgs::msg::Path pathFromCells(const std::vector<CellIndex>& cells);
};

#endif