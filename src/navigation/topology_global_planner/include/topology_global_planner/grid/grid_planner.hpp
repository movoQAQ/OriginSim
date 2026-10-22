// 读取代价地图并生成栅格路径
#pragma once

#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "map_server/map_view.hpp"
#include "rclcpp/rclcpp.hpp"



namespace topology_global_planner::grid
{

enum class GoalPolicy
{
  EXACT,
  WITHIN_TOLERANCE
};

class GridPlanner
{
public:
  struct Options
  {
    double tolerance{0.2};
    bool use_final_approach_orientation{true};
  };

  GridPlanner(const std::string & global_frame,
    rclcpp::Clock::SharedPtr clock,
    Options options);

  nav_msgs::msg::Path plan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const map_server::MapView & view,
    GoalPolicy goal_policy,
    std::function<bool()> cancel_checker);

private:
  std::string global_frame_;
  rclcpp::Clock::SharedPtr clock_;
  Options options_;


};

}