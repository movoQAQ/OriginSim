#pragma once
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "map_server/costmap_2d.hpp"
#include "map_server/map_geometry.hpp"
#include "map_server/msg/derived_map.hpp"
#include "map_server/msg/dynamic_obstacles.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include <chrono>
namespace map_server
{
// ROS consumer adapter. Naming retained for existing consumer constructor APIs.
class CostmapClient : public rclcpp::Node
{
public:
  explicit CostmapClient(const std::string& name,
                         const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  Costmap2D* getCostmap() { return &grid_; }
  std::shared_ptr<const DerivedMaps> getDerivedMaps() const { return grid_.getDerivedMaps(); }
  std::shared_ptr<const DynamicObstacleView> getLatestDynamicObstacles() const;
  DynamicHealth getDynamicHealth() const;
  RegionId regionAt(const Eigen::Vector2d& p) const;
  std::optional<float> heightAt(const Eigen::Vector2d& p) const;
  std::optional<float> depthAt(const Eigen::Vector2d& p) const;
  const MapSettings& settings() const { return settings_; }
  const std::string& getGlobalFrameID() const { return settings_.global_frame; }
  const std::string& getBaseFrameID() const { return settings_.robot_base_frame; }
  const Footprint& getRobotFootprint() const { return settings_.footprint; }
  std::shared_ptr<tf2_ros::Buffer> getTfBuffer() { return tf_; }
  bool getRobotPose(geometry_msgs::msg::PoseStamped& pose);
  bool isCurrent() const;
  rclcpp::Time snapshotStamp() const;

private:
  MapSettings settings_;
  Costmap2D grid_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<msg::DerivedMap>::SharedPtr subscription_;
  rclcpp::Subscription<msg::DynamicObstacles>::SharedPtr dynamic_sub_;
  std::shared_ptr<const DynamicObstacleView> dynamic_;
  std::vector<RegionId> regions_;
  std::vector<float> height_, depth_;
  std::chrono::steady_clock::time_point received_{};
  rclcpp::Time stamp_{ 0, 0, RCL_ROS_TIME };
  uint64_t map_producer_id_{0}, dynamic_producer_id_{0};
  bool valid_{ false }, require_cloud_, require_radar_;
  double timeout_, transform_tolerance_;
};
}
