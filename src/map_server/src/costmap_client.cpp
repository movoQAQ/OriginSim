#include "map_server/costmap_client.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
namespace map_server
{
namespace
{
DynamicHealth
health(const msg::DynamicHealth& m)
{
  if (m.cloud > 3 || m.radar_allies > 3 || m.radar_enemies > 3)
    throw std::invalid_argument("Invalid health status");
  return { static_cast<SourceStatus>(m.cloud),
           static_cast<SourceStatus>(m.radar_allies),
           static_cast<SourceStatus>(m.radar_enemies),
           m.cloud_observation_age_s,
           m.ally_observation_age_s,
           m.enemy_observation_age_s,
           m.self_filter_verified,
           m.radar_time_degraded, m.map_error };
}
GridGeometry
geometry(const nav_msgs::msg::MapMetaData& m)
{
  const auto& p = m.origin.position;
  const auto& q = m.origin.orientation;
  if (!std::isfinite(p.z) || std::abs(p.z) > 1e-9 || !std::isfinite(q.x) || !std::isfinite(q.y) ||
      !std::isfinite(q.z) || !std::isfinite(q.w) || std::abs(q.x) > 1e-9 || std::abs(q.y) > 1e-9 ||
      std::abs(q.z) > 1e-9 || std::abs(q.w - 1) > 1e-9)
    throw std::invalid_argument("Only axis-aligned map geometry is supported");
  GridGeometry g{ m.width, m.height, m.resolution, p.x, p.y };
  g.validate();
  return g;
}
}
CostmapClient::CostmapClient(const std::string& name, const rclcpp::NodeOptions& options)
  : Node(name, options)
{
  settings_ = declareMapSettings(*this);
  timeout_ = declare_parameter("map_timeout", 1.0);
  transform_tolerance_ = declare_parameter("transform_tolerance", 0.5);
  require_cloud_ = declare_parameter("require_cloud", true);
  require_radar_ = declare_parameter("require_radar", false);
  if (!std::isfinite(timeout_) || timeout_ <= 0 || !std::isfinite(transform_tolerance_) ||
      transform_tolerance_ <= 0)
    throw std::invalid_argument("Invalid map client timeout");
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
  subscription_ = create_subscription<msg::DerivedMap>(
    "/map_server/derived",
    rclcpp::QoS(1).reliable().transient_local(),
    [this](msg::DerivedMap::ConstSharedPtr m)
    {
      try
      {
        if (m->header.frame_id != "map" || !m->producer_id)
          throw std::invalid_argument("Derived map frame mismatch");
        auto g = geometry(m->info);
        const auto n = g.size();
        if (m->fused_obstacles.size() != n || m->costmap.size() != n ||
            m->distance_field.size() != n || m->collision_mask.size() != n ||
            m->region_id.size() != n || (!m->height.empty() && m->height.size() != n) ||
            (!m->depth.empty() && m->depth.size() != n) || !m->version ||
            !std::isfinite(m->hard_radius_m) || m->hard_radius_m < settings_.robot_radius ||
            !std::isfinite(m->generated_at_s))
          throw std::invalid_argument("Derived map sizes or robot envelope mismatch");
        for (size_t i = 0; i < n; ++i)
        {
          const float c = m->costmap[i], d = m->distance_field[i];
          if (m->fused_obstacles[i] > 1 || m->collision_mask[i] > 1 || std::isnan(d) || d < 0 ||
              std::isnan(c) || c < 1 || (m->collision_mask[i] != (std::isinf(c) ? 1 : 0)) ||
              (m->fused_obstacles[i] && (d != 0 || !m->collision_mask[i])) ||
              (!m->height.empty() && !std::isfinite(m->height[i])) ||
              (!m->depth.empty() && !std::isfinite(m->depth[i])))
            throw std::invalid_argument("Invalid derived values");
        }
        auto next = std::make_shared<DerivedMaps>();
        next->geometry = g;
        next->version = m->version;
        next->input_version = m->input_version;
        next->fused_obstacles = { g, m->fused_obstacles };
        next->distance_field = { g, m->distance_field };
        next->costmap = { g, m->costmap };
        next->collision_mask = { g, m->collision_mask };
        next->health_at_build = health(m->health_at_build);
        next->generated_at_s = m->generated_at_s;
        next->hard_radius_m = m->hard_radius_m;
        const rclcpp::Time stamp(m->header.stamp, get_clock()->get_clock_type());
        std::lock_guard<Costmap2D::mutex_t> lock(*grid_.getMutex());
        if (stamp < stamp_)
        {
          dynamic_.reset();
          valid_ = false;
        }
        else if (auto current = grid_.getDerivedMaps();
                 current && m->producer_id == map_producer_id_ && m->version <= current->version && valid_)
          return;
        map_producer_id_ = m->producer_id;
        grid_.setSnapshot(next);
        regions_ = m->region_id;
        height_ = m->height;
        depth_ = m->depth;
        stamp_ = stamp;
        valid_ = true;
      }
      catch (const std::exception& e)
      {
        std::lock_guard<Costmap2D::mutex_t> lock(*grid_.getMutex());
        valid_ = false;
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "Rejected derived map: %s", e.what());
      }
    });
  dynamic_sub_ = create_subscription<msg::DynamicObstacles>(
    "/map_server/dynamic",
    rclcpp::QoS(1).reliable().transient_local(),
    [this](msg::DynamicObstacles::ConstSharedPtr m)
    {
      try
      {
        if (m->header.frame_id != "map" || !m->producer_id)
          throw std::invalid_argument("Dynamic frame mismatch");
        auto v = std::make_shared<DynamicObstacleView>();
        v->input_version = m->input_version;
        v->health = health(m->health);
        if (!m->cloud_mask.empty())
        {
          auto g = geometry(m->cloud_info);
          if (g.size() != m->cloud_mask.size())
            throw std::invalid_argument("Dynamic mask size mismatch");
          for (auto c : m->cloud_mask)
            if (c > 1)
              throw std::invalid_argument("Nonbinary dynamic mask");
          v->cloud_mask = std::make_shared<BinaryGrid>(BinaryGrid{ g, m->cloud_mask });
        }
        const auto targets = [](const auto& in)
        {
          std::vector<RadarTarget> out;
          if (in.size() > 1000)
            throw std::invalid_argument("Excessive radar targets");
          for (const auto& t : in)
          {
            if (!std::isfinite(t.position.x) || !std::isfinite(t.position.y) ||
                !std::isfinite(t.physical_radius_m) || t.physical_radius_m < 0)
              throw std::invalid_argument("Invalid radar circle");
            out.push_back({ t.id,
                            { t.position.x, t.position.y },
                            rclcpp::Time(t.observed_at).seconds(),
                            0,
                            t.physical_radius_m });
          }
          return out;
        };
        v->allies = targets(m->allies);
        v->enemies = targets(m->enemies);
        const auto stamp = rclcpp::Time(m->header.stamp, get_clock()->get_clock_type());
        const double age = (now() - stamp).seconds();
        if (age < -0.1 || age > timeout_)
          throw std::invalid_argument("Stale dynamic message");
        std::lock_guard<Costmap2D::mutex_t> lock(*grid_.getMutex());
        dynamic_producer_id_ = m->producer_id;
        dynamic_ = v;
        received_ = std::chrono::steady_clock::now();
      }
      catch (const std::exception& e)
      {
        std::lock_guard<Costmap2D::mutex_t> lock(*grid_.getMutex());
        dynamic_.reset();
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "Rejected dynamic map: %s", e.what());
      }
    });
}
std::shared_ptr<const DynamicObstacleView>
CostmapClient::getLatestDynamicObstacles() const
{
  std::lock_guard<Costmap2D::mutex_t> lock(*grid_.getMutex());
  return dynamic_;
}
DynamicHealth
CostmapClient::getDynamicHealth() const
{
  std::lock_guard<Costmap2D::mutex_t> lock(*grid_.getMutex());
  return dynamic_ ? dynamic_->health : DynamicHealth{};
}
RegionId
CostmapClient::regionAt(const Eigen::Vector2d& p) const
{
  std::lock_guard<Costmap2D::mutex_t> lock(*grid_.getMutex());
  auto m = grid_.getDerivedMaps();
  auto i = m ? m->geometry.index(p) : std::optional<size_t>{};
  return i ? regions_.at(*i) : INVALID_REGION;
}
std::optional<float>
CostmapClient::heightAt(const Eigen::Vector2d& p) const
{
  std::lock_guard<Costmap2D::mutex_t> lock(*grid_.getMutex());
  auto m = grid_.getDerivedMaps();
  auto i = m ? m->geometry.index(p) : std::optional<size_t>{};
  return i && !height_.empty() ? std::optional<float>(height_.at(*i)) : std::nullopt;
}
std::optional<float>
CostmapClient::depthAt(const Eigen::Vector2d& p) const
{
  std::lock_guard<Costmap2D::mutex_t> lock(*grid_.getMutex());
  auto m = grid_.getDerivedMaps();
  auto i = m ? m->geometry.index(p) : std::optional<size_t>{};
  return i && !depth_.empty() ? std::optional<float>(depth_.at(*i)) : std::nullopt;
}
rclcpp::Time
CostmapClient::snapshotStamp() const
{
  std::lock_guard<Costmap2D::mutex_t> lock(*grid_.getMutex());
  return stamp_;
}
bool
CostmapClient::isCurrent() const
{
  std::lock_guard<Costmap2D::mutex_t> lock(*grid_.getMutex());
  // The map can be unchanged indefinitely; freshness comes from the lightweight heartbeat.
  if (!valid_ || !dynamic_ || map_producer_id_ != dynamic_producer_id_ || dynamic_->health.map_error ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() >
        timeout_)
    return false;
  return (!require_cloud_ || dynamic_->health.cloud == SourceStatus::VALID) &&
         (!require_radar_ || (dynamic_->health.radar_allies == SourceStatus::VALID &&
                              dynamic_->health.radar_enemies == SourceStatus::VALID &&
                              dynamic_->health.self_filter_verified));
}
bool
CostmapClient::getRobotPose(geometry_msgs::msg::PoseStamped& pose)
{
  try
  {
    const auto t = tf_->lookupTransform("map", settings_.robot_base_frame, tf2::TimePointZero);
    const auto stamp = rclcpp::Time(t.header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (stamp.nanoseconds() != 0 && (age < -0.1 || age > transform_tolerance_))
      return false;
    pose.header = t.header;
    pose.pose.position.x = t.transform.translation.x;
    pose.pose.position.y = t.transform.translation.y;
    pose.pose.position.z = t.transform.translation.z;
    pose.pose.orientation = t.transform.rotation;
    return true;
  }
  catch (const tf2::TransformException&)
  {
    return false;
  }
}
}
