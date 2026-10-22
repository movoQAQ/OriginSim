#include "map_server/map_server.hpp"
#include "map_server/msg/derived_map.hpp"
#include "map_server/msg/dynamic_obstacles.hpp"
#include "map_server/ros_adapters.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.hpp"
#include <algorithm>
#include <chrono>
namespace map_server
{
namespace
{
double
steadySeconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
nav_msgs::msg::MapMetaData
metadata(const GridGeometry& g)
{
  nav_msgs::msg::MapMetaData m;
  m.width = g.width;
  m.height = g.height;
  m.resolution = g.resolution;
  m.origin.position.x = g.origin_x;
  m.origin.position.y = g.origin_y;
  m.origin.orientation.w = 1;
  return m;
}
msg::DynamicHealth
healthMessage(const DynamicHealth& h, const std_msgs::msg::Header& header)
{
  msg::DynamicHealth out;
  out.header = header;
  out.cloud = static_cast<uint8_t>(h.cloud);
  out.radar_allies = static_cast<uint8_t>(h.radar_allies);
  out.radar_enemies = static_cast<uint8_t>(h.radar_enemies);
  out.cloud_observation_age_s = h.cloud_observation_age_s;
  out.ally_observation_age_s = h.ally_observation_age_s;
  out.enemy_observation_age_s = h.enemy_observation_age_s;
  out.self_filter_verified = h.self_filter_verified;
  out.radar_time_degraded = h.radar_time_degraded;
  out.map_error = h.map_error;
  return out;
}
std::vector<msg::RadarTarget>
targetMessages(const std::vector<RadarTarget>& targets)
{
  std::vector<msg::RadarTarget> out;
  for (const auto& t : targets)
  {
    msg::RadarTarget m;
    m.id = t.id;
    m.position.x = t.xy_map.x();
    m.position.y = t.xy_map.y();
    m.physical_radius_m = t.physical_radius_m;
    m.observed_at = rclcpp::Time(static_cast<int64_t>(t.observed_at_s * 1e9));
    out.push_back(m);
  }
  return out;
}
}
class MapServerNode : public rclcpp::Node
{
public:
  MapServerNode()
    : Node("map_server")
    , tf_(get_clock())
    , listener_(tf_)
  {
    MapServerConfig c;
    c.cost.robot_radius = declare_parameter("robot_radius", 0.43);
    c.cost.safety_margin = declare_parameter("safety_margin", 0.05);
    c.cost.soft_radius = declare_parameter("soft_radius", 1.5);
    c.cost.alpha = declare_parameter("alpha", 1.0);
    c.cost.beta = declare_parameter("beta", 0.1);
    c.cost.tdt_unbounded_inverse_distance =
      declare_parameter("tdt_unbounded_inverse_distance", false);
    c.update_frequency = declare_parameter("update_frequency", 20.0);
    c.cloud_timeout = declare_parameter("cloud_timeout", 0.5);
    c.accumulate_duration = declare_parameter("accumulate_duration", 0.35);
    const int frames = declare_parameter("accumulate_frames", 4),
              max_points = declare_parameter("max_cloud_points", 200000),
              max_accumulated = declare_parameter("max_accumulated_points", 500000);
    if (frames <= 0 || max_points <= 0 || max_accumulated <= 0)
      throw std::invalid_argument("Cloud budgets must be positive");
    c.accumulate_frames = frames;
    c.max_cloud_points = max_points;
    c.max_accumulated_points = max_accumulated;
    c.localization_jump_m = declare_parameter("localization_jump_m", 0.5);
    c.localization_jump_rad = declare_parameter("localization_jump_rad", 0.35);
    c.cloud_filter.roi_min_radius = declare_parameter("roi.min_radius", 0.4);
    c.cloud_filter.roi_max_radius = declare_parameter("roi.max_radius", 8.0);
    c.cloud_filter.roi_min_z = declare_parameter("roi.min_z", -0.45);
    c.cloud_filter.roi_max_z = declare_parameter("roi.max_z", 0.8);
    c.cloud_filter.voxel_size = declare_parameter("voxel_size", 0.03);
    c.cloud_filter.static_voxel_size = declare_parameter("static_voxel_size", 0.05);
    c.cloud_filter.static_distance = declare_parameter("static_distance_threshold", 0.2);
    c.cloud_filter.sor_neighbors = declare_parameter("sor.neighbors", 12);
    c.cloud_filter.sor_stddev = declare_parameter("sor.stddev_multiplier", 2.0);
    c.cloud_filter.min_density = declare_parameter("min_density_per_m2", 400.0);
    c.cloud_filter.min_component_area = declare_parameter("min_component_area", 0.0075);
    c.cloud_filter.component_connection_distance =
      declare_parameter("component_connection_distance", 0.55);
    c.cloud_memory.hit_rate = declare_parameter("memory.hit_rate", 5.0);
    c.cloud_memory.decay_rate = declare_parameter("memory.decay_rate", 0.7);
    c.cloud_memory.on_threshold = declare_parameter("memory.on_threshold", 0.6);
    c.cloud_memory.off_threshold = declare_parameter("memory.off_threshold", 0.25);
    c.cloud_memory.obstacle_timeout = declare_parameter("memory.obstacle_timeout", 1.0);
    c.cloud_memory.nominal_dt = declare_parameter("memory.nominal_dt", 0.05);
    c.cloud_memory.maximum_evidence_dt = declare_parameter("memory.maximum_evidence_dt", 0.2);
    c.radar.stale_after = declare_parameter("radar.stale_after", 2.5);
    c.radar.confidence_half_life = declare_parameter("radar.confidence_half_life", 2.0);
    c.radar.target_max_age = declare_parameter("radar.target_max_age", 4.0);
    c.radar.active_confidence_min = declare_parameter("radar.active_confidence_min", 0.25);
    c.radar.uncertainty_rate = declare_parameter("radar.uncertainty_rate", 0.1);
    c.radar.uncertainty_max = declare_parameter("radar.uncertainty_max", 0.4);
    c.radar.max_physical_radius = declare_parameter("radar.max_physical_radius", 2.0);
    int self = declare_parameter("radar.self_id", -1);
    if (self > 65535 || self < -1)
      throw std::invalid_argument("Invalid self ID");
    if (self >= 0)
      c.radar.self_id = static_cast<uint16_t>(self);
    robot_ = declare_parameter("robot_base_frame", std::string("base_link_fake"));
    max_points_ = c.max_cloud_points;
    tf_wait_ = declare_parameter("tf_wait_timeout", 0.15);
    if (!std::isfinite(tf_wait_) || tf_wait_ <= 0)
      throw std::invalid_argument("Invalid TF wait timeout");
    StaticMapFiles files{ declare_parameter("yaml_filename", std::string{}),
                          declare_parameter("region_yaml", std::string{}),
                          declare_parameter("height_yaml", std::string{}),
                          declare_parameter("depth_yaml", std::string{}),
                          declare_parameter("static_pcd", std::string{}) };
    server_ = std::make_unique<MapServer>(c);
    if (!server_->loadStaticMap(files))
      throw std::runtime_error(server_->lastError());
    static_ = loadStaticMapFiles(files);
    if (files.static_pcd.empty())
      RCLCPP_WARN(get_logger(),
                  "static_pcd is unset: cloud source unavailable; no fallback localization-package "
                  "PCD is used");
    if (files.region_yaml.empty())
      RCLCPP_WARN(get_logger(), "No Region ID raster: regionAt returns INVALID_REGION");
    if (self < 0)
      RCLCPP_WARN(get_logger(), "Radar self ID unset: self-obstacle filtering is unverified");
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    derived_pub_ = create_publisher<msg::DerivedMap>("/map_server/derived", qos);
    health_pub_ = create_publisher<msg::DynamicHealth>("/map_server/health", qos);
    dynamic_pub_ = create_publisher<msg::DynamicObstacles>("/map_server/dynamic", qos);
    debug_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map_server/costmap", qos);
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    map.header.stamp = now();
    map.info = metadata(static_.geometry);
    for (auto v : static_.occupancy.data)
      map.data.push_back(v ? 100 : 0);
    map_pub_->publish(map);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      declare_parameter("cloud_topic", std::string("/cloud_registered")),
      rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m)
      {
        const auto stamp = rclcpp::Time(m->header.stamp).nanoseconds();
        if (stamp <= last_cloud_stamp_ ||
            (latest_ && stamp <= rclcpp::Time(latest_->header.stamp).nanoseconds()) ||
            (pending_ && stamp <= rclcpp::Time(pending_->header.stamp).nanoseconds()))
          return;
        latest_ = m;
        latest_received_ = steadySeconds();
      });
    const auto radar = [this](RadarSource source, msg::RadarTargets::ConstSharedPtr m)
    {
      try
      {
        double s = steadySeconds();
        server_->advanceTime(now().seconds(), s);
        auto f = adaptRadar(*m, source, s);
        if (source == RadarSource::ALLY)
          server_->updateRadarAllies(f);
        else
          server_->updateRadarEnemies(f);
      }
      catch (const std::exception& e)
      {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected radar: %s", e.what());
      }
    };
    allies_sub_ = create_subscription<msg::RadarTargets>(
      declare_parameter("radar.allies_topic", std::string("/radar/allies")),
      10,
      [radar](msg::RadarTargets::ConstSharedPtr m) { radar(RadarSource::ALLY, m); });
    enemies_sub_ = create_subscription<msg::RadarTargets>(
      declare_parameter("radar.enemies_topic", std::string("/radar/enemies")),
      10,
      [radar](msg::RadarTargets::ConstSharedPtr m) { radar(RadarSource::ENEMY, m); });
    server_->advanceTime(now().seconds(), steadySeconds());
    server_->start();
    timer_ =
      create_wall_timer(std::chrono::duration<double>(1 / c.update_frequency), [this] { tick(); });
  }

private:
  void tick()
  {
    const double time = now().seconds(), steady = steadySeconds();
    if (time < last_time_)
    {
      latest_.reset();
      pending_.reset();
      last_cloud_stamp_ = -1;
    }
    last_time_ = time;
    server_->advanceTime(time, steady);
    if (!pending_ && latest_)
    {
      pending_ = std::move(latest_);
      pending_received_ = latest_received_;
    }
    if (pending_)
    {
      try
      {
        auto f = adaptCloud(*pending_, tf_, robot_, pending_received_, max_points_);
        server_->updateRegisteredCloud(f);
        last_cloud_stamp_ = rclcpp::Time(pending_->header.stamp).nanoseconds();
        pending_.reset();
      }
      catch (const tf2::TransformException& e)
      {
        if (steady - pending_received_ > tf_wait_)
        {
          server_->reportCloudError();
          pending_.reset();
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Cloud measurement TF unavailable: %s", e.what());
        }
      }
      catch (const std::exception& e)
      {
        server_->reportCloudError();
        pending_.reset();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected cloud: %s", e.what());
      }
    }
    std_msgs::msg::Header header;
    header.frame_id = "map";
    header.stamp = now();
    auto view = server_->getLatestDynamicObstacles();
    auto h = healthMessage(view->health, header);
    const auto worker_error = server_->getWorkerError();
    if (!worker_error.empty())
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Map worker: %s", worker_error.c_str());
    health_pub_->publish(h);
    msg::DynamicObstacles dynamic;
    dynamic.header = header;
    dynamic.producer_id = producer_id_;
    dynamic.input_version = view->input_version;
    dynamic.health = h;
    if (view->cloud_mask)
    {
      dynamic.cloud_info = metadata(view->cloud_mask->geometry);
      dynamic.cloud_mask = view->cloud_mask->data;
    }
    dynamic.allies = targetMessages(view->allies);
    dynamic.enemies = targetMessages(view->enemies);
    dynamic_pub_->publish(dynamic);
    auto m = server_->getDerivedMaps();
    if (!m || m->version == published_)
      return;
    msg::DerivedMap out;
    out.header = header;
    out.producer_id = producer_id_;
    out.info = metadata(m->geometry);
    out.version = m->version;
    out.input_version = m->input_version;
    out.fused_obstacles = m->fused_obstacles.data;
    out.distance_field = m->distance_field.data;
    out.costmap = m->costmap.data;
    out.collision_mask = m->collision_mask.data;
    out.health_at_build = healthMessage(m->health_at_build, header);
    out.generated_at_s = m->generated_at_s;
    out.hard_radius_m = m->hard_radius_m;
    out.fusion_ms = m->fusion_ms;
    out.edt_ms = m->edt_ms;
    out.costmap_ms = m->costmap_ms;
    out.cloud_ms = m->cloud_ms;
    out.region_id = static_.region_id.data;
    if (static_.height)
      out.height = static_.height->data;
    if (static_.depth)
      out.depth = static_.depth->data;
    derived_pub_->publish(out);
    published_ = m->version;
    nav_msgs::msg::OccupancyGrid debug;
    debug.header = header;
    debug.info = out.info;
    for (float v : out.costmap)
      debug.data.push_back(
        std::isfinite(v) ? static_cast<int8_t>(std::clamp((v - 1) * 20.0f, 0.0f, 99.0f)) : 100);
    debug_pub_->publish(debug);
  }
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  std::unique_ptr<MapServer> server_;
  StaticMap static_;
  std::string robot_;
  size_t max_points_;
  double tf_wait_;
  double latest_received_{ 0 }, pending_received_{ 0 }, last_time_{ 0 };
  int64_t last_cloud_stamp_{ -1 };
  uint64_t published_{ 0 };
  const uint64_t producer_id_{static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count())};
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_, pending_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<msg::RadarTargets>::SharedPtr allies_sub_, enemies_sub_;
  rclcpp::Publisher<msg::DerivedMap>::SharedPtr derived_pub_;
  rclcpp::Publisher<msg::DynamicHealth>::SharedPtr health_pub_;
  rclcpp::Publisher<msg::DynamicObstacles>::SharedPtr dynamic_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_pub_, map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try
  {
    rclcpp::spin(std::make_shared<map_server::MapServerNode>());
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("map_server"), "%s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
