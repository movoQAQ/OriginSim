#include "map_server/map_geometry.hpp"
namespace map_server
{
MapSettings
declareMapSettings(rclcpp::Node& node)
{
  MapSettings s;
  s.global_frame = node.declare_parameter("global_frame", std::string("map"));
  s.robot_base_frame = node.declare_parameter("robot_base_frame", std::string("base_link_fake"));
  s.robot_radius = node.declare_parameter("robot_radius", 0.43);
  if (s.global_frame != "map" || s.robot_base_frame.empty() || !std::isfinite(s.robot_radius) ||
      s.robot_radius <= 0)
    throw std::invalid_argument("Map clients require map frame and positive robot radius");
  for (int i = 0; i < 32; ++i)
  {
    geometry_msgs::msg::Point p;
    double a = i * 2 * std::acos(-1.0) / 32;
    p.x = s.robot_radius * std::cos(a);
    p.y = s.robot_radius * std::sin(a);
    s.footprint.push_back(p);
  }
  return s;
}
}
