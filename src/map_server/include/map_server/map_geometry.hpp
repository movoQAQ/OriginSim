#pragma once
#include "map_server/footprint.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
namespace map_server
{
struct MapSettings
{
  std::string global_frame{ "map" }, robot_base_frame;
  Footprint footprint;
  double robot_radius{ 0.3 };
};
MapSettings
declareMapSettings(rclcpp::Node& node);
}
