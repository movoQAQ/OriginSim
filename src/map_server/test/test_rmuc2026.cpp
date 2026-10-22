#include "map_server/costmap_generator.hpp"
#include "map_server/edt_2d.hpp"
#include "map_server/static_map.hpp"
#include "topology_global_planner/grid/yastar_core.hpp"
#include <algorithm>
#include <fstream>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

namespace {
using namespace map_server;
using topology_global_planner::grid::YAstarCore;

CostmapOptions configured(const std::string &path) {
  const auto p = YAML::LoadFile(path)["map_server"]["ros__parameters"];
  CostmapOptions o;
  o.robot_radius = p["robot_radius"].as<double>();
  o.safety_margin = p["safety_margin"].as<double>();
  o.soft_radius = p["soft_radius"].as<double>();
  o.alpha = p["alpha"].as<double>();
  o.beta = p["beta"].as<double>();
  o.tdt_unbounded_inverse_distance = p["tdt_unbounded_inverse_distance"].as<bool>();
  o.validate();
  return o;
}

class Rmuc2026 : public ::testing::Test {
protected:
  void SetUp() override {
    StaticMapFiles files;
    files.occupancy_yaml = std::string(RMUC2026_BRINGUP) + "/map/rmuc_2026.yaml";
    const auto map = loadStaticMapFiles(files);
    tuned.geometry = map.geometry;
    tuned.fused_obstacles = map.occupancy;
    tuned.distance_field = preciseEdt(map.occupancy);
    options = configured(std::string(RMUC2026_BRINGUP) + "/params/params.yaml");
    generateCosts(tuned, options);
  }
  size_t pixel(unsigned column, unsigned row) const {
    return (tuned.geometry.height - 1 - row) * tuned.geometry.width + column;
  }
  DerivedMaps tuned;
  CostmapOptions options;
};

TEST_F(Rmuc2026, SofterRealMapCostsPreserveConfiguredCollisionEnvelope) {
  ASSERT_EQ(tuned.geometry.width, 583u);
  ASSERT_EQ(tuned.geometry.height, 300u);
  ASSERT_DOUBLE_EQ(tuned.geometry.resolution, 0.05);
  EXPECT_DOUBLE_EQ(options.robot_radius, 0.3);
  EXPECT_DOUBLE_EQ(options.safety_margin, 0.05);
  EXPECT_FALSE(options.tdt_unbounded_inverse_distance);
  const auto standalone = configured(RMUC2026_SERVER_PARAMS);
  EXPECT_DOUBLE_EQ(options.robot_radius, standalone.robot_radius);
  EXPECT_DOUBLE_EQ(options.soft_radius, standalone.soft_radius);
  EXPECT_DOUBLE_EQ(options.alpha, standalone.alpha);
  EXPECT_DOUBLE_EQ(options.beta, standalone.beta);
  EXPECT_DOUBLE_EQ(options.soft_radius, CostmapOptions{}.soft_radius);
  EXPECT_DOUBLE_EQ(options.alpha, CostmapOptions{}.alpha);
  EXPECT_DOUBLE_EQ(options.beta, CostmapOptions{}.beta);

  auto old = tuned;
  auto previous = options;
  previous.soft_radius = 1.5;
  previous.alpha = 1.0;
  previous.beta = 0.1;
  generateCosts(old, previous);
  EXPECT_EQ(tuned.collision_mask.data, old.collision_mask.data);
  size_t finite = 0, old_flat = 0, new_flat = 0;
  float maximum = 1;
  std::vector<float> clearance;
  for (size_t i = 0; i < tuned.geometry.size(); ++i) {
    if (!std::isfinite(tuned.costmap.data[i])) {continue;}
    ++finite;
    old_flat += old.costmap.data[i] == 1;
    new_flat += tuned.costmap.data[i] == 1;
    maximum = std::max(maximum, tuned.costmap.data[i]);
    clearance.push_back(tuned.distance_field.data[i]);
    EXPECT_GE(tuned.costmap.data[i], 1);
    EXPECT_LE(tuned.costmap.data[i], old.costmap.data[i]);
  }
  ASSERT_GT(finite, 60000u);
  ASSERT_GT(new_flat, finite * 0.4);
  EXPECT_LT(old_flat, finite / 10);
  EXPECT_LT(maximum, 1.6f);
  std::sort(clearance.begin(), clearance.end());
  const double median = clearance[clearance.size() / 2];
  EXPECT_GT(median, 0.65);
  EXPECT_LT(median, 0.9);
  std::ofstream report(RMUC2026_REPORT);
  report << "{\n  \"map\": \"rmuc_2026.pgm\",\n"
    << "  \"resolution_m\": " << tuned.geometry.resolution << ",\n"
    << "  \"robot_radius_m\": " << options.robot_radius << ",\n"
    << "  \"safety_margin_m\": " << options.safety_margin << ",\n"
    << "  \"soft_radius_m\": " << options.soft_radius << ",\n"
    << "  \"alpha\": " << options.alpha << ",\n  \"beta_m\": " << options.beta << ",\n"
    << "  \"finite_cells\": " << finite << ",\n"
    << "  \"old_unit_cost_cells\": " << old_flat << ",\n"
    << "  \"new_unit_cost_cells\": " << new_flat << ",\n"
    << "  \"median_safe_center_distance_m\": " << median << ",\n"
    << "  \"maximum_finite_cost\": " << maximum << ",\n"
    << "  \"collision_mask_changed\": false\n}\n";
  ASSERT_TRUE(report.good());
}

TEST_F(Rmuc2026, BothMeasuredSlotsBecomeTraversableWithSpecifiedRadius) {
  struct Slot {unsigned first_column, first_row, last_row, start_row, goal_row;};
  const Slot slots[] = {{377, 45, 70, 15, 90}, {188, 230, 265, 205, 285}};
  auto legacy = tuned;
  auto old_radius = options;
  old_radius.robot_radius = 0.43;
  generateCosts(legacy, old_radius);
  for (const auto &slot : slots) {
    float max_distance = 0;
    for (unsigned r = slot.first_row; r <= slot.last_row; ++r) {
      for (unsigned c = slot.first_column; c < slot.first_column + 17; ++c) {
        const auto i = pixel(c, r);
        ASSERT_FALSE(tuned.fused_obstacles.data[i]);
        EXPECT_TRUE(legacy.collision_mask.data[i]);
        max_distance = std::max(max_distance, tuned.distance_field.data[i]);
      }
    }
    EXPECT_NEAR(max_distance, 0.45, 1e-6);
    auto passage = tuned;
    // Limit the search to the measured slot; otherwise a global detour could
    // incorrectly be reported as traversal of the tunnel.
    for (unsigned r = 0; r < tuned.geometry.height; ++r) {
      for (unsigned c = 0; c < tuned.geometry.width; ++c) {
        if (c < slot.first_column || c >= slot.first_column + 17 ||
            r < slot.start_row || r > slot.goal_row) {
          const auto i = pixel(c, r);
          passage.collision_mask.data[i] = 1;
          passage.costmap.data[i] = INFINITY;
        }
      }
    }
    const auto start = pixel(slot.first_column + 8, slot.start_row);
    const auto goal = pixel(slot.first_column + 8, slot.goal_row);
    ASSERT_TRUE(std::isfinite(passage.costmap.data[start]));
    ASSERT_TRUE(std::isfinite(passage.costmap.data[goal]));
    const auto path = YAstarCore::search(passage, start, goal, 0, [] {return false;});
    ASSERT_FALSE(path.empty());
    EXPECT_EQ(path.front(), start);
    EXPECT_EQ(path.back(), goal);
    for (auto i : path) {
      EXPECT_FALSE(passage.collision_mask.data[i]);
      EXPECT_TRUE(std::isfinite(passage.costmap.data[i]));
    }
  }
}

TEST_F(Rmuc2026, RealMapRouteUsesFiniteCostsWithoutDoubleInflation) {
  const auto start = tuned.geometry.index({3.4, 9.5});
  const auto goal = tuned.geometry.index({9.4, 9.5});
  ASSERT_TRUE(start);
  ASSERT_TRUE(goal);
  const auto path = YAstarCore::search(tuned, *start, *goal, 0, [] {return false;});
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(path.front(), *start);
  EXPECT_EQ(path.back(), *goal);
  for (auto i : path) {
    EXPECT_FALSE(tuned.collision_mask.data[i]);
    EXPECT_TRUE(std::isfinite(tuned.costmap.data[i]));
  }
}
}  // namespace
