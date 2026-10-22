#include "map_server/costmap_generator.hpp"
#include "map_server/map_client.hpp"
#include <atomic>
#include <gtest/gtest.h>
#include <thread>
namespace {
class MapClientTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    producer = std::make_shared<rclcpp::Node>("map_test_producer");
    client = std::make_shared<map_server::MapClient>(
        "map_test_client", rclcpp::NodeOptions().parameter_overrides(
                               {rclcpp::Parameter("require_cloud", false)}));
    executor.add_node(client);
    executor.add_node(producer);
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    statics = producer->create_publisher<map_server::msg::StaticMap>(
        "/map_server/static", qos);
    maps = producer->create_publisher<map_server::msg::DerivedMap>(
        "/map_server/derived", qos);
    dynamics = producer->create_publisher<map_server::msg::DynamicObstacles>(
        "/map_server/dynamic", qos);
    statuses = producer->create_publisher<map_server::msg::MapStatus>(
        "/map_server/status", qos);
    m.info.width = 20;
    m.info.height = 20;
    m.info.resolution = .1;
    m.info.origin.orientation.w = 1;
    map_server::DerivedMaps state;
    state.geometry = {20, 20, static_cast<double>(m.info.resolution), 0, 0};
    state.fused_obstacles = {state.geometry, std::vector<uint8_t>(400, 0)};
    state.distance_field = {state.geometry, std::vector<float>(400, INFINITY)};
    map_server::generateCosts(state, {});
    m.header.frame_id = "map";
    m.fused_obstacles = state.fused_obstacles.data;
    m.distance_field = state.distance_field.data;
    m.costmap = state.costmap.data;
    m.collision_mask = state.collision_mask.data;
    m.hard_radius_m = .48;
    s.header.frame_id = d.header.frame_id = h.header.frame_id = "map";
    s.info = m.info;
    s.occupancy.assign(400, 0);
    s.region_id.assign(400, 0);
    ASSERT_TRUE(wait([&] {
      return statics->get_subscription_count() &&
             maps->get_subscription_count() &&
             dynamics->get_subscription_count() &&
             statuses->get_subscription_count();
    }));
  }
  void TearDown() override {
    executor.remove_node(client);
    executor.remove_node(producer);
    client.reset();
    producer.reset();
  }
  template <class F> bool wait(F predicate) {
    for (int i = 0; i < 250; ++i) {
      executor.spin_some();
      if (predicate())
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
    return false;
  }
  void data(uint64_t id, uint64_t version, bool static_map = true) {
    m.header.stamp = producer->now();
    m.generated_at_s = producer->now().seconds();
    m.producer_id = id;
    m.version = version;
    maps->publish(m);
    d.header.stamp = producer->now();
    d.producer_id = id;
    d.version = version;
    dynamics->publish(d);
    if (static_map) {
      s.header.stamp = producer->now();
      s.producer_id = id;
      statics->publish(s);
    }
  }
  void heartbeat(uint64_t id, uint64_t version) {
    h.header.stamp = producer->now();
    h.producer_id = id;
    h.derived_version = h.dynamic_version = version;
    statuses->publish(h);
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<rclcpp::Node> producer;
  std::shared_ptr<map_server::MapClient> client;
  rclcpp::Publisher<map_server::msg::StaticMap>::SharedPtr statics;
  rclcpp::Publisher<map_server::msg::DerivedMap>::SharedPtr maps;
  rclcpp::Publisher<map_server::msg::DynamicObstacles>::SharedPtr dynamics;
  rclcpp::Publisher<map_server::msg::MapStatus>::SharedPtr statuses;
  map_server::msg::StaticMap s;
  map_server::msg::DerivedMap m;
  map_server::msg::DynamicObstacles d;
  map_server::msg::MapStatus h;
};
TEST_F(MapClientTest,
       AllChannelsMustAgreeBeforeCurrentAndRestartCanResetVersions) {
  data(11, 10);
  ASSERT_TRUE(wait([&] { return client->view()->ready(); }));
  EXPECT_FALSE(client->isCurrent());
  heartbeat(11, 10);
  ASSERT_TRUE(wait([&] { return client->isCurrent(); }));
  auto held = client->view();
  data(22, 1, false);
  heartbeat(22, 1);
  ASSERT_TRUE(wait([&] { return client->view()->version() == 1; }));
  EXPECT_FALSE(client->isCurrent());
  s.producer_id = 22;
  statics->publish(s);
  ASSERT_TRUE(wait([&] { return client->isCurrent(); }));
  EXPECT_EQ(held->version(), 10u);
  EXPECT_EQ(client->view()->version(), 1u);
  heartbeat(22, 2);
  ASSERT_TRUE(
      wait([&] { return client->view()->status().derived_version == 2; }));
  EXPECT_FALSE(client->isCurrent());
  heartbeat(22, 1);
  ASSERT_TRUE(wait([&] { return client->isCurrent(); }));
}
TEST_F(MapClientTest,
       RetainedViewIsImmutableAndStaticStorageIsSharedAcrossUpdates) {
  data(1, 1);
  heartbeat(1, 1);
  ASSERT_TRUE(wait([&] { return client->isCurrent(); }));
  auto held = client->view();
  auto buffer = held->derivedMaps()->costmap.data.data();
  auto static_buffer = held->staticMap()->occupancy.data.data();
  std::atomic<bool> stop{false}, intact{true};
  std::thread reader([&] {
    while (!stop.load()) {
      if (held->version() != 1 || held->cost(10, 10) != 1 ||
          held->derivedMaps()->costmap.data.data() != buffer)
        intact = false;
    }
  });
  m.costmap[210] = 5;
  data(1, 2, false);
  heartbeat(1, 2);
  const bool updated = wait(
      [&] { return client->isCurrent() && client->view()->version() == 2; });
  stop = true;
  reader.join();
  ASSERT_TRUE(updated);
  EXPECT_TRUE(intact);
  EXPECT_EQ(held->cost(10, 10), 1);
  EXPECT_EQ(client->view()->cost(10, 10), 5);
  EXPECT_EQ(client->view()->staticMap()->occupancy.data.data(), static_buffer);
  EXPECT_NE(client->view()->derivedMaps()->costmap.data.data(), buffer);
  EXPECT_FALSE(client->view()->depthAt({1, 1}).has_value());
}
TEST_F(MapClientTest, MalformedLayerStopsAdmissionAndValidReplacementRecovers) {
  data(1, 1);
  heartbeat(1, 1);
  ASSERT_TRUE(wait([&] { return client->isCurrent(); }));
  auto held = client->view();
  const float original_cost = m.costmap[0];
  m.costmap[0] = NAN;
  maps->publish(m);
  ASSERT_TRUE(wait([&] { return !client->isCurrent(); }));
  EXPECT_TRUE(held->ready());
  m.costmap[0] = original_cost;
  maps->publish(m);
  heartbeat(1, 1);
  ASSERT_TRUE(wait([&] { return client->view()->ready(); }));
  ASSERT_TRUE(client->isCurrent())
      << "versions=" << client->view()->version() << "/"
      << client->view()->status().derived_version
      << " producer=" << client->view()->status().producer_id;
  s.occupancy.pop_back();
  statics->publish(s);
  ASSERT_TRUE(wait([&] { return !client->isCurrent(); }));
  s.occupancy.push_back(0);
  statics->publish(s);
  ASSERT_TRUE(wait([&] { return client->isCurrent(); }));
}
TEST_F(MapClientTest, LatestDynamicRetractionDoesNotReuseFusedOldObstacle) {
  m.fused_obstacles[210] = 1;
  m.distance_field[210] = 0;
  m.costmap[210] = INFINITY;
  m.collision_mask[210] = 1;
  map_server::msg::ObstacleCircle circle;
  circle.center.x = 1.05;
  circle.center.y = 1.05;
  circle.radius_m = .1;
  d.circles = {circle};
  data(1, 1);
  heartbeat(1, 1);
  ASSERT_TRUE(wait([&] { return client->isCurrent(); }));
  auto held = client->view();
  EXPECT_TRUE(held->latestOccupied(10, 10));
  d.circles.clear();
  d.version = 2;
  dynamics->publish(d);
  h.header.stamp = producer->now();
  h.producer_id = 1;
  h.derived_version = 1;
  h.dynamic_version = 2;
  statuses->publish(h);
  ASSERT_TRUE(wait([&] {
    return client->isCurrent() &&
           client->view()->dynamicObstacles()->circles.empty();
  }));
  EXPECT_TRUE(client->view()->occupied(10, 10));
  EXPECT_FALSE(client->view()->latestOccupied(10, 10));
  EXPECT_TRUE(held->latestOccupied(10, 10));
}
TEST_F(MapClientTest,
       HeartbeatExpiresInEitherClockAndWorkerErrorStopsAdmission) {
  data(1, 1);
  heartbeat(1, 1);
  ASSERT_TRUE(wait([&] { return client->isCurrent(); }));
  const auto held = client->view();
  const auto &status = held->status();
  map_server::MapPolicy policy;
  policy.require_cloud = false;
  EXPECT_FALSE(
      held->current(status.stamp_s + 1.1, status.received_steady_s, policy));
  EXPECT_FALSE(
      held->current(status.stamp_s, status.received_steady_s + 1.1, policy));
  EXPECT_FALSE(
      held->current(status.stamp_s - .2, status.received_steady_s, policy));
  h.health.map_error = true;
  heartbeat(1, 1);
  ASSERT_TRUE(wait([&] { return !client->isCurrent(); }));
  h.health.map_error = false;
  heartbeat(1, 1);
  ASSERT_TRUE(wait([&] { return client->isCurrent(); }));
}

TEST_F(MapClientTest, SourceProgressAdvancesWithoutRepublishingGeometry) {
  data(1, 1);
  h.input_version = 10;
  heartbeat(1, 1);
  ASSERT_TRUE(wait([&] { return client->isCurrent(); }));
  const auto held = client->view();
  h.input_version = 11;
  heartbeat(1, 1);
  ASSERT_TRUE(
      wait([&] { return client->view()->status().input_version == 11; }));
  const auto next = client->view();
  EXPECT_TRUE(client->isCurrent(*next));
  EXPECT_EQ(held->status().input_version, 10u);
  EXPECT_EQ(held->staticMap(), next->staticMap());
  EXPECT_EQ(held->derivedMaps(), next->derivedMaps());
  EXPECT_EQ(held->dynamicObstacles(), next->dynamicObstacles());
  EXPECT_EQ(held->dynamicStampNs(), next->dynamicStampNs());
}

} // namespace
