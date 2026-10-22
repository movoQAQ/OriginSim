#include <gtest/gtest.h>
#include <thread>
#include "map_server/costmap_client.hpp"
#include "map_server/costmap_generator.hpp"
namespace {
TEST(MapClient,RestartedProducerCanRestartVersionAndMismatchedHeartbeatCannotEnableMotion) {
  if(!rclcpp::ok())rclcpp::init(0,nullptr);
  auto publisher=std::make_shared<rclcpp::Node>("map_client_test_producer");
  auto client=std::make_shared<map_server::CostmapClient>("map_client_test",rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("require_cloud",false)}));
  rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(client);executor.add_node(publisher);
  auto qos=rclcpp::QoS(1).reliable().transient_local();
  auto maps=publisher->create_publisher<map_server::msg::DerivedMap>("/map_server/derived",qos);
  auto dynamic=publisher->create_publisher<map_server::msg::DynamicObstacles>("/map_server/dynamic",qos);
  map_server::DerivedMaps state;state.geometry={20,20,0.1,0,0};
  state.fused_obstacles={state.geometry,std::vector<uint8_t>(400,0)};
  state.distance_field={state.geometry,std::vector<float>(400,INFINITY)};
  map_server::generateCosts(state,{});
  map_server::msg::DerivedMap m;m.header.frame_id="map";m.info.width=20;m.info.height=20;m.info.resolution=.1;m.info.origin.orientation.w=1;
  m.fused_obstacles=state.fused_obstacles.data;m.distance_field=state.distance_field.data;m.costmap=state.costmap.data;
  m.collision_mask=state.collision_mask.data;m.region_id.assign(400,0);m.hard_radius_m=.48;
  map_server::msg::DynamicObstacles d;d.header.frame_id="map";
  const auto wait=[&](const auto & predicate) {
    for(int i=0;i<200;++i){executor.spin_some();if(predicate())return true;std::this_thread::sleep_for(std::chrono::milliseconds(5));}return false;
  };
  ASSERT_TRUE(wait([&]{return maps->get_subscription_count()>0&&dynamic->get_subscription_count()>0;}));
  const auto publish=[&](uint64_t producer,uint64_t version,bool send_map) {
    d.header.stamp=publisher->now();d.producer_id=producer;dynamic->publish(d);
    if(send_map){m.header.stamp=publisher->now();m.generated_at_s=publisher->now().seconds();m.producer_id=producer;m.version=version;maps->publish(m);}
  };
  publish(11,10,true);
  ASSERT_TRUE(wait([&]{return client->isCurrent();}));
  EXPECT_EQ(client->getDerivedMaps()->version,10u);
  publish(22,1,false);
  ASSERT_TRUE(wait([&]{return !client->isCurrent();}));
  publish(22,1,true);
  ASSERT_TRUE(wait([&]{return client->isCurrent()&&client->getDerivedMaps()->version==1;}));
  publish(22,0,true);
  ASSERT_TRUE(wait([&]{return !client->isCurrent();}));
  publish(22,1,true);
  ASSERT_TRUE(wait([&]{return client->isCurrent();}));
  executor.remove_node(client);executor.remove_node(publisher);client.reset();publisher.reset();rclcpp::shutdown();
}
}
