#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>
#include "map_server/ros_adapters.hpp"
namespace {
using namespace map_server;
void transform(tf2_ros::Buffer & tf, const std::string & child, int time, double x) {
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id="map";t.child_frame_id=child;t.header.stamp.sec=time;
  t.transform.rotation.w=1;t.transform.translation.x=x;
  ASSERT_TRUE(tf.setTransform(t,"contract_test"));
}
sensor_msgs::msg::PointCloud2 cloud() {
  DynamicCloud points;points.push_back({1,2,3});sensor_msgs::msg::PointCloud2 m;
  pcl::toROSMsg(points,m);m.header.frame_id="sensor";m.header.stamp.sec=10;return m;
}
TEST(CloudAdapter,MeasurementTimeTransformAndFixedMapPcdContract) {
  auto clock=std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);tf2_ros::Buffer tf(clock);
  for(int stamp:{10,20}) {transform(tf,"sensor",stamp,stamp==10?5:50);transform(tf,"base",stamp,0);transform(tf,"odom",stamp,0);}
  auto result=adaptCloud(cloud(),tf,"base",100,10);
  ASSERT_EQ(result.cloud->size(),1u);
  EXPECT_FLOAT_EQ(result.cloud->points[0].x,6);
  EXPECT_EQ(result.frame_id,"map");EXPECT_EQ(result.observation_stamp_s,10);
  EXPECT_EQ(result.received_stamp_s,100);
}
TEST(CloudAdapter,MissingMeasurementTfCannotUseLatest) {
  auto clock=std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);tf2_ros::Buffer tf(clock);
  transform(tf,"sensor",20,50);transform(tf,"base",20,0);transform(tf,"odom",20,0);
  EXPECT_THROW(adaptCloud(cloud(),tf,"base",100,10),tf2::TransformException);
}
TEST(CloudAdapter,RejectMalformedLayoutAndPointBudget) {
  auto clock=std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);tf2_ros::Buffer tf(clock);
  auto m=cloud();m.data.pop_back();
  EXPECT_THROW(adaptCloud(m,tf,"base",100,10),std::invalid_argument);
  m=cloud();EXPECT_THROW(adaptCloud(m,tf,"base",100,0),std::invalid_argument);
}
TEST(RadarAdapter,SeparateSourceAndReceptionClock) {
  msg::RadarTargets m;m.header.frame_id="map";m.header.stamp.sec=10;m.observation_valid=true;m.stable_ids=true;
  msg::RadarTarget t;t.id=2;t.position.x=19;t.observed_at.sec=9;t.physical_radius_m=.3;m.targets.push_back(t);
  auto result=adaptRadar(m,RadarSource::ENEMY,100);
  EXPECT_EQ(result.source,RadarSource::ENEMY);EXPECT_EQ(result.targets.front().observed_at_s,9);
  EXPECT_EQ(result.targets.front().received_at_s,100);EXPECT_EQ(result.observation_stamp_s,10);
}
TEST(RadarAdapter,RejectUnstampedOrWrongFrame) {
  msg::RadarTargets m;m.header.frame_id="odom";
  EXPECT_THROW(adaptRadar(m,RadarSource::ALLY,100),std::invalid_argument);
  m.header.frame_id="map";
  EXPECT_THROW(adaptRadar(m,RadarSource::ALLY,100),std::invalid_argument);
}
}
