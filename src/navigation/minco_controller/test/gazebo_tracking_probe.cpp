// Manual integration probe: the real solver/reference drives Gazebo wheel physics.
#include "minco_controller/path_trajectory.hpp"
#include "minco_controller/mpc_solver.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <optional>
#include <thread>
using namespace minco_controller;
using namespace std::chrono_literals;
int main(int argc,char **argv) {
 if(argc<2)return 2;
 const std::string report_file=argv[1];
 const bool smooth=argc>2&&std::string(argv[2])=="smooth";
 rclcpp::init(argc,argv);
 auto node=std::make_shared<rclcpp::Node>("gazebo_mpc_tracking_probe");
 std::optional<nav_msgs::msg::Odometry> odom;
 auto sub=node->create_subscription<nav_msgs::msg::Odometry>("/odometry",rclcpp::SensorDataQoS().keep_last(1),
  [&](nav_msgs::msg::Odometry::ConstSharedPtr msg){odom=*msg;});
 auto pub=node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
 auto stamp=[&](){return odom?odom->header.stamp.sec+odom->header.stamp.nanosec*1e-9:0.;};
 const auto deadline=std::chrono::steady_clock::now()+60s;
 while(rclcpp::ok()&&std::chrono::steady_clock::now()<deadline&&stamp()<3){rclcpp::spin_some(node);std::this_thread::sleep_for(1ms);}
 if(!odom||stamp()<3){rclcpp::shutdown();return 3;}
 const Eigen::Vector2d origin(odom->pose.pose.position.x,odom->pose.pose.position.y);
 std::vector<ReferencePoint> points(1);points[0].pos=origin;
 ReferencePoint point;point.pos=origin+Eigen::Vector2d(2,0);points.push_back(point);
 if(smooth){
  const double angle=5*std::acos(-1.0)/6;
  for(int i=1;i<=60;++i){double a=angle*i/60;point.pos=origin+Eigen::Vector2d(2+.12*std::sin(a),.12*(1-std::cos(a)));points.push_back(point);}
  point.pos+=2*Eigen::Vector2d(std::cos(angle),std::sin(angle));points.push_back(point);
 }else{point.pos=origin+Eigen::Vector2d(.6,1.4);points.push_back(point);}
 MPCConfig cfg;if(argc>3)cfg.q_cross=std::stod(argv[3]);
 cfg.dt=.02;cfg.control_period=.02;cfg.horizon=25;
 cfg.vx_min=cfg.vy_min=-1.7;cfg.vx_max=cfg.vy_max=1.7;
 PathTrajectory path;path.setPath(points,1.2,1,1,false);MpcSolver solver(cfg);
 double progress=0,max_cross=0,last_stamp=-1,final_error=INFINITY,corner_miss=INFINITY;
 size_t samples=0;bool reached=false,solved=true;
 std::vector<Eigen::Vector2d> trace;
 const double began=stamp();
 while(rclcpp::ok()&&std::chrono::steady_clock::now()<deadline&&stamp()-began<30){
  rclcpp::spin_some(node);
  const double now=stamp();if(now-last_stamp<.019){std::this_thread::sleep_for(1ms);continue;}last_stamp=now;
  const auto &q=odom->pose.pose.orientation;
  State state;state.x=odom->pose.pose.position.x;state.y=odom->pose.pose.position.y;
  state.yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  const double c=std::cos(state.yaw),s=std::sin(state.yaw);
  const auto &v=odom->twist.twist;
  state.vx=c*v.linear.x-s*v.linear.y;state.vy=s*v.linear.x+c*v.linear.y;state.omega=v.angular.z;
  progress=path.nearestTime(state,progress);
  State predicted=state;predicted.x+=state.vx*.05;predicted.y+=state.vy*.05;predicted.yaw+=state.omega*.05;
  std::vector<ReferencePoint> refs;
  for(int k=0;k<cfg.horizon;++k){auto ref=path.sample(progress+.05+k*cfg.dt);ref.yaw=predicted.yaw;refs.push_back(ref);}
  Control command;if(!solver.solve(predicted,refs,command)){solved=false;break;}
  if(std::hypot(command.vx,command.vy)<.02){command.vx=command.vy=0;}
  geometry_msgs::msg::Twist msg;msg.linear.x=c*command.vx+s*command.vy;msg.linear.y=-s*command.vx+c*command.vy;pub->publish(msg);
  const Eigen::Vector2d p(state.x,state.y);double error=INFINITY;
  for(size_t i=1;i<points.size();++i){Eigen::Vector2d d=points[i].pos-points[i-1].pos;
   double f=std::clamp((p-points[i-1].pos).dot(d)/d.squaredNorm(),0.,1.);error=std::min(error,(p-points[i-1].pos-f*d).norm());}
  max_cross=std::max(max_cross,error);corner_miss=std::min(corner_miss,(p-points[1].pos).norm());
  final_error=(p-points.back().pos).norm();if(samples++%5==0)trace.push_back(p);
  if(final_error<.1&&std::hypot(state.vx,state.vy)<.1){reached=true;break;}
 }
 for(int i=0;i<10;++i){pub->publish(geometry_msgs::msg::Twist{});rclcpp::spin_some(node);std::this_thread::sleep_for(10ms);}
 const bool passed=reached&&solved&&max_cross<.1;
 std::ofstream out(report_file);out<<std::boolalpha<<"{\n\"source\":\"real Gazebo SwerveDrive contact physics on flat ground\",\n"
  <<"\"mode\":\""<<(smooth?"tight_150_degree_smooth_bend":"135_degree_polyline_turn")<<"\",\n"
  <<"\"q_cross\":"<<cfg.q_cross<<",\n\"travel_direction_rate_limit_rad_s\":1,\n"
  <<"\"passed\":"<<passed<<",\n\"reached\":"<<reached<<",\n\"solver_ok\":"<<solved
  <<",\n\"max_cross_track_m\":"<<max_cross<<",\n\"corner_miss_m\":"<<corner_miss
  <<",\n\"final_error_m\":"<<final_error<<",\n\"samples\":"<<samples<<",\n\"trace\":[";
 for(size_t i=0;i<trace.size();++i){if(i)out<<',';out<<'['<<trace[i].x()<<','<<trace[i].y()<<']';}out<<"]\n}\n";
 std::cout<<"Gazebo turn reached="<<reached<<" max_cross="<<max_cross<<" final="<<final_error<<std::endl;
 rclcpp::shutdown();return passed?0:1;
}
