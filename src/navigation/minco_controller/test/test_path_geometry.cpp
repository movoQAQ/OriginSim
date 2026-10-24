#include <gtest/gtest.h>
#include "minco_controller/path_trajectory.hpp"
using namespace minco_controller;
TEST(PathGeometry,ProjectionDoesNotJumpToNearbyLaterBranch){
 PathTrajectory path;std::vector<ReferencePoint> points(4);
 points[0].pos={0,0};points[1].pos={2,0};points[2].pos={2,.1};points[3].pos={0,.1};
 path.setPath(points,1.2,1,1,false);State state;state.y=.1;
 EXPECT_LE(path.nearestTime(state),1.0);
}
TEST(PathGeometry,SharpBendSlowsWithoutDependingOnBodyYaw){
 PathTrajectory path;std::vector<ReferencePoint> points(3);
 points[0].pos={0,0};points[1].pos={1,0};points[2].pos={1,1};path.setPath(points,1.2,1,1,false);
 bool sampled=false;
 for(double t=0;t<path.duration();t+=.001){auto p=path.sample(t);if((p.pos-Eigen::Vector2d(1,0)).norm()<.01){sampled=true;EXPECT_LT(p.vel.norm(),.6);}}
 EXPECT_TRUE(sampled);
}
#include "minco_controller/mpc_solver.hpp"
TEST(PathGeometry,CrossTrackPenaltyFollowsVelocityWhenBodyYawIsFixed){
 MPCConfig config;config.q_along=1;config.q_cross=40;config.use_acc_constraints=false;
 MpcSolver first(config),rotated(config);State a,b;a.y=.2;b.x=-.2;
 std::vector<ReferencePoint> x(config.horizon),y(config.horizon);
 for(int i=0;i<config.horizon;++i){x[i].pos={.03*(i+1),0};x[i].vel={.5,0};y[i].pos={0,.03*(i+1)};y[i].vel={0,.5};}
 Control u,v;ASSERT_TRUE(first.solve(a,x,u));ASSERT_TRUE(rotated.solve(b,y,v));
 EXPECT_NEAR(v.vx,-u.vy,1e-5);EXPECT_NEAR(v.vy,u.vx,1e-5);
}

#include <deque>
namespace {
// Fixed-yaw swerve plant: same shortest-heading, steering slew and rolling
// projection rules as hxy's Gazebo SwerveDrive. This is a deterministic
// actuator regression, not a substitute for the Gazebo contact model.
void followWithSwerve(const std::vector<ReferencePoint> & points) {
 MPCConfig cfg;cfg.dt=.02;cfg.control_period=.02;cfg.horizon=25;
 cfg.vx_min=cfg.vy_min=-1.7;cfg.vx_max=cfg.vy_max=1.7;
 PathTrajectory path;path.setPath(points,1.2,1,1,false);
 MpcSolver solver(cfg);State state;
 state.x=points.front().pos.x();state.y=points.front().pos.y();
 double progress=0,max_cross=0,heading=0,steering_speed=0,rolling_speed=0;
 std::deque<Eigen::Vector2d> delay(3,Eigen::Vector2d::Zero());
 bool reached=false;
 for(int k=0;k<1500;++k){
  State predicted=state;predicted.x+=state.vx*.05;predicted.y+=state.vy*.05;
  progress=path.nearestTime(state,progress);
  std::vector<ReferencePoint> refs;
  for(int j=0;j<cfg.horizon;++j)refs.push_back(path.sample(progress+.05+j*cfg.dt));
  Control command;ASSERT_TRUE(solver.solve(predicted,refs,command));
  if(std::hypot(command.vx,command.vy)<.02){command.vx=command.vy=0;}
  delay.push_back({command.vx,command.vy});const auto requested=delay.front();delay.pop_front();
  const double desired_heading=requested.norm()>.005?std::atan2(requested.y(),requested.x()):heading;
  double error=std::remainder(desired_heading-heading,2*std::acos(-1.0));
  double target_speed=requested.norm();
  if(std::abs(error)>std::acos(-1.0)/2){error-=std::copysign(std::acos(-1.0),error);target_speed=-target_speed;}
  const double steering_target=std::clamp(6*error,-3.,3.);
  steering_speed+=std::clamp(steering_target-steering_speed,-10*cfg.dt,10*cfg.dt);
  heading+=steering_speed*cfg.dt;
  rolling_speed+=std::clamp(target_speed*std::cos(error)-rolling_speed,-3.032*cfg.dt,3.032*cfg.dt);
  state.vx=rolling_speed*std::cos(heading);state.vy=rolling_speed*std::sin(heading);
  state.x+=state.vx*cfg.dt;state.y+=state.vy*cfg.dt;
  const Eigen::Vector2d position(state.x,state.y);
  double distance=INFINITY;
  for(size_t i=1;i<points.size();++i){
   const Eigen::Vector2d delta=points[i].pos-points[i-1].pos;
   const double fraction=std::clamp((position-points[i-1].pos).dot(delta)/delta.squaredNorm(),0.,1.);
   distance=std::min(distance,(position-points[i-1].pos-fraction*delta).norm());
  }
  max_cross=std::max(max_cross,distance);
  if((position-points.back().pos).norm()<.05&&std::hypot(state.vx,state.vy)<.1){reached=true;break;}
 }
 EXPECT_TRUE(reached);
 EXPECT_LT(max_cross,.04);
 ::testing::Test::RecordProperty("maximum_cross_track_m",max_cross);
}
}
TEST(PathGeometry,AcuteCornerFollowsWithSwerveSteeringLimits){
 std::vector<ReferencePoint> points(3);
 points[0].pos={0,0};points[1].pos={2,0};points[2].pos={.6,1.4};
 followWithSwerve(points);
}
TEST(PathGeometry,TightSmoothBendFollowsWithSwerveSteeringLimits){
 std::vector<ReferencePoint> points(1);points[0].pos={0,0};
 ReferencePoint point;point.pos={2,0};points.push_back(point);
 const double angle=5*std::acos(-1.0)/6;
 for(int i=1;i<=60;++i){const double a=angle*i/60;point.pos={2+.12*std::sin(a),.12*(1-std::cos(a))};points.push_back(point);}
 point.pos+=2*Eigen::Vector2d(std::cos(angle),std::sin(angle));points.push_back(point);
 followWithSwerve(points);
}
TEST(PathGeometry,GridSpacedAcuteCornerDoesNotStallAtCommandDeadzone){
 std::vector<ReferencePoint> points;
 ReferencePoint point;
 for(int i=0;i<=40;++i){point.pos={.05*i,0};points.push_back(point);}
 for(int i=1;i<=40;++i){point.pos={2-.035*i,.035*i};points.push_back(point);}
 followWithSwerve(points);
}

TEST(PathGeometry,SpatialPreviewEscapesTinyGridEntryCornerWithoutChangingPath) {
 PathTrajectory path; std::vector<ReferencePoint> points(3);
 points[0].pos={.04,0};points[1].pos={.05,-.01};points[2].pos={.05,-4};
 path.setPath(points,.5,.5,1,false);
 const auto ahead=path.positionAhead(0,.8);
 EXPECT_NEAR(ahead.x(),.05,1e-9); EXPECT_LT(ahead.y(),-.75);
 EXPECT_NEAR(path.positionAhead(0,0).x(),.04,1e-9);
 EXPECT_NEAR(path.positionAhead(path.duration(),1).y(),-4,1e-9);
 EXPECT_THROW(path.positionAhead(0,-1),std::invalid_argument);
}
