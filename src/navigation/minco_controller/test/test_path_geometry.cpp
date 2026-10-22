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
