#include <gtest/gtest.h>
#include "corrected_velocity.hpp"

TEST(CorrectedVelocity,StationaryMatchesRemoveBiasedFilterVelocity) {
  CorrectedVelocity estimator; Eigen::Vector3d result;
  for (int i=0;i<30;++i) {
    const Eigen::Vector3d noise(0.002*std::sin(i),0.002*std::cos(i),0);
    result=estimator.update(i*.1,Eigen::Vector3d(2,3,.2)+noise,Eigen::Vector3d(.12,-.08,0));
  }
  EXPECT_LT(result.head<2>().norm(),.02);
}
TEST(CorrectedVelocity,TracksTranslationAndSettlesAfterStop) {
  CorrectedVelocity estimator; Eigen::Vector3d result;
  for (int i=0;i<=20;++i)result=estimator.update(i*.1,Eigen::Vector3d(.03*i,0,0),Eigen::Vector3d::Zero());
  EXPECT_NEAR(result.x(),.3,1e-10);
  for (int i=21;i<=30;++i)result=estimator.update(i*.1,Eigen::Vector3d(.6,0,0),Eigen::Vector3d(.1,0,0));
  EXPECT_LT(result.norm(),1e-10);
}
TEST(CorrectedVelocity,DropoutAndPositionJumpResetHistory) {
  CorrectedVelocity estimator; const Eigen::Vector3d fallback(.1,.2,0);
  for (int i=0;i<10;++i)estimator.update(i*.1,Eigen::Vector3d(.03*i,0,0),fallback);
  EXPECT_TRUE(estimator.update(3,Eigen::Vector3d(.9,0,0),fallback).isApprox(fallback));
  for (int i=1;i<10;++i)estimator.update(3+i*.1,Eigen::Vector3d(.9+.03*i,0,0),fallback);
  EXPECT_TRUE(estimator.update(4,Eigen::Vector3d(20,0,0),fallback).isApprox(fallback));
}
