#include <gtest/gtest.h>
#include <random>
#include "../plugins/swerve_drive/PassiveAngularDamping.hh"

TEST(PassiveAngularDamping,AlwaysDissipatesEnergyInAnyBodyAttitude) {
  std::mt19937 rng(2026);
  std::uniform_real_distribution<double> angle(-3.14,3.14), speed(-20,20), c(0,2000);
  for (int i=0;i<1000;++i) {
    const gz::math::Quaterniond q(angle(rng),angle(rng),angle(rng));
    const gz::math::Vector3d omega(speed(rng),speed(rng),speed(rng));
    const gz::math::Vector3d coefficient(c(rng),c(rng),c(rng));
    const auto torque=originsim::PassiveAngularDamping(q,omega,coefficient);
    EXPECT_LE(torque.Dot(omega),1e-9);
  }
}
TEST(PassiveAngularDamping,IsotropicDragDoesNotDependOnAttitude) {
  const gz::math::Vector3d omega(1,-2,3), c(5,5,5);
  for (double pitch : {0.,1.,1.57,2.,3.14}) {
    const auto torque=originsim::PassiveAngularDamping(
        gz::math::Quaterniond(.5,pitch,-.3),omega,c);
    EXPECT_LT((torque+5*omega).Length(),1e-10);
  }
}
TEST(PassiveAngularDamping,DisabledDragAndStationaryBodyProduceNoTorque) {
  const gz::math::Quaterniond q(.2,.3,.4);
  EXPECT_DOUBLE_EQ(originsim::PassiveAngularDamping(q,{1,2,3},{0,0,0}).Length(),0);
  EXPECT_DOUBLE_EQ(originsim::PassiveAngularDamping(q,{0,0,0},{1500,1500,2}).Length(),0);
}
