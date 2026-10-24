#include <gtest/gtest.h>
#include "ground_heading.hpp"

TEST(GroundHeading, PitchSingularityKeepsWheelHeading)
{
  for (double pitch : {0.0, 1.4, 1.5707963267948966, 1.7, 3.0}) {
    tf2::Quaternion q; q.setRPY(0.0, pitch, 0.4);
    EXPECT_NEAR(fake_baselink::groundHeading(q, 0), 0.4, 1e-12);
  }
  tf2::Quaternion q; q.setRPY(1.5707963267948966, 0, 0);
  EXPECT_EQ(fake_baselink::groundHeading(q, 0.7), 0.7);
}

TEST(GroundHeading, AlignsLongAxisBeforeTranslating)
{
  const double pi = std::acos(-1.0);
  auto c = fake_baselink::convertCommand(0, -1, 0, 0, 1.5, 0.8, true);
  EXPECT_EQ(c.x, 0); EXPECT_EQ(c.y, 0); EXPECT_NEAR(std::abs(c.yaw_rate), 0.8, 1e-12);
  c = fake_baselink::convertCommand(0, -1, 0, pi/2, 1.5, 0.8, true);
  EXPECT_NEAR(c.x, -1, 1e-12); EXPECT_NEAR(c.y, 0, 1e-12);
  EXPECT_NEAR(c.yaw_rate, 0, 1e-12);
  c = fake_baselink::convertCommand(-1, 0, 0, 0, 1.5, 0.8, true);
  EXPECT_EQ(c.x, -1); EXPECT_NEAR(c.yaw_rate, 0, 1e-12);
}

TEST(GroundHeading, StopAndExplicitRotationRemainAuthoritative)
{
  auto c = fake_baselink::convertCommand(0, 0, 0, 1, 1.5, 0.8, true);
  EXPECT_EQ(c.x, 0); EXPECT_EQ(c.y, 0); EXPECT_EQ(c.yaw_rate, 0);
  c = fake_baselink::convertCommand(0.2, 0, 0.3, 1, 1.5, 0.8, true);
  EXPECT_NEAR(std::hypot(c.x, c.y), 0.2, 1e-12); EXPECT_EQ(c.yaw_rate, 0.3);
}
