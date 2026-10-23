#include "nav_executor/recovery.hpp"
#include "nav_executor/task_manager.hpp"
#include <gtest/gtest.h>
using namespace nav_executor;
map_server::MapView map(bool obstacle = false) {
  auto s = std::make_shared<map_server::StaticMap>();
  s->geometry.width = 100;
  s->geometry.height = 100;
  s->geometry.resolution = 0.05;
  s->occupancy.geometry = s->geometry;
  s->occupancy.data.resize(10000, 0);
  if (obstacle)
    s->occupancy.data[50 * 100 + 44] = 1;
  auto d = std::make_shared<map_server::DerivedMaps>();
  d->geometry = s->geometry;
  d->fused_obstacles = s->occupancy;
  return {s, d};
}
bool step(Recovery &r, double t, bool hazard = true, bool demand = false,
          double x = 2.5, double speed = 0, bool stalled = false) {
  return r.update(t, x, 2.5, -1, 0, speed, hazard, demand, stalled, map(), 0.3);
}
TEST(Recovery, HazardZeroCommandWaitsThreeSeconds) {
  Recovery r;
  for (int i = 0; i < 30; i++)
    EXPECT_FALSE(step(r, i * 0.1));
  EXPECT_TRUE(step(r, 3.0));
  EXPECT_LT(r.vx(), 0);
  EXPECT_DOUBLE_EQ(r.vy(), 0);
}
TEST(Recovery, IdleAndMovingDoNotReverse) {
  Recovery idle, moving;
  for (int i = 0; i < 50; i++) {
    EXPECT_FALSE(step(idle, i * 0.1, false));
    EXPECT_FALSE(step(moving, i * 0.1, true, true, 2.5, 0.2));
  }
}
TEST(Recovery, StalledCommandAndFreshThreeSecondEvidence) {
  Recovery r;
  for (int i = 0; i < 30; i++)
    EXPECT_FALSE(step(r, i * 0.1, false, true));
  EXPECT_TRUE(step(r, 3.0, false, true));
  Recovery forced;
  EXPECT_TRUE(step(forced, 0, false, false, 2.5, 0, true));
}
TEST(Recovery, ProgressResetsStationaryWindow) {
  Recovery r;
  for (int i = 0; i < 60; i++)
    EXPECT_FALSE(step(r, i * 0.1, true, false, 2.5 + 0.06 * (i / 20)));
}
TEST(Recovery, DistanceCompletesAndCooldownPreventsImmediateRestart) {
  Recovery r;
  ASSERT_TRUE(step(r, 0, true, false, 2.5, 0, true));
  EXPECT_FALSE(step(r, 0.1, true, false, 2.19));
  EXPECT_EQ(r.status(), "COMPLETE");
  EXPECT_FALSE(step(r, 0.2, true, false, 2.19, 0, true));
  EXPECT_DOUBLE_EQ(r.vx(), 0);
}
TEST(Recovery, TimeoutStopsAndAttemptsAreBounded) {
  RecoveryConfig c;
  c.max_attempts = 1;
  Recovery r(c);
  ASSERT_TRUE(step(r, 0, true, false, 2.5, 0, true));
  for (int i = 1; i < 30; i++)
    ASSERT_TRUE(step(r, i * 0.1));
  EXPECT_FALSE(step(r, 3.0));
  for (int i = 31; i < 120; i++)
    EXPECT_FALSE(step(r, i * 0.1));
  EXPECT_EQ(r.status(), "ATTEMPTS_EXHAUSTED");
  r.reset();
  EXPECT_TRUE(step(r, 12, true, false, 2.5, 0, true));
}
TEST(Recovery, GapAndInvalidObservationStop) {
  Recovery r;
  ASSERT_TRUE(step(r, 0, true, false, 2.5, 0, true));
  EXPECT_FALSE(step(r, 1));
  EXPECT_EQ(r.status(), "INPUT_INVALID");
  EXPECT_FALSE(step(r, 1.1, true, false, NAN));
}
TEST(Recovery, SweptDiscRejectsRearObstacleAndBoundary) {
  EXPECT_TRUE(Recovery::corridorClear(map(), 2.5, 2.5, 2.2, 2.5, 0.31));
  EXPECT_FALSE(Recovery::corridorClear(map(true), 2.5, 2.5, 2.2, 2.5, 0.31));
  EXPECT_FALSE(Recovery::corridorClear(map(), 0.4, 2.5, 0.1, 2.5, 0.31));
}
TEST(Recovery, NewObstacleStopsActiveReverse) {
  Recovery r;
  ASSERT_TRUE(step(r, 0, true, false, 2.5, 0, true));
  EXPECT_FALSE(
      r.update(0.1, 2.5, 2.5, -1, 0, 0, true, false, false, map(true), 0.3));
  EXPECT_EQ(r.status(), "REVERSE_BLOCKED");
  EXPECT_DOUBLE_EQ(r.vx(), 0);
}
TEST(Recovery, UnsafeStartAndDisabledNeverCommandMotion) {
  Recovery r;
  EXPECT_FALSE(
      r.update(0, 2.5, 2.5, -1, 0, 0, true, false, true, map(true), 0.3));
  EXPECT_EQ(r.status(), "REVERSE_BLOCKED");
  RecoveryConfig c;
  c.enabled = false;
  Recovery disabled(c);
  EXPECT_FALSE(step(disabled, 0, true, true, 2.5, 0, true));
}
TEST(Recovery, DirectionRemainsFixedAndDeviationStops) {
  Recovery r;
  ASSERT_TRUE(step(r, 0, true, false, 2.5, 0, true));
  EXPECT_TRUE(
      r.update(0.1, 2.49, 2.5, 1, 0, 0, true, false, false, map(), 0.3));
  EXPECT_LT(r.vx(), 0);
  EXPECT_FALSE(
      r.update(0.2, 2.49, 2.7, 1, 0, 0, true, false, false, map(), 0.3));
}
TEST(Recovery, RejectsInvalidConfig) {
  RecoveryConfig c;
  c.timeout = 0;
  EXPECT_THROW(Recovery r(c), std::invalid_argument);
}

TEST(Recovery, RecoveryReplanRevokesOldWorkerAndPreservesGoal) {
  TaskManager tasks;
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 4;
  goal.pose.orientation.w = 1;
  ASSERT_TRUE(tasks.setGoal(goal));
  const auto t = TaskManager::Clock::now();
  auto old = tasks.nextRequest(true, t);
  ASSERT_TRUE(old);
  ASSERT_TRUE(
      tasks.replan()); // recovery entry; scheduler is suspended until exit
  PlanningResult stale;
  stale.identity = old->identity;
  EXPECT_FALSE(tasks.accept(stale, t));
  ASSERT_TRUE(tasks.replan()); // recovery exit: new start pose and generation
  auto fresh = tasks.nextRequest(true, t);
  ASSERT_TRUE(fresh);
  EXPECT_EQ(fresh->goal, goal);
  EXPECT_NE(fresh->identity, old->identity);
  tasks.cancel();
  EXPECT_FALSE(tasks.nextRequest(true, t));
  EXPECT_FALSE(tasks.isPlanConfirmed(fresh->identity));
}
