#include <gtest/gtest.h>

#include "swerve_vehicle_control/temporal_validity.hpp"

using swerve_vehicle_control::CommandTimeGate;
using swerve_vehicle_control::OdomSampleGate;
using swerve_vehicle_control::source_stamp_ns;

TEST(CommandTimeGate, RejectsMalformedSourceStamp)
{
  EXPECT_EQ(source_stamp_ns(-1, 0), 0);
  EXPECT_EQ(source_stamp_ns(1, 1'000'000'000U), 0);
  EXPECT_EQ(source_stamp_ns(1, 25U), 1'000'000'025LL);
}

TEST(CommandTimeGate, RejectsWrongFrameAndUnusableSourceTime)
{
  CommandTimeGate gate;
  constexpr int64_t now = 1'000'000'000;
  constexpr int64_t timeout = 500'000'000;
  EXPECT_TRUE(gate.observe_update(now, now, timeout));
  EXPECT_FALSE(gate.accept("odom", now, now, timeout));
  EXPECT_FALSE(gate.accept("", now, now, timeout));
  EXPECT_FALSE(gate.accept("base_footprint", 0, now, timeout));
  EXPECT_FALSE(gate.accept("base_footprint", now + 1, now, timeout));
  EXPECT_FALSE(gate.accept("base_footprint", now - timeout - 1, now, timeout));
  EXPECT_TRUE(gate.accept("base_footprint", now - 1, now, timeout));
  EXPECT_TRUE(gate.usable(now - 1, now, now, timeout));
  EXPECT_FALSE(gate.usable(now - 1, now, now + timeout + 1, timeout));
}

TEST(CommandTimeGate, SourceExpiryCannotBeHiddenByARecentReceive)
{
  CommandTimeGate gate;
  constexpr int64_t timeout = 500'000'000;
  EXPECT_TRUE(gate.accept("base_footprint", 1'000'000'000, 1'200'000'000, timeout));
  EXPECT_TRUE(gate.usable(1'000'000'000, 1'200'000'000,
                          1'500'000'000, timeout));
  EXPECT_FALSE(gate.usable(1'000'000'000, 1'200'000'000,
                           1'510'000'000, timeout));
}

TEST(CommandTimeGate, RejectsReplayAndOutOfOrderEvenWhenReceiveTimeIsFresh)
{
  CommandTimeGate gate;
  constexpr int64_t timeout = 500'000'000;
  EXPECT_TRUE(gate.accept("base_footprint", 1'000'000'000, 1'010'000'000, timeout));
  EXPECT_FALSE(gate.accept("base_footprint", 1'000'000'000, 1'020'000'000, timeout));
  EXPECT_FALSE(gate.accept("base_footprint", 999'000'000, 1'020'000'000, timeout));
  EXPECT_TRUE(gate.accept("base_footprint", 1'020'000'000, 1'030'000'000, timeout));
}

TEST(CommandTimeGate, PausedClockStopsTemporarilyWithoutRearmingAReplay)
{
  CommandTimeGate gate;
  constexpr int64_t timeout = 500'000'000;
  EXPECT_TRUE(gate.observe_update(1'000'000'000, 1'000'000'000, timeout));
  EXPECT_TRUE(gate.accept("base_footprint", 1'000'000'000, 1'000'000'000, timeout));
  EXPECT_FALSE(gate.observe_update(1'000'000'000, 1'010'000'000, timeout));
  EXPECT_FALSE(gate.reset_latched());
  EXPECT_FALSE(gate.accept("base_footprint", 1'000'000'000, 1'000'000'000, timeout));
  EXPECT_TRUE(gate.observe_update(1'001'000'000, 1'020'000'000, timeout));
  EXPECT_TRUE(gate.accept("base_footprint", 1'001'000'000, 1'001'000'000, timeout));
}

TEST(CommandTimeGate, PausedClockRequiresNewCommandBeforeMotionResumes)
{
  CommandTimeGate gate;
  constexpr int64_t timeout = 500'000'000;
  constexpr int64_t paused_at = 1'000'000'000;
  EXPECT_TRUE(gate.observe_update(paused_at, paused_at, timeout));
  EXPECT_TRUE(gate.accept("base_footprint", paused_at - 10'000'000,
                          paused_at, timeout));
  EXPECT_TRUE(gate.usable(paused_at - 10'000'000, paused_at,
                          paused_at, timeout));
  EXPECT_FALSE(gate.observe_update(paused_at, paused_at + 1'000'000, timeout));
  EXPECT_FALSE(gate.accept("base_footprint", paused_at, paused_at, timeout));
  EXPECT_TRUE(gate.observe_update(paused_at + 10'000'000, paused_at + 20'000'000, timeout));
  EXPECT_FALSE(gate.usable(paused_at - 10'000'000, paused_at,
                           paused_at + 10'000'000, timeout));
  EXPECT_TRUE(gate.accept("base_footprint", paused_at + 10'000'000,
                          paused_at + 10'000'000, timeout));
  EXPECT_TRUE(gate.usable(paused_at + 10'000'000,
                          paused_at + 10'000'000,
                          paused_at + 10'000'000, timeout));
}

TEST(CommandTimeGate, LongWallGapWithoutUpdatesStopsFirstResumeTick)
{
  CommandTimeGate gate;
  constexpr int64_t timeout = 500'000'000;
  EXPECT_TRUE(gate.observe_update(1'000'000'000, 1'000'000'000, timeout));
  EXPECT_TRUE(gate.accept("base_footprint", 1'000'000'000,
                          1'000'000'000, timeout));
  // Gazebo publishes no control updates for 800 ms of wall time while
  // simulation advances only 10 ms at the first tick after resume.
  EXPECT_FALSE(gate.observe_update(1'010'000'000, 1'800'000'000, timeout));
  EXPECT_FALSE(gate.usable(1'000'000'000, 1'000'000'000,
                           1'010'000'000, timeout));
  EXPECT_FALSE(gate.accept("base_footprint", 1'010'000'000,
                           1'010'000'000, timeout));
  EXPECT_TRUE(gate.observe_update(1'020'000'000, 1'810'000'000, timeout));
  EXPECT_TRUE(gate.accept("base_footprint", 1'020'000'000,
                          1'020'000'000, timeout));
  EXPECT_TRUE(gate.usable(1'020'000'000, 1'020'000'000,
                          1'020'000'000, timeout));
}

TEST(CommandTimeGate, BackwardClockLatchesStopUntilControllerReactivation)
{
  CommandTimeGate gate;
  constexpr int64_t timeout = 500'000'000;
  EXPECT_TRUE(gate.observe_update(10'000'000'000, 1'000'000'000, timeout));
  EXPECT_TRUE(gate.accept("base_footprint", 10'000'000'000, 10'000'000'000, timeout));
  EXPECT_FALSE(gate.observe_update(1'000'000'000, 1'010'000'000, timeout));
  EXPECT_TRUE(gate.reset_latched());
  EXPECT_FALSE(gate.accept("base_footprint", 1'000'000'000, 1'000'000'000, timeout));
  EXPECT_FALSE(gate.observe_update(10'000'000'000, 1'020'000'000, timeout));
  EXPECT_FALSE(gate.accept("base_footprint", 10'000'000'000, 10'000'000'000, timeout));
  gate.reset();
  EXPECT_TRUE(gate.observe_update(1'000'000'000, 1'030'000'000, timeout));
  EXPECT_TRUE(gate.accept("base_footprint", 1'000'000'000, 1'000'000'000, timeout));
}

TEST(OdomSampleGate, NeverPresentsAStalledUpdateAsFresh)
{
  OdomSampleGate gate;
  EXPECT_FALSE(gate.should_publish(0, 1'000'000'000));
  EXPECT_FALSE(gate.should_publish(2, 0));
  EXPECT_TRUE(gate.should_publish(2, 1'000'000'000));
  EXPECT_FALSE(gate.should_publish(2, 1'100'000'000));
  EXPECT_FALSE(gate.should_publish(4, 900'000'000));
  EXPECT_FALSE(gate.should_publish(4, 1'000'000'000));
  EXPECT_TRUE(gate.should_publish(4, 1'020'000'000));
  EXPECT_FALSE(gate.should_publish(4, 1'030'000'000));
}
