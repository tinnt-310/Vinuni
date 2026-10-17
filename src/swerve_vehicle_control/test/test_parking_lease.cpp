#include <gtest/gtest.h>
#include "swerve_vehicle_control/parking_lease.hpp"

using namespace swerve_vehicle_control;

TEST(ParkingLease, ExclusiveOwnerAndReplayProtection) {
  ParkingLease lease;
  EXPECT_TRUE(lease.apply({4, 1, 100, 100, false}, 100));
  EXPECT_TRUE(lease.inhibited());
  EXPECT_FALSE(lease.apply({5, 1, 110, 100, false}, 110));
  EXPECT_FALSE(lease.apply({4, 1, 150, 100, false}, 150));
  EXPECT_TRUE(lease.expired(200));
  EXPECT_TRUE(lease.inhibited());
  EXPECT_TRUE(lease.apply({4, 2, 201, 100, false}, 201));
  EXPECT_FALSE(lease.expired(250));
}

TEST(ParkingLease, ReleaseDoesNotReplayOldDriveCommand) {
  ParkingLease lease;
  ASSERT_TRUE(lease.apply({4, 1, 100, 100, false}, 100));
  EXPECT_FALSE(lease.commandAllowed(101));
  EXPECT_FALSE(lease.apply({5, 2, 200, 0, true}, 200));
  ASSERT_TRUE(lease.apply({4, 2, 201, 0, true}, 205));
  EXPECT_FALSE(lease.inhibited());
  EXPECT_FALSE(lease.commandAllowed(204));
  EXPECT_TRUE(lease.commandAllowed(206));
  EXPECT_FALSE(lease.apply({4, 1, 206, 100, false}, 206));
}

TEST(ParkingLease, InvalidDurationAndClockAreRejected) {
  ParkingLease lease;
  EXPECT_FALSE(lease.apply({0, 1, 100, 100, false}, 100));
  EXPECT_FALSE(lease.apply({4, 0, 100, 100, false}, 100));
  EXPECT_FALSE(lease.apply({4, 1, 100, 0, false}, 100));
  EXPECT_FALSE(lease.apply({4, 1, 100, 100, false}, 99));
  EXPECT_FALSE(lease.apply({4, 1, 100, 1000000001, false}, 100));
  EXPECT_FALSE(lease.inhibited());
}
