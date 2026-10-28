// Copyright 2026, SANDO Lyrical / PX4 integration
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cmath>

#include "px4_sando_bridge/frame_transforms.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTolerance = 1e-9;

double quaternionDot(
  const px4_sando_bridge::Quaternion & a,
  const px4_sando_bridge::Quaternion & b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

}  // namespace

TEST(FrameTransforms, PositionNedToEnuAndBack)
{
  const px4_sando_bridge::Vector3 ned = {10.0, 5.0, -2.5};
  const px4_sando_bridge::Vector3 enu = px4_sando_bridge::nedToEnu(ned);

  EXPECT_NEAR(enu[0], 5.0, kTolerance);
  EXPECT_NEAR(enu[1], 10.0, kTolerance);
  EXPECT_NEAR(enu[2], 2.5, kTolerance);

  const px4_sando_bridge::Vector3 roundtrip = px4_sando_bridge::enuToNed(enu);
  EXPECT_NEAR(roundtrip[0], ned[0], kTolerance);
  EXPECT_NEAR(roundtrip[1], ned[1], kTolerance);
  EXPECT_NEAR(roundtrip[2], ned[2], kTolerance);
}

TEST(FrameTransforms, VelocityUsesSameBasisChange)
{
  const px4_sando_bridge::Vector3 ned_velocity = {1.5, 2.5, -0.8};
  const px4_sando_bridge::Vector3 enu_velocity =
    px4_sando_bridge::nedToEnu(ned_velocity);

  EXPECT_NEAR(enu_velocity[0], 2.5, kTolerance);
  EXPECT_NEAR(enu_velocity[1], 1.5, kTolerance);
  EXPECT_NEAR(enu_velocity[2], 0.8, kTolerance);
}

TEST(FrameTransforms, YawNedToEnu)
{
  EXPECT_NEAR(px4_sando_bridge::nedYawToEnu(0.0), kPi / 2.0, kTolerance);
  EXPECT_NEAR(px4_sando_bridge::nedYawToEnu(kPi / 2.0), 0.0, kTolerance);
  EXPECT_NEAR(px4_sando_bridge::enuYawToNed(0.0), kPi / 2.0, kTolerance);
  EXPECT_NEAR(px4_sando_bridge::enuYawToNed(kPi / 2.0), 0.0, kTolerance);

  const double original = 0.75;
  const double roundtrip =
    px4_sando_bridge::nedYawToEnu(px4_sando_bridge::enuYawToNed(original));
  EXPECT_NEAR(roundtrip, original, kTolerance);
}

TEST(FrameTransforms, IdentityNedQuaternionBecomesEastFacingEnu)
{
  const px4_sando_bridge::Quaternion q_ned = {1.0, 0.0, 0.0, 0.0};
  const px4_sando_bridge::Quaternion q_enu =
    px4_sando_bridge::nedToEnuQuaternion(q_ned);

  EXPECT_NEAR(
    px4_sando_bridge::yawFromQuaternion(q_enu), kPi / 2.0, kTolerance);

  const px4_sando_bridge::Quaternion expected =
    px4_sando_bridge::quaternionFromYaw(kPi / 2.0);
  EXPECT_NEAR(std::abs(quaternionDot(q_enu, expected)), 1.0, kTolerance);
}

TEST(FrameTransforms, YawOnlyQuaternionMatchesYawFormula)
{
  const double psi_ned = 0.3;
  const px4_sando_bridge::Quaternion q_ned = {std::cos(psi_ned / 2.0),
    0.0, 0.0, std::sin(psi_ned / 2.0)};

  const px4_sando_bridge::Quaternion q_enu =
    px4_sando_bridge::nedToEnuQuaternion(q_ned);

  EXPECT_NEAR(
    px4_sando_bridge::yawFromQuaternion(q_enu),
    px4_sando_bridge::nedYawToEnu(psi_ned),
    kTolerance);
}

TEST(FrameTransforms, QuaternionRoundTrip)
{
  const px4_sando_bridge::Quaternion q_ned =
    px4_sando_bridge::normalizeQuaternion({0.5, -0.5, 0.5, -0.5});

  const px4_sando_bridge::Quaternion q_enu =
    px4_sando_bridge::nedToEnuQuaternion(q_ned);
  const px4_sando_bridge::Quaternion roundtrip =
    px4_sando_bridge::enuToNedQuaternion(q_enu);

  EXPECT_NEAR(std::abs(quaternionDot(roundtrip, q_ned)), 1.0, kTolerance);
}

TEST(FrameTransforms, NormalizeAngleWraps)
{
  EXPECT_NEAR(px4_sando_bridge::normalizeAngle(0.0), 0.0, kTolerance);
  EXPECT_NEAR(
    px4_sando_bridge::normalizeAngle(4.0), 4.0 - 2.0 * kPi, kTolerance);
  EXPECT_NEAR(
    px4_sando_bridge::normalizeAngle(-4.0), 2.0 * kPi - 4.0, kTolerance);
  EXPECT_NEAR(px4_sando_bridge::normalizeAngle(1.0 + 2.0 * kPi), 1.0, kTolerance);
}

TEST(FrameTransforms, NormalizeQuaternionHandlesDegenerateInput)
{
  const px4_sando_bridge::Quaternion identity =
    px4_sando_bridge::normalizeQuaternion({0.0, 0.0, 0.0, 0.0});
  EXPECT_NEAR(identity[0], 1.0, kTolerance);

  const px4_sando_bridge::Quaternion unit =
    px4_sando_bridge::normalizeQuaternion({2.0, 0.0, 0.0, 0.0});
  EXPECT_NEAR(unit[0], 1.0, kTolerance);
}

TEST(FrameTransforms, YawQuaternionRoundTrip)
{
  const double yaw = -1.234;
  const px4_sando_bridge::Quaternion q = px4_sando_bridge::quaternionFromYaw(yaw);
  EXPECT_NEAR(px4_sando_bridge::yawFromQuaternion(q), yaw, kTolerance);
}
