// Copyright 2026, SANDO Lyrical / PX4 integration
// SPDX-License-Identifier: MIT

#include "px4_sando_bridge/frame_transforms.hpp"

#include <cmath>

#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace px4_sando_bridge
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

// NED -> ENU basis change. This is a proper 180 deg rotation about the
// (1, 1, 0) axis, so it can be applied to rotation matrices as-is.
const tf2::Matrix3x3 & worldBasisChange()
{
  static const tf2::Matrix3x3 kWorldBasis(0.0, 1.0, 0.0,
                                          1.0, 0.0, 0.0,
                                          0.0, 0.0, -1.0);
  return kWorldBasis;
}

// FRD -> FLU body basis change (180 deg about the body X / Forward axis).
const tf2::Matrix3x3 & bodyBasisChange()
{
  static const tf2::Matrix3x3 kBodyBasis(1.0, 0.0, 0.0,
                                         0.0, -1.0, 0.0,
                                         0.0, 0.0, -1.0);
  return kBodyBasis;
}

tf2::Quaternion toTf2(const Quaternion & q)
{
  const Quaternion unit = normalizeQuaternion(q);
  return tf2::Quaternion(unit[1], unit[2], unit[3], unit[0]);
}

Quaternion fromTf2(const tf2::Quaternion & q)
{
  tf2::Quaternion unit = q;
  unit.normalize();
  return {unit.w(), unit.x(), unit.y(), unit.z()};
}

}  // namespace

Vector3 nedToEnu(const Vector3 & ned)
{
  return {ned[1], ned[0], -ned[2]};
}

Vector3 enuToNed(const Vector3 & enu)
{
  return {enu[1], enu[0], -enu[2]};
}

Quaternion nedToEnuQuaternion(const Quaternion & q_ned)
{
  const tf2::Matrix3x3 r_ned_frd(toTf2(q_ned));
  const tf2::Matrix3x3 r_enu_flu =
    worldBasisChange() * r_ned_frd * bodyBasisChange();

  tf2::Quaternion q_enu;
  r_enu_flu.getRotation(q_enu);
  return fromTf2(q_enu);
}

Quaternion enuToNedQuaternion(const Quaternion & q_enu)
{
  const tf2::Matrix3x3 r_enu_flu(toTf2(q_enu));
  const tf2::Matrix3x3 r_ned_frd =
    worldBasisChange().transpose() * r_enu_flu * bodyBasisChange().transpose();

  tf2::Quaternion q_ned;
  r_ned_frd.getRotation(q_ned);
  return fromTf2(q_ned);
}

double normalizeAngle(double angle)
{
  if (!std::isfinite(angle)) {
    return 0.0;
  }

  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle < 0.0) {
    angle += 2.0 * kPi;
  }
  return angle - kPi;
}

double nedYawToEnu(double yaw_ned)
{
  return normalizeAngle(kPi / 2.0 - yaw_ned);
}

double enuYawToNed(double yaw_enu)
{
  return normalizeAngle(kPi / 2.0 - yaw_enu);
}

Quaternion normalizeQuaternion(const Quaternion & q)
{
  const double norm = std::sqrt(
    q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

  if (!std::isfinite(norm) || norm < 1e-12) {
    return {1.0, 0.0, 0.0, 0.0};
  }

  return {q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm};
}

Quaternion quaternionFromYaw(double yaw_enu)
{
  const double half = normalizeAngle(yaw_enu) * 0.5;
  return {std::cos(half), 0.0, 0.0, std::sin(half)};
}

double yawFromQuaternion(const Quaternion & q)
{
  const Quaternion unit = normalizeQuaternion(q);

  const double siny_cosp = 2.0 * (unit[0] * unit[3] + unit[1] * unit[2]);
  const double cosy_cosp =
    1.0 - 2.0 * (unit[2] * unit[2] + unit[3] * unit[3]);

  return normalizeAngle(std::atan2(siny_cosp, cosy_cosp));
}

}  // namespace px4_sando_bridge
