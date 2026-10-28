// Copyright 2026, SANDO Lyrical / PX4 integration
// SPDX-License-Identifier: MIT

#ifndef PX4_SANDO_BRIDGE__FRAME_TRANSFORMS_HPP_
#define PX4_SANDO_BRIDGE__FRAME_TRANSFORMS_HPP_

#include <array>

namespace px4_sando_bridge
{

using Vector3 = std::array<double, 3>;

/// Quaternion in [w, x, y, z] order (PX4 and ROS message storage convention).
using Quaternion = std::array<double, 4>;

/// Convert a vector from local NED to world ENU.
Vector3 nedToEnu(const Vector3 & ned);

/// Convert a vector from world ENU to local NED.
Vector3 enuToNed(const Vector3 & enu);

/// Convert an orientation quaternion from NED/FRD to ENU/FLU.
Quaternion nedToEnuQuaternion(const Quaternion & q_ned);

/// Convert an orientation quaternion from ENU/FLU to NED/FRD.
Quaternion enuToNedQuaternion(const Quaternion & q_enu);

/// Convert a heading from NED (clockwise from North) to ENU (CCW from East).
double nedYawToEnu(double yaw_ned);

/// Convert a heading from ENU (CCW from East) to NED (clockwise from North).
double enuYawToNed(double yaw_enu);

/// Wrap an angle to (-pi, pi].
double normalizeAngle(double angle);

/// Build a [w, x, y, z] quaternion for a yaw-only rotation in ENU/FLU.
Quaternion quaternionFromYaw(double yaw_enu);

/// Extract yaw (rad) from a [w, x, y, z] quaternion.
double yawFromQuaternion(const Quaternion & q);

/// Return a unit-length quaternion. A zero quaternion yields identity.
Quaternion normalizeQuaternion(const Quaternion & q);

}  // namespace px4_sando_bridge

#endif  // PX4_SANDO_BRIDGE__FRAME_TRANSFORMS_HPP_
