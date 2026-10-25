// MovingDeckLab: unilateral, friction-limited support for near-level landing.
// This is a reduced-order flat landing-gear model on a kinematic support. It
// does not attach either body or copy the support's position to the vehicle.
#pragma once
#include <algorithm>
#include <cmath>
#include "core_sim/physics_common_types.hpp"

namespace microsoft { namespace projectairsim {
struct SupportStep {
  bool supported = false;
  bool sliding = false;
  Vector3 velocity = Vector3::Zero();
  Vector3 position = Vector3::Zero();
  float normal_delta_v = 0;
};

inline SupportStep SolveSupportContact(
    const Vector3& position, const Vector3& velocity,
    const Vector3& free_acceleration, const CollisionInfo& contact,
    float dt, double now_sec, float friction) {
  SupportStep out;
  out.velocity = velocity + free_acceleration * dt;
  out.position = position + out.velocity * dt;
  const double age = now_sec - contact.time_stamp * 1e-9;
  if (!(dt > 0) || !contact.support_valid || age < -0.001 || age > 0.2 ||
      !contact.normal.allFinite() || contact.normal.z() > -0.95f ||
      !contact.contact_velocity.allFinite()) return out;
  const Vector3 n = contact.normal.normalized();
  // Extrapolate only the measured contact plane during the render/physics lag.
  // No tangential position correction is permitted.
  const float gap = contact.support_gap +
      (position - contact.position - contact.contact_velocity * float(std::max(0.0, age))).dot(n);
  if (gap > 0.03f || gap < -0.08f) return out;
  const float vn = (out.velocity - contact.contact_velocity).dot(n);
  // No tensile contact force: upward thrust/separation immediately releases.
  if (vn > 0.001f) return out;
  out.normal_delta_v = std::max(0.0f, -vn);
  out.velocity += n * out.normal_delta_v;
  const Vector3 relative = out.velocity - contact.contact_velocity;
  const Vector3 tangent = relative - n * relative.dot(n);
  const float slip = tangent.norm();
  const float limit = std::max(0.0f, std::min(friction, contact.support_friction)) * out.normal_delta_v;
  out.sliding = slip > limit + 1e-5f;
  if (slip > 1e-8f) out.velocity -= tangent * (std::min(slip, limit) / slip);
  out.position = position + out.velocity * dt;
  // Resolve penetration along the normal only. Positive gaps are not pulled in.
  if (gap < 0) out.position -= n * gap;
  out.supported = true;
  return out;
}
}}  // namespace
