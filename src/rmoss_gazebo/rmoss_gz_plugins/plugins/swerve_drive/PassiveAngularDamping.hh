#pragma once
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

namespace originsim {
// Nonnegative coefficients produce tau_world dot omega_world <= 0.
inline gz::math::Vector3d PassiveAngularDamping(
    const gz::math::Quaterniond &worldFromBody,
    const gz::math::Vector3d &worldOmega,
    const gz::math::Vector3d &coefficients)
{
  const auto bodyOmega = worldFromBody.RotateVectorReverse(worldOmega);
  const gz::math::Vector3d bodyTorque(-coefficients.X() * bodyOmega.X(),
      -coefficients.Y() * bodyOmega.Y(), -coefficients.Z() * bodyOmega.Z());
  return worldFromBody.RotateVector(bodyTorque);
}
}
