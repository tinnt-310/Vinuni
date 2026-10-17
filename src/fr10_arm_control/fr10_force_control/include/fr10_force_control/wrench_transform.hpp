#pragma once

#include "fr10_force_control/force_types.hpp"

namespace fr10_force {

class WrenchTransform {
public:
  [[nodiscard]] static bool valid(const RigidTransform &transform);
  [[nodiscard]] static Wrench apply(const Wrench &source,
                                    const RigidTransform &transform);
};

} // namespace fr10_force
