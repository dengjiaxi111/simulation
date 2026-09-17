#pragma once

#include <cmath>

namespace seu_sentry_sim_control
{
inline bool is_startup_motion_command(double vx, double vy, double wz, double deadband)
{
  return std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz) &&
         (std::hypot(vx, vy) > deadband || std::abs(wz) > deadband);
}
}  // namespace seu_sentry_sim_control
