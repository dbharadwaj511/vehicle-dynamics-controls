
#include "steering_actuator.hpp"

#include <algorithm>
#include <cmath>

namespace vehicle_control {

bool tryApplySteeringLimitsRad(
    double requested_rad,
    double previous_rad,
    double dt_s,
    double& applied_rad,
    const SteeringActuatorConfig& config
) noexcept {

    // Validate actuator inputs.
    if (!std::isfinite(requested_rad) ||
        !std::isfinite(previous_rad) ||
        !std::isfinite(dt_s) ||
        dt_s <= 0.0) {
        return false;
    }

    // Validate actuator configuration.
    if (!std::isfinite(config.max_angle_rad) ||
        !std::isfinite(config.max_rate_rad_s) ||
        config.max_angle_rad <= 0.0 ||
        config.max_rate_rad_s <= 0.0) {
        return false;
    }

    // Apply steering angle constraint.
    const double angle_limited = std::clamp(
        requested_rad,
        -config.max_angle_rad,
        config.max_angle_rad
    );

    // Maximum allowed steering change during this timestep.
    const double max_delta_change =
        config.max_rate_rad_s * dt_s;

    if (!std::isfinite(max_delta_change)) {
        return false;
    }

    // Apply steering rate constraint.
    const double delta_change = std::clamp(
        angle_limited - previous_rad,
        -max_delta_change,
        max_delta_change
    );

    const double result = previous_rad + delta_change;

    // Validate calculated output.
    if (!std::isfinite(result)) {
        return false;
    }

    applied_rad = result;

    return true;
}

} // namespace vehicle_control
