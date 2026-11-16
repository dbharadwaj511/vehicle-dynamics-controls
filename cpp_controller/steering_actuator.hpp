
#pragma once

namespace vehicle_control {

struct SteeringActuatorConfig {
    double max_angle_rad = 0.5;
    double max_rate_rad_s = 2.0;
};

// Apply steering angle and steering rate constraints.
//
// Returns true if all inputs and the calculated output are valid.
// Returns false for invalid inputs or non-finite output.
//
// applied_rad is only modified on success.

[[nodiscard]] bool tryApplySteeringLimitsRad(
    double requested_rad,
    double previous_rad,
    double dt_s,
    double& applied_rad,
    const SteeringActuatorConfig& config = {}
) noexcept;

} // namespace vehicle_control
