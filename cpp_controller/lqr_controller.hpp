
#pragma once

namespace vehicle_control {

// Tracking errors supplied to the LQR controller.
//
// Units:
//   lateral_velocity_error_mps : m/s
//   yaw_rate_error_rad_s       : rad/s
//   lateral_error_m            : m
//   heading_error_rad          : rad
//   feedforward_steering_rad   : rad

struct LqrInput {
    double lateral_velocity_error_mps;
    double yaw_rate_error_rad_s;
    double lateral_error_m;
    double heading_error_rad;
    double feedforward_steering_rad;
};

// Calculate the requested steering angle.
//
// Returns true if the inputs and calculated output are valid.
// Returns false if any input or the result is non-finite.
//
// steering_command_rad is only modified on success.

[[nodiscard]] bool tryComputeLqrSteeringCommandRad(
    const LqrInput& input,
    double& steering_command_rad
) noexcept;

} // namespace vehicle_control
