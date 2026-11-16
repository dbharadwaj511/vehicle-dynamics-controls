
#include "steering_actuator.hpp"

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

struct ActuatorTest {
    const char* name;
    double requested_rad;
    double previous_rad;
    double dt_s;
    double expected_rad;
};

int main() {

    constexpr double tolerance = 1e-12;

    const std::array<ActuatorTest, 5> tests = {{

        {"Upper angle limit", 1.0, 0.4, 0.1, 0.5},
        {"Lower angle limit", -1.0, -0.4, 0.1, -0.5},
        {"Positive rate limit", 0.5, 0.0, 0.1, 0.2},
        {"Negative rate limit", -0.5, 0.3, 0.1, 0.1},
        {"No constraint active", 0.1, -0.05, 0.1, 0.1}

    }};

    int passed = 0;
    int failed = 0;

    std::cout << std::setprecision(15);
    std::cout << "Steering Actuator Verification\n\n";

    // Test normal actuator behavior.
    for (const auto& test : tests) {

        double actual = 0.0;

        const bool calculation_succeeded =
            vehicle_control::tryApplySteeringLimitsRad(
                test.requested_rad,
                test.previous_rad,
                test.dt_s,
                actual
            );

        const double error =
            std::abs(actual - test.expected_rad);

        const bool success =
            calculation_succeeded && error <= tolerance;

        std::cout << test.name << '\n';
        std::cout << "  Expected: " << test.expected_rad << " rad\n";
        std::cout << "  Actual:   " << actual << " rad\n";
        std::cout << "  Result:   "
                  << (success ? "PASS" : "FAIL") << "\n\n";

        if (success) {
            ++passed;
        } else {
            ++failed;
        }
    }

    // Verify invalid inputs are rejected without modifying output.
    constexpr double sentinel = 123.456;

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    auto expect_rejected = [&](const char* name,
                               double requested,
                               double previous,
                               double dt,
                               const vehicle_control::SteeringActuatorConfig& config = {}) {

        double output = sentinel;

        const bool accepted =
            vehicle_control::tryApplySteeringLimitsRad(
                requested, previous, dt, output, config
            );

        const bool success =
            !accepted && output == sentinel;

        std::cout << name << ": "
                  << (success ? "PASS" : "FAIL") << '\n';

        if (success) {
            ++passed;
        } else {
            ++failed;
        }
    };

    std::cout << "\nInvalid Input Tests\n";

    expect_rejected("NaN requested steering", nan, 0.0, 0.01);
    expect_rejected("Infinite previous steering", 0.0, inf, 0.01);
    expect_rejected("Zero timestep", 0.0, 0.0, 0.0);
    expect_rejected("Negative timestep", 0.0, 0.0, -0.01);

    vehicle_control::SteeringActuatorConfig bad_config;
    bad_config.max_angle_rad = 0.0;

    expect_rejected("Invalid angle limit", 0.0, 0.0, 0.01, bad_config);

    bad_config = {};
    bad_config.max_rate_rad_s = inf;

    expect_rejected("Infinite rate limit", 0.0, 0.0, 0.01, bad_config);

    std::cout << "\n===============================\n";
    std::cout << "Tests passed: " << passed << '\n';
    std::cout << "Tests failed: " << failed << '\n';

    if (failed > 0) {
        std::cout << "VERIFICATION FAILED\n";
        return 1;
    }

    std::cout << "ALL TESTS PASSED\n";
    return 0;
}
