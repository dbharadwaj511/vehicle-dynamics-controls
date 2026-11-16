
#include "lqr_controller.hpp"

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

struct TestCase {
    const char* name;
    vehicle_control::LqrInput input;
    double expected_steering_rad;
};

int main() {

    constexpr double tolerance = 1e-12;

    const std::array<TestCase, 3> test_cases = {{

        {
            "MATLAB case at t = 1 s",
            {0.33225028797125311, 0.49163917743704488,
             -0.011353950085768793, 0.034103859975540107,
             -0.15173264175620615},
            -0.56770557973869151
        },

        {
            "MATLAB case at t = 3 s",
            {-0.074629181764601232, -0.034158776757072268,
             0.0098431251781399935, 0.044808783485848491,
             -0.039902754660131531},
            -0.26425915048438731
        },

        {
            "MATLAB case at t = 8 s",
            {0.037071216163936438, 0.013479755407310945,
             0.0026753307155150492, -0.0059614589675066896,
             -0.19622733408229959},
            -0.2010832497886044
        }

    }};

    int passed = 0;
    int failed = 0;

    std::cout << std::setprecision(15);
    std::cout << "LQR Controller Verification\n\n";

    // Test valid MATLAB reference cases.
    for (const auto& test : test_cases) {

        double actual = 0.0;

        const bool calculation_succeeded =
            vehicle_control::tryComputeLqrSteeringCommandRad(
                test.input, actual
            );

        const double error =
            std::abs(actual - test.expected_steering_rad);

        const bool success =
            calculation_succeeded && error <= tolerance;

        std::cout << test.name << '\n';
        std::cout << "  Expected: " << test.expected_steering_rad << " rad\n";
        std::cout << "  Actual:   " << actual << " rad\n";
        std::cout << "  Result:   "
                  << (success ? "PASS" : "FAIL") << "\n\n";

        if (success) {
            ++passed;
        } else {
            ++failed;
        }
    }

    // Test that invalid inputs are rejected without modifying output.
    const vehicle_control::LqrInput valid_input{
        0.0, 0.0, 0.0, 0.0, 0.0
    };

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    auto expect_rejected = [&](const char* name,
                               const vehicle_control::LqrInput& input) {

        constexpr double sentinel = 123.456;
        double output = sentinel;

        const bool accepted =
            vehicle_control::tryComputeLqrSteeringCommandRad(
                input, output
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

    auto bad = valid_input;
    bad.lateral_velocity_error_mps = nan;
    expect_rejected("NaN lateral velocity", bad);

    bad = valid_input;
    bad.yaw_rate_error_rad_s = inf;
    expect_rejected("Infinite yaw rate", bad);

    bad = valid_input;
    bad.lateral_error_m = -inf;
    expect_rejected("Infinite lateral error", bad);

    bad = valid_input;
    bad.heading_error_rad = nan;
    expect_rejected("NaN heading error", bad);

    bad = valid_input;
    bad.feedforward_steering_rad = inf;
    expect_rejected("Infinite feedforward steering", bad);

    // Finite input that produces a non-finite output.
    bad = valid_input;
    bad.lateral_error_m = 1e308;
    expect_rejected("Non-finite calculated output", bad);

    std::cout << "\n===========================\n";
    std::cout << "Tests passed: " << passed << '\n';
    std::cout << "Tests failed: " << failed << '\n';

    if (failed > 0) {
        std::cout << "VERIFICATION FAILED\n";
        return 1;
    }

    std::cout << "ALL TESTS PASSED\n";
    return 0;
}
