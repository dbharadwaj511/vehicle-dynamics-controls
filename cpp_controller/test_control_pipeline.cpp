
#include "lqr_controller.hpp"
#include "steering_actuator.hpp"

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>

struct PipelineTest {
    const char* name;
    vehicle_control::LqrInput input;
    double previous_steering_rad;
    double dt_s;
    double expected_command_rad;
    double expected_applied_rad;
};

int main() {

    constexpr double tolerance = 1e-12;

    
    const std::array<PipelineTest, 3> tests = {{

        {
            "MATLAB actuator transition at t = 1 s",
            {
                0.33225028797125311,
                0.49163917743704488,
                -0.011353950085768793,
                0.034103859975540107,
                -0.15173264175620615
            },
            -0.047768962876906289,
            0.0003125,
            -0.56770557973869151,
            -0.048393962876906289
        },

        {
            "MATLAB actuator transition at t = 3 s",
            {
                -0.074629181764601232,
                -0.034158776757072268,
                0.0098431251781399935,
                0.044808783485848491,
                -0.039902754660131531
            },
            -0.26610236710808421,
            0.0003125,
            -0.26425915048438731,
            -0.26547736710808423
        },

        {
            "MATLAB actuator transition at t = 8 s",
            {
                0.037071216163936438,
                0.013479755407310945,
                0.0026753307155150492,
                -0.0059614589675066896,
                -0.19622733408229959
            },
            -0.2009401935140688,
            0.0003125,
            -0.2010832497886044,
            -0.2010832497886044
        }

    }};


    int passed = 0;
    int failed = 0;

    std::cout << std::setprecision(15);
    std::cout << "LQR Control Pipeline Verification\n";
    std::cout << "=================================\n\n";

    for (const auto& test : tests) {

        // Stage 1: Calculate requested steering.
        double requested = 0.0;

        const bool controller_ok =
            vehicle_control::tryComputeLqrSteeringCommandRad(
                test.input, requested
            );

        // Stage 2: Apply actuator constraints.
        double applied = 0.0;

        const bool actuator_ok =
            vehicle_control::tryApplySteeringLimitsRad(
                requested,
                test.previous_steering_rad,
                test.dt_s,
                applied
            );

        const double command_error =
            std::abs(requested - test.expected_command_rad);

        const double applied_error =
            std::abs(applied - test.expected_applied_rad);

        const bool success =
            controller_ok &&
            actuator_ok &&
            command_error <= tolerance &&
            applied_error <= tolerance;

        std::cout << test.name << '\n';

        std::cout << "  Requested: " << requested << " rad\n";
        std::cout << "  Applied:   " << applied << " rad\n";

        std::cout << "  Result: "
                  << (success ? "PASS" : "FAIL") << "\n\n";

        if (success) {
            ++passed;
        } else {
            ++failed;
        }
    }

    std::cout << "=================================\n";
    std::cout << "Tests passed: " << passed << '\n';
    std::cout << "Tests failed: " << failed << '\n';

    if (failed > 0) {
        std::cout << "VERIFICATION FAILED\n";
        return 1;
    }

    std::cout << "ALL TESTS PASSED\n";
    return 0;
}
