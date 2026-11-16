#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <random>


bool invertMatrix3x3(
    double matrix[3][3],
    double inverse[3][3]
) {
    double determinant =
          matrix[0][0] * (matrix[1][1] * matrix[2][2]
                        - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2]
                        - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1]
                        - matrix[1][1] * matrix[2][0]);

    if (std::abs(determinant) < 1e-12) {
        return false;
    }

    inverse[0][0] =
        (matrix[1][1] * matrix[2][2]
       - matrix[1][2] * matrix[2][1]) / determinant;

    inverse[0][1] =
        (matrix[0][2] * matrix[2][1]
       - matrix[0][1] * matrix[2][2]) / determinant;

    inverse[0][2] =
        (matrix[0][1] * matrix[1][2]
       - matrix[0][2] * matrix[1][1]) / determinant;

    inverse[1][0] =
        (matrix[1][2] * matrix[2][0]
       - matrix[1][0] * matrix[2][2]) / determinant;

    inverse[1][1] =
        (matrix[0][0] * matrix[2][2]
       - matrix[0][2] * matrix[2][0]) / determinant;

    inverse[1][2] =
        (matrix[0][2] * matrix[1][0]
       - matrix[0][0] * matrix[1][2]) / determinant;

    inverse[2][0] =
        (matrix[1][0] * matrix[2][1]
       - matrix[1][1] * matrix[2][0]) / determinant;

    inverse[2][1] =
        (matrix[0][1] * matrix[2][0]
       - matrix[0][0] * matrix[2][1]) / determinant;

    inverse[2][2] =
        (matrix[0][0] * matrix[1][1]
       - matrix[0][1] * matrix[1][0]) / determinant;

    return true;
}

int main() {

    // std::random_device rd;
    // std::default_random_engine generator(rd());
    std::default_random_engine generator(42);

    std::normal_distribution<double> gps_noise(0.0, 0.1); // 0.1
    std::normal_distribution<double> imu_noise(0.0, 0.02); // 0.02

    // Generate reference trajectory
    std::vector<double> reference_x;
    std::vector<double> reference_y;

    std::ofstream reference_file("reference.csv");

    reference_file << "x,y\n";

    for (double x_ref = 0.0; x_ref <= 50.0; x_ref += 0.1) {

        double y_ref = 2.0 * std::sin(x_ref / 5.0);

        reference_x.push_back(x_ref);
        reference_y.push_back(y_ref);

        reference_file << x_ref << "," << y_ref << "\n";
    }

    reference_file.close();


    // Vehicle state
    double x = 0.0;
    double y = 0.0;
    double heading = std::atan(0.4);

    double velocity_x = 5.0;
    double velocity_y = 0.0;
    double yaw_rate = 0.0;

    double steering_angle = 0.0;

    // Vehicle parameters
    double mass = 200.0;
    double inertia_z = 250.0;

    double front_length = 1.25;
    double rear_length = 1.25;

    double cornering_stiffness_front = 5000.0;
    double cornering_stiffness_rear = 5000.0;

    double wheelbase = front_length + rear_length;

    double dt = 0.01;
    double simulation_time = 10.0;

    double max_steering_angle = 0.524; // 30 degrees
    double max_steering_rate = 2.0; // 0.5 rad = ~29 degrees

    double lookahead_distance = 2.0; // 1.2

    // EKF state estimate
    double estimated_x = 0.0;
    double estimated_y = 0.0;
    double estimated_heading = std::atan(0.4);
    double estimated_velocity = 4.5; // 4.0

    // EKF covariance matrix
    double P[4][4] = {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 0.1, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };

    // EKF process noise: motion model uncertainty
    double Q[4][4] = {
        {0.01, 0.0, 0.0, 0.0},
        {0.0, 0.01, 0.0, 0.0},
        {0.0, 0.0, 0.001, 0.0},
        {0.0, 0.0, 0.0, 0.001}
    };

    // EKF Measurement Matrix: converts P into sensor units
    // Is 3x4 matrix because we don't have a velocity sensor
    double H[3][4] = {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0}
    };

    // EKF measurement noise: sensor uncertainty
    double R[3][3] = {
        {0.01, 0.0, 0.0},
        {0.0, 0.01, 0.0},
        {0.0, 0.0, 0.0004}
    };

    std::ofstream file("trajectory.csv");

    
    file << "time,"
        // True vehicle state
        << "true_x,true_y,true_heading,velocity_x,velocity_y,yaw_rate,"
        // EKF state estimates
        << "estimated_x,estimated_y,estimated_heading,estimated_velocity,"
        // Sensor measurements
        << "gps_x,gps_y,imu_heading,"
        // Control inputs
        << "desired_steering_angle,steering_angle,"
        // Tracking errors
        << "position_error,heading_error,"
        // EKF diagnostic
        << "velocity_correction\n";
    double sum_squared_position_error = 0.0;
    double sum_squared_heading_error = 0.0;

    double max_position_error = 0.0;
    double max_heading_error = 0.0;

    int sample_count = 0;

    // Simulation loop
    for (double time = 0.0; time <= simulation_time; time += dt) {

        // Simulated sensor measurements
        double measured_x = x + gps_noise(generator);
        double measured_y = y + gps_noise(generator);
        double measured_heading = heading + imu_noise(generator);
        
        // Measurement residual
        double residual[3];

        residual[0] = measured_x - estimated_x;
        residual[1] = measured_y - estimated_y;
        residual[2] = measured_heading - estimated_heading;

        while (residual[2] > M_PI)
            residual[2] -= 2.0 * M_PI;

        while (residual[2] < -M_PI)
            residual[2] += 2.0 * M_PI;

        double HP[3][4] = {};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    HP[i][j] += H[i][k] * P[k][j];
                }
            }
        }

        double HPHt[3][3] = {};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 4; k++) {
                    HPHt[i][j] += HP[i][k] * H[j][k];
                }
            }
        }

        double S[3][3] = {};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                S[i][j] = HPHt[i][j] + R[i][j];
            }
        }

        // Calculate inverse of S
        double S_inverse[3][3] = {};
        if (!invertMatrix3x3(S, S_inverse)) {
            std::cerr << "Failed to invert S matrix.\n";
            return 1;
        }

        double PHt[4][3] = {};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 4; k++) {
                    PHt[i][j] += P[i][k] * H[j][k];
                }
            }
        }

        double K[4][3] = {};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    K[i][j] += PHt[i][k] * S_inverse[k][j];
                }
            }
        }

        // Correct EKF state estimate
        double state_correction[4] = {};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                state_correction[i] += K[i][j] * residual[j];
            }
        }

        estimated_x += state_correction[0];
        estimated_y += state_correction[1];
        estimated_heading += state_correction[2];
        estimated_velocity += state_correction[3];

        // Normalize heading
        while (estimated_heading > M_PI)
            estimated_heading -= 2.0 * M_PI;

        while (estimated_heading < -M_PI)
            estimated_heading += 2.0 * M_PI;


        // Joseph form covariance update
        double KH[4][4] = {};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 3; k++) {
                    KH[i][j] += K[i][k] * H[k][j];
                }
            }
        }

        double I_KH[4][4] = {};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                if (i == j)
                    I_KH[i][j] = 1.0 - KH[i][j];
                else
                    I_KH[i][j] = -KH[i][j];
            }
        }

        // Calculate (I - KH)P
        double term1[4][4] = {};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    term1[i][j] += I_KH[i][k] * P[k][j];
                }
            }
        }

        // Calculate (I - KH)P(I - KH)^T
        double term2[4][4] = {};

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    term2[i][j] += term1[i][k] * I_KH[j][k];
                }
            }
        }

        // Calculate K R
        double KR[4][3] = {};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    KR[i][j] += K[i][k] * R[k][j];
                }
            }
        }

        // Calculate K R K^T
        double term3[4][4] = {};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 3; k++) {
                    term3[i][j] += KR[i][k] * K[j][k];
                }
            }
        }

        // Final Joseph form covariance
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                P[i][j] = term2[i][j] + term3[i][j];
            }
        }

        // Find lookahead point
        double target_x = estimated_x;
        double target_y = estimated_y;

        for (size_t i = 0; i < reference_x.size(); i++) {

            double dx = reference_x[i] - estimated_x;
            double dy = reference_y[i] - estimated_y;

            double distance = std::sqrt(dx * dx + dy * dy);

            double forward_direction =
                dx * std::cos(estimated_heading) +
                dy * std::sin(estimated_heading);

            if (distance >= lookahead_distance &&
                forward_direction > 0) {

                target_x = reference_x[i];
                target_y = reference_y[i];

                break;
            }
        }

        // Calculate angle to lookahead point
        double target_angle =
            std::atan2(target_y - estimated_y,
                    target_x - estimated_x);

        double alpha = target_angle - estimated_heading;

        while (alpha > M_PI)
            alpha -= 2.0 * M_PI;

        while (alpha < -M_PI)
            alpha += 2.0 * M_PI;

        // Pure Pursuit controller
        double desired_steering_angle =
            std::atan(
                (2.0 * wheelbase * std::sin(alpha))
                / lookahead_distance
            );

        // Limit desired steering angle
        if (desired_steering_angle > max_steering_angle)
            desired_steering_angle = max_steering_angle;

        if (desired_steering_angle < -max_steering_angle)
            desired_steering_angle = -max_steering_angle;

        // Apply steering rate limit
        double steering_difference =
            desired_steering_angle - steering_angle;

        double max_steering_change =
            max_steering_rate * dt;

        if (steering_difference > max_steering_change)
            steering_difference = max_steering_change;

        if (steering_difference < -max_steering_change)
            steering_difference = -max_steering_change;

        steering_angle += steering_difference;

        // Find closest point on reference path
        double closest_distance = 1e9;
        size_t closest_index = 0;

        for (size_t i = 0; i < reference_x.size(); i++) {

            double dx = reference_x[i] - x;
            double dy = reference_y[i] - y;

            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance < closest_distance) {
                closest_distance = distance;
                closest_index = i;
            }
        }

        // Calculate signed cross-track error
        double dx = reference_x[closest_index] - x;
        double dy = reference_y[closest_index] - y;

        double reference_heading =
            std::atan((2.0 / 5.0) *
            std::cos(reference_x[closest_index] / 5.0));

        double normal_x = -std::sin(reference_heading);
        double normal_y =  std::cos(reference_heading);

        double position_error =
            dx * normal_x +
            dy * normal_y;

        // Calculate heading error
        double heading_error = reference_heading - heading;

        while (heading_error > M_PI)
            heading_error -= 2.0 * M_PI;

        while (heading_error < -M_PI)
            heading_error += 2.0 * M_PI;

        sum_squared_position_error += position_error * position_error;
        sum_squared_heading_error += heading_error * heading_error;

        if (std::abs(position_error) > max_position_error)
            max_position_error = std::abs(position_error);

        if (std::abs(heading_error) > max_heading_error)
            max_heading_error = std::abs(heading_error);

        sample_count++;

        // Save data
        file << time << ","
            // True vehicle state
            << x << ","
            << y << ","
            << heading << ","
            << velocity_x << ","
            << velocity_y << ","
            << yaw_rate << ","
            // EKF state estimates
            << estimated_x << ","
            << estimated_y << ","
            << estimated_heading << ","
            << estimated_velocity << ","
            // Sensor measurements
            << measured_x << ","
            << measured_y << ","
            << measured_heading << ","
            // Control inputs
            << desired_steering_angle << ","
            << steering_angle << ","
            // Tracking errors
            << position_error << ","
            << heading_error << ","
            // EKF diagnostic
            << state_correction[3] << "\n";

        // Calculate tire slip angles
        double front_slip_angle =
            steering_angle
            - std::atan2(
                velocity_y + front_length * yaw_rate,
                velocity_x
            );

        double rear_slip_angle =
            -std::atan2(
                velocity_y - rear_length * yaw_rate,
                velocity_x
            );

        // Calculate lateral tire forces
        double front_lateral_force =
            cornering_stiffness_front * front_slip_angle;

        double rear_lateral_force =
            cornering_stiffness_rear * rear_slip_angle;

        // Calculate vehicle lateral and yaw accelerations
        double lateral_acceleration =
            (front_lateral_force * std::cos(steering_angle)
            + rear_lateral_force) / mass
            - velocity_x * yaw_rate;

        double yaw_acceleration =
            (front_length * front_lateral_force * std::cos(steering_angle)
            - rear_length * rear_lateral_force) / inertia_z;

        // Update vehicle lateral velocity and yaw rate
        velocity_y += lateral_acceleration * dt;
        yaw_rate += yaw_acceleration * dt;

        // Update vehicle position using body-frame velocities
        x += (velocity_x * std::cos(heading)
            - velocity_y * std::sin(heading)) * dt;

        y += (velocity_x * std::sin(heading)
            + velocity_y * std::cos(heading)) * dt;

        // Update vehicle heading
        heading += yaw_rate * dt;

            double F[4][4] = {
            {1.0, 0.0, -estimated_velocity * std::sin(estimated_heading) * dt, std::cos(estimated_heading) * dt},
            {0.0, 1.0, estimated_velocity * std::cos(estimated_heading) * dt, std::sin(estimated_heading) * dt},
            {0.0, 0.0, 1.0, (std::tan(steering_angle) / wheelbase) * dt},
            {0.0, 0.0, 0.0, 1.0}
        };

        // Calculate A = F * P
        double A[4][4] = {};

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    A[i][j] += F[i][k] * P[k][j];
                }
            }
        }

        // Calculate new_P = A * F^T
        double new_P[4][4] = {};

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    new_P[i][j] += A[i][k] * F[j][k];
                }
            }
        }

        // Update covariance
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                P[i][j] = new_P[i][j] + Q[i][j];
            }
        }

        // EKF prediction test: estimated vel
        estimated_x =
            estimated_x
            + estimated_velocity * std::cos(estimated_heading) * dt;

        estimated_y =
            estimated_y
            + estimated_velocity * std::sin(estimated_heading) * dt;

        estimated_heading =
            estimated_heading
            + (estimated_velocity / wheelbase)
            * std::tan(steering_angle)
            * dt;
        
        while (estimated_heading > M_PI)
            estimated_heading -= 2.0 * M_PI;

        while (estimated_heading < -M_PI)
            estimated_heading += 2.0 * M_PI;
    }

    file.close();

    std::cout << "Simulation complete.\n";

    double rms_position_error =
        std::sqrt(sum_squared_position_error / sample_count);

    double rms_heading_error =
        std::sqrt(sum_squared_heading_error / sample_count);

    std::cout << "RMS cross-track error: "
            << rms_position_error << " m\n";

    std::cout << "Maximum cross-track error: "
            << max_position_error << " m\n";

    std::cout << "RMS heading error: "
            << rms_heading_error << " rad\n";

    std::cout << "Maximum heading error: "
            << max_heading_error << " rad\n";

    return 0;
}