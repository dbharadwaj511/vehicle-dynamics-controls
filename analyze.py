import pandas as pd
import numpy as np

# Load simulation data
data = pd.read_csv("trajectory.csv")


# -------------------------
# EKF Estimation Errors
# -------------------------

# Position error
position_error = np.sqrt(
    (data["true_x"] - data["estimated_x"]) ** 2
    + (data["true_y"] - data["estimated_y"]) ** 2
)

# Heading error
heading_error = data["true_heading"] - data["estimated_heading"]

# Wrap heading error to [-pi, pi]
heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

# Velocity error
velocity_error = data["velocity_x"] - data["estimated_velocity"]


# -------------------------
# Overall EKF Metrics
# -------------------------

position_rmse = np.sqrt(np.mean(position_error ** 2))
position_max = np.max(np.abs(position_error))

heading_rmse = np.sqrt(np.mean(heading_error ** 2))
heading_max = np.max(np.abs(heading_error))

velocity_rmse = np.sqrt(np.mean(velocity_error ** 2))
velocity_max = np.max(np.abs(velocity_error))


# -------------------------
# Steady-State Velocity
# -------------------------

steady_state = data["time"] >= 1.0

velocity_rmse_steady = np.sqrt(
    np.mean(velocity_error[steady_state] ** 2)
)

velocity_max_steady = np.max(
    np.abs(velocity_error[steady_state])
)


# -------------------------
# Raw Sensor Errors
# -------------------------

# GPS position error
gps_position_error = np.sqrt(
    (data["true_x"] - data["gps_x"]) ** 2
    + (data["true_y"] - data["gps_y"]) ** 2
)

# IMU heading error
imu_heading_error = data["true_heading"] - data["imu_heading"]

# Wrap IMU heading error to [-pi, pi]
imu_heading_error = (
    imu_heading_error + np.pi
) % (2 * np.pi) - np.pi


# -------------------------
# Sensor RMSE
# -------------------------

gps_position_rmse = np.sqrt(
    np.mean(gps_position_error ** 2)
)

imu_heading_rmse = np.sqrt(
    np.mean(imu_heading_error ** 2)
)


# -------------------------
# Print Results
# -------------------------

print("EKF Performance")
print("----------------")

print(f"Position RMSE:              {position_rmse:.4f} m")
print(f"Maximum position error:     {position_max:.4f} m")

print(f"Heading RMSE:               {heading_rmse:.4f} rad")
print(f"Maximum heading error:      {heading_max:.4f} rad")

print(f"Velocity RMSE:              {velocity_rmse:.4f} m/s")
print(f"Maximum velocity error:     {velocity_max:.4f} m/s")


print()
print("Velocity After 1 s")
print("------------------")

print(f"Velocity RMSE:              {velocity_rmse_steady:.4f} m/s")
print(f"Maximum velocity error:     {velocity_max_steady:.4f} m/s")


print()
print("Raw Sensors vs EKF")
print("------------------")

print(f"GPS position RMSE:          {gps_position_rmse:.4f} m")
print(f"EKF position RMSE:          {position_rmse:.4f} m")

print(f"IMU heading RMSE:            {imu_heading_rmse:.4f} rad")
print(f"EKF heading RMSE:            {heading_rmse:.4f} rad")


# -------------------------
# Estimation Improvement
# -------------------------

if gps_position_rmse > 0:
    position_improvement = (
        1 - position_rmse / gps_position_rmse
    ) * 100

    print(
        f"Position error improvement: {position_improvement:.2f}%"
    )

if imu_heading_rmse > 0:
    heading_improvement = (
        1 - heading_rmse / imu_heading_rmse
    ) * 100

    print(
        f"Heading error improvement:  {heading_improvement:.2f}%"
    )


# Evaluate velocity convergence
data = pd.read_csv("trajectory.csv")
data = data[data["time"] >= 2.0].copy()

velocity_error = (
    data["estimated_velocity"] - data["velocity_x"]
).abs()

velocity_rmse = np.sqrt(
    np.mean(
        (data["estimated_velocity"] - data["velocity_x"]) ** 2
    )
)

print(f"Velocity RMSE after 2 seconds: {velocity_rmse:.4f} m/s")

# Find the first time the estimate stays within 0.1 m/s
# of the true velocity for the rest of the simulation.
within_tolerance = velocity_error <= 0.1
convergence_time = None

for i in range(len(data)):
    if within_tolerance.iloc[i:].all():
        convergence_time = data["time"].iloc[i]
        break

if convergence_time is not None:
    print(f"Convergence time (0.1 m/s tolerance): {convergence_time:.2f} s")
else:
    print("Estimate did not remain within 0.1 m/s of true velocity.")