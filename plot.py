import pandas as pd
import matplotlib.pyplot as plt

# Load simulation data
trajectory = pd.read_csv("trajectory.csv")
reference = pd.read_csv("reference.csv")

# Create one window with four subplots
fig, axes = plt.subplots(2, 2, figsize=(14, 9))
fig.suptitle("Vehicle Model Simulation", fontsize=16)

# 1. Vehicle trajectory vs. reference path
ax = axes[0, 0]
ax.plot(reference["x"], reference["y"],
        "--", label="Reference path")
ax.plot(trajectory["true_x"], trajectory["true_y"],
        label="Actual vehicle path")
ax.set_xlabel("X position (m)")
ax.set_ylabel("Y position (m)")
ax.set_title("Vehicle Trajectory")
ax.axis("equal")
ax.grid(True)
ax.legend()

# 2. Lateral velocity over time
ax = axes[0, 1]
ax.plot(trajectory["time"], trajectory["velocity_y"])
ax.set_xlabel("Time (s)")
ax.set_ylabel("Lateral velocity (m/s)")
ax.set_title("Lateral Velocity")
ax.grid(True)

# 3. Yaw rate over time
ax = axes[1, 0]
ax.plot(trajectory["time"], trajectory["yaw_rate"])
ax.set_xlabel("Time (s)")
ax.set_ylabel("Yaw rate (rad/s)")
ax.set_title("Yaw Rate")
ax.grid(True)

# 4. Actual vs. desired steering angle
ax = axes[1, 1]
ax.plot(trajectory["time"], trajectory["steering_angle"],
        label="Actual")
ax.plot(trajectory["time"], trajectory["desired_steering_angle"],
        "--", label="Desired")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Steering angle (rad)")
ax.set_title("Steering Angle")
ax.grid(True)
ax.legend()

# Adjust spacing and display the first window
fig.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()

# 5–6. EKF velocity estimation and correction
fig, axes = plt.subplots(1, 2, figsize=(14, 5))
fig.suptitle("EKF Velocity Analysis", fontsize=15)

# Left: True vs. estimated longitudinal velocity
ax = axes[0]
ax.plot(
    trajectory["time"],
    trajectory["velocity_x"],
    label="True velocity"
)
ax.plot(
    trajectory["time"],
    trajectory["estimated_velocity"],
    "--",
    label="EKF estimated velocity"
)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Longitudinal velocity (m/s)")
ax.set_title("Velocity Estimation")
ax.grid(True)
ax.legend()

# Right: EKF velocity correction
ax = axes[1]
ax.plot(
    trajectory["time"],
    trajectory["velocity_correction"],
    label="Velocity correction"
)
ax.axhline(0, linestyle="--", linewidth=1)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Velocity correction (m/s)")
ax.set_title("Velocity Correction")
ax.grid(True)
ax.legend()

fig.tight_layout(rect=[0, 0, 1, 0.92])
plt.show()


# 7. True vs. GPS vs. EKF position estimates
fig, axes = plt.subplots(2, 1, figsize=(12, 8))
fig.suptitle("EKF Position Estimation", fontsize=15)

# X position
ax = axes[0]
ax.plot(trajectory["time"], trajectory["true_x"],
        label="True X")
ax.plot(trajectory["time"], trajectory["gps_x"],
        ".", markersize=2, alpha=0.5, label="GPS X")
ax.plot(trajectory["time"], trajectory["estimated_x"],
        "--", label="EKF estimated X")
ax.set_xlabel("Time (s)")
ax.set_ylabel("X position (m)")
ax.set_title("X Position")
ax.grid(True)
ax.legend()

# Y position
ax = axes[1]
ax.plot(trajectory["time"], trajectory["true_y"],
        label="True Y")
ax.plot(trajectory["time"], trajectory["gps_y"],
        ".", markersize=2, alpha=0.5, label="GPS Y")
ax.plot(trajectory["time"], trajectory["estimated_y"],
        "--", label="EKF estimated Y")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Y position (m)")
ax.set_title("Y Position")
ax.grid(True)
ax.legend()

fig.tight_layout(rect=[0, 0, 1, 0.94])
plt.show()


# 8. True vs. IMU vs. EKF heading estimates
plt.figure(figsize=(12, 5))

plt.plot(trajectory["time"], trajectory["heading"],
         label="True heading")
plt.plot(trajectory["time"], trajectory["imu_heading"],
         ".", markersize=2, alpha=0.5, label="IMU heading")
plt.plot(trajectory["time"], trajectory["estimated_heading"],
         "--", label="EKF estimated heading")

plt.xlabel("Time (s)")
plt.ylabel("Heading (rad)")
plt.title("EKF Heading Estimation")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()