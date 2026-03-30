"""
UAV Altitude Simulation — Base Model
=====================================
Maincrafts Technology | UAV Internship Task 1 — Deliverable 3

Description:
    Simple 1D quadcopter altitude simulation using Newton's Second Law.
    The drone is subject to a constant upward thrust and gravity.
    This is the base model provided in the task brief.

Physics:
    Net Force      = Thrust - (mass × gravity)
    Acceleration   = Net Force / mass
    Velocity      += Acceleration × dt
    Height        += Velocity × dt

Usage:
    python uav_altitude_simulation.py

Requirements:
    pip install numpy matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt

# ─── Constants ────────────────────────────────────────────────────────────────
mass    = 1.5        # kg  — drone mass
gravity = 9.81       # m/s² — gravitational acceleration
thrust  = 20         # N   — constant upward thrust
dt      = 0.1        # s   — time step
time    = np.arange(0, 10, dt)  # 0 to 10 seconds

# ─── Initial Conditions ───────────────────────────────────────────────────────
velocity = 0.0
height   = 0.0
heights  = []

# ─── Simulation Loop ──────────────────────────────────────────────────────────
for t in time:
    net_force    = thrust - (mass * gravity)
    acceleration = net_force / mass
    velocity    += acceleration * dt
    height      += velocity * dt
    heights.append(height)

# ─── Analysis ─────────────────────────────────────────────────────────────────
weight         = mass * gravity
net_force_val  = thrust - weight
max_height     = max(heights)

print("=" * 50)
print("  UAV Altitude Simulation — Base Model")
print("=" * 50)
print(f"  Drone mass       : {mass} kg")
print(f"  Weight force     : {weight:.3f} N")
print(f"  Applied thrust   : {thrust} N")
print(f"  Net upward force : {net_force_val:.3f} N")
print(f"  Acceleration     : {net_force_val / mass:.3f} m/s²")
print(f"  Max height (10s) : {max_height:.2f} m")
print("=" * 50)

# ─── Plot ─────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(12, 5))
fig.suptitle("UAV Altitude Simulation — Base Model\n"
             f"(mass={mass}kg, thrust={thrust}N, constant)",
             fontsize=13, fontweight='bold')

# Height vs Time
axes[0].plot(time, heights, color='royalblue', linewidth=2, label='Height (m)')
axes[0].axhline(y=0, color='brown', linestyle='--', linewidth=1, label='Ground level')
axes[0].fill_between(time, heights, alpha=0.15, color='royalblue')
axes[0].set_xlabel("Time (s)")
axes[0].set_ylabel("Height (m)")
axes[0].set_title("Height vs Time")
axes[0].legend()
axes[0].grid(True, alpha=0.3)

# Velocity vs Time
velocities = []
v = 0.0
for t in time:
    net_force = thrust - (mass * gravity)
    a = net_force / mass
    v += a * dt
    velocities.append(v)

axes[1].plot(time, velocities, color='darkorange', linewidth=2, label='Velocity (m/s)')
axes[1].axhline(y=0, color='gray', linestyle='--', linewidth=1)
axes[1].set_xlabel("Time (s)")
axes[1].set_ylabel("Velocity (m/s)")
axes[1].set_title("Velocity vs Time")
axes[1].legend()
axes[1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("simulation_base_output.png", dpi=150, bbox_inches='tight')
print("\n  Graph saved as: simulation_base_output.png")
plt.show()
