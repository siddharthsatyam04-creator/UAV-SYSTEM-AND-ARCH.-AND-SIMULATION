"""
UAV Altitude Simulation — Stabilized Hover (P-Controller)
===========================================================
Maincrafts Technology | UAV Internship Task 1 — Deliverable 3

Description:
    Extended simulation that implements a simple Proportional (P) controller
    to stabilize the drone at a constant target altitude.

    This satisfies the intern task requirement:
        "Try stabilizing at constant height"

    The controller adjusts thrust dynamically based on altitude error:
        error  = target_height - current_height
        thrust = hover_thrust + Kp × error

    Where hover_thrust = mass × gravity (exact thrust to counteract gravity).

Physics:
    Net Force      = thrust(t) - (mass × gravity)
    Acceleration   = Net Force / mass
    Velocity      += Acceleration × dt
    Height        += Velocity × dt

Usage:
    python uav_altitude_stabilized.py

Requirements:
    pip install numpy matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt

# ─── Drone Parameters ─────────────────────────────────────────────────────────
mass          = 1.5     # kg
gravity       = 9.81    # m/s²
hover_thrust  = mass * gravity  # 14.715 N — exact thrust to hover

# ─── Controller Parameters ────────────────────────────────────────────────────
target_height = 10.0    # m  — desired stabilization altitude
Kp            = 5.0     # Proportional gain
Kd            = 3.0     # Derivative gain (damps oscillation)

# ─── Simulation Parameters ────────────────────────────────────────────────────
dt   = 0.05             # s  — smaller timestep for better accuracy
time = np.arange(0, 20, dt)

# ─── Initial Conditions ───────────────────────────────────────────────────────
velocity = 0.0
height   = 0.0

heights    = []
velocities = []
thrusts    = []

# ─── Phase 1 + 2 Setup ────────────────────────────────────────────────────────
# Phase 1: Takeoff with full thrust (0–3 seconds)
# Phase 2: P-controller engages to stabilize at target height

takeoff_thrust = 25.0   # N — initial boost for takeoff
takeoff_end    = 3.0    # s — switch to P-controller after this

# ─── Simulation Loop ──────────────────────────────────────────────────────────
prev_error = 0.0

for t in time:
    if t < takeoff_end:
        # Phase 1: Takeoff boost
        thrust = takeoff_thrust
    else:
        # Phase 2: PD Controller
        error      = target_height - height
        d_error    = (error - prev_error) / dt
        thrust     = hover_thrust + Kp * error + Kd * d_error
        thrust     = max(0, min(thrust, 40))  # clamp thrust: 0–40 N
        prev_error = error

    net_force    = thrust - (mass * gravity)
    acceleration = net_force / mass
    velocity    += acceleration * dt
    height      += velocity * dt
    height       = max(0, height)   # can't go below ground

    heights.append(height)
    velocities.append(velocity)
    thrusts.append(thrust)

# ─── Analysis ─────────────────────────────────────────────────────────────────
final_height  = heights[-1]
steady_start  = int(15.0 / dt)     # last 5 seconds
steady_heights = heights[steady_start:]
steady_error  = abs(np.mean(steady_heights) - target_height)

print("=" * 55)
print("  UAV Altitude Simulation — Stabilized Hover")
print("=" * 55)
print(f"  Target altitude     : {target_height:.1f} m")
print(f"  Final altitude      : {final_height:.3f} m")
print(f"  Steady-state error  : {steady_error:.4f} m")
print(f"  Hover thrust        : {hover_thrust:.3f} N")
print(f"  Controller (Kp, Kd) : ({Kp}, {Kd})")
print("=" * 55)

# ─── Plot ─────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(11, 10))
fig.suptitle("UAV Altitude Simulation — Stabilized Hover (PD Controller)\n"
             f"Target: {target_height}m | mass={mass}kg | Kp={Kp}, Kd={Kd}",
             fontsize=13, fontweight='bold')

# — Plot 1: Height vs Time —
axes[0].plot(time, heights, color='royalblue', linewidth=2, label='Height (m)')
axes[0].axhline(y=target_height, color='red', linestyle='--',
                linewidth=1.5, label=f'Target = {target_height}m')
axes[0].axvline(x=takeoff_end, color='green', linestyle=':', linewidth=1.2,
                label='Controller engages')
axes[0].fill_between(time, heights, alpha=0.1, color='royalblue')
axes[0].set_ylabel("Height (m)")
axes[0].set_title("Altitude Response")
axes[0].legend(loc='upper left')
axes[0].grid(True, alpha=0.3)

# — Plot 2: Velocity vs Time —
axes[1].plot(time, velocities, color='darkorange', linewidth=2, label='Velocity (m/s)')
axes[1].axhline(y=0, color='gray', linestyle='--', linewidth=1)
axes[1].axvline(x=takeoff_end, color='green', linestyle=':', linewidth=1.2)
axes[1].set_ylabel("Velocity (m/s)")
axes[1].set_title("Velocity Response")
axes[1].legend()
axes[1].grid(True, alpha=0.3)

# — Plot 3: Thrust vs Time —
axes[2].plot(time, thrusts, color='mediumseagreen', linewidth=2, label='Applied Thrust (N)')
axes[2].axhline(y=hover_thrust, color='purple', linestyle='--', linewidth=1.5,
                label=f'Hover thrust = {hover_thrust:.2f} N')
axes[2].axvline(x=takeoff_end, color='green', linestyle=':', linewidth=1.2)
axes[2].set_xlabel("Time (s)")
axes[2].set_ylabel("Thrust (N)")
axes[2].set_title("Thrust Command vs Time")
axes[2].legend()
axes[2].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("simulation_stabilized_output.png", dpi=150, bbox_inches='tight')
print("\n  Graph saved as: simulation_stabilized_output.png")
plt.show()
