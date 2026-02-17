"""
Generate ArduPilot add_motor_raw_6dof mixing factors for an 8-thruster vehicle
and visualize thrust lines of action in 3D.

Thruster layout:
  Motors 1-4 (YZ plane, x=0): canted 30 deg from Z axis, inward toward center
  Motors 5-8 (XZ plane, y=0): aimed toward origin in XZ, canted 15 deg toward Y
"""

import math
import numpy as np
import plotly.graph_objects as go

# ============================================================================
# Thruster geometry
# ============================================================================

# YZ motors (1-4): positions at x=0, thrust 30 deg from Z axis inward
YZ_ANGLE_DEG = 30  # angle from Z axis
YZ_POSITIONS = {
    1: (0.0, -9.865, +8.117),  # top left (port-top)
    2: (0.0, +9.865, +8.117),  # top right (stbd-top)
    3: (0.0, -9.865, -8.117),  # bottom left (port-bottom)
    4: (0.0, +9.865, -8.117),  # bottom right (stbd-bottom)
}

# XZ motors (5-8): positions at y=0, primary thrust toward origin in XZ, 15 deg Y cant
XZ_CANT_DEG = 15  # cant angle toward Y
XZ_POSITIONS = {
    5: (+8.9, 0.0, +6.793),   # front top
    6: (-8.9, 0.0, +6.793),   # back top
    7: (+8.9, 0.0, -6.793),   # front bottom
    8: (-8.9, 0.0, -6.793),   # back bottom
}

MOTOR_LABELS = {
    1: "port-top",
    2: "stbd-top",
    3: "port-bottom",
    4: "stbd-bottom",
    5: "front-top",
    6: "back-top",
    7: "front-bottom",
    8: "back-bottom",
}

# ============================================================================
# Compute thrust directions
# ============================================================================

def compute_yz_direction(pos):
    """YZ motor: thrust 30 deg from Z axis, outward in Y, toward XY plane in Z."""
    _, y, z = pos
    angle = math.radians(YZ_ANGLE_DEG)
    dx = 0.0
    dy = math.copysign(1, y) * math.sin(angle)
    dz = -math.copysign(1, z) * math.cos(angle)
    return np.array([dx, dy, dz])


def compute_xz_direction(pos):
    """XZ motor: faces +Y, rotated 15 deg about Z axis toward origin."""
    x, _, z = pos
    angle = math.radians(XZ_CANT_DEG)
    # Rotate (0, 1, 0) about Z by sign(x)*15 deg toward origin
    dx = -math.copysign(1, x) * math.sin(angle)
    dy = math.cos(angle)
    dz = 0.0
    return np.array([dx, dy, dz])


# Build motor data: position, direction, factors
motors = {}

for mid, pos in YZ_POSITIONS.items():
    d = compute_yz_direction(pos)
    motors[mid] = {"pos": np.array(pos), "dir": d}

for mid, pos in XZ_POSITIONS.items():
    d = compute_xz_direction(pos)
    motors[mid] = {"pos": np.array(pos), "dir": d}

# ============================================================================
# Compute ArduPilot mixing factors (FRU frame: X=fwd, Y=right, Z=up)
#
#   torque = pos x thrust_dir
#   forward_fac = t_x,  lateral_fac = t_y,  climb_fac = t_z
#   roll_fac = -tau_x,  pitch_fac = -tau_y,  yaw_fac = tau_z
# ============================================================================

factor_names = ["roll", "pitch", "yaw", "climb", "forward", "lateral"]

for mid in sorted(motors):
    m = motors[mid]
    t = m["dir"]
    r = m["pos"]
    tau = np.cross(r, t)

    m["factors"] = {
        "forward": t[0],
        "lateral": t[1],
        "climb":   t[2],
        "roll":   -tau[0],
        "pitch":  -tau[1],
        "yaw":     tau[2],
    }

# Normalize: max |value| per DoF column = 1.0
for fname in factor_names:
    max_abs = max(abs(motors[mid]["factors"][fname]) for mid in motors)
    if max_abs > 1e-9:
        for mid in motors:
            motors[mid]["factors"][fname] /= max_abs

# ============================================================================
# Write thrust-sample.csv
# ============================================================================

MOTOR_ENUMS = {
    1: "AP_MOTORS_MOT_1", 2: "AP_MOTORS_MOT_2",
    3: "AP_MOTORS_MOT_3", 4: "AP_MOTORS_MOT_4",
    5: "AP_MOTORS_MOT_5", 6: "AP_MOTORS_MOT_6",
    7: "AP_MOTORS_MOT_7", 8: "AP_MOTORS_MOT_8",
}

csv_path = "thrust-sample.csv"
with open(csv_path, "w") as f:
    f.write("\n//// YZ motors\n")
    for mid in [1, 2, 3, 4]:
        fac = motors[mid]["factors"]
        f.write(f"// {MOTOR_LABELS[mid]}\n")
        f.write(
            f"add_motor_raw_6dof({MOTOR_ENUMS[mid]}, "
            f"{fac['roll']:.4f}f, "
            f"{fac['pitch']:.4f}f, "
            f"{fac['yaw']:.4f}f, "
            f"{fac['climb']:.4f}f, "
            f"{fac['forward']:.4f}f, "
            f"{fac['lateral']:.4f}f, "
            f"{mid});\n"
        )

    f.write("\n//// XZ motors\n")
    for mid in [5, 6, 7, 8]:
        fac = motors[mid]["factors"]
        f.write(f"// {MOTOR_LABELS[mid]}\n")
        f.write(
            f"add_motor_raw_6dof({MOTOR_ENUMS[mid]}, "
            f"{fac['roll']:.4f}f, "
            f"{fac['pitch']:.4f}f, "
            f"{fac['yaw']:.4f}f, "
            f"{fac['climb']:.4f}f, "
            f"{fac['forward']:.4f}f, "
            f"{fac['lateral']:.4f}f, "
            f"{mid});\n"
        )

print(f"Wrote {csv_path}")
print()

# Print factor table
header = f"{'Motor':>12}  {'roll':>8}  {'pitch':>8}  {'yaw':>8}  {'climb':>8}  {'fwd':>8}  {'lat':>8}"
print(header)
print("-" * len(header))
for mid in sorted(motors):
    fac = motors[mid]["factors"]
    print(f"{MOTOR_LABELS[mid]:>12}  {fac['roll']:+8.4f}  {fac['pitch']:+8.4f}  "
          f"{fac['yaw']:+8.4f}  {fac['climb']:+8.4f}  {fac['forward']:+8.4f}  {fac['lateral']:+8.4f}")

# ============================================================================
# 3D Plotly visualization
# ============================================================================

ARROW_LEN = 5.0  # length of thrust direction arrows

traces = []
colors = [
    "#e6194b", "#3cb44b", "#4363d8", "#f58231",
    "#911eb4", "#42d4f4", "#f032e6", "#bfef45",
]

for i, mid in enumerate(sorted(motors)):
    m = motors[mid]
    pos = m["pos"]
    d_hat = m["dir"] / np.linalg.norm(m["dir"])
    fac = m["factors"]
    col = colors[i % len(colors)]
    label = MOTOR_LABELS[mid]

    # Arrow: from position along thrust direction
    tip = pos + ARROW_LEN * d_hat

    hover = (
        f"Motor {mid}: {label}<br>"
        f"pos=({pos[0]:+.1f}, {pos[1]:+.1f}, {pos[2]:+.1f})<br>"
        f"dir=({d_hat[0]:+.3f}, {d_hat[1]:+.3f}, {d_hat[2]:+.3f})<br>"
        f"roll={fac['roll']:+.4f} pitch={fac['pitch']:+.4f} yaw={fac['yaw']:+.4f}<br>"
        f"climb={fac['climb']:+.4f} fwd={fac['forward']:+.4f} lat={fac['lateral']:+.4f}"
    )

    # Line of action
    traces.append(go.Scatter3d(
        x=[pos[0], tip[0]], y=[pos[1], tip[1]], z=[pos[2], tip[2]],
        mode="lines",
        line=dict(color=col, width=5),
        name=f"M{mid} {label}",
        legendgroup=f"m{mid}",
        hovertext=hover,
        hoverinfo="text",
    ))

    # Motor position marker
    traces.append(go.Scatter3d(
        x=[pos[0]], y=[pos[1]], z=[pos[2]],
        mode="markers+text",
        marker=dict(size=6, color=col, symbol="diamond"),
        text=[f"M{mid}"],
        textposition="top center",
        textfont=dict(size=10, color=col),
        showlegend=False,
        legendgroup=f"m{mid}",
        hovertext=hover,
        hoverinfo="text",
    ))

    # Arrowhead cone
    cone_pos = pos + 0.7 * ARROW_LEN * d_hat
    traces.append(go.Cone(
        x=[cone_pos[0]], y=[cone_pos[1]], z=[cone_pos[2]],
        u=[d_hat[0]], v=[d_hat[1]], w=[d_hat[2]],
        sizemode="absolute", sizeref=0.8,
        colorscale=[[0, col], [1, col]],
        showscale=False, showlegend=False,
        legendgroup=f"m{mid}",
        hoverinfo="skip",
    ))

# Reference frame axes at origin
axis_len = 3.0
for axis, ax_label, color in [
    ([axis_len, 0, 0], "X (fwd)", "red"),
    ([0, axis_len, 0], "Y (right)", "green"),
    ([0, 0, axis_len], "Z (up)", "blue"),
]:
    traces.append(go.Scatter3d(
        x=[0, axis[0]], y=[0, axis[1]], z=[0, axis[2]],
        mode="lines+text",
        line=dict(color=color, width=3),
        text=["", ax_label],
        textposition="top center",
        textfont=dict(size=9, color=color),
        showlegend=False,
        hoverinfo="skip",
    ))

# Origin marker
traces.append(go.Scatter3d(
    x=[0], y=[0], z=[0],
    mode="markers",
    marker=dict(size=5, color="black"),
    name="Origin (CoM)",
    hoverinfo="name",
))

fig = go.Figure(data=traces)
fig.update_layout(
    title="Thruster Positions & Thrust Directions (FRU frame)",
    scene=dict(
        xaxis_title="X  (forward →)",
        yaxis_title="Y  (right →)",
        zaxis_title="Z  (up →)",
        aspectmode="data",
        camera=dict(eye=dict(x=1.8, y=1.8, z=1.2)),
    ),
    legend=dict(x=0.01, y=0.99, bgcolor="rgba(255,255,255,0.8)"),
    margin=dict(l=0, r=0, t=40, b=0),
)

fig.show()
