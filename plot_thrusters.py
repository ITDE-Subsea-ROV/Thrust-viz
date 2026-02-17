"""
3D interactive visualization of ArduPilot 6DoF thruster lines of action.

Parses add_motor_raw_6dof calls and computes each thruster's line of action
from the mixing matrix factors.

Signature: add_motor_raw_6dof(motor_num, roll_fac, pitch_fac, yaw_fac,
                               climb_fac, forward_fac, lateral_fac, testing_order)

Translational factors (forward, lateral, climb) → thrust direction.
Rotational factors (roll, pitch, yaw) → torque = position × thrust direction.
Line of action point closest to origin: r = (d × m) / |d|²
"""

import re
import numpy as np
import plotly.graph_objects as go

# ---------------------------------------------------------------------------
# Parse the CSV / source file
# ---------------------------------------------------------------------------
motors = []
with open("thrust-sample.csv") as f:
    for line in f:
        m = re.search(
            r"add_motor_raw_6dof\(\s*\w+\s*,"  # motor enum
            r"\s*([-\d.f]+)\s*,"                 # roll
            r"\s*([-\d.f]+)\s*,"                 # pitch
            r"\s*([-\d.f]+)\s*,"                 # yaw
            r"\s*([-\d.f]+)\s*,"                 # climb
            r"\s*([-\d.f]+)\s*,"                 # forward
            r"\s*([-\d.f]+)\s*,"                 # lateral
            r"\s*(\d+)",                          # testing_order
            line,
        )
        if not m:
            continue

        def f2v(s):
            return float(s.replace("f", ""))

        roll, pitch, yaw = f2v(m.group(1)), f2v(m.group(2)), f2v(m.group(3))
        climb, fwd, lat  = f2v(m.group(4)), f2v(m.group(5)), f2v(m.group(6))
        order             = int(m.group(7))

        motors.append(dict(
            id=order, roll=roll, pitch=pitch, yaw=yaw,
            climb=climb, forward=fwd, lateral=lat,
        ))

    # Also grab comment labels (line before each call)
    f.seek(0)
    lines = f.readlines()
    motor_idx = 0
    for i, line in enumerate(lines):
        if "add_motor_raw_6dof" in line and motor_idx < len(motors):
            # look for comment on previous line
            if i > 0 and "//" in lines[i - 1]:
                label = lines[i - 1].strip().lstrip("/ ").strip()
                motors[motor_idx]["label"] = label
            motor_idx += 1

for mot in motors:
    mot.setdefault("label", f"Motor {mot['id']}")

print(f"Parsed {len(motors)} motors:")
for mot in motors:
    print(f"  Motor {mot['id']:>2} ({mot['label']:>15}): "
          f"d=({mot['forward']:+.1f}, {mot['lateral']:+.1f}, {mot['climb']:+.1f})  "
          f"m=({mot['roll']:+.1f}, {mot['pitch']:+.1f}, {mot['yaw']:+.1f})")

# ---------------------------------------------------------------------------
# Compute lines of action
#
# Body frame (NED-like): X = forward, Y = right, Z = down
#   thrust direction  d = (forward_fac,  lateral_fac,  -climb_fac)
#                         climb>0 means "push up" = force in -Z_NED
#   torque factors    m = (roll_fac, -pitch_fac, -yaw_fac)
#                         pitch>0 = nose-up = -My_NED,  yaw>0 = nose-right = -Mz_NED
#
# Display frame: X = forward, Y = right, Z = UP  (flip Z for readability)
# ---------------------------------------------------------------------------

LINE_HALF_LEN = 2.0  # how far to extend each line of action from the base point

traces = []
annotations = []

# Colour palette for motors
colors = [
    "#e6194b", "#3cb44b", "#4363d8", "#f58231",
    "#911eb4", "#42d4f4", "#f032e6", "#bfef45",
]

for i, mot in enumerate(motors):
    # --- Thrust direction in NED body frame ---
    d_ned = np.array([mot["forward"], mot["lateral"], -mot["climb"]])

    # --- Torque in NED body frame ---
    m_ned = np.array([mot["roll"], -mot["pitch"], -mot["yaw"]])

    d_norm_sq = np.dot(d_ned, d_ned)
    if d_norm_sq < 1e-9:
        print(f"  Motor {mot['id']}: zero thrust direction, skipping")
        continue

    # Closest point on line of action to origin
    r_ned = np.cross(d_ned, m_ned) / d_norm_sq

    # Convert NED → display (X fwd, Y right, Z up): flip Z
    r_disp = np.array([r_ned[0], r_ned[1], -r_ned[2]])
    d_disp = np.array([d_ned[0], d_ned[1], -d_ned[2]])

    # Normalize direction for consistent line length
    d_hat = d_disp / np.linalg.norm(d_disp)

    # Line of action endpoints
    p0 = r_disp - LINE_HALF_LEN * d_hat
    p1 = r_disp + LINE_HALF_LEN * d_hat

    col = colors[i % len(colors)]

    # Line
    traces.append(go.Scatter3d(
        x=[p0[0], p1[0]], y=[p0[1], p1[1]], z=[p0[2], p1[2]],
        mode="lines",
        line=dict(color=col, width=5),
        name=f"M{mot['id']} {mot['label']}",
        legendgroup=f"m{mot['id']}",
        hovertext=f"Motor {mot['id']}: {mot['label']}<br>"
                  f"dir=({d_disp[0]:+.2f},{d_disp[1]:+.2f},{d_disp[2]:+.2f})<br>"
                  f"pt=({r_disp[0]:+.2f},{r_disp[1]:+.2f},{r_disp[2]:+.2f})",
        hoverinfo="text",
    ))

    # Base point marker
    traces.append(go.Scatter3d(
        x=[r_disp[0]], y=[r_disp[1]], z=[r_disp[2]],
        mode="markers+text",
        marker=dict(size=6, color=col, symbol="diamond"),
        text=[f"M{mot['id']}"],
        textposition="top center",
        textfont=dict(size=10, color=col),
        showlegend=False,
        legendgroup=f"m{mot['id']}",
        hovertext=f"Motor {mot['id']} base point<br>"
                  f"({r_disp[0]:+.3f}, {r_disp[1]:+.3f}, {r_disp[2]:+.3f})",
        hoverinfo="text",
    ))

    # Arrowhead cone at the positive-thrust end
    traces.append(go.Cone(
        x=[r_disp[0] + 0.6 * d_hat[0]],
        y=[r_disp[1] + 0.6 * d_hat[1]],
        z=[r_disp[2] + 0.6 * d_hat[2]],
        u=[d_hat[0]], v=[d_hat[1]], w=[d_hat[2]],
        sizemode="absolute", sizeref=0.25,
        colorscale=[[0, col], [1, col]],
        showscale=False, showlegend=False,
        legendgroup=f"m{mot['id']}",
        hoverinfo="skip",
    ))

    print(f"  Motor {mot['id']:>2}: base=({r_disp[0]:+.3f}, {r_disp[1]:+.3f}, {r_disp[2]:+.3f})  "
          f"dir=({d_hat[0]:+.3f}, {d_hat[1]:+.3f}, {d_hat[2]:+.3f})")

# ---------------------------------------------------------------------------
# Reference frame axes (at origin)
# ---------------------------------------------------------------------------
axis_len = 0.5
for axis, label, color in [
    ([axis_len, 0, 0], "X (fwd)", "red"),
    ([0, axis_len, 0], "Y (right)", "green"),
    ([0, 0, axis_len], "Z (up)", "blue"),
]:
    traces.append(go.Scatter3d(
        x=[0, axis[0]], y=[0, axis[1]], z=[0, axis[2]],
        mode="lines+text",
        line=dict(color=color, width=3),
        text=["", label],
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

# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------
fig = go.Figure(data=traces)
fig.update_layout(
    title="Thruster Lines of Action (ArduPilot 6DoF)",
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
