"""
Generate an ArduPilot Lua motor-configuration script from the thrust CSV.

Reads the same add_motor_raw_6dof() calls used by plot_thrusters.py and
emits a .lua file ready to drop into APM/scripts/ on the Navigator/Pi.

ArduPilot Lua uses 0-based motor indices; AP_MOTORS_MOT_N → N-1.
"""

import re
import sys
from pathlib import Path

CSV_FILE = "thrust-sample.csv"
LUA_FILE = "motor_config.lua"

# ---------------------------------------------------------------------------
# Parse
# ---------------------------------------------------------------------------
motors = []
motor_re = re.compile(
    r"add_motor_raw_6dof\(\s*AP_MOTORS_MOT_(\d+)\s*,"
    r"\s*([-\d.f]+)\s*,"   # roll
    r"\s*([-\d.f]+)\s*,"   # pitch
    r"\s*([-\d.f]+)\s*,"   # yaw
    r"\s*([-\d.f]+)\s*,"   # climb
    r"\s*([-\d.f]+)\s*,"   # forward
    r"\s*([-\d.f]+)\s*,"   # lateral
    r"\s*(\d+)"             # testing_order
)

def fv(s):
    return float(s.replace("f", ""))

src = Path(CSV_FILE).read_text()
lines = src.splitlines()

for i, line in enumerate(lines):
    m = motor_re.search(line)
    if not m:
        continue
    mot_num = int(m.group(1))          # 1-based from CSV
    label = ""
    if i > 0 and "//" in lines[i - 1]:
        label = lines[i - 1].strip().lstrip("/ ").strip()

    motors.append(dict(
        mot_num=mot_num,               # 1-based (CSV)
        lua_idx=mot_num - 1,           # 0-based (Lua)
        roll=fv(m.group(2)),
        pitch=fv(m.group(3)),
        yaw=fv(m.group(4)),
        climb=fv(m.group(5)),
        forward=fv(m.group(6)),
        lateral=fv(m.group(7)),
        order=int(m.group(8)),
        label=label or f"Motor {mot_num}",
    ))

if not motors:
    print(f"No motors found in {CSV_FILE}", file=sys.stderr)
    sys.exit(1)

print(f"Parsed {len(motors)} motors from {CSV_FILE}")

# ---------------------------------------------------------------------------
# Emit Lua
# ---------------------------------------------------------------------------
call_lines = []
for mot in sorted(motors, key=lambda m: m["mot_num"]):
    comment = f"  -- {mot['label']}" if mot["label"] else ""
    call_lines.append(
        f"  motors:add_motor_raw_6dof("
        f"{mot['lua_idx']}, "
        f"{mot['roll']:>8.4f}, {mot['pitch']:>8.4f}, {mot['yaw']:>8.4f}, "
        f"{mot['climb']:>8.4f}, {mot['forward']:>8.4f}, {mot['lateral']:>8.4f}, "
        f"{mot['order']}"
        f"){comment}"
    )

mapping_lines = "\n".join(
    f"--   motors[{m['lua_idx']}]  ->  PWM output {m['mot_num']}  ({m['label']})"
    for m in sorted(motors, key=lambda x: x["mot_num"])
)

lua = (
    "-- Auto-generated motor configuration for ArduSub 6DoF custom frame.\n"
    f"-- Source: {CSV_FILE}\n"
    "-- Drop this file into APM/scripts/ on the Navigator / Raspberry Pi.\n"
    "-- Requires FRAME_CONFIG = 0 (scripted/custom) in ArduSub parameters.\n"
    "--\n"
    "-- Motor index mapping (0-based Lua -> Navigator PWM output):\n"
    f"{mapping_lines}\n"
    "\n"
    "local function init()\n"
    + "\n".join(call_lines) + "\n"
    "end\n"
    "\n"
    "local function update()\n"
    "  return update, 1000\n"
    "end\n"
    "\n"
    "init()\n"
    "return update, 1000\n"
)

Path(LUA_FILE).write_text(lua)
print(f"Written: {LUA_FILE}")
