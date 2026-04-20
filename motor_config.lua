-- Auto-generated motor configuration for ArduSub 6DoF custom frame.
-- Source: thrust-sample.csv
-- Drop this file into APM/scripts/ on the Navigator / Raspberry Pi.
-- Requires FRAME_CONFIG = 0 (scripted/custom) in ArduSub parameters.
--
-- Motor index mapping (0-based Lua -> Navigator PWM output):
--   motors[0]  ->  PWM output 1  (port-top)
--   motors[1]  ->  PWM output 2  (stbd-top)
--   motors[2]  ->  PWM output 3  (port-bottom)
--   motors[3]  ->  PWM output 4  (stbd-bottom)
--   motors[4]  ->  PWM output 5  (front-top)
--   motors[5]  ->  PWM output 6  (back-top)
--   motors[6]  ->  PWM output 7  (front-bottom)
--   motors[7]  ->  PWM output 8  (back-bottom)

local function init()
  motors:add_motor_raw_6dof(0,  -1.0000,  -0.0000,   0.0000,  -1.0000,   0.0000,  -0.5176, 1)  -- port-top
  motors:add_motor_raw_6dof(1,   1.0000,  -0.0000,   0.0000,  -1.0000,   0.0000,   0.5176, 2)  -- stbd-top
  motors:add_motor_raw_6dof(2,   1.0000,   0.0000,   0.0000,   1.0000,   0.0000,  -0.5176, 3)  -- port-bottom
  motors:add_motor_raw_6dof(3,  -1.0000,   0.0000,   0.0000,   1.0000,   0.0000,   0.5176, 4)  -- stbd-bottom
  motors:add_motor_raw_6dof(4,   0.5207,   1.0000,   1.0000,   0.0000,  -1.0000,   1.0000, 5)  -- front-top
  motors:add_motor_raw_6dof(5,   0.5207,  -1.0000,  -1.0000,   0.0000,   1.0000,   1.0000, 6)  -- back-top
  motors:add_motor_raw_6dof(6,  -0.5207,  -1.0000,   1.0000,   0.0000,  -1.0000,   1.0000, 7)  -- front-bottom
  motors:add_motor_raw_6dof(7,  -0.5207,   1.0000,  -1.0000,   0.0000,   1.0000,   1.0000, 8)  -- back-bottom
end

local function update()
  return update, 1000
end

init()
return update, 1000
