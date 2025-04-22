# Serial Robot Control Arduino

This code is designed to control the motors os a differential drive robot that uses Sabertooth 2X12 Motor Driver.

## Serial Commands
- Set speed for left and right motor in rad/s:
  - `m left_speed right_speed`
- Get encoder counts:
  - `e` -> Returns encoders counts for left and right motor (response format: count_left count_right)
- Set PID Values:
  - `p Kp Ki Kd`
- Reset encoder counts:
  - `r` -> Resets encoder counts
