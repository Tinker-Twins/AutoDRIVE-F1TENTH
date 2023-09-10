# F1TENTH Car Parameters

<img src="https://github.com/Tinker-Twins/AutoDRIVE-F1TENTH/blob/main/Media/Fall%202022/SLAM.jpg" alt="AutoDRIVE-F1TENTH-ARMLab" width="525"/>

## Kinematic Parameters
| Parameter | Value |
| :-------: | :---: |
| Car Length   | 0.50 m |
| Car Width    | 0.27 m |
| Wheelbase    | 0.324 m |
| Track Width  | 0.236 m |
| Front Offset | 0.09 m |
| Rear Offset  | 0.08 m |
| Min Turning Radius (@ Max Steering Angle) | 0.5716 m |
| Max Steering Angle (Wheel-Road Angle) | 0.523599 rad (30 deg) |
| Max Speed :warning: Physical Car Limit :warning: | 8.9408 m/s (20 mph) |

### Reference:
`src/autodrive/config/planner_params.yaml`

## VESC Parametrs

| Parameter | Value |
| :-------: | :---: |
| Max Steering Rate | 3.2 rad/s |
| Max Acceleration | 2.5 m/s<sup>2</sup> |
| Min Braking Command | -20000 |
| Max Braking Command | 200000 |
| Min Speed Command | -23250 |
| Max Speed Command | 23250 |
| Min Braking Command | -20000 |
| Max Braking Command | 200000 |
| Min Steering Command | 0.15 |
| Max Steering Command | 0.85 |

### Notes:

- #### Drive motor erpm (electrical rpm) = speed_to_erpm_gain * speed (meters / second) + speed_to_erpm_offset
  - `speed_to_erpm_gain`: 7300
  - `speed_to_erpm_offset`: 250

- #### Steer servo value (0 to 1) =  steering_angle_to_servo_gain * steering angle (radians) + steering_angle_to_servo_offset
  - `steering_angle_to_servo_gain`: -0.6
  - `steering_angle_to_servo_offset`: 0.435 (zero steer offset)

### Reference:
`src/f1tenth/racecar/racecar/config/racecar-v2/vesc.yaml`
