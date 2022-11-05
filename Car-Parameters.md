# F1TENTH Car Parameters

<img src="https://github.com/Tinker-Twins/AutoDRIVE-F1TENTH-ARMLab/blob/main/AutoDRIVE-F1TENTH-ARMLab.jpg" alt="AutoDRIVE-F1TENTH-ARMLab" width="525"/>

## Kinematic Parameters
| Parameter | Value |
| :-------: | :---: |
| Car Length   | 0.50 m |
| Car Width    | 0.27 m |
| Wheelbase    | 0.33 m |
| Track Width  | 0.23 m |
| Front Offset | 0.09 m |
| Rear Offset  | 0.08 m |
| Min Turning Radius (@ Max Steering Angle) | 0.5716 m |
| Max Steering Angle | 0.523599 rad (30 deg) |
| Max Speed (Car Limit) | 8.9408 m/s (20 mph) |

#### References:
`src/autodrive/config/planner_params.yaml`

## VESC Parametrs

| Parameter | Value |
| :-------: | :---: |
| Max Steering Rate | 3.2 rad/s |

1. erpm (electrical rpm) = speed_to_erpm_gain * speed (meters / second) + speed_to_erpm_offset

speed_to_erpm_gain: 4614

speed_to_erpm_offset: 0.0

tachometer_ticks_to_meters_gain: 0.00225

2. servo smoother - limits rotation speed and smooths anything above limit

max_servo_speed: 3.2 # radians/second

servo_smoother_rate: 75.0 # messages/sec

3. servo smoother - limits acceleration and smooths anything above limit

max_acceleration: 2.5 # meters/second^2

throttle_smoother_rate: 75.0 # messages/sec

4. servo value (0 to 1) =  steering_angle_to_servo_gain * steering angle (radians) + steering_angle_to_servo_offset

steering_angle_to_servo_gain: -1.2135

steering_angle_to_servo_offset: 0.435 # 0.5304

5. publish odom to base link tf

vesc_to_odom/publish_tf: false

6. car wheelbase is about 0.33 m 

wheelbase: 0.33

7. vesc_driver:

port: /dev/sensors/vesc

duty_cycle_min: 0.0

duty_cycle_max: 0.0

current_min: 0.0

current_max: 100.0

brake_min: -20000.0

brake_max: 200000.0

speed_min: -23250

speed_max: 23250

position_min: 0.0

position_max: 0.0

servo_min: 0.15

servo_max: 0.85

`src/f1tenth/racecar/racecar/config/racecar-v2/vesc.yaml`
