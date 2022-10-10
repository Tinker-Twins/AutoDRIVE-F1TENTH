#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

################################################################################

DRIVE_LIMIT = 1
STEER_LIMIT = 1
THROTTLE_CONST = 4
STEERING_CONST = 1

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input
    return input

def boundSteer(steer_cmd):
    steer_cmd = constrain(steer_cmd, -STEER_LIMIT, STEER_LIMIT)
    return steer_cmd

def boundDrive(drive_cmd):
    drive_cmd = constrain(drive_cmd, -DRIVE_LIMIT, DRIVE_LIMIT)
    return drive_cmd

################################################################################

control_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
control_msg = AckermannDriveStamped()

def navCtrlCallback(data):
    global throttle, steering
    throttle = boundDrive(THROTTLE_CONST * data.linear.x)
    steering = boundSteer(STEERING_CONST * data.angular.z)

    control_msg.header.stamp = rospy.Time.now()
    control_msg.header.frame_id = 'base_link'
    control_msg.drive.speed = throttle
    control_msg.drive.steering_angle = steering
    control_pub.publish(control_msg)

################################################################################

if __name__=="__main__":

    rospy.init_node('nav_ctrl')
    rospy.Subscriber("cmd_vel", Twist, navCtrlCallback)
    rospy.spin()
