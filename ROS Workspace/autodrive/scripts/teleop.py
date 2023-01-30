#!/usr/bin/env python

'''
BSD 2-Clause License

Copyright (c) 2022, Tinker Twins
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

################################################################################

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

################################################################################

DRIVE_LIMIT = 5 # 3
STEER_LIMIT = 0.52
DRIVE_STEP_SIZE = 1
STEER_STEP_SIZE = 0.104 # 0.52

info = """
-------------------------------------
AutoDRIVE-F1TENTH Teleoperation Panel
Developers - Chinmay and Tanmay Samak
-------------------------------------
             Q   W   E
             A   S   D
                 X
W/S : Increase/decrease drive command
D/A : Increase/decrease steer command
Q   : Zero steer
E   : Emergency brake
X   : Force stop and reset
Press CTRL+C to quit
NOTE: Press keys within this terminal
-------------------------------------
"""

error = """
ERROR: Communication failed!
"""

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

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

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('key_teleop')
    key_teleop_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
    key_teleop_msg = AckermannDriveStamped()

    throttle = 0.0
    steering = 0.0

    try:
        print(info)

        while(1):
            key = getKey()
            if key == 'w' :
                throttle = boundDrive(throttle + DRIVE_STEP_SIZE)
            elif key == 's' :
                throttle = boundDrive(throttle - DRIVE_STEP_SIZE)
            elif key == 'a' :
                steering = boundSteer(steering + STEER_STEP_SIZE)
            elif key == 'd' :
                steering = boundSteer(steering - STEER_STEP_SIZE)
            elif key == 'q' :
                steering = 0.0
            elif key == 'e' :
                throttle = 0.0
            elif key == 'x' :
                throttle = 0.0
                steering = 0.0
            else:
                if (key == '\x03'): # CTRL+C
                    break

            key_teleop_msg.header.stamp = rospy.Time.now()
            key_teleop_msg.header.frame_id = 'base_link'
            key_teleop_msg.drive.speed = throttle
            key_teleop_msg.drive.steering_angle = steering
            key_teleop_pub.publish(key_teleop_msg)

    except:
        print(error)

    finally:
        throttle = 0.0
        steering = 0.0
        key_teleop_msg.header.stamp = rospy.Time.now()
        key_teleop_msg.header.frame_id = 'base_link'
        key_teleop_msg.drive.speed = throttle
        key_teleop_msg.drive.steering_angle = steering
        key_teleop_pub.publish(key_teleop_msg)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
