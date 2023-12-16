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
import numpy as np

################################################################################

if __name__=="__main__":
    
    rospy.init_node('open_loop_ctrl')

    velocity = rospy.get_param('/open_loop_ctrl/velocity')
    steering = rospy.get_param('/open_loop_ctrl/steering')
    velocity_noise = rospy.get_param('/open_loop_ctrl/velocity_noise')
    steering_noise = rospy.get_param('/open_loop_ctrl/steering_noise')

    ol_ctrl_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
    ol_ctrl_msg = AckermannDriveStamped()

    rate = rospy.Rate(10) # Control frequency (Hz)

    try:
        while not rospy.is_shutdown():
################################################################################
            # Sample test (constant control inputs)
            '''
            velocity_cmd = 1.0
            steering_cmd = 0.523599
            ol_ctrl_msg.header.stamp = rospy.Time.now()
            ol_ctrl_msg.header.frame_id = 'base_link'
            ol_ctrl_msg.drive.speed = velocity_cmd
            ol_ctrl_msg.drive.steering_angle = steering_cmd
            #print("Lin Vel : {:.4} m/s".format(ol_ctrl_msg.drive.speed))
            #print("Ang Vel : {:.4} rad/s".format(ol_ctrl_msg.drive.steering_angle))
            ol_ctrl_pub.publish(ol_ctrl_msg)
            '''
################################################################################
            # Skidpad test (constant control inputs with Gaussian noise - sampled from normal distribution)
            # Reference Example: noise = np.random.normal(0,1) 0 mean and 1 std dev
            #velocity = 1.5 + np.random.normal(0,0.1)
            #steering = 0.4 + np.random.normal(0,0.2)
            #'''
            # Skidpad
	    t_delay = rospy.Duration.from_sec(20).to_sec()
	    t_start = rospy.Time.now().to_sec()
	    while (rospy.Time.now().to_sec() - t_start) <= t_delay:
	        velocity_cmd = velocity + np.random.normal(0,velocity_noise)
	        steering_cmd = steering + np.random.normal(0,steering_noise)
	        ol_ctrl_msg.header.stamp = rospy.Time.now()
	        ol_ctrl_msg.header.frame_id = 'base_link'
	        ol_ctrl_msg.drive.speed = velocity_cmd
	        ol_ctrl_msg.drive.steering_angle = steering_cmd
	        #print("Lin Vel : {:.4} m/s".format(ol_ctrl_msg.drive.speed))
	        #print("Ang Vel : {:.4} rad/s".format(ol_ctrl_msg.drive.steering_angle))
	        ol_ctrl_pub.publish(ol_ctrl_msg)
	    # Stop
	    velocity_cmd = 0
            steering_cmd = 0
            ol_ctrl_msg.header.stamp = rospy.Time.now()
            ol_ctrl_msg.header.frame_id = 'base_link'
            ol_ctrl_msg.drive.speed = velocity_cmd
            ol_ctrl_msg.drive.steering_angle = steering_cmd
            #print("Lin Vel : {:.4} m/s".format(ol_ctrl_msg.drive.speed))
            #print("Ang Vel : {:.4} rad/s".format(ol_ctrl_msg.drive.steering_angle))
            ol_ctrl_pub.publish(ol_ctrl_msg)

            rospy.signal_shutdown('Skidpad test completed!')
            #'''
################################################################################
            # Slalom test (constant linear velocity and time-delayed varying steering with Gaussian noise - sampled from normal distribution)
            # Reference Example: noise = np.random.normal(0,1) 0 mean and 1 std dev
            #velocity = 1.5 + np.random.normal(0,0.1)
            #steering = 0.4 + np.random.normal(0,0.2)
            '''
            # Straight
            t_delay = rospy.Duration.from_sec(1.5/velocity).to_sec()
            t_start = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec() - t_start) <= t_delay:
                velocity_cmd = velocity + np.random.normal(0,velocity_noise)
                steering_cmd = 0
                ol_ctrl_msg.header.stamp = rospy.Time.now()
                ol_ctrl_msg.header.frame_id = 'base_link'
                ol_ctrl_msg.drive.speed = velocity_cmd
                ol_ctrl_msg.drive.steering_angle = steering_cmd
                #print("Lin Vel : {:.4} m/s".format(ol_ctrl_msg.drive.speed))
                #print("Ang Vel : {:.4} rad/s".format(ol_ctrl_msg.drive.steering_angle))
                ol_ctrl_pub.publish(ol_ctrl_msg)
            # Slalom
            for i in range(3):
		    # Left
		    t_delay = rospy.Duration.from_sec(0.6/velocity).to_sec()
		    t_start = rospy.Time.now().to_sec()
		    while (rospy.Time.now().to_sec() - t_start) <= t_delay:
		        velocity_cmd = velocity + np.random.normal(0,velocity_noise)
		        steering_cmd = steering + np.random.normal(0,steering_noise)
		        ol_ctrl_msg.header.stamp = rospy.Time.now()
		        ol_ctrl_msg.header.frame_id = 'base_link'
		        ol_ctrl_msg.drive.speed = velocity_cmd
		        ol_ctrl_msg.drive.steering_angle = steering_cmd
		        #print("Lin Vel : {:.4} m/s".format(ol_ctrl_msg.drive.speed))
		        #print("Ang Vel : {:.4} rad/s".format(ol_ctrl_msg.drive.steering_angle))
		        ol_ctrl_pub.publish(ol_ctrl_msg)
		    # Right
		    t_delay = rospy.Duration.from_sec(0.6/velocity).to_sec()
		    t_start = rospy.Time.now().to_sec()
		    while (rospy.Time.now().to_sec() - t_start) <= t_delay:
		        velocity_cmd = velocity + np.random.normal(0,velocity_noise)
		        steering_cmd = -steering - np.random.normal(0,steering_noise)
		        ol_ctrl_msg.header.stamp = rospy.Time.now()
		        ol_ctrl_msg.header.frame_id = 'base_link'
		        ol_ctrl_msg.drive.speed = velocity_cmd
		        ol_ctrl_msg.drive.steering_angle = steering_cmd
		        #print("Lin Vel : {:.4} m/s".format(ol_ctrl_msg.drive.speed))
		        #print("Ang Vel : {:.4} rad/s".format(ol_ctrl_msg.drive.steering_angle))
		        ol_ctrl_pub.publish(ol_ctrl_msg)
	    # Stop
	    velocity_cmd = 0
            steering_cmd = 0
            ol_ctrl_msg.header.stamp = rospy.Time.now()
            ol_ctrl_msg.header.frame_id = 'base_link'
            ol_ctrl_msg.drive.speed = velocity_cmd
            ol_ctrl_msg.drive.steering_angle = steering_cmd
            #print("Lin Vel : {:.4} m/s".format(ol_ctrl_msg.drive.speed))
            #print("Ang Vel : {:.4} rad/s".format(ol_ctrl_msg.drive.steering_angle))
            ol_ctrl_pub.publish(ol_ctrl_msg)

            rospy.signal_shutdown('Slalom test completed!')
            '''
################################################################################
            # Fishhook test (constant linear velocity and time-delayed varying steering with Gaussian noise - sampled from normal distribution)
            # Reference Example: noise = np.random.normal(0,1) 0 mean and 1 std dev
            #velocity = 1.5 + np.random.normal(0,0.1)
            #steering = 0.4 + np.random.normal(0,0.2)
            # Fishhook
            '''
            for delta in [0.312,0.416,0.52]:
                #print(delta)
            	turn_rad = 0.33/np.tan(delta)
	    	t_delay = rospy.Duration.from_sec(10).to_sec()
	    	t_start = rospy.Time.now().to_sec()
	   	while (rospy.Time.now().to_sec() - t_start) <= t_delay:
	     	    velocity_cmd = velocity + np.random.normal(0,velocity_noise)
	    	    steering_cmd = delta + np.random.normal(0,steering_noise)
	     	    ol_ctrl_msg.header.stamp = rospy.Time.now()
	      	    ol_ctrl_msg.header.frame_id = 'base_link'
	      	    ol_ctrl_msg.drive.speed = velocity_cmd
	      	    ol_ctrl_msg.drive.steering_angle = steering_cmd
	      	    #print("Lin Vel : {:.4} m/s".format(ol_ctrl_msg.drive.speed))
	      	    #print("Ang Vel : {:.4} rad/s".format(ol_ctrl_msg.drive.steering_angle))
	      	    ol_ctrl_pub.publish(ol_ctrl_msg)
	    # Stop
	    velocity_cmd = 0
            steering_cmd = 0
            ol_ctrl_msg.header.stamp = rospy.Time.now()
            ol_ctrl_msg.header.frame_id = 'base_link'
            ol_ctrl_msg.drive.speed = velocity_cmd
            ol_ctrl_msg.drive.steering_angle = steering_cmd
            #print("Lin Vel : {:.4} m/s".format(ol_ctrl_msg.drive.speed))
            #print("Ang Vel : {:.4} rad/s".format(ol_ctrl_msg.drive.steering_angle))
            ol_ctrl_pub.publish(ol_ctrl_msg)

            rospy.signal_shutdown('Fishhook test completed!')
            '''
################################################################################
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
