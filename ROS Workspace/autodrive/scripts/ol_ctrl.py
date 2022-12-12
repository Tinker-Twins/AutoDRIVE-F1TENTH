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

    lin_vel = rospy.get_param('/open_loop_ctrl/lin_vel')
    ang_vel = rospy.get_param('/open_loop_ctrl/ang_vel')
    lin_noise = rospy.get_param('/open_loop_ctrl/lin_noise')
    ang_noise = rospy.get_param('/open_loop_ctrl/ang_noise')

    ol_ctrl_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
    ol_ctrl_msg = AckermannDriveStamped()

    rate = rospy.Rate(10) # Control frequency (Hz)

    try:
        while not rospy.is_shutdown():
################################################################################
            # Sample test (constant control inputs)
            '''
            lin_vel_cmd = 1.0
            ang_vel_cmd = 0.523599
            ol_ctrl_msg.header.stamp = rospy.Time.now()
            ol_ctrl_msg.header.frame_id = 'base_link'
            ol_ctrl_msg.drive.speed = lin_vel_cmd
            ol_ctrl_msg.drive.steering_angle = ang_vel_cmd
            #print("Lin Vel : {:.4} m/s".format(ol_ctrl_msg.drive.speed))
            #print("Ang Vel : {:.4} rad/s".format(ol_ctrl_msg.drive.steering_angle))
            ol_ctrl_pub.publish(ol_ctrl_msg)
            '''
################################################################################
            # Skid-pad test (constant control inputs with Gaussian noise - sampled from normal distribution)
            # Reference Example: noise = np.random.normal(0,1) 0 mean and 1 std dev
            #lin_vel = 1.5 + np.random.normal(0,0.1)
            #ang_vel = 0.4 + np.random.normal(0,0.2)
            '''
            lin_vel_cmd = lin_vel + np.random.normal(0,lin_noise)
            ang_vel_cmd = ang_vel + np.random.normal(0,ang_noise)
            ol_ctrl_msg.header.stamp = rospy.Time.now()
            ol_ctrl_msg.header.frame_id = 'base_link'
            ol_ctrl_msg.drive.speed = lin_vel_cmd
            ol_ctrl_msg.drive.steering_angle = ang_vel_cmd
            #print("Lin Vel : {:.4} m/s".format(ol_ctrl_msg.drive.speed))
            #print("Ang Vel : {:.4} rad/s".format(ol_ctrl_msg.drive.steering_angle))
            ol_ctrl_pub.publish(ol_ctrl_msg)
            '''
################################################################################
            # Slalom test (constant linear velocity and time-delayed varying steering with Gaussian noise - sampled from normal distribution)
            # Reference Example: noise = np.random.normal(0,1) 0 mean and 1 std dev
            #lin_vel = 1.5 + np.random.normal(0,0.1)
            #ang_vel = 0.4 + np.random.normal(0,0.2)
            '''
            # Straight
            t_delay = rospy.Duration.from_sec(2.5/lin_vel).to_sec()
            t_start = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec() - t_start) <= t_delay:
                lin_vel_cmd = lin_vel + np.random.normal(0,lin_noise)
                ang_vel_cmd = 0
                ol_ctrl_msg.header.stamp = rospy.Time.now()
                ol_ctrl_msg.header.frame_id = 'base_link'
                ol_ctrl_msg.drive.speed = lin_vel_cmd
                ol_ctrl_msg.drive.steering_angle = ang_vel_cmd
                #print("Lin Vel : {:.4} m/s".format(ol_ctrl_msg.drive.speed))
                #print("Ang Vel : {:.4} rad/s".format(ol_ctrl_msg.drive.steering_angle))
                ol_ctrl_pub.publish(ol_ctrl_msg)
            # Slalom
            for i in range(3):
		    # Left
		    t_delay = rospy.Duration.from_sec(1.0/lin_vel).to_sec()
		    t_start = rospy.Time.now().to_sec()
		    while (rospy.Time.now().to_sec() - t_start) <= t_delay:
		        lin_vel_cmd = lin_vel + np.random.normal(0,lin_noise)
		        ang_vel_cmd = ang_vel + np.random.normal(0,ang_noise)
		        ol_ctrl_msg.header.stamp = rospy.Time.now()
		        ol_ctrl_msg.header.frame_id = 'base_link'
		        ol_ctrl_msg.drive.speed = lin_vel_cmd
		        ol_ctrl_msg.drive.steering_angle = ang_vel_cmd
		        #print("Lin Vel : {:.4} m/s".format(ol_ctrl_msg.drive.speed))
		        #print("Ang Vel : {:.4} rad/s".format(ol_ctrl_msg.drive.steering_angle))
		        ol_ctrl_pub.publish(ol_ctrl_msg)
		    # Right
		    t_delay = rospy.Duration.from_sec(1.0/lin_vel).to_sec()
		    t_start = rospy.Time.now().to_sec()
		    while (rospy.Time.now().to_sec() - t_start) <= t_delay:
		        lin_vel_cmd = lin_vel + np.random.normal(0,lin_noise)
		        ang_vel_cmd = -ang_vel - np.random.normal(0,ang_noise)
		        ol_ctrl_msg.header.stamp = rospy.Time.now()
		        ol_ctrl_msg.header.frame_id = 'base_link'
		        ol_ctrl_msg.drive.speed = lin_vel_cmd
		        ol_ctrl_msg.drive.steering_angle = ang_vel_cmd
		        #print("Lin Vel : {:.4} m/s".format(ol_ctrl_msg.drive.speed))
		        #print("Ang Vel : {:.4} rad/s".format(ol_ctrl_msg.drive.steering_angle))
		        ol_ctrl_pub.publish(ol_ctrl_msg)
	    # Stop
	    lin_vel_cmd = 0
            ang_vel_cmd = 0
            ol_ctrl_msg.header.stamp = rospy.Time.now()
            ol_ctrl_msg.header.frame_id = 'base_link'
            ol_ctrl_msg.drive.speed = lin_vel_cmd
            ol_ctrl_msg.drive.steering_angle = ang_vel_cmd
            #print("Lin Vel : {:.4} m/s".format(ol_ctrl_msg.drive.speed))
            #print("Ang Vel : {:.4} rad/s".format(ol_ctrl_msg.drive.steering_angle))
            ol_ctrl_pub.publish(ol_ctrl_msg)

            rospy.signal_shutdown('Test completed!')
            '''
################################################################################
            # Fishhook test (constant linear velocity and time-delayed varying steering with Gaussian noise - sampled from normal distribution)
            # Reference Example: noise = np.random.normal(0,1) 0 mean and 1 std dev
            #lin_vel = 1.5 + np.random.normal(0,0.1)
            #ang_vel = 0.4 + np.random.normal(0,0.2)
            # Fishhook
            for delta in [0.15,0.175,0.2,0.225,0.25]:
                print(delta)
            	turn_rad = 0.33/np.tan(delta)
	    	t_delay = rospy.Duration.from_sec(2*np.pi*turn_rad/lin_vel).to_sec()
	    	t_start = rospy.Time.now().to_sec()
	   	while (rospy.Time.now().to_sec() - t_start) <= t_delay:
	     	    lin_vel_cmd = lin_vel + np.random.normal(0,lin_noise)
	    	    ang_vel_cmd = delta
	     	    ol_ctrl_msg.header.stamp = rospy.Time.now()
	      	    ol_ctrl_msg.header.frame_id = 'base_link'
	      	    ol_ctrl_msg.drive.speed = lin_vel_cmd
	      	    ol_ctrl_msg.drive.steering_angle = ang_vel_cmd
	      	    #print("Lin Vel : {:.4} m/s".format(ol_ctrl_msg.drive.speed))
	      	    #print("Ang Vel : {:.4} rad/s".format(ol_ctrl_msg.drive.steering_angle))
	      	    ol_ctrl_pub.publish(ol_ctrl_msg)
	    # Stop
	    lin_vel_cmd = 0
            ang_vel_cmd = 0
            ol_ctrl_msg.header.stamp = rospy.Time.now()
            ol_ctrl_msg.header.frame_id = 'base_link'
            ol_ctrl_msg.drive.speed = lin_vel_cmd
            ol_ctrl_msg.drive.steering_angle = ang_vel_cmd
            #print("Lin Vel : {:.4} m/s".format(ol_ctrl_msg.drive.speed))
            #print("Ang Vel : {:.4} rad/s".format(ol_ctrl_msg.drive.steering_angle))
            ol_ctrl_pub.publish(ol_ctrl_msg)

            rospy.signal_shutdown('Test completed!')
################################################################################
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
