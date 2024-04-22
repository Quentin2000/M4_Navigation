#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64

pub1 = rospy.Publisher('/m4assembly/front_left_hip_position_controller/command', Float64, queue_size=10)
pub2 = rospy.Publisher('/m4assembly/front_right_hip_position_controller/command', Float64, queue_size=10)
pub3 = rospy.Publisher('/m4assembly/rear_left_hip_position_controller/command', Float64, queue_size=10)
pub4 = rospy.Publisher('/m4assembly/rear_right_hip_position_controller/command', Float64, queue_size=10)
# pub9 = rospy.Publisher('/m4assembly/blade1_position_controller/command', Float64, queue_size=10)
#pub10 = rospy.Publisher('/m4assembly/blade2_position_controller/command', Float64, queue_size=10)
#pub11 = rospy.Publisher('/m4assembly/blade3_position_controller/command', Float64, queue_size=10)
#pub12 = rospy.Publisher('/m4assembly/blade4_position_controller/command', Float64, queue_size=10)


rospy.init_node('joint_position_publisher')

while not rospy.is_shutdown():
        setup = input("Enter M4 mode ('1' for Flight, '0' for Ground): ")
        print(setup)
        if float(setup) == 1:
            for i in range(1, 90):
                pub1.publish(Float64(np.deg2rad(i)))
                pub2.publish(Float64(np.deg2rad(i)))
                pub3.publish(Float64(np.deg2rad(i)))
                pub4.publish(Float64(np.deg2rad(i)))
                rospy.sleep(0.01)

        else:
            for i in range(1, 90):
                pub1.publish(Float64(np.deg2rad(90-i)))
                pub2.publish(Float64(np.deg2rad(90-i)))
                pub3.publish(Float64(np.deg2rad(90-i)))
                pub4.publish(Float64(np.deg2rad(90-i)))
                rospy.sleep(0.01)
