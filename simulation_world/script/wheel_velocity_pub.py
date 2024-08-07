#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Float64

def callback(data):
    wheel_indices = {
        "front_left_wheel": -1,
        "front_right_wheel": -1,
        "rear_left_wheel": -1,
        "rear_right_wheel": -1
    }
    
    # Find the indices of the wheels
    for i, name in enumerate(data.name):
        if "front_left_wheel" in name:
            wheel_indices["front_left_wheel"] = i
        elif "front_right_wheel" in name:
            wheel_indices["front_right_wheel"] = i
        elif "rear_left_wheel" in name:
            wheel_indices["rear_left_wheel"] = i
        elif "rear_right_wheel" in name:
            wheel_indices["rear_right_wheel"] = i
    
    # Publish the angular z velocities
    if wheel_indices["front_left_wheel"] != -1:
        front_left_pub.publish(data.twist[wheel_indices["front_left_wheel"]].angular.z)
    if wheel_indices["front_right_wheel"] != -1:
        front_right_pub.publish(data.twist[wheel_indices["front_right_wheel"]].angular.z)
    if wheel_indices["rear_left_wheel"] != -1:
        rear_left_pub.publish(data.twist[wheel_indices["rear_left_wheel"]].angular.z)
    if wheel_indices["rear_right_wheel"] != -1:
        rear_right_pub.publish(data.twist[wheel_indices["rear_right_wheel"]].angular.z)

if __name__ == '__main__':
    rospy.init_node('wheel_angular_velocity_publisher', anonymous=True)

    front_left_pub = rospy.Publisher('/wheel_velocity/FL', Float64, queue_size=10)
    front_right_pub = rospy.Publisher('/wheel_velocity/FR', Float64, queue_size=10)
    rear_left_pub = rospy.Publisher('/wheel_velocity/RL', Float64, queue_size=10)
    rear_right_pub = rospy.Publisher('/wheel_velocity/RR', Float64, queue_size=10)

    rospy.Subscriber('/gazebo/link_states', LinkStates, callback)

    rospy.spin()