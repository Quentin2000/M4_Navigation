#!/usr/bin/env python

import rospy
from simulation_slam.msg import NavigationState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math

class NavigationStateNode:
    def __init__(self):
        rospy.init_node('navigation_state_estimator_node')

        # Subscribers
        rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)

        # Subscribe to the joint states topic
        rospy.Subscriber('/m4assembly/joint_states', JointState, self.joint_states_callback)

        # Publishers
        self.pub_nav_state = rospy.Publisher('/nav_state', NavigationState, queue_size=10)

        # Variables
        self.linear_speed = None        # float64 x, float64 y, float64 z
        self.angular_speed = None       # float64 x, float64 y, float64 z
        self.is_moving = False
        self.is_transitioning = False
        self.is_transitioning = False
        self.joint_names = ['rear_left_hip', 'rear_right_hip', 'front_left_hip', 'front_right_hip']
        self.current_velocities = {}
        self.transition_speed_threshold = 0.5
        self.linear_speed_threshold = 0.1
        self.angular_speed_threshold = 0.1

    def joint_states_callback(self, msg):
        # Update the current joint positions
        for i, joint_name in enumerate(msg.name):
            joint_name_cut = joint_name.split('_joint')[0]
            if joint_name_cut in self.joint_names:
                self.current_velocities[joint_name_cut] = msg.velocity[i]

    def odom_callback(self, msg):
        self.linear_speed = msg.twist.twist.linear      # float64 x, float64 y, float64 z
        self.angular_speed = msg.twist.twist.angular    # float64 x, float64 y, float64 z

    def estimate_state(self):
        # Wait for Odometry to start
        if (self.linear_speed is not None and self.angular_speed is not None):
            # Check if robot is moving by computing normal of linear speed and angular speed vectors 
            if  (math.sqrt( self.linear_speed.x * self.linear_speed.x +
                            self.linear_speed.y * self.linear_speed.y +
                            self.linear_speed.z * self.linear_speed.z ) > self.linear_speed_threshold or
                math.sqrt(  self.angular_speed.x * self.angular_speed.x +
                            self.angular_speed.y * self.angular_speed.y +
                            self.angular_speed.z * self.angular_speed.z)  > self.angular_speed_threshold ):
                self.is_moving = True
            else :
                self.is_moving = False

            # Check if hips are moving (assuming hip movement => transition)
            if (self.current_velocities['rear_left_hip'] > self.transition_speed_threshold or
                self.current_velocities['rear_right_hip'] > self.transition_speed_threshold or
                self.current_velocities['front_left_hip'] > self.transition_speed_threshold or
                self.current_velocities['front_right_hip'] > self.transition_speed_threshold):
                self.is_transitioning = True
            else:
                self.is_transitioning = False

            # Publish navigation state
            navigation_state = NavigationState()
            navigation_state.moving = self.is_moving
            navigation_state.transitioning = self.is_transitioning
            self.pub_nav_state.publish(navigation_state)

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.estimate_state()
            rate.sleep()


if __name__ == '__main__':
    try:
        node = NavigationStateNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
