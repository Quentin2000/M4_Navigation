#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from simulation_slam.msg import NavigationState
import math

class PowerComputationNode:
    def __init__(self):
        rospy.init_node('power_computation_node')

        # Power consumption parameters
        self.wheel_power_min = rospy.get_param('~wheel_power_min', 0.25*12)     # Power at 0 m/s (see 25D 9.7:1 motor specs)
        self.wheel_power_max = rospy.get_param('~wheel_power_max', 10)          # Power at 1 m/s
        self.hip_power_min = rospy.get_param('~hip_power_min', 20)              # Power at 0 rad (see XXXXX motor specs)
        self.hip_power_max = rospy.get_param('~hip_power_max', 35)              # Power at 1 rad

        # Subscribers
        rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/m4assembly/joint_states', JointState, self.joint_states_callback)

        # Publishers
        self.pub_nav_state = rospy.Publisher('/nav_state', NavigationState, queue_size=10)
        self.pub_total_power_consumption = rospy.Publisher('/power_consumption_total', Float32, queue_size=10)
        self.pub_hip_power_consumption = rospy.Publisher('/power_consumption_hip', Float32, queue_size=10)
        self.pub_wheel_power_consumption = rospy.Publisher('/power_consumption_wheel', Float32, queue_size=10)

        # Variables
        self.joint_names = ['rear_left_hip', 'rear_right_hip', 'front_left_hip', 'front_right_hip']
        self.hip_angles = {}
        self.linear_speed = None
        self.angular_speed = None
        self.is_moving = False
        self.total_power = 0.0
        self.hip_power_total = 0.0
        self.wheel_power_total = 0.0

    def joint_states_callback(self, msg):
        for i, joint_name in enumerate(msg.name):
            joint_name_cut = joint_name.split('_joint')[0]
            if joint_name_cut in self.joint_names:
                self.hip_angles[joint_name_cut] = msg.position[i]
        self.calculate_and_publish_power()

    def odom_callback(self, msg):
        self.linear_speed = msg.twist.twist.linear
        self.angular_speed = msg.twist.twist.angular
        self.evaluate_motion_state()

    def evaluate_motion_state(self):
        if self.linear_speed and self.angular_speed:
            self.is_moving = math.hypot(self.linear_speed.x, self.linear_speed.y) > 0.01
            self.publish_navigation_state()

    def publish_navigation_state(self):
        nav_state = NavigationState()
        nav_state.moving = self.is_moving
        self.pub_nav_state.publish(nav_state)

    def calculate_and_publish_power(self):
        # Calculate power based on hip angles
        hip_power = 0.0
        for angle in self.hip_angles.values():
            power = self.hip_power_min + (self.hip_power_max - self.hip_power_min) * (abs(angle) / math.pi)
            hip_power += power
        
        # Update total hip power
        self.hip_power_total += hip_power

        # Calculate power based on wheel speed
        wheel_power = 0.0
        if self.is_moving:
            wheel_speed = math.hypot(self.linear_speed.x, self.linear_speed.y)
            wheel_power = self.wheel_power_min + (self.wheel_power_max - self.wheel_power_min) * wheel_speed * 4    # For 4 wheels
        
        # Update total wheel power
        self.wheel_power_total += wheel_power

        # Update total power
        self.total_power = self.hip_power_total + self.wheel_power_total

        # Publish power consumption
        self.pub_hip_power_consumption.publish(Float32(self.hip_power_total))
        self.pub_wheel_power_consumption.publish(Float32(self.wheel_power_total))
        self.pub_total_power_consumption.publish(Float32(self.total_power))

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        node = PowerComputationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
