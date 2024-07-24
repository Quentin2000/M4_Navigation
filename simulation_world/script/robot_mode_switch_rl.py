#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from simulation_world.msg import HipPos 

class HipCommander:
    def __init__(self):
        rospy.init_node('hip_commander')

        # Subscribers
        # rospy.Subscriber('/m4assembly/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('rl/hip_pos', HipPos, self.command_callback)

        # Variables
        self.current_positions = {}
        self.initial_position = {}
        self.target_values = {}
        self.current_mode = 0
        self.joint_names = ['rear_left_hip', 'rear_right_hip', 'front_left_hip', 'front_right_hip']

        # Publishers
        # Create publishers for each joint command topic
        self.joint_publishers = {}
        for joint_name in self.joint_names:
            topic_name = '/m4assembly/' + joint_name + '_position_controller/command'
            self.joint_publishers[joint_name] = rospy.Publisher(topic_name, Float64, queue_size=10)


    def command_callback(self, msg):
        # Extract values from the message [rad]
        hip_positions = {
            'front_left_hip': msg.FL_hip, # [rad]
            'front_right_hip': msg.FR_hip, # [rad]
            'rear_left_hip': msg.RL_hip, # [rad]
            'rear_right_hip': msg.RR_hip # [rad]
        }
        
        # Publish the commands to each respective joint
        for joint, pos in hip_positions.items():
            if joint in self.joint_publishers:
                self.joint_publishers[joint].publish(pos)
        

if __name__ == '__main__':
    try:
        commander = HipCommander()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
