#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class JointCommander:
    def __init__(self):
        rospy.init_node('joint_commander')

        # Initialize dictionary to store current joint positions
        self.current_positions = {}
        self.initial_position = {}

        self.target_values = {}

        self.current_mode = 0

        # Define the joint names
        self.joint_names = ['rear_left_hip', 'rear_right_hip', 'front_left_hip', 'front_right_hip']

        # Create publishers for each joint command topic
        self.joint_publishers = {}
        for joint_name in self.joint_names:
            topic_name = '/m4assembly/' + joint_name + '_position_controller/command'
            self.joint_publishers[joint_name] = rospy.Publisher(topic_name, Float64, queue_size=10)

        # Subscribe to the joint states topic
        rospy.Subscriber('/m4assembly/joint_states', JointState, self.joint_states_callback)

        # Subscribe to the command topic
        rospy.Subscriber('/m4assembly/mode/command', Float64, self.command_callback)

    def joint_states_callback(self, msg):
        # Update the current joint positions
        for i, joint_name in enumerate(msg.name):
            joint_name_cut = joint_name.split('_joint')[0]
            if joint_name_cut in self.joint_names:
                self.current_positions[joint_name_cut] = msg.position[i]

    def command_callback(self, msg):
        # Determine the target positions based on the command value
        command_value = msg.data

        increments = 90
        speed = 0.01
        
        if self.current_mode != command_value:

            self.current_mode = command_value
            
            for joint_name in self.joint_names:
                if command_value == 0:   # ROLLING mode
                    self.target_values[joint_name] = np.linspace(self.current_positions.get(joint_name, 0.0), np.deg2rad(0), increments)
                elif command_value == 1: # CRAWLING mode 
                    self.target_values[joint_name] = np.linspace(self.current_positions.get(joint_name, 0.0), np.deg2rad(30.0), increments)
                elif command_value == 2: # FLIGHT mode
                    normal_speed_increments = np.linspace(self.current_positions.get(joint_name, 0.0), np.deg2rad(90.0-20.0), increments)
                    low_speed_increments = np.linspace(np.deg2rad(90.0-20.0), np.deg2rad(90.0), 20)
                    self.target_values[joint_name] = np.concatenate((normal_speed_increments, low_speed_increments))
                    # self.target_values[joint_name] = np.linspace(self.current_positions.get(joint_name, 0.0), np.deg2rad(90.0), increments)
                else:
                    rospy.logwarn("Invalid command value: {}. Command value must be 0, 1, or 2.".format(command_value))
                    return
                
                    # Publish the command for the specified joint
            
            for i in range(len(self.target_values['rear_left_hip'])):
                for joint_name in self.joint_names:
                    self.joint_publishers[joint_name].publish(Float64(self.target_values[joint_name][i]))
                rospy.sleep(speed)
        
        # increments = 90
        # normal_speed = 0.04
        # high_speed = 0.01
        # speed_array = np.full(increments, normal_speed)
        
        # if self.current_mode != command_value:

        #     self.current_mode = command_value
            
        #     for joint_name in self.joint_names:
        #         if command_value == 0:   # ROLLING mode
        #             self.target_values[joint_name] = np.linspace(self.current_positions.get(joint_name, 0.0), np.deg2rad(0), increments)
        #         elif command_value == 1: # CRAWLING mode 
        #             self.target_values[joint_name] = np.linspace(self.current_positions.get(joint_name, 0.0), np.deg2rad(30.0), increments)
        #         elif command_value == 2: # FLIGHT mode
        #             # Trickier because the robot oscillates and falls in simulation if hip deployment speed not tuned  
        #             normal_speed_increments_array = np.linspace(self.current_positions.get(joint_name, 0.0), np.deg2rad(90.0-20.0), increments)
        #             normal_speed_array = np.full(increments, normal_speed)
        #             high_speed_increments_array = np.linspace(np.deg2rad(90.0-20.0), np.deg2rad(90.0), 20)
        #             high_speed_array = np.full(increments, high_speed)
        #             self.target_values[joint_name] = np.concatenate((normal_speed_increments_array, high_speed_increments_array))
        #             speed_array = np.concatenate((normal_speed_array, high_speed_array))
        #         else:
        #             rospy.logwarn("Invalid command value: {}. Command value must be 0, 1, or 2.".format(command_value))
        #             return

        #     # Publish the command for the specified joint
            
        #     for i in range(len(self.target_values['rear_left_hip'])):
        #         for joint_name in self.joint_names:
        #             self.joint_publishers[joint_name].publish(Float64(self.target_values[joint_name][i]))
        #         rospy.sleep(speed_array[i])


if __name__ == '__main__':
    try:
        commander = JointCommander()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
