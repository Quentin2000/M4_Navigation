#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64, Bool

class JointCommander:
    def __init__(self):
        rospy.init_node('joint_commander')

        # Subscribers
        rospy.Subscriber('/m4assembly/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('/m4assembly/mode/command', String, self.command_callback)

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
            
            # Create target angles depending on mode
            for joint_name in self.joint_names:
                # ROLLING mode
                if command_value == "ROLLING":      
                    self.target_values[joint_name] = np.linspace(self.current_positions.get(joint_name, 0.0), np.deg2rad(0), increments)
                # CRAWLING mode 
                elif command_value == "CRAWLING":
                    self.target_values[joint_name] = np.linspace(self.current_positions.get(joint_name, 0.0), np.deg2rad(30.0), increments)
                # FLIGHT mode
                elif command_value == "FLIGHT":
                    normal_speed_increments = np.linspace(self.current_positions.get(joint_name, 0.0), np.deg2rad(90.0-20.0), increments)
                    low_speed_increments = np.linspace(np.deg2rad(90.0-20.0), np.deg2rad(90.0), 20)
                    self.target_values[joint_name] = np.concatenate((normal_speed_increments, low_speed_increments))
                else:
                    rospy.logwarn("Invalid command value: {}. Command value must be ROLLING, CRAWLING, or FLIGHT.".format(command_value))
                    return
                
            # Publish the command for the specified joint
            for i in range(len(self.target_values['rear_left_hip'])):
                for joint_name in self.joint_names:
                    self.joint_publishers[joint_name].publish(Float64(self.target_values[joint_name][i]))
                rospy.sleep(speed)


if __name__ == '__main__':
    try:
        commander = JointCommander()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
