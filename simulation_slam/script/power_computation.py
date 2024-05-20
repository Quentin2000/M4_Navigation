#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32
from simulation_slam.msg import NavigationState


class PowerComputationNode:
    def __init__(self):
        rospy.init_node('power_computation_node')

        # Parameters
        self.cost_crawling = rospy.get_param('~cost_crawling', 45)
        self.cost_rolling = rospy.get_param('~cost_rolling', 30)
        self.cost_transitioning = rospy.get_param('~cost_transitioning', 50)

        # Subscribers
        rospy.Subscriber('/m4assembly/mode/command', String, self.mode_command_callback)
        rospy.Subscriber('/nav_state', NavigationState, self.navigation_state_callback)

        # Publishers
        self.pub_cost_rolling = rospy.Publisher('/computed_cost_rolling', Float32, queue_size=10)
        self.pub_cost_crawling = rospy.Publisher('/computed_cost_crawling', Float32, queue_size=10)
        self.pub_cost_transitioning = rospy.Publisher('/computed_cost_transitioning', Float32, queue_size=10)
        self.pub_cost_total = rospy.Publisher('/computed_cost_total', Float32, queue_size=10)

        # Variables
        self.current_mode = None                           # "CRAWLING", "ROLLING", "FLIGHT"
        self.prev_mode = None                              # "CRAWLING", "ROLLING", "FLIGHT"
        self.is_moving = False
        self.is_transitioning = False
        self.prev_is_transitioning = False
        self.last_transitioning_time = rospy.Time.now()
        self.start_time = rospy.Time.now()
        self.last_time = self.start_time
        self.computed_cost_rolling = 0.0                    # Energy [J]
        self.computed_cost_crawling = 0.0                   # Energy [J]
        self.computed_cost_transitioning = 0.0              # Energy [J]

    def mode_command_callback(self, msg):
        self.current_mode = msg.data

    def navigation_state_callback(self, msg):
        self.is_moving = msg.moving
        self.is_transitioning = msg.transitioning 

    def compute_costs(self):
        
        # Wait for current_mode to become available
        if self.current_mode is not None:
            
            # Wait for robot to move to compute energy consumption (driving motors)
            if self.is_moving:
                
                # Restart timer if mode has changed [sec]
                if (self.current_mode != self.prev_mode):
                    self.last_time = rospy.Time.now()
                    self.prev_mode = self.current_mode

                # Get current time [sec]
                current_time = rospy.Time.now()

                # Compute time since last mode change
                elapsed_time = (current_time - self.last_time).to_sec()

                # Compute and publish energy comsumption per mode: E [J] = P [W] * dT [sec]
                if self.current_mode == "ROLLING":
                    self.computed_cost_rolling = self.computed_cost_rolling + self.cost_rolling * elapsed_time
                elif self.current_mode == "CRAWLING":
                    self.computed_cost_crawling = self.computed_cost_crawling + self.cost_crawling * elapsed_time
           
            else:
                # Restart timer until the robot starts moving again to avoid integrating energy when robot is idle 
                self.last_time = rospy.Time.now()
            
            # Compute mode transition power consumption (hip servo motors)
            if self.is_transitioning:
            
                # Restart timer if just started transitioning [sec]
                if (self.prev_is_transitioning == False):
                    self.last_transitioning_time = rospy.Time.now()
                    self.prev_is_transitioning = True
                
                # Get current time [sec]
                current_time = rospy.Time.now()

                # Compute time since start of transition [sec]
                elapsed_time = (current_time - self.last_transitioning_time).to_sec()

                # Compute and publish energy comsumption for transitioning: E [J] = P [W] * dT [sec]
                self.computed_cost_transitioning = self.computed_cost_transitioning + self.cost_transitioning * elapsed_time
        
            else:
                # If stopped transitioning, reset the transition state to restart the last_transitioning_time timer
                self.prev_is_transitioning = False
        
        # Publish separate energy consumptions in [J]
        self.pub_cost_rolling.publish(self.computed_cost_rolling)
        self.pub_cost_crawling.publish(self.computed_cost_crawling)
        self.pub_cost_transitioning.publish(self.computed_cost_transitioning)

        # Compute the total energy consumption in [J]
        total_cost = self.computed_cost_rolling + self.computed_cost_crawling + self.computed_cost_transitioning
        rospy.loginfo("Power consumption: %f", total_cost)
        self.pub_cost_total.publish(total_cost)

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.compute_costs()
            rate.sleep()


if __name__ == '__main__':
    try:
        node = PowerComputationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
