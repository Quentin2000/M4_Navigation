#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
import time

class KeyTeleop:
    def __init__(self):
        rospy.init_node('key_controller_node')

        # Define the topic to publish Twist messages
        self.cmd_vel_pub = rospy.Publisher('/m4assembly/wheel_velocity_controller/cmd_vel', Twist, queue_size=10)

        # Subscribe to the key teleop topic
        # rospy.Subscriber('/cmd_vel', Twist, self.key_callback)

        self.max_linear_speed = 0.4  # Adjust as needed
        self.max_angular_speed = 2.0  # Adjust as needed
        self.acceleration = 0.1  # Adjust as needed
        self.deceleration = 0.5  # Adjust as needed

        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0

        self.last_key_press_time = 0.0
        self.key_press_interval = 0.1  # Adjust as needed (in seconds)

        # Start the keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    # def key_callback(self, msg):
    #     # This function is called whenever a key is pressed or released in the key teleop node
    #     pass

    def ramp_up_speed(self, target_speed, current_speed):
        # Increase the current speed gradually up to the target speed
        if target_speed > current_speed:
            return min(target_speed, current_speed + self.acceleration)
        elif target_speed < current_speed:
            return max(target_speed, current_speed - self.deceleration)
        else:
            return target_speed

    def on_press(self, key):
        # This function is called whenever a key is pressed

        # Add debouncing of keyboard
        current_time = time.time()
        if current_time - self.last_key_press_time < self.key_press_interval:
            return
        
        twist_msg = Twist()

        if key == keyboard.Key.up:
            self.current_linear_speed = self.ramp_up_speed(self.max_linear_speed, self.current_linear_speed)
            twist_msg.linear.x = self.current_linear_speed
        elif key == keyboard.Key.down:
            self.current_linear_speed = self.ramp_up_speed(-self.max_linear_speed, self.current_linear_speed)
            twist_msg.linear.x = self.current_linear_speed
        if key == keyboard.Key.left:
            self.current_angular_speed = self.ramp_up_speed(self.max_angular_speed, self.current_angular_speed)
            twist_msg.angular.z = self.current_angular_speed
        elif key == keyboard.Key.right:
            self.current_angular_speed = self.ramp_up_speed(-self.max_angular_speed, self.current_angular_speed)
            twist_msg.angular.z = self.current_angular_speed

        # Publish the Twist message
        self.cmd_vel_pub.publish(twist_msg)
        self.last_key_press_time = current_time
        rospy.sleep(0.1)

    def on_release(self, key):
        # This function is called whenever a key is released
        twist_msg = Twist()

        if key == keyboard.Key.up or key == keyboard.Key.down:
            # Decelerate linear speed to 0
            while abs(self.current_linear_speed) > 0.1:
                self.current_linear_speed = self.ramp_up_speed(0, self.current_linear_speed)
                twist_msg.linear.x = self.current_linear_speed
                self.cmd_vel_pub.publish(twist_msg)
                rospy.sleep(0.1)  # Adjust sleep time as needed

        if key == keyboard.Key.left or key == keyboard.Key.right:
            # Decelerate angular speed to 0
            while abs(self.current_angular_speed) > 0.1:
                self.current_angular_speed = self.ramp_up_speed(0, self.current_angular_speed)
                twist_msg.angular.z = self.current_angular_speed
                self.cmd_vel_pub.publish(twist_msg)
                rospy.sleep(0.1)  # Adjust sleep time as needed

if __name__ == '__main__':
    try:
        key_teleop = KeyTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass