import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
import time
import threading

class KeyTeleop:
    def __init__(self):
        rospy.init_node('key_controller_node')
        self.cmd_vel_pub = rospy.Publisher('/m4assembly/wheel_velocity_controller/cmd_vel', Twist, queue_size=10)

        self.max_linear_speed = 0.3
        self.max_angular_speed = 2.0
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.key_state = {
            "up": False,
            "down": False,
            "left": False,
            "right": False
        }

        # Start the keyboard listener
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        # Start the command thread
        self.running = True
        self.send_command = False
        self.command_thread = threading.Thread(target=self.send_continuous_commands)
        self.command_thread.start()

    def send_continuous_commands(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown() and self.running:
            twist_msg = Twist()
            self.send_command = False
            self.current_linear_speed = 0
            self.current_angular_speed = 0

            if self.key_state["up"]:
                self.current_linear_speed = self.max_linear_speed
                self.send_command = True
            elif self.key_state["down"]:
                self.current_linear_speed = -self.max_linear_speed
                self.send_command = True

            if self.key_state["left"]:
                self.current_angular_speed = self.max_angular_speed
                self.send_command = True
            elif self.key_state["right"]:
                self.current_angular_speed = -self.max_angular_speed
                self.send_command = True

            if self.send_command:
                twist_msg.linear.x = self.current_linear_speed
                twist_msg.angular.z = self.current_angular_speed
                self.cmd_vel_pub.publish(twist_msg)

            rate.sleep()

    def on_press(self, key):
        if key == keyboard.Key.up:
            self.key_state["up"] = True
        elif key == keyboard.Key.down:
            self.key_state["down"] = True
        elif key == keyboard.Key.left:
            self.key_state["left"] = True
        elif key == keyboard.Key.right:
            self.key_state["right"] = True

    def on_release(self, key):
        if key == keyboard.Key.up:
            self.key_state["up"] = False
        elif key == keyboard.Key.down:
            self.key_state["down"] = False
        elif key == keyboard.Key.left:
            self.key_state["left"] = False
        elif key == keyboard.Key.right:
            self.key_state["right"] = False

    def __del__(self):
        self.running = False
        self.command_thread.join()

if __name__ == '__main__':
    try:
        key_teleop = KeyTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
