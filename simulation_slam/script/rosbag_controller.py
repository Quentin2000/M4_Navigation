# #!/usr/bin/env python3

# import rospy
# from std_msgs.msg import Int32
# import subprocess
# import os
# import signal
# import time  # Import the time module

# class RosbagController:
#     def __init__(self):
#         rospy.init_node('rosbag_controller')
        
#         # Subscriber to control the rosbag play
#         rospy.Subscriber("/goal_status", Int32, self.control_callback)
        
#         # Fetch the rosbag name and construct the full path
#         self.rosbag_file = rospy.get_param("~rosbag", "rosbag.bag")
#         self.rosbag_path = os.path.join(os.path.expanduser('~/catkin_ws/src/M4_Local_Planner/rosbags/bagfiles'), self.rosbag_file)
        
#         # Topics to record
#         self.topics = [
#             "/power_consumption_total",
#             "/power_consumption_wheel",
#             "/power_consumption_hip",
#             "/goal_status"
#         ]
        
#         # Process handle for rosbag play
#         self.play_process = None

#         # Process handle for rosbag record
#         self.record_process = None

#     def control_callback(self, msg):
#         # Start the rosbag if it isn't already running
#         if self.play_process is None:
#             self.play_process = subprocess.Popen(['rosbag', 'play', self.rosbag_path])
#             rospy.loginfo("Rosbag play started.")
        
#         if self.record_process is None:
#             record_cmd = ['rosbag', 'record', '-O', self.rosbag_file] + self.topics
#             self.record_process = subprocess.Popen(record_cmd)
#             rospy.loginfo("Rosbag record started.")

#         # If the message data is 3, terminate this script by sending SIGINT to itself
#         if msg.data == 3:

#             # Wait for 2 seconds to log everything
#             rospy.logwarn("Waiting 2 seconds before sending CTRL+C to self...")
#             time.sleep(2)
            
#             if self.play_process is not None:
#                 self.play_process.terminate()
#                 self.play_process = None
#                 rospy.loginfo("Rosbag play stopped.")
            
#             if self.record_process is not None:
#                 self.record_process.send_signal(signal.SIGINT)  # Sending SIGINT to ensure proper rosbag closure
#                 self.record_process = None
#                 rospy.loginfo("Rosbag record stopped.")

#             os.kill(os.getpid(), signal.SIGINT)

#     def on_shutdown(self):
#         # Ensure that the rosbag processes are terminated when the node is shut down
#         if self.play_process is not None:
#             self.play_process.terminate()
#             rospy.loginfo("Rosbag play process terminated on shutdown.")
#         if self.record_process is not None:
#             self.record_process.send_signal(signal.SIGINT)
#             rospy.loginfo("Rosbag record process terminated on shutdown.")

# if __name__ == '__main__':
#     node = RosbagController()
#     rospy.on_shutdown(node.on_shutdown)
#     rospy.spin()
