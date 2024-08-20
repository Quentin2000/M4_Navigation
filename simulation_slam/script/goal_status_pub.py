#!/usr/bin/env python3
import rospy
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Int32

class StatusPublisher:
    def __init__(self):
        rospy.init_node('goal_status_pub')

        # Publisher that will publish the status of the first element in status_list
        self.status_pub = rospy.Publisher('/goal_status', Int32, queue_size=10)

        # Subscriber to the move_base status list
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)

        # File to write the status
        self.file_path = "/home/m4/catkin_ws/src/M4_Local_Planner/rosbags/sim_status.txt"

        # Last known status
        self.last_status = None

    def status_callback(self, msg):
        if msg.status_list:
            # Update the last known status from the first element of status_list
            self.last_status = msg.status_list[0].status
        else:
            rospy.loginfo("Status list is empty.")

    def run(self):
        rate = rospy.Rate(50)  # 50 Hz
        while not rospy.is_shutdown():
            if self.last_status is not None:
                # Publish the last known status
                self.status_pub.publish(self.last_status)

                # Write the status to the file
                with open(self.file_path, 'w') as file:
                    file.write(str(self.last_status))
            rate.sleep()

if __name__ == '__main__':
    try:
        sp = StatusPublisher()
        sp.run()
    except rospy.ROSInterruptException:
        pass
