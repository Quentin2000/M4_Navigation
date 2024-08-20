#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def send_goal():
    rospy.init_node('send_nav_goal')

    # Publisher to send the goal
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    # Set the rate of the loop to 1 Hz
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Create the PoseStamped message
        goal = PoseStamped()
        goal.header.frame_id = "map"  # Typically the goal is specified in the map frame
        goal.header.stamp = rospy.Time.now()

        # Set position (x, y, z)
        goal.pose.position.x = 5.0
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.0

        # Set orientation (quaternion)
        q = quaternion_from_euler(0, 0, 0.5)  # Roll, Pitch, Yaw
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        # Publish the goal
        goal_pub.publish(goal)
        rospy.loginfo("Goal published to /move_base_simple/goal")

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        send_goal()
    except rospy.ROSInterruptException:
        pass
