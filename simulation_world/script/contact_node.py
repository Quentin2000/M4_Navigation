#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Int32

class ContactCounter:
    def __init__(self):
        rospy.init_node('contact_counter_node')

        # Subscriber to the bumper states topic
        self.contact_sub = rospy.Subscriber('/bumper_states', ContactsState, self.contact_callback)

        # Publisher to publish the contact count
        self.contact_count_pub = rospy.Publisher('/base_link_contacts', Int32, queue_size=10)

        # Counter to keep track of contacts
        self.contact_count = 0

    def contact_callback(self, msg):
        # Check if there are any contacts
        if msg.states:
            self.contact_count += 1
            rospy.loginfo("Contact detected! Total contacts: %s", self.contact_count)

    def run(self):
        # Create a rate object for a loop running at 50 Hz
        rate = rospy.Rate(50)  # 50 Hz

        while not rospy.is_shutdown():
            # Publish the count of contacts at a rate of 50 Hz, regardless of callback activity
            self.contact_count_pub.publish(self.contact_count)
            rate.sleep()

if __name__ == '__main__':
    contact_counter = ContactCounter()
    try:
        contact_counter.run()
    except rospy.ROSInterruptException:
        pass
