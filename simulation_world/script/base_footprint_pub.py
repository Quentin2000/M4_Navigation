import rospy
import tf
import tf.msg
import math
import geometry_msgs.msg

class DynamicTFBroadcaster:

    def __init__(self):
        
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage)

        self.ref_link = "base_link"
        self.target_link = "rear_right_wheel"
        self.m4_ground_holder_tf_z = 0.19
        self.wheel_radius_transformed = 0.20 # Wheel radius reported on line between base_link and rear_right_wheel link

        listener = tf.TransformListener()

        timeout_duration = rospy.Duration(20.0)  # Timeout duration for waiting for transform

        # Wait for the transform between the two links to become available
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time) < timeout_duration:
            try:
                listener.waitForTransform(self.ref_link, self.target_link, rospy.Time(), rospy.Duration(1.0))
                rospy.loginfo("Transform between {} and {} found.".format(self.ref_link, self.target_link))
                break  # Exit the loop if transform is found
            except tf.Exception as ex:
                rospy.logwarn("Failed to find transform between {} and {}: {}".format(self.ref_link, self.target_link, ex))
                rospy.sleep(0.5)  # Wait for a short duration before retrying


        while not rospy.is_shutdown():
            # Run this loop at about 50Hz
            rospy.sleep(0.02)

            try:
                # Get the transform between the two links
                (trans, rot) = listener.lookupTransform(self.ref_link, self.target_link, rospy.Time(0))
                rot_x = rot[0]
                distance_z = -trans[2]  # Z translation
                rospy.loginfo("Distance in X rotation: {}".format(rot_x))
                # rospy.loginfo("Distance in Z translation: {}".format(distance_z))

                self.compute_ground_contact_transform(distance_z, rot_x)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


    def compute_ground_contact_transform(self, distance_z, rot_x):
        
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "base_footprint"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = self.ref_link
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = distance_z + self.wheel_radius_transformed * math.sin(rot_x)

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        if t.transform.translation.z < self.m4_ground_holder_tf_z:
            t.transform.translation.z = self.m4_ground_holder_tf_z

        tfm = tf.msg.tfMessage([t])
        self.pub_tf.publish(tfm)

            
if __name__ == '__main__':
    rospy.init_node('base_footprint_pub')
    tfb = DynamicTFBroadcaster()
    rospy.spin()