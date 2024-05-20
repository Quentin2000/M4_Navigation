import rospy
import tf
import tf.msg
import math
import geometry_msgs.msg

class DynamicTFBroadcaster:

    def __init__(self):
        
        # Publisher
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=10)

        # Listener
        listener = tf.TransformListener()

        # Variables
        self.ref_link = "base_link"
        self.target_link = "rear_right_wheel"
        self.m4_ground_holder_tf_z = 0.19
        self.wheel_radius_transformed = 0.20        # Wheel radius projected on line between base_link and rear_right_wheel link

        timeout_duration = rospy.Duration(20.0)     # Timeout duration for waiting for ref_link to target_link transform

        # Wait for the transform between the ref_link and target_link to become available
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time) < timeout_duration:
            
            try:
                listener.waitForTransform(self.ref_link, self.target_link, rospy.Time(), rospy.Duration(1.0))
                rospy.loginfo("Transform between {} and {} found.".format(self.ref_link, self.target_link))
                # Exit the loop if transform is found
                break  
            
            except tf.Exception as ex:
                rospy.logwarn("Failed to find transform between {} and {}: {}".format(self.ref_link, self.target_link, ex))
                # Wait for a short duration before retrying
                rospy.sleep(0.5)

        # Expect the transform between the ref_link and target_link to be available after timeout
        while not rospy.is_shutdown():
            
            # Run loop at about 50Hz
            rospy.sleep(0.02)

            try:
                # Get the transform between the two links
                (trans, rot) = listener.lookupTransform(self.ref_link, self.target_link, rospy.Time(0))
                rot_x = rot[0]
                distance_z = -trans[2]  # Z distance between base_link (at center top of robot to right_rear_wheel center)
                self.compute_ground_contact_transform(distance_z, rot_x)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


    def compute_ground_contact_transform(self, distance_z, rot_x):
        
        # Create transform message
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "base_footprint"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = self.ref_link
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        # Ground_link tranform computation: base_link_2_ground_contact = dist_z(base_link, center of wheel) + dist_z(center of wheel, ground) (depending on hip angle)
        t.transform.translation.z = distance_z + self.wheel_radius_transformed * math.sin(rot_x)
        # Set the rest of the transforms as default value since the transform is only in Z.
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # If the wheel contact is higher than the robot supports, set transform to dist_z(base_link, robot_supports)
        if t.transform.translation.z < self.m4_ground_holder_tf_z:
            t.transform.translation.z = self.m4_ground_holder_tf_z

        # Publish transform
        tfm = tf.msg.tfMessage([t])
        self.pub_tf.publish(tfm)

            
if __name__ == '__main__':
    rospy.init_node('base_footprint_pub')
    tfb = DynamicTFBroadcaster()
    rospy.spin()