import rospy
import tf
import tf.msg
import tf.transformations as tft
import math
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates

class DynamicTFBroadcaster:

    def __init__(self):

        # Params
        self.use_sim_z_pose = rospy.get_param("~use_sim_z_pose", True)  # If True (for simulation purposes), uses Gazebo elevation to compute distance to ground. Else, (for real world use), uses hip joints position to compute distance to ground.
        
        # Publisher
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=10)

        # Listener
        self.listener = tf.TransformListener()
        if self.use_sim_z_pose:
            self.z_pose = 30.0
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.pose_callback)

        # Variables
        self.rotation_x = 0
        self.rotation_y = 0
        self.rotation_z = 0
        self.rotation_w = 1
        self.ref_link = "base_link"
        self.rear_right_wheel = "rear_right_wheel"
        self.rear_left_wheel = "rear_left_wheel"
        self.front_right_wheel = "front_right_wheel"
        self.front_left_wheel = "front_left_wheel"
        self.m4_ground_holder_tf_z = 0.19
        self.wheel_radius_transformed = 0.35        # Length of the extension of the line "base_link to wheel_center" between wheel_center and the ground when the wheel is vertical

        timeout_duration = rospy.Duration(10.0)     # Timeout duration for waiting for ref_link to wheel link transform

        # Wait for the transform between the ref_link and target_link to become available
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time) < timeout_duration:
            
            try:
                self.listener.waitForTransform(self.ref_link, self.rear_right_wheel, rospy.Time(), rospy.Duration(1.0))
                self.listener.waitForTransform(self.ref_link, self.rear_left_wheel, rospy.Time(), rospy.Duration(1.0))
                self.listener.waitForTransform(self.ref_link, self.front_right_wheel, rospy.Time(), rospy.Duration(1.0))
                self.listener.waitForTransform(self.ref_link, self.front_left_wheel, rospy.Time(), rospy.Duration(1.0))
                # Exit the loop if transform is found
                break  
            
            except tf.Exception as ex:
                rospy.logwarn("Failed to find transform between {} and wheels link: {}".format(self.ref_link, ex))
                # Wait for a short duration before retrying
                rospy.sleep(0.5)

        # Expect the transform between the ref_link and wheel links to be available after timeout
        while not rospy.is_shutdown():
            
            # Run loop at about 50Hz
            rospy.sleep(0.02)

            try:
 
                # Compute separate ground transforms for each wheel 
                t1 = self.compute_ground_contact_transform(self.rear_right_wheel)
                t2 = self.compute_ground_contact_transform(self.rear_left_wheel)
                t3 = self.compute_ground_contact_transform(self.front_right_wheel)
                t4 = self.compute_ground_contact_transform(self.front_left_wheel)

                # Initialize ground transformation as average distance to ground between base_link and all wheel links
                t = t1
                t.transform.translation.z = (t1.transform.translation.z + t2.transform.translation.z + t3.transform.translation.z + t4.transform.translation.z) / 4

                # Publish transform
                tfm = tf.msg.tfMessage([t])
                self.pub_tf.publish(tfm)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def pose_callback(self, msg):
        rospy.logwarn("Received ModelStates message with {} poses".format(len(msg.pose)))
        if len(msg.pose) > 1:
            self.z_pose = msg.pose[1].position.z

            original_quaternion = (msg.pose[1].orientation.x, msg.pose[1].orientation.y, msg.pose[1].orientation.z, msg.pose[1].orientation.w)
            
            # Convert quaternion to Euler angles
            euler_angles = tft.euler_from_quaternion(original_quaternion)

            # Modify Euler angles: set yaw (rotation around z-axis) to 0
            modified_euler_angles = (euler_angles[0], euler_angles[1], 0)

            # Convert modified Euler angles back to quaternion
            new_quaternion = tft.quaternion_from_euler(*modified_euler_angles)

            self.rotation_x, self.rotation_y, self.rotation_z, self.rotation_w = new_quaternion
            
        else:
            rospy.logwarn("ModelStates message does not contain enough models.")

    def compute_ground_contact_transform(self, wheel_link):

        # Get the transform between the two links
        (trans, rot) = self.listener.lookupTransform(self.ref_link, wheel_link, rospy.Time(0))
        rot_x = rot[0]
        distance_z = -trans[2]  # Z distance between base_link (at center top of robot to right_rear_wheel center)
        
        # Create transform message
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "base_footprint"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = self.ref_link
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        # Ground_link tranform computation: base_link_2_ground_contact = dist_z(base_link, center of wheel) + dist_z(center of wheel, ground) (depending on hip angle)
        if self.use_sim_z_pose:
            t.transform.translation.z = self.z_pose
        else:
            t.transform.translation.z = distance_z + self.wheel_radius_transformed * math.sin(rot_x)        
            # If the wheel contact is higher than the robot supports, set transform to dist_z(base_link, robot_supports)
            if t.transform.translation.z < self.m4_ground_holder_tf_z:
                t.transform.translation.z = self.m4_ground_holder_tf_z
        
            # Set the rest of the transforms as default value since the transform is only in Z.
        t.transform.rotation.x = self.rotation_x
        t.transform.rotation.y = self.rotation_y
        t.transform.rotation.z = self.rotation_z
        t.transform.rotation.w = self.rotation_w

        return t


            
if __name__ == '__main__':
    rospy.init_node('base_footprint_pub')
    tfb = DynamicTFBroadcaster()
    rospy.spin()