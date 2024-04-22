#!/usr/bin/env python  
import roslib
roslib.load_manifest('simulation_world')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('base_footprint_node')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, -0.5),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "base_link",
                         "base_footprint")
        rate.sleep()