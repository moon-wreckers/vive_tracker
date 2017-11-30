#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import time
import sys
import tf

def vive_world():
    rospy.init_node('vive_tracker_frame')
    listener = tf.TransformListener()
    rate = rospy.Rate(100) # 10hz
    translation = [0,0,0]
    rotation = [0.707,0.0,0.0,0.707]
    broadcaster = tf.TransformBroadcaster()
    broadcaster.sendTransform(translation,
                    rotation,
                    rospy.Time.now(),
                    'vive_world',
                    'map')
    sys.stdout.write('Generating vive_world frame.')
    frame_generated = False
    while not rospy.is_shutdown():
        try:
            (translation, rotation) = listener.lookupTransform('map', 'vive_world', rospy.Time(0))
            if frame_generated is False:
                print ""
                frame_generated = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            sys.stdout.write('.') 
        broadcaster.sendTransform(translation,
                        rotation,
                        rospy.Time.now(),
                        "vive_world",
                        "map")
        rate.sleep()

if __name__ == '__main__':
    try:
        vive_world()
    except rospy.ROSInterruptException:
        pass
