#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import triad_openvr
import time
import sys
import tf

v = triad_openvr.triad_openvr()
v.print_discovered_objects()

def vive_tracker():
    rospy.init_node('vive_tracker_frame')
    br = tf.TransformBroadcaster()
    pub = rospy.Publisher('vive_tracker_str', String, queue_size=10)
    #rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        txt = ""
        tfVals = [None] * 7
        for i, each in enumerate(v.devices["tracker_1"].get_pose_quaternion()):
            txt += "%.4f" % each
            txt += " "
            tfVals[i] = each
            print(tfVals[i])
        print("")
        #print("\r" + txt)
        #rospy.loginfo(txt)
        br.sendTransform((tfVals[0],tfVals[1],tfVals[2]),
                         (tfVals[3], tfVals[4], tfVals[5], tfVals[6]),
                         rospy.Time.now(),
                         "vive_tracker",
                         "map")

        pub.publish(txt)
        rate.sleep()

if __name__ == '__main__':
    try:
        vive_tracker()
    except rospy.ROSInterruptException:
        pass
