#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import triad_openvr
import time
import sys

v = triad_openvr.triad_openvr()
v.print_discovered_objects()

def vive_tracker():
    pub = rospy.Publisher('vive_tracker', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        txt = ""
        for each in v.devices["tracker_1"].get_pose_euler():
            txt += "%.4f" % each
            txt += " "
        #print("\r" + txt)
        rospy.loginfo(txt)
        pub.publish(txt)
        rate.sleep()

if __name__ == '__main__':
    try:
        vive_tracker()
    except rospy.ROSInterruptException:
        pass
