#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import triad_openvr
import time
import sys
import tf
from pyquaternion import Quaternion

try:
  v = triad_openvr.triad_openvr()
except Exception as ex:
  if (type(ex).__name__ == 'OpenVRError' and ex.args[0] == 'VRInitError_Init_HmdNotFoundPresenceFailed (error number 126)'):
    print('Is SteamVR turned on? Cannot contact VR server.')
  else:
    template = "An exception of type {0} occurred. Arguments:\n{1!r}"
    message = template.format(type(ex).__name__, ex.args)
    print message
  print(ex.args)
  quit()

v.print_discovered_objects()

def vive_tracker():
    rospy.init_node('vive_tracker_frame')
    br = tf.TransformBroadcaster()
    pub = rospy.Publisher('vive_tracker_euler', String, queue_size=10)
    #rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(30) # 10hz
    tfVals = [None] * 7
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        txt = ""
        eulerPose = [None] * 6
        for i, each in enumerate(v.devices["tracker_1"].get_pose_euler()):
            txt += "%.4f" % each
            txt += " "
            eulerPose[i] = each
        for i, each in enumerate(v.devices["tracker_1"].get_pose_quaternion()):
            tfVals[i] = each
        
        br.sendTransform((tfVals[0],tfVals[1],tfVals[2]),
                         (tfVals[3],tfVals[4],tfVals[5],tfVals[6]),
                         rospy.Time.now(),
                         "vive_tracker",
                         "new_map")
#        pub.publish(txt)
        rate.sleep()

if __name__ == '__main__':
    try:
        vive_tracker()
    except rospy.ROSInterruptException:
        pass
