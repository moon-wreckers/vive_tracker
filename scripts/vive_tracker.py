#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import triad_openvr
import time
import sys
import tf

print('\n')
try:
  v = triad_openvr.triad_openvr()
except Exception as ex:
  if (type(ex).__name__ == 'OpenVRError' and ex.args[0] == 'VRInitError_Init_HmdNotFoundPresenceFailed (error number 126)'):
    print('Cannot find the tracker.')
    print('Is SteamVR running?')
    print('Is the Vive Tracker turned on, connected, and paired with SteamVR?')
    print('Are the Lighthouse Base Stations powered and in view of the Tracker?\n\n')
  else:
    template = "An exception of type {0} occurred. Arguments:\n{1!r}"
    message = template.format(type(ex).__name__, ex.args)
    print message
  #print(ex.args)
  quit()

v.print_discovered_objects()

def vive_tracker():
    rospy.init_node('vive_tracker_frame')
    br = tf.TransformBroadcaster()
    pub_tracker = rospy.Publisher('vive_tracker_euler', String, queue_size=10)
    pub_reference_1 = rospy.Publisher('vive_reference_euler_1', String, queue_size=10)
    pub_reference_2 = rospy.Publisher('vive_reference_euler_2', String, queue_size=10)
    #rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(30) # 10hz
    tfVals = [None] * 7
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        txt = ""
        eulerPose = [None] * 6

        for i, each in enumerate(v.devices["tracking_reference_1"].get_pose_euler()):
            txt += "%.4f" % each
            txt += " "
            eulerPose[i] = each
        for i, each in enumerate(v.devices["tracking_reference_1"].get_pose_quaternion()):
            tfVals[i] = each
        
        br.sendTransform((tfVals[0],tfVals[1],tfVals[2]),
                         (tfVals[3],tfVals[4],tfVals[5],tfVals[6]),
                         rospy.Time.now(),
                         "vive_tracking_reference_1",
                         "vive_world")
        pub_reference_1.publish('  X: ' + str(eulerPose[0]) + '  Y: ' + str(eulerPose[1]) + '  Z: ' + str(eulerPose[2]) + '  Pitch: ' + str(eulerPose[5]) + '  Yaw: ' + str(eulerPose[4]) + '  Roll: ' + str(eulerPose[3]))
        if "tracking_reference_2" in v.devices:
            for i, each in enumerate(v.devices["tracking_reference_2"].get_pose_euler()):
                txt += "%.4f" % each
                txt += " "
                eulerPose[i] = each
            for i, each in enumerate(v.devices["tracking_reference_2"].get_pose_quaternion()):
                tfVals[i] = each
            
            br.sendTransform((tfVals[0],tfVals[1],tfVals[2]),
                             (tfVals[3],tfVals[4],tfVals[5],tfVals[6]),
                             rospy.Time.now(),
                             "vive_tracking_reference_2",
                             "vive_world")
            pub_reference_2.publish('  X: ' + str(eulerPose[0]) + '  Y: ' + str(eulerPose[1]) + '  Z: ' + str(eulerPose[2]) + '  Pitch: ' + str(eulerPose[5]) + '  Yaw: ' + str(eulerPose[4]) + '  Roll: ' + str(eulerPose[3]))

        for i, each in enumerate(v.devices["tracker_1"].get_pose_euler()):
            txt += "%.4f" % each
            txt += " "
            eulerPose[i] = each
        for i, each in enumerate(v.devices["tracker_1"].get_pose_quaternion()):
            tfVals[i] = each
        
        br.sendTransform((tfVals[0],tfVals[1],tfVals[2]),
                         (tfVals[3],tfVals[4],tfVals[5],tfVals[6]),
                         rospy.Time.now(),
                         "vive_tracker_1",
                         "vive_world")
        pub_tracker.publish('  X: ' + str(eulerPose[0]) + '  Y: ' + str(eulerPose[1]) + '  Z: ' + str(eulerPose[2]) + '  Pitch: ' + str(eulerPose[5]) + '  Yaw: ' + str(eulerPose[4]) + '  Roll: ' + str(eulerPose[3]))

        rate.sleep()

if __name__ == '__main__':
    try:
        vive_tracker()
    except rospy.ROSInterruptException:
        pass
