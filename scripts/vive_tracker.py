#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import triad_openvr
import time
import sys
import tf
import numpy as np

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
    broadcaster = { }
    publisher = { }
    rate = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
        # For each Vive Device
        for deviceName in v.devices:
            
            # Broadcast the TF as a quaternion
            [x, y, z, qw, qx, qy, qz] = v.devices[deviceName].get_pose_quaternion()
            time = rospy.Time.now()
            if deviceName not in broadcaster:
                broadcaster[deviceName] = tf.TransformBroadcaster()
            
            broadcaster[deviceName].sendTransform((x,y,z),
                            (qw,qx,qy,qz),
                            time,
                            deviceName,
                            "vive_world")
            # Publish a topic as euler angles
            [x,y,z,yaw,pitch,roll] = v.devices[deviceName].get_pose_euler()
            if deviceName not in publisher:
                publisher[deviceName] = rospy.Publisher(deviceName, String, queue_size=10)
            
            publisher[deviceName].publish('  X: ' + str(x) + '  Y: ' + str(y) + '  Z: ' + str(z) + '  Pitch: ' + str(pitch) + '  Roll: ' + str(roll) + '  Yaw: ' + str(yaw))

        rate.sleep()


if __name__ == '__main__':
    try:
        vive_tracker()
    except rospy.ROSInterruptException:
        pass
