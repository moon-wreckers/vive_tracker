#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import triad_openvr
import time
import sys
import tf
import numpy as np
import math

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
    last_time = { }
    last_pose = { }
    rate = rospy.Rate(30) # 10hz
    while not rospy.is_shutdown():
        # For each Vive Device
        for deviceName in v.devices:
            
            # Broadcast the TF as a quaternion
            [x, y, z, qw, qx, qy, qz] = v.devices[deviceName].get_pose_quaternion()
            time = rospy.Time.now()
            if deviceName not in broadcaster:
                broadcaster[deviceName] = tf.TransformBroadcaster()
                broadcaster[deviceName + "_euler"] = tf.TransformBroadcaster()
                
            
            broadcaster[deviceName].sendTransform((x,y,z),
                            (qw,qx,qy,qz),
                            time,
                            deviceName,
                            "vive_world")
            # Publish a topic as euler angles
            [x,y,z,roll,pitch,yaw] = v.devices[deviceName].get_pose_euler()
            y_rot = math.radians(pitch)
            
            if deviceName not in publisher:
                publisher[deviceName] = rospy.Publisher(deviceName, String, queue_size=10)
                
            
            publisher[deviceName].publish('  X: ' + str(x) + '  Y: ' + str(y) + '  Z: ' + str(z) + '  Pitch: ' + str(pitch) + '  Roll: ' + str(roll) + '  Yaw: ' + str(yaw))
            
            if "reference" not in deviceName:
                if deviceName + "_odom" not in publisher:
                    publisher[deviceName + "_odom"] = rospy.Publisher(deviceName + "_odom", Odometry, queue_size=50)
                # next, we'll publish the odometry message over ROS
                odom = Odometry()
                odom.header.stamp = time
                odom.header.frame_id = "vive_world"
                # set the position
                odom.pose.pose = Pose(Point(x, y, z), Quaternion(qx,qy,qz,qw))
    
                # set the velocity
                odom.child_frame_id = "vive_world"
                if deviceName in last_time and deviceName in last_pose:
                    [vx, vy, vz, v_roll, v_pitch, v_yaw] = v.devices[deviceName].get_velocities()
                    
                    #t = time - last_time[deviceName]
                    #vx = (x - last_pose[deviceName].position.x) / t.to_sec()
                    #vy = (y - last_pose[deviceName].position.y) / t.to_sec()
                    #vz = (z - last_pose[deviceName].position.z) / t.to_sec()
                    #last_quat = last_pose[deviceName].orientation
                    #explicit_quat = [last_quat.x, last_quat.y, last_quat.z, last_quat.w]
                    #euler = tf.transformations.euler_from_quaternion(explicit_quat)
                    #last_roll = euler[0]
                    #last_pitch = euler[1]
                    #last_yaw = euler[2]
#                   # print("Last Roll: " + str(euler[0]) + " This Roll: " + str(math.radians(roll)))
                    #v_roll  = (math.radians(roll) - last_roll  ) / t.to_sec()
                    #v_pitch = (math.radians(pitch) - last_pitch) / t.to_sec()
                    #v_yaw   = (math.radians(yaw) - last_yaw    ) / t.to_sec()
                    odom.twist.twist = Twist(Vector3(vx, vy, vz), Vector3(v_roll, v_pitch, v_yaw))
    
                # publish the message
                publisher[deviceName + "_odom"].publish(odom)
                last_time[deviceName] = time
                last_pose[deviceName] = odom.pose.pose
        rate.sleep()


if __name__ == '__main__':
    try:
        vive_tracker()
    except rospy.ROSInterruptException:
        pass
