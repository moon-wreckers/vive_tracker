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
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])
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
                broadcaster[deviceName + "_matrix"] = tf.TransformBroadcaster()
                
            
            broadcaster[deviceName].sendTransform((x,y,z),
                            (qw,qx,qy,qz),
                            time,
                            deviceName,
                            "vive_world")
            # Publish a topic as euler angles
            [x,y,z,yaw,pitch,roll] = v.devices[deviceName].get_pose_euler()
            y_rot = math.radians(pitch)
            
            broadcaster[deviceName + "_euler"].sendTransform((x,y,z),
                            tf.transformations.quaternion_from_euler(roll,yaw,pitch),
                            time,
                            deviceName + "_euler",
                            "vive_world")
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
                pose_mat = v.devices[deviceName].get_pose_matrix()
                R = [[0 for x in range(3)] for y in range(3)]
                for i in range(0,3):
                    for j in range(0,3):
                        R[i][j] = pose_mat[i][j]
                print(str(round(R[0][0],3)) + "\t" + str(round(R[0][1],3)) + "\t" + str(round(R[0][2],3)))
                print(str(round(R[1][0],3)) + "\t" + str(round(R[1][1],3)) + "\t" + str(round(R[1][2],3)))
                print(str(round(R[2][0],3)) + "\t" + str(round(R[2][1],3)) + "\t" + str(round(R[2][2],3)))
                [xrot,yrot,zrot] = rotationMatrixToEulerAngles(np.asarray(R))
                print("")
                broadcaster[deviceName + "_matrix"].sendTransform((pose_mat[0][3],pose_mat[1][3],pose_mat[2][3]),
                                tf.transformations.quaternion_from_euler(xrot,yrot,zrot),
                                time,
                                deviceName + "_matrix",
                                "vive_world")
                # set the position
                odom.pose.pose = Pose(Point(x, y, z), Quaternion(qx,qy,qz,qw))
    
                # set the velocity
                odom.child_frame_id = "vive_world"
                if deviceName in last_time and deviceName in last_pose:
                    t = time - last_time[deviceName]
                    vx = (x - last_pose[deviceName].position.x) / t.to_sec()
                    vy = (y - last_pose[deviceName].position.y) / t.to_sec()
                    vz = (z - last_pose[deviceName].position.z) / t.to_sec()
                    last_quat = last_pose[deviceName].orientation
                    explicit_quat = [last_quat.x, last_quat.y, last_quat.z, last_quat.w]
                    euler = tf.transformations.euler_from_quaternion(explicit_quat)
                    last_roll = euler[0]
                    last_pitch = euler[1]
                    last_yaw = euler[2]
#                    print("Last Roll: " + str(euler[0]) + " This Roll: " + str(math.radians(roll)))
                    v_roll  = (math.radians(roll) - last_roll  ) / t.to_sec()
                    v_pitch = (math.radians(pitch) - last_pitch) / t.to_sec()
                    v_yaw   = (math.radians(yaw) - last_yaw    ) / t.to_sec()
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
