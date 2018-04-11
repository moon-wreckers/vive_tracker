#!/usr/bin/env python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
import rospy
from std_msgs.msg import String
import triad_openvr
import time
import sys
import tf
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_multiply
import numpy as np
import math
import pdb

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
    listener = tf.TransformListener()

    rate = rospy.Rate(30) # 10hz

    P = np.mat([[1e-6, 0, 0], [0, 1e-6, 0], [0, 0, 1e-3]])
    p_cov = np.zeros((6, 6))
    # position covariance
    p_cov[0:2,0:2] = P[0:2,0:2]
    # orientation covariance for Yaw
    # x and Yaw
    p_cov[5,0] = p_cov[0,5] = P[2,0]
    # y and Yaw
    p_cov[5,1] = p_cov[1,5] = P[2,1]
    # Yaw and Yaw
    p_cov[5,5] = P[2,2]

    p_cov[0,:] =   [0.0000349162103240595,  -0.0000018202960310455,  -0.0000339898160507969,  -0.0000081126791170800,   0.0000001353045808767,   0.0000032202291901186]
    p_cov[1,:] =    [-0.0000018202960310455,   0.0000011910722363973,   0.0000020423436706964,   0.0000010961526869235,  -0.0000000333091396801,  -0.0000001408541892558]
    p_cov[2,:] =    [-0.0000339898160507969,   0.0000020423436706964,   0.0000341312090595451,   0.0000060715616751347,  -0.0000000237628610568,  -0.0000029217229365340]
    p_cov[3,:] =    [-0.0000081126791170800,   0.0000010961526869235,   0.0000060715616751347,   0.0000165832615351042,  -0.0000004759697840205,  -0.0000024486872043021]
    p_cov[4,:] =    [0.0000001353045808767,  -0.0000000333091396801,  -0.0000000237628610568,  -0.0000004759697840205,   0.0000003366392930324,  -0.0000000030521109214]
    p_cov[5,:] =    [0.0000032202291901186,  -0.0000001408541892558,  -0.0000029217229365340,  -0.0000024486872043021,  -0.0000000030521109214,   0.0000007445433570531]
    # rospy.loginfo(p_cov)
    while not rospy.is_shutdown():
        # For each Vive Device
        for deviceName in v.devices:

            if 'Null' in v.devices[deviceName].get_serial():
              del v.devices[deviceName]
              break
            publish_name_str = v.devices[deviceName].get_serial().replace("-","_")
            # Broadcast the TF as a quaternion
            [x, y, z, qx, qy, qz, qw] = v.devices[deviceName].get_pose_quaternion()
            time = rospy.Time.now()
            if deviceName not in broadcaster:
                broadcaster[deviceName] = tf.TransformBroadcaster()

            # Rotate Vive Trackers 180, so Z+ comes out of the top of the Tracker
            if "LHR" in v.devices[deviceName].get_serial():
                [qx, qy, qz, qw] = tf.transformations.quaternion_multiply([qx, qy, qz, qw], tf.transformations.quaternion_from_euler(math.pi, 0.0, -math.pi/2.0))

            broadcaster[deviceName].sendTransform((x,y,z),
                            (qx,qy,qz,qw),
                            time,
                            publish_name_str,
                            "vive_world")

            # Publish a topic as euler angles
            [x,y,z,roll,pitch,yaw] = v.devices[deviceName].get_pose_euler()
            y_rot = math.radians(pitch)

            if deviceName not in publisher:
                publisher[deviceName] = rospy.Publisher(publish_name_str, String, queue_size=10)


            publisher[deviceName].publish('  X: ' + str(x) + '  Y: ' + str(y) + '  Z: ' + str(z) + '  Pitch: ' + str(pitch) + '  Roll: ' + str(roll) + '  Yaw: ' + str(yaw))

            if "reference" not in deviceName:
                if deviceName + "_odom" not in publisher:
                    publisher["ring_odom"] = rospy.Publisher("ring_odom", Odometry, queue_size=50)
                    publisher[deviceName + "_odom"] = rospy.Publisher(publish_name_str + "_odom", Odometry, queue_size=50)
                    publisher[deviceName + "_inverted_odom"] = rospy.Publisher(publish_name_str + "_inverted_odom", Odometry, queue_size=50)
                    publisher[deviceName + "_pose"] = rospy.Publisher(publish_name_str + "_pose",PoseWithCovarianceStamped,queue_size=10)
                # next, we'll publish the odometry message over ROS
                odom = Odometry()
                odom.header.stamp = time
                odom.header.frame_id = "vive_world"
                # set the position
                odom.pose.pose = Pose(Point(x, y, z), Quaternion(qx,qy,qz,qw))

                inv_odom = Odometry()
                inv_odom.header.stamp = time
                inv_odom.header.frame_id = "vive_world"
                # set the position
                [inv_qx, inv_qy, inv_qz, inv_qw] = tf.transformations.quaternion_multiply([qx, qy, qz, qw], [-0.707, 0.707, 0, 0])
                q = Quaternion(inv_qx,inv_qy,inv_qz,inv_qw)
                # pdb.set_trace()
                # q_new = q * q_inv
                inv_odom.pose.pose = Pose(Point(x, y+0.05, z), q)

                # set the velocity
                odom.child_frame_id = "ak1_base_link"
                [vx, vy, vz, v_roll, v_pitch, v_yaw] = v.devices[deviceName].get_velocities()
                odom.twist.twist = Twist(Vector3(vx, vy, vz), Vector3(v_roll, v_pitch, v_yaw))
                # This is all wrong but close enough for now
                # np.mat([1[1e-6, 0, 0], [0, 1e-6, 0], [0, 0, 1e-3]])
                odom.pose.covariance = tuple(p_cov.ravel().tolist())
                odom.twist.covariance = tuple(p_cov.ravel().tolist())
                # rospy.loginfo(p_cov)

                # Create a pose with covariance stamped topic
                pose = PoseWithCovarianceStamped()
                pose.pose.pose = Pose(Point(x, y, z), Quaternion(qx,qy,qz,qw))
                pose.pose.covariance = tuple(p_cov.ravel().tolist())
                if not (x == 0.0 and y == 0.0 and z == 0.0 and vx == 0.0 and vy == 0.0 and vz == 0.0 and v_roll == 0.0 and v_pitch == 0.0 and v_yaw == 0.0):
                    # publish the message
                    publisher[deviceName + "_odom"].publish(odom)
                    publisher[deviceName + "_inverted_odom"].publish(inv_odom)
                    publisher[deviceName + "_pose"].publish(pose)
                    # Publish the ring position
                    try:
                        (ring_trans,ring_rot) = listener.lookupTransform("vive_world", "ring" , time)
                        odom.pose.pose = Pose(Point(ring_trans[0],ring_trans[1],ring_trans[2]), Quaternion(qx, qy, qz, qw))
                        publisher["ring_odom"].publish(odom)
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue

        rate.sleep()


if __name__ == '__main__':
    try:
        vive_tracker()
    except rospy.ROSInterruptException:
        pass
