#!/usr/bin/env python  
import roslib
import rospy
import tf


# This node adjusts the vive_world frame, such that all child frames are
# shifted so that the tracked device is at the origin.
# i.e. Everything moves so that Vive Tracker is moved to 0,0,0
if __name__ == '__main__':
    rospy.init_node('vive_tracker_reset')

    listener = tf.TransformListener()

    tf_count = 0
    zero_tf = listener.lookupTransform('vive_world', 'vive_world', rospy.Time(0))
    br = tf.TransformBroadcaster() 
    topic_names = rospy.get_published_topics("/vive/")
    tracker_name = ""
    # Get the first tracked device that isn't a reference frame.
    for name in topic_names:
        if "LHB" not in name[0]:
            tracker_name = name[0].replace('/vive/','').replace('_odom','')
            print(tracker_name)
            break
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        if tracker_name is "":
            continue
        try:
            (trans,rot) = listener.lookupTransform('vive_world', tracker_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
 
        if tf_count < 10:
            (trans,rot) = listener.lookupTransform('vive_world', tracker_name, rospy.Time(0))
            br.sendTransform(trans, rot, rospy.Time.now(),'zero_location','vive_world')
            inv_trans = trans[:]
            inv_rot = rot[:]
            #rospy.loginfo('Inverse transform found.')
            tf_count += 1
        rate.sleep()
        break
    inv_trans[0] = -trans[0]
    inv_trans[1] = -trans[1]
    inv_trans[2] = -trans[2]
    inv_rot[0] = -rot[0]
    inv_rot[1] = -rot[1]
    inv_rot[2] = -rot[2]
    inv_rot[3] = -rot[3]

    br.sendTransform(inv_trans, [0,0,0,1], rospy.Time.now(),'vive_world','vive_world_orientation')
    while  not rospy.is_shutdown():
        br.sendTransform(trans, rot, rospy.Time.now(),'zero_location','vive_world')
    #rospy.signal_shutdown("Tracker reset to origin.")
