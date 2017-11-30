#!/usr/bin/env python  
import roslib
import rospy
import tf


# This node adjusts the vive_world frame, such that all child frames are
# shifted so that the tracked device is at the origin.
# i.e. Everything moves so that Vive Tracker is moved to 0,0,0
if __name__ == '__main__':
    rospy.init_node('vive_tracker_goal')

    listener = tf.TransformListener()

    tf_count = 0
    goal_tf = listener.lookupTransform('vive_world', 'vive_world', rospy.Time(0))
    br = tf.TransformBroadcaster() 
    topic_names = rospy.get_published_topics("/vive/")
    tracker_name = ""
    base_station_frame = ""
    # Get the first tracked device that isn't a reference frame.
    for name in topic_names:
        if "LHB" not in name[0]:
            tracker_name = name[0].replace('/vive/','').replace('_odom','')
            print(tracker_name)
        elif "LHB-D3CC0B26" not in name[0]:
            base_station_frame = name[0].replace('/vive','')
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        time = rospy.Time(0)
        if tracker_name is "":
            continue
        try:
            (trans,rot) = listener.lookupTransform(base_station_frame, tracker_name, time)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
 
        if tf_count < 10:
            (trans,rot) = listener.lookupTransform(base_station_frame, tracker_name, time)
            #br.sendTransform(trans, rot, time,'goal_location',base_station_frame)
            inv_trans = trans[:]
            #rospy.loginfo('Inverse transform found.')
            tf_count += 1
        rate.sleep()
        break
    while  not rospy.is_shutdown():
        br.sendTransform(trans, rot, rospy.Time.now(),'goal_location',base_station_frame)
    #rospy.signal_shutdown("Tracker reset to origin.")
