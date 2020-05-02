#!/usr/bin/env python

import sys

import rospy
import tf2_ros

if __name__ == "__main__":

    arg_num = len(sys.argv) - 1

    if arg_num < 2:
        print "Only {} arguments. Need 2".format(arg_num)
        sys.exit()
    else:
        print "Have {} argements. Going on".format(arg_num)

    print "two args are {} and {}".format(sys.argv[1], sys.argv[2])

    rospy.init_node("tf_echo")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(1.0)

    transform_stamped = None

    source_link = sys.argv[1]
    target_link = sys.argv[2]
    
    while not rospy.is_shutdown():

        try:
            transform_stamped = tfBuffer.lookup_transform(source_link, target_link, rospy.Time())
            trans = transform_stamped.transform.translation
            rot = transform_stamped.transform.rotation
            rospy.loginfo("Transform from {} to {} is".format(source_link, target_link))
            print "- Translation: [{}, {}, {}]".format(trans.x, trans.y, trans.z)
            print "- Rotation: in Quaternion [{}, {}, {}, {}]".format(rot.x, rot.y, rot.z, rot.w)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to lookup transform from {} to {}".format(source_link, target_link))
        
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            print "Manual shut down"
            break