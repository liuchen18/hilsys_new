#!/usr/bin/env python

import rospy


def print_tf(source_link, target_link, transform):
    trans = transform.translation
    rot = transform.rotation
    rospy.loginfo("Transform from {} to {} is".format(source_link, target_link))
    print "- Translation: [{}, {}, {}]".format(trans.x, trans.y, trans.z)
    print "- Rotation: in Quaternion [{}, {}, {}, {}]".format(rot.x, rot.y, rot.z, rot.w)