#!/usr/bin/env python

import rospy
import tf2_ros


tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

transform_stamped = tfBuffer.lookup_transform('/parallel/wx', '/iiwa/link_7', rospy.Time())