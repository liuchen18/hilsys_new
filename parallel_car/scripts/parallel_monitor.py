#!/usr/bin/env python

import rospy

from parallel_car.IKSolver import ParallelIKSolver

# either 'rviz' or 'gazebo'
RUN_ENV = 'gazebo'

if __name__ == "__main__":
    rospy.init_node("parallel_monitor")

    para_ik = ParallelIKSolver(run_env=RUN_ENV)

    rate_slow = rospy.Rate(1.0)

    rate_fast = rospy.Rate(20.0)

    listened_to_fixed = False

    max_length = 0.0
    min_length = 1.0

    while not listened_to_fixed:
        # listen to fixed tf to initialize
        if para_ik.listen_to_up_down_fixed_tf():
            rospy.loginfo("listend to parallel fixed tf successfully")
            listened_to_fixed = True
        else:
            rospy.logerr("failed to listen to fixed")
        rate_slow.sleep()

    while not rospy.is_shutdown():
        if para_ik.listen_to_tf():
            para_ik.calculate_pole_length_from_inherent()
            para_ik.print_pole_length()
            pole_length_list = para_ik.get_pole_length_list()
            max_length_in_list = max(pole_length_list)
            if max_length_in_list > max_length:
                max_length = max_length_in_list
            min_length_in_list = min(pole_length_list)
            if min_length_in_list < min_length:
                min_length = min_length_in_list
        else:
            rospy.logerr("failed to listen to tf from down_num to up_num")

        try:
            rate_fast.sleep()
        except rospy.ROSInterruptException:
            print "max is {}".format(max_length)
            print "min is {}".format(min_length)