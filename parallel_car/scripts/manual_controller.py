#!/usr/bin/env python  

import rospy
from parallel_car.Optimizer import GradientOptimizer, SimpleOptimizer
from parallel_car.IKSolver import ParallelPose ,ParallelIKSolver, SerialIKSolver
from parallel_car.TransRotGen import quaternion_to_rotation_matrix, vector3_to_translation_matrix, transform_to_matrix, yaw_from_quaternion_only_Z
from geometry_msgs.msg import Pose, Point
from parallel_car.Driver import BaseAndMechDriver

# either 'rviz' or 'gazebo'
RUN_ENV = 'gazebo'

if __name__ == "__main__":
    rospy.init_node("auto_controller")

    seri_ik = SerialIKSolver(run_env=RUN_ENV)

    optimizer = SimpleOptimizer()

    driver = BaseAndMechDriver()

    listened_to_fixed = False

    rate = rospy.Rate(1.0)

    if RUN_ENV == 'rviz':
        origin = 'car_link'
    else: # RUN_ENV == 'gazebo'
        origin = 'world'

    while not rospy.is_shutdown():

        if not listened_to_fixed:
            # control the pace
            raw_input()
            if seri_ik.listen_to_fixed_tf():
                rospy.loginfo("listend to fixed tf successfully")
                listened_to_fixed = True
            else:
                rospy.logerr("listening to fixed failed")
        else:
            rospy.loginfo("have listend to fixed tf")

        try:
            # control the pace
            raw_input()
        except EOFError: # catch EOFError
            print "Manual ending"
            break

        (o_to_wx_succ, o_to_wx_tf) = seri_ik.get_transform(origin, "wx_link")
        if o_to_wx_succ:
            rospy.loginfo("Successfully get the transfrom from {} to wx_link".format(origin))
            # look for optimal alpha and modified transformation matrix of wx_link after let its y-axis to level in global coordinate
            (optimal_alpha, T_o_to_wx_modified) = optimizer.compute_optimal_alpha(o_to_wx_tf)
            # from T_o_to_wx_modified get disired parallel pose and serial pose
            (parallel_pose_desired, serial_pose_desired) = seri_ik.compute_ik_from_modified_matrix(T_o_to_wx_modified)
            # need to revert optimal alpha for parallel pose
            parallel_pose_desired.alpha = -optimal_alpha
            
            rospy.loginfo("parallel_pose_desired: ")
            print parallel_pose_desired

            rospy.loginfo("serial_pose_desired: ")
            print serial_pose_desired

            # go to desired pose by driver
            driver.send_trajectory_from_controller(parallel_pose_desired, serial_pose_desired)
        else:
            rospy.logerr("listening to {}-wx_link transform failed.".format(origin))

        
        '''
        try:
            rate.sleep()
        except ROSInterruptException:
            print "end while sleeping"
            break
        '''