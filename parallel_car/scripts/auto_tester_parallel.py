#!/usr/bin/env python  

import rospy
from parallel_car.Optimizer import GradientOptimizer
from parallel_car.IKSolver import ParallelPose ,ParallelIKSolver
from parallel_car.TransRotGen import quaternion_to_rotation_matrix, vector3_to_translation_matrix, transform_to_matrix, quaternion_to_euler
from geometry_msgs.msg import Pose, Point
from time import sleep

# either 'rviz' or 'gazebo'
RUN_ENV = 'rviz'

if __name__ == "__main__":
    rospy.init_node("auto_tester")

    para_ik = ParallelIKSolver(run_env=RUN_ENV)

    optimizer = GradientOptimizer()

    rate = rospy.Rate(1.0)

    listened_to_fixed = False

    first_time_listen = True

    if RUN_ENV == 'rviz':
        origin = 'car_link'
    else: # RUN_ENV == 'gazebo'
        origin = 'odom'

    while not rospy.is_shutdown():
        if not listened_to_fixed: # have not listned to fixed tf
            if para_ik.listen_to_up_down_fixed_tf():
                listened_to_fixed = True
            else:
                rospy.logerr("listening to fixed tf failed")
        else: # listned_to_fixed is True
            # listen to tf from down_num to up_num
            listen_to_tf_succ = para_ik.listen_to_tf()
            if listen_to_tf_succ:
                rospy.loginfo("listen to down_i-up_i transform successfully!")
                # calculate the num from listened tf, this should be 100 percent correct
                para_ik.calculate_pole_length_from_inherent()
            else:
                rospy.logerr("listening to down_i-up_i transform failed.")
                
            parallel_pose_desired = ParallelPose()

            # get transfrom from origin to down_link
            (o_to_down_succ, o_to_down_tf) = para_ik.get_transform(origin, "down_link")
            if o_to_down_succ:
                parallel_pose_desired.x = o_to_down_tf.translation.x
                parallel_pose_desired.y = o_to_down_tf.translation.y
                parallel_pose_desired.theta = quaternion_to_euler(o_to_down_tf.rotation)[0]
            else:
                rospy.logerr("listening to {}-down_link transform failed.".format(origin))

            # get transform from up_link to wx_link
            (up_to_wx_succ, up_to_wx_tf) = para_ik.get_transform("up_link", "wx_link")
            if up_to_wx_succ:
                parallel_pose_desired.alpha = quaternion_to_euler(up_to_wx_tf.rotation)[0]
            else:
                rospy.logerr("listening to up_link-wx_link transform failed.")
            
            # get transform from origin to wx_link
            (o_to_wx_succ, o_to_wx_tf) = para_ik.get_transform(origin, "wx_link")
            if o_to_wx_succ:
                wx_pose = Pose()
                wx_pose.position.x = o_to_wx_tf.translation.x
                wx_pose.position.y = o_to_wx_tf.translation.y
                wx_pose.position.z = o_to_wx_tf.translation.z
                wx_pose.orientation = o_to_wx_tf.rotation
            
            if listen_to_tf_succ and o_to_down_succ and up_to_wx_succ and o_to_wx_succ:
                if first_time_listen:
                    original_pole_length_list = para_ik.get_pole_length_list()
                    first_time_listen = False
                para_ik.print_pole_length()
                (pole_length_list_from_target, T_down_num_to_up_num_from_target_list) = para_ik.calculate_pole_length_from_target(parallel_pose_desired, wx_pose)
                cost = optimizer.compute_cost(ParallelPose(), para_ik.get_pole_length_list(), original_pole_length_list)
                rospy.loginfo("Current cost is {}".format(cost))
            
        rate.sleep()
            