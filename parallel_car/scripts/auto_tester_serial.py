#!/usr/bin/env python  

import rospy
from parallel_car.Optimizer import GradientOptimizer, SimpleOptimizer
from parallel_car.IKSolver import ParallelPose ,ParallelIKSolver, SerialIKSolver
from parallel_car.TransRotGen import quaternion_to_rotation_matrix, vector3_to_translation_matrix, transform_to_matrix, yaw_from_quaternion_only_Z
from geometry_msgs.msg import Pose, Point

# either 'rviz' or 'gazebo'
RUN_ENV = 'rviz'

if __name__ == "__main__":
    rospy.init_node("auto_controller")

    seri_ik = SerialIKSolver(run_env=RUN_ENV)

    listened_to_fixed = False

    rate = rospy.Rate(1.0)

    if RUN_ENV == 'rviz':
        origin = 'car_link'
    else: # RUN_ENV == 'gazebo'
        origin = 'odom'

    while not rospy.is_shutdown():

        if not listened_to_fixed: # have not listend to fixed tf
            if seri_ik.listen_to_fixed_tf():
                listened_to_fixed = True
            else:
                rospy.logerr("listening to fixed tf failed")
        else:
            # listen to tf from down_link to up_link
            listen_to_tf_succ = seri_ik.listen_to_tf()
            if listen_to_tf_succ:
                rospy.loginfo("listen to down_link-up_link transform successfully!")
            else:
                rospy.logerr("listening to down_link-up_link transform failed.")

            parallel_pose_desired = ParallelPose()

            # get transfrom from origin to down_link
            (o_to_down_succ, o_to_down_tf) = seri_ik.get_transform(origin, "down_link")
            if o_to_down_succ:
                parallel_pose_desired.x = o_to_down_tf.translation.x
                parallel_pose_desired.y = o_to_down_tf.translation.y
                parallel_pose_desired.theta = yaw_from_quaternion_only_Z(o_to_down_tf.rotation)
            else:
                rospy.logerr("listening to {}-down_link transform failed.".format(origin))

            # get transform from addon_Tilt_link to wx_link
            (addon_Tilt_to_wx_succ, addon_Tilt_to_wx_tf) = seri_ik.get_transform("addon_Tilt_link", "wx_link")
            if addon_Tilt_to_wx_succ:
                parallel_pose_desired.alpha = yaw_from_quaternion_only_Z(addon_Tilt_to_wx_tf.rotation)
            else:
                rospy.logerr("listening to up_link-wx_link transform failed.")

            # get transform from origin to wx_link
            (o_to_wx_succ, o_to_wx_tf) = seri_ik.get_transform(origin, "wx_link")
            if o_to_wx_succ:
                wx_pose = Pose()
                wx_pose.position.x = o_to_wx_tf.translation.x
                wx_pose.position.y = o_to_wx_tf.translation.y
                wx_pose.position.z = o_to_wx_tf.translation.z
                wx_pose.orientation = o_to_wx_tf.rotation

            if listen_to_tf_succ and o_to_down_succ and addon_Tilt_to_wx_succ and o_to_wx_succ:
                # seri_ik.print_T_down_to_up()
                seri_ik.compute_ik_from_inherent()
                seri_ik.compute_ik_from_target(parallel_pose_desired, wx_pose)
         
        rate.sleep()