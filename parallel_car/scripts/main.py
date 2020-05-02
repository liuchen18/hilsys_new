#!/usr/bin/env python  
import rospy
from parallel_car.IKSolver import ParallelPose ,ParallelIKSolver
from parallel_car.TransRotGen import quaternion_to_rotation_matrix, vector3_to_translation_matrix
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    rospy.init_node('iksolver')

    para_ik = ParallelIKSolver()

    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        if para_ik.listen_to_tf():
            # must be correct here
            correct_trans_matrix = vector3_to_translation_matrix(para_ik._transform_list[0].translation)
            correct_rot_matrix = quaternion_to_rotation_matrix(para_ik._transform_list[0].rotation)
            correct_matrix = correct_trans_matrix*correct_rot_matrix
            rospy.loginfo("Correct matrix is ")
            print correct_matrix
            para_ik.calculate_pole_length_from_inherent()
            para_ik.print_pole_length()
        if para_ik.listen_to_up_down_fixed_tf():
            parallel_pose_desired = ParallelPose()
            wx_pose = Pose()
            wx_pose.position.z = 1.255
            para_ik.calculate_pole_length_from_target(parallel_pose_desired, wx_pose)
        rate.sleep()
