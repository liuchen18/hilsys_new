#!/usr/bin/env python

import rospy
import math
import moveit_commander
from moveit_commander import MoveGroupCommander
import numpy as np
from moveit_msgs.srv import GetPositionIK,GetPositionIKRequest,GetPositionIKResponse
from gazebo_msgs.srv import GetLinkState,GetLinkStateRequest,GetLinkStateResponse
from geometry_msgs.msg import Pose
import numpy.matlib
import eigenpy
from geometry_msgs.msg import Twist
import PyKDL as kdl
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import Pose2D

class manipulator():
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "sim_sys"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)


    def compute_jacobian_matrix(self,joint_value):
        '''
        compute the jacobian matrix of the given joint value,the given joint value should be (1,6) array
        '''
        if len(joint_value) < 6:
            r=[]
            return []
        else:
            jacobian_matrix_m=self.group.get_jacobian_matrix(joint_value)
            jacobian_matrix=np.asarray(jacobian_matrix_m)
            return jacobian_matrix

    def compute_manipulability(self,joint_value):
        '''
        compute the manipulability of the given end_effector_pose,return manipulability
        '''
        jacobian_matrix=self.compute_jacobian_matrix(joint_value)
        if jacobian_matrix != []:
            manipulability=math.sqrt(np.linalg.det(np.dot(jacobian_matrix,np.transpose(jacobian_matrix))))
        else:
            manipulability=0
        return manipulability

def main():
    rospy.init_node('mani_test')
    mani=manipulator()
    joint_value=[-0.174533,1.64433,0.583764, -0.115503, 0.0798634, 1.04467, -0.101824, 0.0251186, -0.0999235, -1.03462, 0.138063]

    manipulability=mani.compute_manipulability(joint_value)

    print(manipulability)


if __name__ == '__main__':
    main()
