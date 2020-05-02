#!/usr/bin/env python 

import rospy

import numpy as np

from parallel_car.TransRotGen import quaternion_to_rotation_matrix, vector3_to_translation_matrix, Rotation

class GradientOptimizer:
    """
    A class to perform gradient optimization for both car movement and wx rotation
    """
    def __init__(self, car_weight=1.0, wx_weight=1.0, pole_weight=1.0, car_rate=1.0, wx_rate=1.0):
        """Construction function for gradient optimizer
        
        Keyword Arguments:
            car_weight {float} -- car (mobile base) weight in cost function, applied to car's cost considering translation in x, y and rotation in yaw (default: {1.0})
            wx_weight {float} -- wx weight in cost function, applied to wx's cost considering rotation between wx_link and up_link (default: {1.0})
            pole_weight {float} -- pole weight in cost function, applied to poles' cost considering pole length (default: {1.0})
            car_rate {float} -- [description] (default: {1.0})
            wx_rate {float} -- [description] (default: {1.0})
        """
        self._car_weight = car_weight
        self._wx_weight = wx_weight
        self._pole_weight = pole_weight
        self._car_rate = car_rate
        self._wx_rate = wx_rate

    def compute_cost(self, parallel_increment, pole_length_list, old_pole_length_list):
        """A function to compute the cost of a potential pose of the parallel_car
        
        Arguments:
            parallel_increment {ParallelPose} -- the desired incremental position of the parallel car
            pole_length_list {list} -- the length of the poles at the desired pose of the parallel mechanism 
            old_pole_length_list {list} -- the length of the poles at the previous namingly optimized pose of the parallel mechanism
        
        Returns:
            [double] -- the total cost for the desired incremental pose of the parallel_car
        """
        delta_x = parallel_increment.x
        delta_y = parallel_increment.y
        delta_theta = parallel_increment.theta
        delta_alpha = parallel_increment.alpha

        car_cost = np.square(delta_x)+np.square(delta_y)+np.square(delta_theta)
        car_cost *= self._car_weight

        wx_cost = np.square(delta_alpha)
        wx_cost *= self._wx_weight

        pole_cost = 0.0
        for idx in range(0, len(pole_length_list)):
            pole_cost += np.square(pole_length_list[idx]-old_pole_length_list[idx])
        pole_cost *= self._pole_weight

        total_cost = car_cost+wx_cost+pole_cost

        return total_cost

class SimpleOptimizer:
    def __init__(self):
        pass

    def compute_optimal_alpha(self, o_to_wx_tf):
        """A function to compute the optimal alpha from specified wx pose

        Arguments:
            o_to_wx_tf {Transform} -- the transformation from origin to wx_link

        Returns:
            [tuple] -- a tuple containing the optimal alpha and the homogeneous transformation matrix neutralizing the optimal alpha
        """

        # alpha to rotate z-axis of wx_link to let y-axis of wx_link maintain level
        optimal_alpha = 0.0
        
        T_wx_rot = quaternion_to_rotation_matrix(o_to_wx_tf.rotation)
        T_wx_trans = vector3_to_translation_matrix(o_to_wx_tf.translation)

        # actually gamma in z-y-z pattern euler rotation
        alpha1 = np.arctan2(T_wx_rot[2, 1], T_wx_rot[2, 0])

        alpha2 = np.arctan2(-T_wx_rot[2, 1], -T_wx_rot[2, 0])

        # rospy.loginfo("alpha1 = {}, alpha2 = {}".format(alpha1, alpha2))

        # i_global_z is z component of the representation of x-axis
        # alpha will be chosen if i_global_z < 0
        i_global_z1 = T_wx_rot[2, 0]*np.cos(alpha1) + T_wx_rot[2, 1]*np.sin(alpha1)

        i_global_z2 = T_wx_rot[2, 0]*np.cos(alpha2) + T_wx_rot[2, 1]*np.sin(alpha2)

        # rospy.loginfo("i_global_z1 = {}, i_global_z2 = {}".format(i_global_z1, i_global_z2))

        if i_global_z1 < 0:
            optimal_alpha = alpha1
        else: # i_global_x2 < 0
            optimal_alpha = alpha2

        rospy.loginfo("optimal_alpha = {}".format(optimal_alpha))

        T_wx_rot_modified = T_wx_rot * Rotation('z', optimal_alpha)

        T_o_to_wx_modified = T_wx_trans * T_wx_rot * Rotation('z', optimal_alpha)

        j_standard = np.mat(np.array([0, 1, 0, 1]).reshape((-1, 1)))

        # rospy.loginfo("After rotation about z-axis {}, y-axis in global is".format(optimal_alpha))
        # print T_wx_rot_modified * j_standard
        

        return (optimal_alpha, T_o_to_wx_modified)