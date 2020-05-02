#! /usr/bin/env python  

import sys

import rospy
from parallel_car.Optimizer import SimpleOptimizer
from parallel_car.IKSolver import SerialIKSolver
from parallel_car.Driver import BaseAndMechDriver
from parallel_car.PrintFunc import print_tf
from geometry_msgs.msg import Transform, Vector3, Quaternion

# either 'rviz' or 'gazebo'
RUN_ENV = 'gazebo'

def go_to_gazebo_target():
    while not rospy.is_shutdown():

        try:
            # uncomment raw_input if you want to control the pace of sending goals
            raw_input()
        except EOFError:
            print "Manual Ending"
            sys.exit()
        
        # get the transform from origin to wx_link
        (o_to_wx_succ, o_to_wx_tf) = seri_ik.get_transform(origin, "wx_link")

        if o_to_wx_succ:
            # from o_to_wx_tf get disired parallel pose and serial pose
            (parallel_pose_desired, serial_pose_desired) = seri_ik.compute_ik_from_o_to_wx_tf(o_to_wx_tf)
        
        # go to desired pose by driver
        driver.send_trajectory_from_controller(parallel_pose_desired, serial_pose_desired)

def auto_move_one_by_one():

    driver.read_trajectory()

    o_to_wx_tf_list = driver.get_o_to_wx_tf_list()

    for tf_idx in range(len(o_to_wx_tf_list)):
        # get a tf from o_to_wx_tf_list
        o_to_wx_tf = o_to_wx_tf_list[tf_idx]
        # compute the parallel pose and serial pose from modified matrix
        (parallel_pose_desired, serial_pose_desired) = seri_ik.compute_ik_from_o_to_wx_tf(o_to_wx_tf)
        # drive store both parallel pose and serial pose inside
        driver.append_pose_desired(parallel_pose_desired, serial_pose_desired)

    # go to initial pose
    driver.init_pose()

    raw_input("Enter to go to resting poses")

    # send the trajectory point one by one
    driver.send_trajectory_one_by_one()

def auto_move():

    driver.read_trajectory()

    o_to_wx_tf_list = driver.get_o_to_wx_tf_list()

    for tf_idx in range(len(o_to_wx_tf_list)):
        # get a tf from o_to_wx_tf_list
        o_to_wx_tf = o_to_wx_tf_list[tf_idx]
        # compute the parallel pose and serial pose from modified matrix
        (parallel_pose_desired, serial_pose_desired) = seri_ik.compute_ik_from_o_to_wx_tf(o_to_wx_tf)
        # drive store both parallel pose and serial pose inside
        driver.append_pose_desired(parallel_pose_desired, serial_pose_desired)

    # go to initial pose
    driver.init_pose()

    raw_input("Enter to go to resting poses")

    # send the trajectory point one by one
    driver.send_trajectory()

def go_to_specified_target():
        
    while not rospy.is_shutdown():
    
        # construct a tf from origin to wx_link manually
        o_to_wx_tf = Transform()

        # get the translation in string
        vec3_str = raw_input("Give me translation in x, y, z order: ")

        # map sting to list
        vec3 = map(float,vec3_str.split())

        o_to_wx_tf.translation = Vector3(vec3[0], vec3[1], vec3[2])

        # get the rotation in string
        quat_str = raw_input("Give me rotation in qx, qy, qz, qw order: ")

        # map string to list
        quat = map(float, quat_str.split())

        o_to_wx_tf.rotation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        
        # print o_to_wx_tf target
        print "From command line"
        print_tf(origin, 'wx_link', o_to_wx_tf)
        
        # from o_to_wx_tf get disired parallel pose and serial pose
        (parallel_pose_desired, serial_pose_desired) = seri_ik.compute_ik_from_o_to_wx_tf(o_to_wx_tf)

        print "parallel_pose_desired"
        print parallel_pose_desired

        print "serial_pose_desired"
        print serial_pose_desired
        
        # go to desired pose by driver
        driver.send_trajectory_from_controller(parallel_pose_desired, serial_pose_desired)

        # get the transform from origin to wx_link to see whether wx has gone to target pose
        (o_to_wx_succ, o_to_wx_tf_fact) = seri_ik.get_transform(origin, "wx_link")

        if o_to_wx_succ:
            print "In Fact"
            print_tf(origin, 'wx_link', o_to_wx_tf_fact)

def go_to_specified_line_in_file():

    driver.read_trajectory()

    o_to_wx_tf_list = driver.get_o_to_wx_tf_list()
        
    while not rospy.is_shutdown():

        try:
            # get the line number in string
            line_str = raw_input("Line: ")
        except EOFError:
            print "Manual ending"
            break

        # transfer string into int
        line_num = int(line_str)
    
        # construct a tf from origin to wx_link manually
        o_to_wx_tf = o_to_wx_tf_list[line_num-2]

        # print o_to_wx_tf target
        print "From File"
        print_tf(origin, 'wx_link', o_to_wx_tf)
        
        # from o_to_wx_tf get disired parallel pose and serial pose
        (parallel_pose_desired, serial_pose_desired) = seri_ik.compute_ik_from_o_to_wx_tf(o_to_wx_tf)
        
        # go to desired pose by driver
        driver.send_trajectory_from_controller(parallel_pose_desired, serial_pose_desired)

        # get the transform from origin to wx_link to see whether wx has gone to target pose
        (o_to_wx_succ, o_to_wx_tf_fact) = seri_ik.get_transform(origin, "wx_link")

        if o_to_wx_succ:
            print "In Fact"
            print_tf(origin, 'wx_link', o_to_wx_tf_fact)

if __name__ == "__main__":
    rospy.init_node("auto_controller")

    seri_ik = SerialIKSolver(run_env=RUN_ENV)

    mbx_file_path = "/home/chen/ws_chen/src/hilsys/sim_sys//data/mbx_planned_trajectory_he.txt"

    driver = BaseAndMechDriver(file_path=mbx_file_path)

    listened_to_fixed = False

    rate = rospy.Rate(1.0)

    if RUN_ENV == 'rviz':
        origin = 'car_link'
    else: # RUN_ENV == 'gazebo'
        origin = 'world'

    # wait until having successfully listened to fixed tf
    while not listened_to_fixed:
        if seri_ik.listen_to_fixed_tf():
            rospy.loginfo("listend to fixed tf successfully")
            listened_to_fixed = True
        else:
            rospy.logerr("listening to fixed failed")
        rate.sleep()
    
    # go_to_gazebo_target()

    # go_to_specified_target()

    # go_to_specified_line_in_file()

    # auto_move_one_by_one()

    auto_move()