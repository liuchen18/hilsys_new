#!/usr/bin/env python 

import sys
import rospy
from parallel_car.Optimizer import SimpleOptimizer
from parallel_car.IKSolver import SerialIKSolver
from parallel_car.Driver import BaseAndMechDriver
from parallel_car.PrintFunc import print_tf
from iiwa_agv.Executor import Executor
from geometry_msgs.msg import Transform, Vector3, Quaternion
from threading import Thread

# either 'rviz' or 'gazebo'
RUN_ENV = 'gazebo'

def init_pose(ob):
    # go to initial pose
    ob.init_pose()

def send_path(ob):
    ob.send_trajectory()


if __name__ == "__main__":
    rospy.init_node("start_sim")

    seri_ik = SerialIKSolver(run_env=RUN_ENV)

    mbx_file_path = "/home/chen/ws_chen/src/hilsys/sim_sys/data/mbx_planned_trajectory_he.txt"

    driver = BaseAndMechDriver(file_path=mbx_file_path)

    exe = Executor(file_path='/home/chen/ws_chen/src/hilsys/sim_sys/data/fwx_planned_trajectory_he.txt')


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
    while not exe.listen_to_initial_offset():
        rospy.logerr("Waiting for listening to initial offset")
        rate.sleep()
    rospy.loginfo("Have successfully listened to initial offset")

        # read trajectory from the txt file
    exe.read_trajectory()
    driver.read_trajectory()

    o_to_wx_tf_list = driver.get_o_to_wx_tf_list()
    for tf_idx in range(len(o_to_wx_tf_list)):
        # get a tf from o_to_wx_tf_list
        o_to_wx_tf = o_to_wx_tf_list[tf_idx]
        # compute the parallel pose and serial pose from modified matrix
        (parallel_pose_desired, serial_pose_desired) = seri_ik.compute_ik_from_o_to_wx_tf(o_to_wx_tf)
        # drive store both parallel pose and serial pose inside
        driver.append_pose_desired(parallel_pose_desired, serial_pose_desired)

    #drive the robot to the init pose
    try:
        fwx_init_thread=Thread(target=init_pose,args=(exe,))
        mbx_init_thread=Thread(target=init_pose,args=(driver,))
        fwx_init_thread.start()
        mbx_init_thread.start()
        fwx_init_thread.join()
        mbx_init_thread.join()
    except:
        rospy.logerr('unable to start new thread, plz try again')
        exit()

    rospy.loginfo('the robots are initialized! ')
    raw_input("Enter to start the simulation")

    #send the trajectory to the robot
    try:
        fwx_thread=Thread(target=send_path,args=(exe,))
        mbx_thread=Thread(target=send_path,args=(driver,))
        fwx_thread.start()
        mbx_thread.start()
        fwx_thread.join()
        mbx_thread.join()
        
    except:
        rospy.logerr('unable to start new thread, plz try again')
        exit()
    