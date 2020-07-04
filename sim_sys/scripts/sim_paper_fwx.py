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

def send_path_one_by_one(ob):
    ob.send_trajectory_one_by_one(0.1)


if __name__ == "__main__":
    rospy.init_node("start_sim")

    #trajectory 1
    fwx_file_path = '/home/chen/ws_chen/src/hilsys/sim_sys/data/fwx.txt'
    exe = Executor(file_path=fwx_file_path)


    listened_to_fixed = False

    rate = rospy.Rate(1.0)

    if RUN_ENV == 'rviz':
        origin = 'car_link'
    else: # RUN_ENV == 'gazebo'
        origin = 'world'

    while not exe.listen_to_initial_offset():
        rospy.logerr("Waiting for listening to initial offset")
        rate.sleep()
    rospy.loginfo("Have successfully listened to initial offset")

        # read trajectory from the txt file
    exe.read_trajectory()

    try:
        fwx_init_thread=Thread(target=init_pose,args=(exe,))
        fwx_init_thread.start()
        fwx_init_thread.join()
    except:
        rospy.logerr('unable to start new thread, plz try again')
        exit()
    raw_input('press inter to start simulation')

    try:
        fwx_sim_thread=Thread(target=send_path,args=(exe,))
        fwx_sim_thread.start()
        fwx_sim_thread.join()
    except:
        rospy.logerr('unable to start new thread, plz try again')
        exit()
    