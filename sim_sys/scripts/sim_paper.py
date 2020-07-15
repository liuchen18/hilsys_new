#!/usr/bin/env python 

import sys
import rospy
from parallel_car.Optimizer import SimpleOptimizer
from parallel_car.IKSolver import SerialIKSolver
from parallel_car.Driver import BaseAndMechDriver
from parallel_car.PrintFunc import print_tf
from iiwa_agv.Executor import Executor
from geometry_msgs.msg import Transform, Vector3, Quaternion,TransformStamped
from threading import Thread
import tf2_ros
import tf

# either 'rviz' or 'gazebo'
RUN_ENV = 'gazebo'

def init_pose(ob):
    # go to initial pose
    ob.init_pose()

def send_path(ob):
    ob.send_trajectory()

def send_path_one_by_one(ob):
    ob.send_trajectory_one_by_one(0.1)

def tf_listener(tf_buf,start_time):
    rate=rospy.Rate(50)

    while rospy.get_time()-start_time<20 and not rospy.is_shutdown():
        try:
            transform_stamped = tf_buf.lookup_transform('wx_virtual', 'link_7', rospy.Time())
            with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mixed/tf.txt','a') as f:
                v=transform_stamped.transform.translation
                q=transform_stamped.transform.rotation
                with open(output_file_path,'a') as f:
                    f.write(str(rospy.get_time()-start_time)+' '+str(v.x)+' '+str(v.y)+' '+str(v.z)+' '+str(q.x)+' '+str(q.y)+' '+str(q.z)+' '+str(q.w)+'\r\n')
                '''
                print('current time: '+str(rospy.get_time()-start_time))
                print('translation:')
                print(str(v.x)+' '+str(v.y)+' '+str(v.z))
                print('rotation')
                print(str(q.x)+' '+str(q.y)+' '+str(q.z)+' '+str(q.w))    
                '''      

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Transform lookup exception")
            print(rospy.get_time())
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("start_sim")

    seri_ik = SerialIKSolver(run_env=RUN_ENV)

    #trajectory 1
    mbx_file_path = "/home/chen/ws_chen/src/hilsys/sim_sys/data/mbx.txt"
    fwx_file_path = '/home/chen/ws_chen/src/hilsys/sim_sys/data/fwx.txt'
    output_file_path='/home/chen/ws_chen/src/hilsys/sim_sys/data/real_data.txt'

    #trajectory 2
    #mbx_file_path = "/home/chen/ws_chen/src/hilsys/sim_sys/data/mbx_planned_trajectory_zheng.txt"
    #fwx_file_path = '/home/chen/ws_chen/src/hilsys/sim_sys/data/fwx_planned_trajectory_zheng.txt'

    #trajectory 3
    #mbx_file_path = "/home/chen/ws_chen/src/hilsys/sim_sys/data/mbx_planned_trajectory_cehou.txt"
    #fwx_file_path = '/home/chen/ws_chen/src/hilsys/sim_sys/data/fwx_planned_trajectory_cehou.txt'
    
    driver = BaseAndMechDriver(file_path=mbx_file_path)
    exe = Executor(file_path=fwx_file_path)


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
    '''
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    start_time=rospy.get_time()
    try:
        tf_thratd=Thread(target=tf_listener,args=(tfBuffer,start_time,))
        tf_thratd.start()
        tf_thratd.join()
        
    except:
        rospy.logerr('unable to start new thread, plz try again')
        exit()
    '''
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

    #add tf listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    raw_input("Enter to start the simulation")
    with open(output_file_path,'w') as f:
        f.write('this is the real data. link_7 w r t wx_virtual'+'\r\n') 
    start_time=rospy.get_time()

    #send the trajectory to the robot
    
    try:
        fwx_thread=Thread(target=send_path,args=(exe,))
        mbx_thread=Thread(target=send_path,args=(driver,))
        tf_thratd=Thread(target=tf_listener,args=(tfBuffer,start_time,))
        fwx_thread.start()
        mbx_thread.start()
        tf_thratd.start()
        fwx_thread.join()
        mbx_thread.join()
        tf_thratd.join()
        
    except:
        rospy.logerr('unable to start new thread, plz try again')
        exit()
    
    