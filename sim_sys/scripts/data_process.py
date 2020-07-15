#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from sim_sys.quaternion_euler import *
import math

rospy.init_node('data_process')

dt=0.01

#trajectory output file dir
mbx_file_path = "/home/chen/ws_chen/src/hilsys/sim_sys/data/mbx.txt"
fwx_file_path = '/home/chen/ws_chen/src/hilsys/sim_sys/data/fwx.txt'

#clear the output file
with open(mbx_file_path,'w') as f:
    f.write('this is the wx pose.'+'\r\n')

with open(fwx_file_path,'w') as f:
    f.write('this is the fwx joint values'+'\r\n')

#input trajectory file dir
input_file_path='/home/chen/ws_chen/src/hilsys/sim_sys/data/mixed/planned_joint_path_opt_both.txt'

joints_values=dict()
for i in range(11):
    joints_values[i]=list()

with open(input_file_path,'r') as f:
    for line in f.readlines():
        data=list(map(float,line.split()))
        for i in range(11):
            joints_values[i].append(data[i])

with open(mbx_file_path,'a') as f:
    for i in range(len(joints_values[0])):
        wx_pose=Pose()
        wx_pose.position.x=-(joints_values[1][i])/2 #this means the x position of the car
        wx_pose.position.y=-joints_values[2][i]/2 #this means the y position of the car
        #wx_pose.position.z=0.81+0.25*math.tan(joints_values[0][i])
        wx_pose.position.z=0.91

        wx_pose.orientation=euler_to_quaternion(-math.pi/2+joints_values[0][i],0,0)

        f.write(str(i*dt)+' '+str(wx_pose.position.x)+' '+str(wx_pose.position.y)+' '+str(wx_pose.position.z)+' '+str(wx_pose.orientation.w)+\
            ' '+str(wx_pose.orientation.x)+' '+str(wx_pose.orientation.y)+' '+str(wx_pose.orientation.z)+'\r\n')

with open(fwx_file_path,'a') as f:
    for i in range(len(joints_values[0])):
        f.write(str(i*dt)+' '+str((joints_values[1][i])/2)+' '+str(joints_values[2][i]/2)+' '+str(joints_values[3][i])+' '+str(joints_values[4][i])+\
            ' '+str(joints_values[5][i])+' '+str(joints_values[6][i])+' '+str(joints_values[7][i])+' '+str(joints_values[8][i])+' '+str(joints_values[9][i])+\
            ' '+str(joints_values[10][i])+'\r\n')

print('data processing finished!!')
