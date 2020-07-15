#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy
import math
from geometry_msgs.msg import Quaternion


def interpolation(start_orientation,end_orientation,t):
    '''compute the current orientation using Spherical linear interpolation method. return a Quaternion
    '''
    cosa = start_orientation.x*end_orientation.x + start_orientation.y*end_orientation.y + start_orientation.z*end_orientation.z + start_orientation.w*end_orientation.w
    end_o=Quaternion()
    if cosa < 0:
        end_o.x= -end_orientation.x
        end_o.y= -end_orientation.y
        end_o.z= -end_orientation.z
        end_o.w= -end_orientation.w
        cosa = -cosa
    else:
        end_o=end_orientation
    start_o=start_orientation
    k0=0.0
    k1=0.0

    #If the inputs are too close for comfort, linearly interpolate
    if cosa >0.999:
        k0=1.0-t
        k1=t
    else:
        sina=math.sqrt(1.0-cosa*cosa)
        a=math.atan2(sina,cosa)
        k0=math.sin((1.0-t)*a)/sina
        k1=math.sin(t*a)/sina

    res=Quaternion()
    res.x=start_o.x*k0+end_o.x*k1
    res.y=start_o.y*k0+end_o.y*k1
    res.z=start_o.z*k0+end_o.z*k1
    res.w=start_o.w*k0+end_o.w*k1
    return res

def main_joint_values():
    rospy.init_node('plot_fig')
    data_opt_mani=dict()
    data_no_opt=dict()
    data_opt_joint=dict()
    data_opt_jiange=dict()
    data_opt_lianxu=dict()
    index=list()
    idx=0
    for i in range(13):
        data_opt_mani[i]=list()
        data_opt_joint[i]=list()
        data_opt_jiange[i]=list()
        data_opt_lianxu[i]=list()
        data_no_opt[i]=list()
    
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mixed/planned_joint_path_opt_mani.txt','r') as f:
        for line in f.readlines():
            data=list(map(float,line.split()))
            for i in range(13):
                data_opt_mani[i].append(data[i])
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mixed/planned_joint_path_opt_joint.txt','r') as f:
        for line in f.readlines():
            data=list(map(float,line.split()))
            for i in range(13):
                data_opt_joint[i].append(data[i])
    
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mixed/planned_joint_path_opt_no.txt','r') as f:
        for line in f.readlines():
            index.append(idx)
            idx+=1
            data=list(map(float,line.split()))
            for i in range(13):
                data_no_opt[i].append(data[i])
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mixed/planned_joint_path_opt_both.txt','r') as f:
        for line in f.readlines():
            data=list(map(float,line.split()))
            for i in range(13):
                data_opt_lianxu[i].append(data[i])
    upper=[math.pi for i in range(len(index))]
    lower=[-math.pi for i in range(len(index))]

    plt.figure(1)
    plt.subplot(2,3,1)
    upper_10=[170.0/180*math.pi for i in range(len(index))]
    lower_10=[-170.0/180*math.pi for i in range(len(index))]
    plt.plot(index,data_opt_mani[10],label='only manipulability')
    plt.plot(index,data_opt_joint[10],label='only joint limits')
    plt.plot(index,data_opt_lianxu[10],label='mani and joint limits')
    plt.plot(index,data_no_opt[10],label='no optimization')
    plt.plot(index,upper_10,label='upper bound')
    plt.plot(index,lower_10,label='lower bound')
    plt.title('joint 11')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,2)
    upper_9=[120.0/180*math.pi for i in range(len(index))]
    lower_9=[-120.0/180*math.pi for i in range(len(index))]
    plt.plot(index,data_opt_mani[9],label='only manipulability')
    plt.plot(index,data_opt_joint[9],label='only joint limits')
    plt.plot(index,data_opt_lianxu[9],label='mani and joint limits')
    plt.plot(index,data_no_opt[9],label='no optimization')
    plt.plot(index,upper_9,label='upper bound')
    plt.plot(index,lower_9,label='lower bound')
    plt.title('joint 10')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,3)
    upper_8=[170.0/180*math.pi for i in range(len(index))]
    lower_8=[-170.0/180*math.pi for i in range(len(index))]
    plt.plot(index,data_opt_mani[8],label='only manipulability')
    plt.plot(index,data_opt_joint[8],label='only joint limits')
    plt.plot(index,data_opt_lianxu[8],label='mani and joint limits')
    plt.plot(index,data_no_opt[8],label='no optimization')
    plt.plot(index,upper_8,label='upper bound')
    plt.plot(index,lower_8,label='lower bound')
    plt.title('joint 9')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,4)
    upper_7=[120.0/180*math.pi for i in range(len(index))]
    lower_7=[-120.0/180*math.pi for i in range(len(index))]
    plt.plot(index,data_opt_mani[7],label='only manipulability')
    plt.plot(index,data_opt_joint[7],label='only joint limits')
    plt.plot(index,data_opt_lianxu[7],label='mani and joint limits')
    plt.plot(index,data_no_opt[7],label='no optimization')
    plt.plot(index,upper_7,label='upper bound')
    plt.plot(index,lower_7,label='lower bound')
    plt.title('joint 8')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,5)
    upper_6=[170.0/180*math.pi for i in range(len(index))]
    lower_6=[-170.0/180*math.pi for i in range(len(index))]
    plt.plot(index,data_opt_mani[6],label='only manipulability')
    plt.plot(index,data_opt_joint[6],label='only joint limits')
    plt.plot(index,data_opt_lianxu[6],label='mani and joint limits')
    plt.plot(index,data_no_opt[6],label='no optimization')
    plt.plot(index,upper_6,label='upper bound')
    plt.plot(index,lower_6,label='lower bound')
    plt.title('joint 7')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,6)
    upper_5=[120.0/180*math.pi for i in range(len(index))]
    lower_5=[-120.0/180*math.pi for i in range(len(index))]
    plt.plot(index,data_opt_mani[5],label='only manipulability')
    plt.plot(index,data_opt_joint[5],label='only joint limits')
    plt.plot(index,data_opt_lianxu[5],label='mani and joint limits')
    plt.plot(index,data_no_opt[5],label='no optimization')
    plt.plot(index,upper_5,label='upper bound')
    plt.plot(index,lower_5,label='lower bound')
    plt.title('joint 6')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    plt.show()

    plt.figure(2)

    upper_4=[170.0/180*math.pi for i in range(len(index))]
    lower_4=[-170.0/180*math.pi for i in range(len(index))]
    plt.subplot(2,3,1)
    plt.plot(index,data_opt_mani[4],label='only manipulability')
    plt.plot(index,data_opt_joint[4],label='only joint limits')
    plt.plot(index,data_opt_lianxu[4],label='mani and joint limits')
    plt.plot(index,data_no_opt[4],label='no optimization')
    plt.plot(index,upper_4,label='upper bound')
    plt.plot(index,lower_4,label='lower bound')
    plt.title('joint 5')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    upper_3=[math.pi*2 for i in range(len(index))]
    lower_3=[-math.pi*2 for i in range(len(index))]
    plt.subplot(2,3,2)
    plt.plot(index,data_opt_mani[3],label='only manipulability')
    plt.plot(index,data_opt_joint[3],label='only joint limits')
    plt.plot(index,data_opt_lianxu[3],label='mani and joint limits')
    plt.plot(index,data_no_opt[3],label='no optimization')
    plt.plot(index,upper_3,label='upper bound')
    plt.plot(index,lower_3,label='lower bound')
    plt.title('joint 4')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    upper_2=[8 for i in range(len(index))]
    lower_2=[-8 for i in range(len(index))]
    plt.subplot(2,3,3)
    plt.plot(index,data_opt_mani[2],label='only manipulability')
    plt.plot(index,data_opt_joint[2],label='only joint limits')
    plt.plot(index,data_opt_lianxu[2],label='mani and joint limits')
    plt.plot(index,data_no_opt[2],label='no optimization')
    plt.plot(index,upper_2,label='upper bound')
    plt.plot(index,lower_2,label='lower bound')
    plt.title('joint 3')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    upper_1=[8 for i in range(len(index))]
    lower_1=[-8 for i in range(len(index))]
    plt.subplot(2,3,4)
    plt.plot(index,data_opt_mani[1],label='only manipulability')
    plt.plot(index,data_opt_joint[1],label='only joint limits')
    plt.plot(index,data_opt_lianxu[1],label='mani and joint limits')
    plt.plot(index,data_no_opt[1],label='no optimization')
    plt.plot(index,upper_1,label='upper bound')
    plt.plot(index,lower_1,label='lower bound')
    plt.title('joint 2')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    upper_0=[2*math.pi/9 for i in range(len(index))]
    lower_0=[-math.pi*1.5/18 for i in range(len(index))]
    plt.subplot(2,3,5)
    plt.plot(index,data_opt_mani[0],label='only manipulability')
    plt.plot(index,data_opt_joint[0],label='only joint limits')
    plt.plot(index,data_opt_lianxu[0],label='mani and joint limits')
    plt.plot(index,data_no_opt[0],label='no optimization')
    plt.plot(index,upper_0,label='upper bound')
    plt.plot(index,lower_0,label='lower bound')
    plt.title('joint 1')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')


    plt.show()

def main_2():
    rospy.init_node('plot_fig')
    data_opt_mani=dict()
    data_no_opt=dict()
    data_opt_joint=dict()
    data_opt_jiange=dict()
    data_opt_lianxu=dict()
    index=list()
    idx=0
    for i in range(13):
        data_opt_mani[i]=list()
        data_opt_joint[i]=list()
        data_opt_jiange[i]=list()
        data_opt_lianxu[i]=list()
        data_no_opt[i]=list()
    
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mixed/planned_joint_path_opt_mani.txt','r') as f:
        for line in f.readlines():
            data=list(map(float,line.split()))
            for i in range(13):
                data_opt_mani[i].append(data[i])
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mixed/planned_joint_path_opt_joint.txt','r') as f:
        for line in f.readlines():
            data=list(map(float,line.split()))
            for i in range(13):
                data_opt_joint[i].append(data[i])
    
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mixed/planned_joint_path_opt_no.txt','r') as f:
        for line in f.readlines():
            index.append(idx)
            idx+=1
            data=list(map(float,line.split()))
            for i in range(13):
                data_no_opt[i].append(data[i])
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mixed/planned_joint_path_opt_both.txt','r') as f:
        for line in f.readlines():
            data=list(map(float,line.split()))
            for i in range(13):
                data_opt_lianxu[i].append(data[i])
    
    
    plt.figure(1)
    plt.subplot(1,1,1)
    plt.plot(index,data_opt_mani[11],label='only manipulability')
    plt.plot(index,data_opt_joint[11],label='only joint limits')
    plt.plot(index,data_opt_lianxu[11],label='mani and joint limits')
    plt.plot(index,data_no_opt[11],label='no optimization')
    plt.title('different manipulability')
    plt.xlabel('index')
    plt.ylabel('manipulability')
    plt.legend(loc='upper left')

    plt.show()

    plt.figure(2)
    plt.subplot(1,1,1)
    plt.plot(index,data_opt_mani[12],label='only manipulability')
    plt.plot(index,data_opt_joint[12],label='only joint limits')
    plt.plot(index,data_opt_lianxu[12],label='mani and joint limits')
    plt.plot(index,data_no_opt[12],label='no optimization')
    plt.ylim([0,10])
    plt.title('different joint limit function')
    plt.xlabel('index')
    plt.ylabel('joint limit function')
    plt.legend(loc='upper left')

    plt.show()

def main_accurate():
    rospy.init_node('plot_fig')
    data_opt_mani=dict()
    data_no_opt=dict()
    data_opt_joint=dict()
    data_opt_jiange=dict()
    data_opt_lianxu=dict()
    index=list()
    idx=0
    for i in range(7):
        data_opt_mani[i]=list()
        data_opt_joint[i]=list()
        data_opt_jiange[i]=list()
        data_opt_lianxu[i]=list()
        data_no_opt[i]=list()
    
    
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mixed/planned_joint_path_opt_no.txt','r') as f:
        for line in f.readlines():
            index.append(idx)
            idx+=0.01
            data=list(map(float,line.split()))
            for i in range(7):
                data_no_opt[i].append(data[13+i])
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mixed/planned_joint_path_opt_both.txt','r') as f:
        for line in f.readlines():
            data=list(map(float,line.split()))
            for i in range(7):
                data_opt_lianxu[i].append(data[13+i])
    
    d_index=list()
    data_desired=dict()
    for i in range(7):
        data_desired[i]=list()
    id=0
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mixed/desired_path_points.txt','r') as f:
        for line in f.readlines():
            d_index.append(id)
            id+=1
            data=list(map(float,line.split()))
            for i in range(7):
                data_desired[i].append(data[i])

    time=list()
    data_actual=dict()
    for i in range(7):
        data_actual[i]=list()
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/real_data.txt','r') as f:
        for line in f.readlines()[1:]:
            data=list(map(float,line.split()))
            time.append(data[0])
            for i in range(7):
                data_actual[i].append(data[i+1])
        for i in range(len(data_actual[0])):
            data_actual[1][i]-=0.3
            data_actual[2][i]*=0.1


    plt.figure(1)
    plt.subplot(2,3,2)
    plt.plot(index,data_opt_lianxu[1],label='mani and joint limits')
    plt.plot(index,data_no_opt[1],label='no optimization')
    plt.plot(time,data_actual[1],label='actual')
    #plt.plot(d_index,data_desired[1],label='desired')
    plt.scatter(d_index, data_desired[1], s=20, c="#ff1212", marker='*',label='desired path point')
    plt.title('distance in y direction')
    plt.xlabel('time/s')
    plt.ylabel('value/m')
    plt.legend(loc='upper left')

    plt.subplot(2,3,1)
    plt.plot(index,data_opt_lianxu[0],label='mani and joint limits')
    plt.plot(index,data_no_opt[0],label='no optimization')
    plt.plot(time,data_actual[0],label='actual')
    plt.scatter(d_index, data_desired[0], s=20, c="#ff1212", marker='*',label='desired path point')
    plt.title('distance in x direction')
    plt.xlabel('time/s')
    plt.ylabel('value/m')
    plt.legend(loc='upper left')

    plt.subplot(2,3,3)
    plt.plot(index,data_opt_lianxu[2],label='mani and joint limits')
    plt.plot(index,data_no_opt[2],label='no optimization')
    plt.plot(time,data_actual[2],label='actual')
    plt.scatter(d_index, data_desired[2], s=20, c="#ff1212", marker='*',label='desired path point')
    plt.title('distance in z direction')
    plt.xlabel('time/s')
    plt.ylabel('value/m')
    plt.legend(loc='upper left')

    plt.subplot(2,3,4)
    plt.plot(index,data_opt_lianxu[3],label='mani and joint limits')
    plt.plot(index,data_no_opt[3],label='no optimization')
    plt.plot(time,data_actual[3],label='actual')
    plt.scatter(d_index, data_desired[3], s=20, c="#ff1212", marker='*',label='desired path point')
    plt.title('x value of the quaternion')
    plt.xlabel('time/s')
    plt.ylabel('value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,5)
    plt.plot(index,data_opt_lianxu[4],label='mani and joint limits')
    plt.plot(index,data_no_opt[4],label='no optimization')
    plt.plot(time,data_actual[4],label='actual')
    plt.scatter(d_index, data_desired[4], s=20, c="#ff1212", marker='*',label='desired path point')
    plt.title('y value of the quaternion')
    plt.xlabel('time/s')
    plt.ylabel('value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,6)
    plt.plot(index,data_opt_lianxu[5],label='mani and joint limits')
    plt.plot(index,data_no_opt[5],label='no optimization')
    plt.plot(time,data_actual[5],label='actual')
    plt.scatter(d_index, data_desired[5], s=20, c="#ff1212", marker='*',label='desired path point')
    plt.title('z value of the quaternion')
    plt.xlabel('time/s')
    plt.ylabel('value')
    plt.legend(loc='upper left')


    plt.show()


def plot_parallel():
    data=dict()
    for i in range(6):
        data[i]=list()
    index=list()
    
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mixed/pole_length.csv','r') as f:
        for line in f.readlines()[1:]:
            data_line=list(map(float,line.split(',')))
            if data_line[0]<20:
                continue
            index.append(data_line[0]-20)
            for i in range(6):
                data[i].append(data_line[i+1]+0.02)
    
    plt.figure(1)
    plt.plot(index,data[0],label='chain 1')
    plt.plot(index,data[1],label='chain 2')
    plt.plot(index,data[2],label='chain 3')
    plt.plot(index,data[3],label='chain 4')
    plt.plot(index,data[4],label='chain 5')
    plt.plot(index,data[5],label='chain 6')
    plt.title('the length of parallel mechanism chain')
    plt.xlabel('time')
    plt.ylabel('length')
    plt.legend(loc='upper left')
    
    plt.show()

def quaternion_to_euler(given_oriantation):
    '''
    transfor the given orientation into the ruler angle,return R,P,Y
    '''
    R=math.atan2(2*(given_oriantation.w*given_oriantation.x+given_oriantation.y*given_oriantation.z),1-2*(math.pow(given_oriantation.x,2)+math.pow(given_oriantation.y,2)))
    P=math.asin(2*(given_oriantation.w*given_oriantation.y-given_oriantation.x*given_oriantation.z))
    Y=math.atan2(2*(given_oriantation.w*given_oriantation.z+given_oriantation.x*given_oriantation.y),1-2*(math.pow(given_oriantation.y,2)+math.pow(given_oriantation.z,2)))
    return R,P,Y


def main_error():
    rospy.init_node('plot_fig')


    time=list()
    data_actual=dict()
    data_desired=dict()
    for i in range(7):
        data_actual[i]=list()
        data_desired[i]=list()
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/real_data.txt','r') as f:
        for line in f.readlines()[1:]:
            data=list(map(float,line.split()))
            time.append(data[0])
            for i in range(7):
                data_actual[i].append(data[i+1])
        for i in range(len(data_actual[0])):
            data_actual[1][i]-=0.3
            data_actual[2][i]*=0.1

    for i in range(500):
        data_desired[0].append(5-0.3*time[i])
        data_desired[1].append(0.2*time[i])
        data_desired[2].append(0)
        data_desired[3].append(0)
        data_desired[4].append(0)
        data_desired[5].append(0.707)
        data_desired[6].append(0.707)

    start_q=Quaternion(0,0,0.707,0.707)
    end_q=Quaternion(0,0,0,1)

    for i in range(500,len(time)):
        data_desired[0].append(2*math.cos((time[i]-10)/20*math.pi))
        data_desired[1].append(2+2*math.sin((time[i]-10)/20*math.pi))
        data_desired[2].append(0)
        q=interpolation(start_q,end_q,(time[i]-10)/10)
        data_desired[3].append(q.x)
        data_desired[4].append(q.y)
        data_desired[5].append(q.z)
        data_desired[6].append(q.w)

    error=dict()
    for i in range(7):
        error[i]=list()

    for i in range(0,len(time),50):
        for j in range(3):
            error[j].append(data_desired[j][i]-data_actual[j][i])

    t=list()
    for i in range(1,21):
        t.append(i)

    plt.figure(2)
    plt.plot(t,error[0],label='error in x direction')
    plt.plot(t,error[1],label='error in y direction')
    plt.plot(t,error[2],label='error in z direction')
    plt.title('translation error')
    plt.xlabel('time/s')
    plt.ylabel('value/m')
    plt.legend(loc='upper left')
    plt.show()

    for i in range(0,len(time),50):
        d_q=Quaternion(data_desired[3][i],data_desired[4][i],data_desired[5][i],data_desired[6][i])
        a_q=Quaternion(data_actual[3][i],data_actual[4][i],data_actual[5][i],data_actual[6][i])
        d_r,d_p,d_y=quaternion_to_euler(d_q)
        a_r,a_p,a_y=quaternion_to_euler(a_q)
        error[3].append(d_r-a_r)
        error[4].append(d_p-a_p)
        error[5].append(d_y-a_y)


    plt.figure(3)
    plt.plot(t,error[3],label='roll error')
    plt.plot(t,error[4],label='pitch error')
    plt.plot(t,error[5],label='yaw error')
    plt.title('ratation error')
    plt.xlabel('time/s')
    plt.ylabel('value/rad')
    plt.legend(loc='upper left')
    plt.show()


    plt.figure(1)
    plt.subplot(2,3,2)
    plt.plot(time,data_desired[1],label='desired y distance')
    plt.plot(time,data_actual[1],label='actual y distance')
    plt.title('distance in y direction')
    plt.xlabel('time/s')
    plt.ylabel('value/m')
    plt.legend(loc='upper left')

    
    plt.subplot(2,3,1)
    plt.plot(time,data_desired[0],label='desired y distance')
    plt.plot(time,data_actual[0],label='actual y distance')
    plt.title('distance in x direction')
    plt.xlabel('time/s')
    plt.ylabel('value/m')
    plt.legend(loc='upper left')

    plt.subplot(2,3,3)
    plt.plot(time,data_desired[2],label='desired y distance')
    plt.plot(time,data_actual[2],label='actual y distance')
    plt.title('distance in z direction')
    plt.xlabel('time/s')
    plt.ylabel('value/m')
    plt.legend(loc='upper left')

    plt.subplot(2,3,4)
    plt.plot(time,data_desired[3],label='desired y distance')
    plt.plot(time,data_actual[3],label='actual y distance')
    plt.title('x value of the quaternion')
    plt.xlabel('time/s')
    plt.ylabel('value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,5)
    plt.plot(time,data_desired[4],label='desired y distance')
    plt.plot(time,data_actual[4],label='actual y distance')
    plt.title('y value of the quaternion')
    plt.xlabel('time/s')
    plt.ylabel('value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,6)
    plt.plot(time,data_desired[5],label='desired y distance')
    plt.plot(time,data_actual[5],label='actual y distance')
    plt.title('z value of the quaternion')
    plt.xlabel('time/s')
    plt.ylabel('value')
    plt.legend(loc='upper left')
    

    plt.show()




if __name__ == '__main__':
    #main_joint_values()
    #main_2()
    #main_accurate()
    #plot_parallel()
    main_error()