#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy
import math

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
    plt.plot(index,data_opt_mani[10],label='only manipulability')
    plt.plot(index,data_opt_joint[10],label='only joint limits')
    plt.plot(index,data_opt_lianxu[10],label='mani and joint limits')
    plt.plot(index,data_no_opt[10],label='no optimization')
    plt.plot(index,upper,label='upper bound')
    plt.plot(index,lower,label='lower bound')
    plt.title('joint 11')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,2)
    plt.plot(index,data_opt_mani[9],label='only manipulability')
    plt.plot(index,data_opt_joint[9],label='only joint limits')
    plt.plot(index,data_opt_lianxu[9],label='mani and joint limits')
    plt.plot(index,data_no_opt[9],label='no optimization')
    plt.plot(index,upper,label='upper bound')
    plt.plot(index,lower,label='lower bound')
    plt.title('joint 10')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,3)
    plt.plot(index,data_opt_mani[8],label='only manipulability')
    plt.plot(index,data_opt_joint[8],label='only joint limits')
    plt.plot(index,data_opt_lianxu[8],label='mani and joint limits')
    plt.plot(index,data_no_opt[8],label='no optimization')
    plt.plot(index,upper,label='upper bound')
    plt.plot(index,lower,label='lower bound')
    plt.title('joint 9')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,4)
    plt.plot(index,data_opt_mani[7],label='only manipulability')
    plt.plot(index,data_opt_joint[7],label='only joint limits')
    plt.plot(index,data_opt_lianxu[7],label='mani and joint limits')
    plt.plot(index,data_no_opt[7],label='no optimization')
    plt.plot(index,upper,label='upper bound')
    plt.plot(index,lower,label='lower bound')
    plt.title('joint 8')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,5)
    plt.plot(index,data_opt_mani[6],label='only manipulability')
    plt.plot(index,data_opt_joint[6],label='only joint limits')
    plt.plot(index,data_opt_lianxu[6],label='mani and joint limits')
    plt.plot(index,data_no_opt[6],label='no optimization')
    plt.plot(index,upper,label='upper bound')
    plt.plot(index,lower,label='lower bound')
    plt.title('joint 7')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,6)
    plt.plot(index,data_opt_mani[5],label='only manipulability')
    plt.plot(index,data_opt_joint[5],label='only joint limits')
    plt.plot(index,data_opt_lianxu[5],label='mani and joint limits')
    plt.plot(index,data_no_opt[5],label='no optimization')
    plt.plot(index,upper,label='upper bound')
    plt.plot(index,lower,label='lower bound')
    plt.title('joint 6')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    plt.show()

    plt.figure(2)
    plt.subplot(2,3,1)
    plt.plot(index,data_opt_mani[4],label='only manipulability')
    plt.plot(index,data_opt_joint[4],label='only joint limits')
    plt.plot(index,data_opt_lianxu[4],label='mani and joint limits')
    plt.plot(index,data_no_opt[4],label='no optimization')
    plt.plot(index,upper,label='upper bound')
    plt.plot(index,lower,label='lower bound')
    plt.title('joint 5')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    upper_3=[2*math.pi for i in range(len(index))]
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
    lower_0=[-math.pi/9 for i in range(len(index))]
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
    plt.title('different joint limit function')
    plt.xlabel('index')
    plt.ylabel('joint limit function')
    plt.legend(loc='upper left')

    plt.show()

if __name__ == '__main__':
    #main_joint_values()
    main_2()