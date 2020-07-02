#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy

def main():
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
    
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/planned_joint_path_opt_mani.txt','r') as f:
        for line in f.readlines():
            index.append(idx)
            data=list(map(float,line.split()))
            for i in range(13):
                data_opt_mani[i].append(data[i])
            idx+=1
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/planned_joint_path_no_opt.txt','r') as f:
        for line in f.readlines():
            data=list(map(float,line.split()))
            for i in range(13):
                data_no_opt[i].append(data[i])
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/planned_joint_path_opt_joint.txt','r') as f:
        for line in f.readlines():
            data=list(map(float,line.split()))
            for i in range(13):
                data_opt_joint[i].append(data[i])
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/planned_joint_path_opt_jiange.txt','r') as f:
        for line in f.readlines():
            data=list(map(float,line.split()))
            for i in range(13):
                data_opt_jiange[i].append(data[i])
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/planned_joint_path_opt_tongshi.txt','r') as f:
        for line in f.readlines():
            data=list(map(float,line.split()))
            for i in range(13):
                data_opt_lianxu[i].append(data[i])
    
    plt.figure(1)
    plt.subplot(1,1,1)
    plt.plot(index,data_opt_mani[11],label='only manipulability')
    plt.plot(index,data_opt_joint[11],label='only joint limits')
    plt.plot(index,data_opt_jiange[11],label='mani or joint limits')
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
    plt.plot(index,data_opt_jiange[12],label='mani or joint limits')
    plt.plot(index,data_opt_lianxu[12],label='mani and joint limits')
    plt.plot(index,data_no_opt[12],label='no optimization')
    plt.title('different joint limit function')
    plt.xlabel('index')
    plt.ylabel('joint limit function')
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
    
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/planned_joint_path_opt_mani.txt','r') as f:
        for line in f.readlines():
            index.append(idx)
            data=list(map(float,line.split()))
            for i in range(13):
                data_opt_mani[i].append(data[i])
            idx+=1
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/planned_joint_path_no_opt.txt','r') as f:
        for line in f.readlines():
            data=list(map(float,line.split()))
            for i in range(13):
                data_no_opt[i].append(data[i])
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/planned_joint_path_opt_joint.txt','r') as f:
        for line in f.readlines():
            data=list(map(float,line.split()))
            for i in range(13):
                data_opt_joint[i].append(data[i])
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/planned_joint_path_opt_tongshi.txt','r') as f:
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
    #main()
    main_2()