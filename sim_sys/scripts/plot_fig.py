#!/usr/bin/env python

import matplotlib.pyplot as plt
import rospy

def main():
    rospy.init_node('plot_fig')
    data_opt=dict()
    data_no_opt=dict()
    index=list()
    idx=0
    for i in range(13):
        data_opt[i]=list()
        data_no_opt[i]=list()
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/planned_joint_path_opt.txt','r') as f:
        for line in f.readlines():
            index.append(idx)
            data=list(map(float,line.split()))
            for i in range(13):
                data_opt[i].append(data[i])
            idx+=1
    idx=0
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/planned_joint_path_no_opt.txt','r') as f:
        for line in f.readlines():
            data=list(map(float,line.split()))
            for i in range(13):
                data_no_opt[i].append(data[i])
    
    plt.figure(12)
    plt.subplot(2,3,1)
    plt.plot(index,data_opt[11],label='opt')
    plt.plot(index,data_no_opt[11],label='no opt')
    plt.title('different manipulability')
    plt.xlabel('index')
    plt.ylabel('manipulability')
    plt.legend(loc='upper left')

    plt.subplot(2,3,2)
    plt.plot(index,data_opt[9],label='opt')
    plt.plot(index,data_no_opt[9],label='no opt')
    plt.title('joint 10')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,3)
    plt.plot(index,data_opt[7],label='opt')
    plt.plot(index,data_no_opt[7],label='no opt')
    plt.title('joint 8')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,4)
    plt.plot(index,data_opt[3],label='opt')
    plt.plot(index,data_no_opt[3],label='no opt')
    plt.title('joint 4')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,5)
    plt.plot(index,data_opt[5],label='opt')
    plt.plot(index,data_no_opt[5],label='no opt')
    plt.title('joint 6')
    plt.xlabel('index')
    plt.ylabel('joint value')
    plt.legend(loc='upper left')

    plt.subplot(2,3,6)
    plt.plot(index,data_opt[12],label='opt')
    plt.plot(index,data_no_opt[12],label='no opt')
    plt.title('joint coffe')
    plt.xlabel('index')
    plt.ylabel('joint')
    plt.legend(loc='upper left')

    plt.show()


if __name__ == '__main__':
    main()