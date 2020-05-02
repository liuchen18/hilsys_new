#!/usr/bin/env python
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import math

def quaternion_to_euler(given_oriantation):
    '''
    transfor the given orientation into the ruler angle,return R,P,Y
    '''
    R=math.atan2(2*(given_oriantation.w*given_oriantation.x+given_oriantation.y*given_oriantation.z),1-2*(math.pow(given_oriantation.x,2)+math.pow(given_oriantation.y,2)))
    P=math.asin(2*(given_oriantation.w*given_oriantation.y-given_oriantation.x*given_oriantation.z))
    Y=math.atan2(2*(given_oriantation.w*given_oriantation.z+given_oriantation.x*given_oriantation.y),1-2*(math.pow(given_oriantation.y,2)+math.pow(given_oriantation.z,2)))
    return R,P,Y

def euler_to_quaternion(R,P,Y):
    '''
    transfor the given ruler angle to orientation ,return orientation
    '''
    p=Pose()
    p.orientation.w=math.cos(R/2)*math.cos(P/2)*math.cos(Y/2)+math.sin(R/2)*math.sin(P/2)*math.sin(Y/2)
    p.orientation.x=math.sin(R/2)*math.cos(P/2)*math.cos(Y/2)-math.cos(R/2)*math.sin(P/2)*math.sin(Y/2)
    p.orientation.y=math.cos(R/2)*math.sin(P/2)*math.cos(Y/2)+math.sin(R/2)*math.cos(P/2)*math.sin(Y/2)
    p.orientation.z=math.cos(R/2)*math.cos(P/2)*math.sin(Y/2)-math.sin(R/2)*math.sin(P/2)*math.cos(Y/2)
    return p.orientation

def interpolation(start_orientation,end_orientation,t):
    '''compute the current orientation using Spherical linear interpolation method. return a orientation
    '''
    cosa = start_orientation.x*end_orientation.x + start_orientation.y*end_orientation.y + start_orientation.z*end_orientation.z + start_orientation.w*end_orientation.w
    if cosa < 0:
       end_o=-end_orientation
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
        

def main():
    '''
    with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mbx_planned_trajectory_nutation.txt','r') as f:
        for line in f.readlines()[1:10]:
            data=list(map(float,line.split()))
            cur_pose=Pose()
            cur_pose.orientation.w=data[4]
            cur_pose.orientation.x=data[5]
            cur_pose.orientation.y=data[6]
            cur_pose.orientation.z=data[7]
            R,P,Y=quaternion_to_euler(cur_pose.orientation)
            print('R: '+str(R)+' P: '+str(P)+' Y: '+str(Y))
            print(data[4])
    
    cur_pose=Pose()
    cur_pose.orientation.w=0.707
    cur_pose.orientation.x=0.707
    cur_pose.orientation.y=0
    cur_pose.orientation.z=0.0
    R,P,Y=quaternion_to_euler(cur_pose.orientation)
    print('R: '+str(R)+' P: '+str(P)+' Y: '+str(Y))
    
    o=euler_to_quaternion(math.pi/2,0,0)
    print(str(o.w)+' '+str(o.x)+' '+str(o.y)+' '+str(o.z))
    '''
    res=Pose()
    start_orientation=Quaternion(0,0,0.707,0.707)
    print(str(start_orientation.w)+' '+str(start_orientation.x)+' '+str(start_orientation.y)+' '+str(start_orientation.z))
    end_orientation=Quaternion(0,0.707,0,0.707)
    print(str(end_orientation.w)+' '+str(end_orientation.x)+' '+str(end_orientation.y)+' '+str(end_orientation.z))
    res.orientation=interpolation(start_orientation,end_orientation,0.5)
    print(str(res.orientation.w)+' '+str(res.orientation.x)+' '+str(res.orientation.y)+' '+str(res.orientation.z))

if __name__ == '__main__':
    main()