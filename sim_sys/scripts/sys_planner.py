#!/usr/bin/env python
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionFKResponse
from moveit_msgs.msg import RobotState, RobotTrajectory
import numpy as np
import numpy.matlib
from geometry_msgs.msg import Pose,Quaternion
import math
import copy



class sys_class():
    def __init__(self):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "sim_sys"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self.compute_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
        self.group.allow_replanning=True
        self.group.set_pose_reference_frame('wx')
        self.group.set_end_effector_link('pan_link')
        self.group.set_goal_position_tolerance(0.001)
        self.group.set_goal_orientation_tolerance(0.01)
        self.group.set_max_velocity_scaling_factor(1)
    
    def compute_inverse_kinematics(self,end_effector_pose):
        '''
        compute the inverse kinematics of the given end effector pose,return joint values,end_effector_pose should be a pose
        '''
        request=GetPositionIKRequest()
        request.ik_request.group_name=self.group_name
        request.ik_request.ik_link_name = "pan_link"
        request.ik_request.attempts = 100
        request.ik_request.pose_stamped.header.frame_id = "wx"
        request.ik_request.pose_stamped.pose.position.x = end_effector_pose.position.x
        request.ik_request.pose_stamped.pose.position.y = end_effector_pose.position.y
        request.ik_request.pose_stamped.pose.position.z = end_effector_pose.position.z
        request.ik_request.pose_stamped.pose.orientation.x = end_effector_pose.orientation.x
        request.ik_request.pose_stamped.pose.orientation.y = end_effector_pose.orientation.y
        request.ik_request.pose_stamped.pose.orientation.z = end_effector_pose.orientation.z
        request.ik_request.pose_stamped.pose.orientation.w = end_effector_pose.orientation.w
        ik_response=self.compute_ik(request)
        #print(ik_response)
        joint_value=ik_response.solution.joint_state.position
        joint_values=[]
        if len(joint_value) < 10:
            rospy.logerr('the given end_effector_pose has no results')
            return joint_values
        else:
            for i in range(len(joint_value)):
                joint_values.append(joint_value[i])
            #print(joint_value)
            return joint_values
    
    def compute_forward_kinematics(self,joint_values,goal_link):
        '''
        compute the forward kinematics of the given joint values with reference to the reference link, return a posestamped
        '''
        fk_request=GetPositionFKRequest()
        links=self.robot.get_link_names()
        fk_request.fk_link_names=links
        state=RobotState()
        joint_names=['wx_agv2_1','agv2_virx','agv2_viry','agv2_virz','joint_a1','joint_a2','joint_a3','joint_a4','joint_a5','joint_a6','joint_a7']
        state.joint_state.name=joint_names
        state.joint_state.position=joint_values
        fk_request.robot_state=state
        fk_response=self.compute_fk(fk_request)
        index=fk_response.fk_link_names.index(goal_link)
        end_effector_pose=fk_response.pose_stamped[index]
        return end_effector_pose

    
    def compute_jacobian_matrix(self,joint_value):
        '''
        compute the jacobian matrix of the given joint value,the given joint value should be (1,6) array
        '''
        if len(joint_value) < 6:
            r=[]
            return []
        else:
            jacobian_matrix_m=numpy.matlib.zeros((6,6))
            jacobian_matrix_m=self.group.get_jacobian_matrix(joint_value)
            jacobian_matrix=np.asarray(jacobian_matrix_m)
            return jacobian_matrix

    def compute_cartisian_trajectory(self,way_points):
        '''compute the trajectory according to the compute cartisian path api.inputs are waypoints
        '''
        trajectory=RobotTrajectory()
        attempts=0
        max_attempts=1000
        fraction=0
        while fraction<1 and attempts<max_attempts:
            trajectory,fraction=self.group.compute_cartesian_path(way_points,0.01,0)
            attempts+=1
            if attempts % 20 == 0:
                rospy.loginfo('still trying to solve the path after %d appempts',attempts)
        if fraction == 1:
            rospy.loginfo('the cartisian trajectory computed')
            return trajectory
        else:
            rospy.logerr('cannot solve the path 100%,only' +str(fraction*100)+'% planned')
            exit()







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




class trajectory_class():
    def __init__(self):
        self.omega=5.0/180*math.pi
        self.period=10
    
    def desired_trajectory(self,time,type=1,period=10): #period is the interpolation time
        '''
        generate the desired trajectory, return a pose. the function can be replaced by a trajectory from the simulator 
        '''
        desired_pose=Pose()
        if type ==1: # hou
            desired_pose.position.x=0
            desired_pose.position.y=-5+time*0.15
            desired_pose.position.z=0.0
            desired_pose.orientation.x=0
            desired_pose.orientation.y=0.707
            desired_pose.orientation.z=0
            desired_pose.orientation.w=0.707
            return desired_pose
        elif type == 2:
            if time <10:
                desired_pose.position.x=5-time*0.3
                desired_pose.position.y=0
                desired_pose.position.z=0.0
                desired_pose.orientation.x=0
                desired_pose.orientation.y=0
                desired_pose.orientation.z=0.707
                desired_pose.orientation.w=0.707
                return desired_pose
            else:
                desired_pose.position.x=2*math.cos(2*math.pi/40*(time-10))
                desired_pose.position.y=-2*math.sin(2*math.pi/40*(time-10))
                desired_pose.position.z=0.0

                #interpolation of the quaternion:Spherical linear interpolation
                start_orientation=Quaternion(0,0,0.707,0.707)  #ce
                end_orientation=Quaternion(0,0.707,0,0.707)   #hou
                if time <= period +10:
                    desired_pose.orientation=self.interpolation(start_orientation,end_orientation,(time-10)/period)
                    #print(str(desired_pose.orientation.w)+' '+str(desired_pose.orientation.x)+' '+str(desired_pose.orientation.y)+' '+str(desired_pose.orientation.z))
                    return desired_pose
                else:
                    rospy.logerr('input time should be less than period + 10 s, current period is '+str(period))
        elif type == 3:
            if time <10:
                desired_pose.position.x=4*math.cos(2*math.pi/40*(time))
                desired_pose.position.y=4*math.sin(2*math.pi/40*(time))
                desired_pose.position.z=0.0

                #interpolation of the quaternion:Spherical linear interpolation
                start_orientation=Quaternion(0,0,0.707,0.707)  #ce
                end_orientation=Quaternion(0,0,1,6e-17)   #qian
                if time <= period:
                    desired_pose.orientation=self.interpolation(start_orientation,end_orientation,time/period)
                    #print(str(desired_pose.orientation.w)+' '+str(desired_pose.orientation.x)+' '+str(desired_pose.orientation.y)+' '+str(desired_pose.orientation.z))
                    return desired_pose
            else:
                desired_pose.position.x=0
                desired_pose.position.y=4-0.2*(time-10)
                desired_pose.position.z=0.0
                desired_pose.orientation.x=0
                desired_pose.orientation.y=0
                desired_pose.orientation.z=1
                desired_pose.orientation.w=6e-17
                return desired_pose
    
    def interpolation(self,start_orientation,end_orientation,t):
        '''compute the current orientation using Spherical linear interpolation method. return a Quaternion
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
        


    def nutation(self,time):
        '''
        according omega,period and time , this function generate the nutation of the wx.retrun a Pose
        '''
        R=self.omega*math.sin(2*math.pi/self.period*time)
        P=self.omega*math.cos(2*math.pi/self.period*time)
        Y=0
        quaternion=euler_to_quaternion(R,P,Y)
        nutation_pose=Pose()
        nutation_pose.orientation=quaternion

        return nutation_pose

    def desired_trajectory_point(self):
        '''generate a pose, to test it
        '''
        desired_pose=Pose()
        desired_pose.position.x=3
        desired_pose.position.y=0
        desired_pose.position.z=0.0
        desired_pose.orientation.x=0
        desired_pose.orientation.y=0
        desired_pose.orientation.z=1
        desired_pose.orientation.w=6e-17
        return desired_pose



class planner_class():
    def __init__(self):
        self.trajectory=trajectory_class()
        self.sys=sys_class()
        self.debug=False
        self.time_duration=0.2 # add a point from the desired trajectory every time_duration
        self.total_time=20.0 # total time of the trajectory

        # init the txt files
        with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mbx_planned_trajectory.txt','w') as f:
            f.write('this is the MBX pose,the info is time,positionx,positiony,positionz,orientationw,orientationx,orientationy,orientationz'+'\r\n')
        with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/fwx_planned_trajectory.txt','w') as f:
            f.write('this is the FWX joint values,the following info is time,x,y,theta,joint1,joint2,joint3,joint4,joint5,joint6,joint7'+'\r\n')
        with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mbx_planned_trajectory_no_n.txt','w') as f:
            f.write('this is the origin planned trajectory without nutation'+'\r\n')
        #with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mbx_planned_trajectory_nutation.txt','w') as f:
        #    f.write('this is the nutation of wx, only'+'\r\n')

    
    def write_mbx_info(self,time,pose,planned_pose,nutation_pose):
        with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mbx_planned_trajectory.txt','a') as f:
            f.write(str(time)+' '+str(pose.position.x)+' '+str(pose.position.y)+' '+str(pose.position.z)+' '+str(pose.orientation.w)+\
            ' '+str(pose.orientation.x)+' '+str(pose.orientation.y)+' '+str(pose.orientation.z)+'\r\n')
        with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mbx_planned_trajectory_no_n.txt','a') as f:
            f.write(str(time)+' '+str(pose.position.x)+' '+str(pose.position.y)+' '+str(pose.position.z)+' '+str(planned_pose.w)+\
            ' '+str(planned_pose.x)+' '+str(planned_pose.y)+' '+str(planned_pose.z)+'\r\n')
        #with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/mbx_planned_trajectory_nutation.txt','a') as f:
        #    f.write(str(time)+' '+str(nutation_pose.position.x)+' '+str(nutation_pose.position.y)+' '+str(nutation_pose.position.z)+' '+str(nutation_pose.orientation.w)+\
        #    ' '+str(nutation_pose.orientation.x)+' '+str(nutation_pose.orientation.y)+' '+str(nutation_pose.orientation.z)+'\r\n')
    
    def write_fwx_info(self,time,joint_value):
        with open('/home/chen/ws_chen/src/hilsys/sim_sys/data/fwx_planned_trajectory.txt','a') as f:
            f.write(str(time)+' '+str((3+joint_value[1])/2)+' '+str(joint_value[2]/2)+' '+str(joint_value[3])+' '+str(joint_value[4])+\
            ' '+str(joint_value[5])+' '+str(joint_value[6])+' '+str(joint_value[7])+' '+str(joint_value[8])+' '+str(joint_value[9])+\
            ' '+str(joint_value[10])+'\r\n')

    def add_nutation(self,joint_values,time):
        '''
        this function generate the movement of wx with nutation according to the joint values and nutation function. return time and pose
        '''
        nutation_pose=self.trajectory.nutation(time)
        nutation_pose.position.z=1.36
        wx_pose=Pose()
        wx_pose.position.x=-(joint_values[1]+3)/2 #this means the x position of the car
        wx_pose.position.y=-joint_values[2]/2 #this means the y position of the car
        wx_pose.position.z=1.36 #this z value is NOT acurate. need to be confermed
        R,P,Y=quaternion_to_euler(nutation_pose.orientation)
        planned_pose=euler_to_quaternion(-math.pi/2+joint_values[0],0,0)

        # this is the multiply of the quaternion
        wx_pose.orientation.w=nutation_pose.orientation.w*planned_pose.w-nutation_pose.orientation.x*planned_pose.x-nutation_pose.orientation.y*planned_pose.y-nutation_pose.orientation.z*planned_pose.z
        wx_pose.orientation.x=nutation_pose.orientation.w*planned_pose.x+nutation_pose.orientation.x*planned_pose.w+nutation_pose.orientation.y*planned_pose.z-nutation_pose.orientation.z*planned_pose.y
        wx_pose.orientation.y=nutation_pose.orientation.w*planned_pose.y-nutation_pose.orientation.x*planned_pose.z+nutation_pose.orientation.y*planned_pose.w+nutation_pose.orientation.z*planned_pose.x
        wx_pose.orientation.z=nutation_pose.orientation.w*planned_pose.z+nutation_pose.orientation.x*planned_pose.y-nutation_pose.orientation.y*planned_pose.x+nutation_pose.orientation.z*planned_pose.w
        #wx_pose.orientation=euler_to_quaternion(R,P,Y)
        return wx_pose,planned_pose,nutation_pose
    
    def plan(self):
        '''get point from trajectory_class and compute the whole path. the result will be witten into txt file
        '''
        count=0
        way_points=[]
        rospy.loginfo('generate way points of the path')
        while count*self.time_duration < self.total_time:
            desired_pose=self.trajectory.desired_trajectory(count*self.time_duration,3,self.total_time-10) #second param is trajectory type
            way_points.append(copy.deepcopy(desired_pose))
            count+=1
        
        rospy.loginfo('start planner')
        trajectory=self.sys.compute_cartisian_trajectory(way_points)
        #print(trajectory)
        print('got '+str(len(trajectory.joint_trajectory.points))+'points totally')

        #find the joint values that the end effector reaches the first given point
        first_point_num=0
        init_pose=self.trajectory.desired_trajectory(0,3)
        total_error=float('inf')
        goal_link='pan_link'
        while total_error > 0.001 or total_error < -0.001:
            computed_end_effector_posestamped=self.sys.compute_forward_kinematics(trajectory.joint_trajectory.points[first_point_num].positions,goal_link)
            computed_end_effector_pose=computed_end_effector_posestamped.pose
            total_error=init_pose.position.x-computed_end_effector_pose.position.x+init_pose.position.y-computed_end_effector_pose.position.y+init_pose.position.z-computed_end_effector_pose.position.z
            first_point_num+=1
        
        #according to the firet point num, recompute time 
        temp_total_time=trajectory.joint_trajectory.points[-1].time_from_start.to_sec()-trajectory.joint_trajectory.points[first_point_num].time_from_start.to_sec()
        print('origin total time: '+str(temp_total_time))
        print('first_point_num: '+str(first_point_num))
        for i in range(first_point_num,len(trajectory.joint_trajectory.points)):
            cur_point=trajectory.joint_trajectory.points[i]
            time_from_start=cur_point.time_from_start.to_sec()-trajectory.joint_trajectory.points[first_point_num].time_from_start.to_sec()
            new_time=time_from_start*self.total_time/temp_total_time
            self.write_fwx_info(new_time,cur_point.positions)

            #compute the mbx trajectory
            wx_pose,planned_pose,nutation_pose=self.add_nutation(cur_point.positions,new_time)
            self.write_mbx_info(new_time,wx_pose,planned_pose,nutation_pose)
        rospy.loginfo('trajectory generated and writen into txt file')
        #for i in range(10):
        #    print(trajectory.joint_trajectory.points[i])

    def plan_given_pose(self,desired_pose):
        '''plan a given point to test it
        '''
        way_point=[desired_pose]
        trajectory=self.sys.compute_cartisian_trajectory(way_point)
        



def main():

    DEBUG=False

    rospy.init_node('sys_planner')
    while rospy.get_time()==0:
        continue
    planner=planner_class()
    if DEBUG:
        end_pose=Pose()
        end_pose.position.x=3
        end_pose.position.y=0
        end_pose.position.z=0.5
        end_pose.orientation.x=0.5
        end_pose.orientation.y=0.5
        end_pose.orientation.z=0.5
        end_pose.orientation.w=0.5
        joint_values=planner.sys.compute_inverse_kinematics(end_pose)
        print(joint_values)
        goal_link='agv_link'
        e_pose=planner.sys.compute_forward_kinematics(joint_values,goal_link)
        print(e_pose)

    planner.plan()
    #desired_pose=planner.trajectory.desired_trajectory_point()
    #planner.plan_given_pose(desired_pose)



if __name__ == '__main__':
    main()
    #debug_main()
    