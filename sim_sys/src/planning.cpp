//#define DEBUG
//#define TRANSLATE
//#define CIRCLE
#define MIXED

#include "ros/ros.h"
#include <Eigen/Dense>
#include "moveit/robot_state/robot_state.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "geometry_msgs/Pose.h"
#include "quaternion.h"
#include "std_msgs/Bool.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include "inverse_kinematics.h"

const double dt=0.01;
const double T=1.0;

Eigen::Isometry3d pose_to_Isometry(const geometry_msgs::Pose& pose){
    Eigen::Quaterniond q = Eigen::Quaterniond(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z).normalized();
    Eigen::Vector3d t = Eigen::Vector3d(pose.position.x,pose.position.y,pose.position.z);
    Eigen::Isometry3d state = Eigen::Isometry3d::Identity();

    state.rotate(q.toRotationMatrix());
    state.pretranslate(t);
    return state;

}


std::vector<double> generate_way_points(int point_index){
    geometry_msgs::Quaternion start_o,mid_o,end_o;
    initialize_quaternion(start_o,0.0,0.0,0.0,1.0);
    initialize_quaternion(mid_o,0.0,0.0,0.707,0.707);
    initialize_quaternion(end_o,0.0,0.0,1.0,0.0);
    //initialize_quaternion(mid_o,0.0,0.0,0.0,1.0);
    //initialize_quaternion(end_o,0.0,0.0,0.0,1.0);

    std::vector<double> res;
    if(point_index<=10){
        res.emplace_back(5-0.2*point_index);
        res.emplace_back(-2+0.2*point_index);
        res.emplace_back(0);
        geometry_msgs::Quaternion q=slerp(start_o,mid_o,(double)(point_index/10));
        res.push_back(q.x);
        res.push_back(q.y);
        res.push_back(q.z);
        res.push_back(q.w);
        res.emplace_back(-0.2);
        res.emplace_back(0.2);
        res.emplace_back(0);
        res.emplace_back(0);
        res.emplace_back(0);
        res.emplace_back(0);
    }
    else{
        res.emplace_back(3*std::cos(M_PI*(point_index-10)/20.0));
        res.emplace_back(3*std::sin(M_PI*(point_index-10)/20.0));
        res.emplace_back(0);
        geometry_msgs::Quaternion q=slerp(mid_o,end_o,(double)((point_index-10)/10));
        res.emplace_back(q.x);
        res.emplace_back(q.y);
        res.emplace_back(q.z);
        res.emplace_back(q.w);
        res.emplace_back(-M_PI*3/20*std::sin(M_PI*(point_index-10)/20.0));
        res.emplace_back(M_PI*3/20*std::cos(M_PI*(point_index-10)/20.0));
        res.emplace_back(0);
        res.emplace_back(-M_PI*M_PI*3/400*std::cos(M_PI*(point_index-10)/20.0));
        res.emplace_back(-M_PI*M_PI*3/400*std::sin(M_PI*(point_index-10)/20.0));
        res.emplace_back(0);
    }

    return res;
}

double get_next_position_velocities(double a,double b,double c,double d,double e,int index){
    return 5*a*std::pow(index*dt,4)+4*b*std::pow(index*dt,3)+3*c*std::pow(index*dt,2)+2*d*index*dt+e;
}


void compute_x_coffe(double& a,double& b,double& c,double& d,double& e,double& f,int index){
    std::vector<double> current_point=generate_way_points(index);
    std::vector<double> next_point=generate_way_points(index+1);
    a=(12*(next_point[0]-current_point[0])-6*(1*current_point[7]+1*next_point[7])*T-(current_point[10]-next_point[10])*std::pow(T,2))/(2*std::pow(T,5));
    b=(30*(current_point[0]-next_point[0])+2*(7*next_point[7]+8*current_point[7])*T+(3*current_point[10]-2*next_point[10])*std::pow(T,2))/(2*std::pow(T,4));
    c=(20*(next_point[0]-current_point[0])-4*(2*next_point[7]+3*current_point[7])*T-(3*current_point[10]-next_point[10])*std::pow(T,2))/(2*std::pow(T,3));
    d=current_point[10]/2.0;
    e=current_point[7];
    f=current_point[0];
}

void compute_y_coffe(double& a,double& b,double& c,double& d,double& e,double& f,int index){
    std::vector<double> current_point=generate_way_points(index);
    std::vector<double> next_point=generate_way_points(index+1);
    a=(12*(next_point[1]-current_point[1])-6*(next_point[8]+current_point[8])*T-(current_point[11]-next_point[11])*std::pow(T,2))/(2*std::pow(T,5));
    b=(30*(current_point[1]-next_point[1])+2*(7*next_point[8]+8*current_point[8])*T+(3*current_point[11]-2*next_point[11])*std::pow(T,2))/(2*std::pow(T,4));
    c=(20*(next_point[1]-current_point[1])-4*(2*next_point[8]+3*current_point[8])*T-(3*current_point[11]-next_point[11])*std::pow(T,2))/(2*std::pow(T,3));
    d=current_point[11]/2;
    e=current_point[8];
    f=current_point[1];
}
void compute_z_coffe(double& a,double& b,double& c,double& d,double& e,double& f,int index){
    std::vector<double> current_point=generate_way_points(index);
    std::vector<double> next_point=generate_way_points(index+1);
    a=(12*(next_point[2]-current_point[2])-6*(next_point[9]+current_point[9])*T-(current_point[12]-next_point[12])*std::pow(T,2))/(2*std::pow(T,5));
    b=(30*(current_point[2]-next_point[2])+2*(7*next_point[9]+8*current_point[9])*T+(3*current_point[12]-2*next_point[12])*std::pow(T,2))/(2*std::pow(T,4));
    c=(20*(next_point[2]-current_point[2])-4*(2*next_point[9]+3*current_point[9])*T-(3*current_point[12]-next_point[12])*std::pow(T,2))/(2*std::pow(T,3));
    d=current_point[12]/2;
    e=current_point[9];
    f=current_point[2];
}


/*compute the cartisian velocities of the next interpolate point*/
std::vector<double> compute_next_cartisian_velocities(bool &done){
    static int index=(int)(T/dt);
    #ifdef TRANSLATE
        static int index_point=0;
    #endif
    #ifdef CIRCLE
        static int index_point=10;
    #endif
    #ifdef MIXED
        static int index_point=0;
    #endif
    static double ax,ay,az,bx,by,bz,cx,cy,cz,dx,dy,dz,ex,ey,ez,fx,fy,fz;
    static geometry_msgs::Quaternion current_point_orientation,next_point_orientation;
    std::vector<double> velocities;

    std::vector<double> current_point,next_point;
    //move to the next way point
    if(index >=(int)(T/dt)){
        index=0;
        current_point=generate_way_points(index_point);
        next_point=generate_way_points(index_point+1);
        
        compute_x_coffe(ax,bx,cx,dx,ex,fx,index_point);
        compute_y_coffe(ay,by,cy,dy,ey,fy,index_point);
        compute_z_coffe(az,bz,cz,dz,ez,fz,index_point);

        initialize_quaternion(current_point_orientation,current_point[3],current_point[4],current_point[5],current_point[6]);
        initialize_quaternion(next_point_orientation,next_point[3],next_point[4],next_point[5],next_point[6]);

        index_point+=1;

        #ifdef TRANSLATE
        if(index_point==10){
            done=true;
        }
        #endif

        #ifdef TRANSLATE
        if(index_point==20){
            done=true;
        }
        #endif

        #ifdef MIXED
        if(index_point==20){
            done=true;
        }
        #endif
    }
    //compute translate velocity
    double vx=get_next_position_velocities(ax,bx,cx,dx,ex,index);
    double vy=get_next_position_velocities(ay,by,cy,dy,ey,index);
    double vz=get_next_position_velocities(az,bz,cz,dz,ez,index);
    velocities.push_back(vx);
    velocities.push_back(vy);
    velocities.push_back(vz);

    double coff=2.0;
    geometry_msgs::Quaternion current_inter_p=slerp(current_point_orientation,next_point_orientation,index*dt);
    geometry_msgs::Quaternion last_inter_p,next_inter_p;
    if(index>=1){
        last_inter_p=slerp(current_point_orientation,next_point_orientation,(index-1)*dt);
    }
    else{
        last_inter_p=slerp(current_point_orientation,next_point_orientation,index*dt);
        coff-=1.0;
    }
    if(index<=99){
        last_inter_p=slerp(current_point_orientation,next_point_orientation,(index+1)*dt);
    }
    else{
        last_inter_p=slerp(current_point_orientation,next_point_orientation,index*dt);
        coff-=1.0;
    }
    geometry_msgs::Quaternion d_q=get_derivate(next_inter_p,last_inter_p,coff,dt);

    geometry_msgs::Quaternion q_star(current_inter_p);
    q_star.x*=-1.0;
    q_star.y*=-1.0;
    q_star.z*=-1.0;
    std::vector<double> w=quaternion_dot(d_q,q_star);

    velocities.emplace_back(2*w[1]);
    velocities.emplace_back(2*w[2]);
    velocities.emplace_back(2*w[3]);
    
    index+=1;
    return velocities;

}
void print(std::vector<double> &vec){
    std::cout<<"data:";
    for(int i=0;i<vec.size();i++){
        std::cout<<" "<<vec[i];
    }
    std::cout<<std::endl;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("sim_sys");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    Eigen::MatrixXd jacobian;
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);

    //init the manipualtor using init pose
    geometry_msgs::Pose init_pose;
    init_pose.position.x=5;
    init_pose.position.y=-2;
    init_pose.position.z=0;
    geometry_msgs::Quaternion q;
    initialize_quaternion(q,0.0,0.0,0.0,1.0);
    init_pose.orientation=q;
     Eigen::Isometry3d init_state=pose_to_Isometry(init_pose);
    //compute ik
    double timeout = 0.1;
    bool found_ik = kinematic_state->setFromIK(joint_model_group, init_state, timeout);
    std::vector<double> init_joint_values;
    if (found_ik){
        kinematic_state->copyJointGroupPositions(joint_model_group, init_joint_values);
    }
    else{
        ROS_ERROR("the given init pose is not availebal");
        exit(1);
    }
    kinematic_state->setJointGroupPositions(joint_model_group,init_joint_values);
    print(init_joint_values);
    //get jacobian
    kinematic_state->getJacobian(joint_model_group,
                                       kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                       reference_point_position, jacobian);
    std::vector<double> joint_values(init_joint_values);

    std::ofstream out("planned_joint_path_opt_no.txt");

    bool done=false;
    
    ROS_INFO("initialized!! start planning!");
    while(!done){
        kinematic_state->getJacobian(joint_model_group,
                                       kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                       reference_point_position, jacobian);
        double manipulability=compute_manipulability(jacobian);
        double joint_coffe=compute_joint_coffe(joint_values);
        #ifndef DEBUG
        out<<joint_values[0]<<" "<<joint_values[1]<<" "<<joint_values[2]<<" "<<joint_values[3]<<" "<<joint_values[4]<<" "<<joint_values[5]<<" "<<joint_values[6]<<" "
            <<joint_values[7]<<" "<<joint_values[8]<<" "<<joint_values[9]<<" "<<joint_values[10]<<" "<<manipulability<<" "<<joint_coffe<<"\r\n";
            
        #endif
        std::vector<double> cartisian_velocities=compute_next_cartisian_velocities(done);

        
        std::vector<double> delta_manipulability=compute_delta_manipulability(joint_values,kinematic_state,joint_model_group);
        std::vector<double> delta_joint_coffe=compute_delta_joint_coffe(joint_values);
        #ifdef DEBUG
            std::cout<<"delta manipulability:"<<std::endl;
            print(delta_manipulability);
            std::cout<<"delta joint coffe:"<<std::endl;
            print(delta_joint_coffe);
            //break;
        #endif

        //optimize method
        std::vector<double> joint_velocities=compute_joint_velocities(&jacobian,delta_manipulability,delta_joint_coffe,cartisian_velocities);
        //no optmization
        //std::vector<double> joint_velocities=compute_joint_velocities_no_opt(&jacobian,cartisian_velocities);

        joint_values_update(joint_values,joint_velocities,dt);

        kinematic_state->setJointGroupPositions(joint_model_group,joint_values);
    }

    #ifndef DEBUG
    ROS_INFO("mission complited!!");
    #endif
    out.close();

}

