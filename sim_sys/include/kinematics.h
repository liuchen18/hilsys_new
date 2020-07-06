#ifndef _INVERSE_KINEMATICS
#define _INVERSE_KINEMATICS

#include <vector>
#include "moveit/robot_state/robot_state.h"
#include <Eigen/Dense>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "quaternion.h"

Eigen::MatrixXd pseudo_inverse(Eigen::MatrixXd &jacobian){
    auto svd = jacobian.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
 
	const auto &singularValues = svd.singularValues();
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> singularValuesInv(jacobian.cols(), jacobian.rows());
	singularValuesInv.setZero();
	double  pinvtoler = 1.e-6; // choose your tolerance wisely
	for (unsigned int i = 0; i < singularValues.size(); ++i) {
		if (singularValues(i) > pinvtoler)
			singularValuesInv(i, i) = 1.0 / singularValues(i);
		else
			singularValuesInv(i, i) = 0.0;
	}
 
	Eigen::MatrixXd pinvmat = svd.matrixV() * singularValuesInv * (svd.matrixU().transpose());
    return pinvmat;
}

std::vector<double> compute_delta_manipulability(std::vector<double> joint_values,
                                                    robot_state::RobotStatePtr kinematic_state,
                                                    const robot_state::JointModelGroup* joint_model_group){
    std::vector<double> res;
    Eigen::MatrixXd jacobian;
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    kinematic_state->getJacobian(joint_model_group,
                                       kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                       reference_point_position, jacobian);
    Eigen::MatrixXd matrix_A=jacobian*(jacobian.transpose());
    Eigen::MatrixXd matrix_A_inverse=matrix_A.inverse();
    double coffe=0.5*std::sqrt(matrix_A.determinant());

    int joint_num=joint_values.size();
    for(int i=0;i<joint_num;i++){
        joint_values[i]+=0.001;
        kinematic_state->setJointGroupPositions(joint_model_group,joint_values);
        Eigen::MatrixXd cur_jacobian;
        kinematic_state->getJacobian(joint_model_group,
                                       kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                       reference_point_position, cur_jacobian);
        double sum_=0;
        Eigen::MatrixXd cur_matrix_A=cur_jacobian*(cur_jacobian.transpose());
        int rows=cur_matrix_A.rows();
        int culs=rows;
        for(int m=0;m<rows;m++){
            for(int n=0;n<culs;n++){
                double derivative=(cur_matrix_A(n,m)-matrix_A(n,m))/0.001;
                sum_+=matrix_A_inverse(m,n)*derivative;
            }
        }
        double cur_res=sum_*coffe;
        /*if(cur_res>0.5){
            cur_res=0.5;
        }
        else if(cur_res<-0.5){
            cur_res=-0.5;
        }*/
        res.push_back(cur_res);
        joint_values[i]-=0.001;
        kinematic_state->setJointGroupPositions(joint_model_group,joint_values);
    }
    return res;
}

const double max_joint_values[11]={M_PI*2.0/9.0,8.0,8.0,2.0*M_PI,
                                    M_PI*170/180,M_PI*120/180,M_PI*170/180,M_PI*120/180,M_PI*170/180,M_PI*120/180,M_PI*175/180};
const double min_joint_values[11]={-M_PI*1.5/18.0,-8.0,-8.0,-2.0*M_PI,
                                    -M_PI*170/180,-M_PI*120/180,-M_PI*170/180,-M_PI*120/180,-M_PI*170/180,-M_PI*120/180,-M_PI*175/180};

std::vector<double> compute_delta_joint_coffe(std::vector<double> joint_values){
    std::vector<double> res;
    if(joint_values.size()!=11){
        ROS_ERROR("invalid joint values input");
    }
    joint_values[1]+=3;
    for(int i=0;i<11;i++){
        double cur_res=-std::pow(max_joint_values[i]-min_joint_values[i],2)*(2*joint_values[i]-max_joint_values[i]-min_joint_values[i])/
                        std::pow(max_joint_values[i]-joint_values[i],2)/std::pow(joint_values[i]-min_joint_values[i],2);
        if(i==1 || i==2){
            cur_res*=0;
            //std::cout<<cur_res<<std::endl;
        }
        res.push_back(cur_res);
    }
    return res;

}

std::vector<double> compute_joint_velocities(Eigen::MatrixXd* jacobian,std::vector<double> delta_manipulability,
                                                std::vector<double> delta_joint_coffe,std::vector<double> cartisian_velocities){
    static bool flag=true;

    double a=0.1;
    double b=0.025;
    //double a=0;
    //double b=0;
    Eigen::Matrix<double,11,1> joint_v;
    Eigen::Matrix<double,6,1> cartisian_v;
    for(int i=0;i<6;i++){
        cartisian_v(i,0)=cartisian_velocities[i];
    }
    Eigen::MatrixXd matrix_I;
    matrix_I.setIdentity(11,11);

    Eigen::Matrix<double,11,1> delta_j,delta_m;
    for(int i=0;i<11;i++){
        delta_j(i,0)=delta_joint_coffe[i];
        delta_m(i,0)=delta_manipulability[i];
    }
    Eigen::Matrix<double,11,1> opt_vec;
    /*
    if(flag){
        opt_vec=delta_m*a;
        flag=false;
    }
    else{
        opt_vec=delta_j*b;
        flag=true;
    }*/
    opt_vec=delta_m*a+delta_j*b;
    Eigen::MatrixXd jacobian_pseudo_inv=pseudo_inverse(*jacobian);

    joint_v=jacobian_pseudo_inv*cartisian_v+(matrix_I-jacobian_pseudo_inv*(*jacobian))*opt_vec;

    std::vector<double> res;
    for(int i=0;i<11;i++){
        res.push_back(joint_v(i,0));
    }
    return res;
}

std::vector<double> compute_joint_velocities_no_opt(Eigen::MatrixXd* jacobian,std::vector<double> cartisian_velocities){
    Eigen::Matrix<double,11,1> joint_v;
    Eigen::Matrix<double,6,1> cartisian_v;
    for(int i=0;i<6;i++){
        cartisian_v(i,0)=cartisian_velocities[i];
    }

    Eigen::MatrixXd jacobian_pseudo_inv=pseudo_inverse(*jacobian);

    joint_v=jacobian_pseudo_inv*cartisian_v;

    std::vector<double> res;
    for(int i=0;i<11;i++){
        res.push_back(joint_v(i,0));
    }
    return res;
}

void joint_values_update(std::vector<double>& joint_values,std::vector<double> joint_velocities, double dt){
    for(int i=0;i<11;i++){
        joint_values[i]+=joint_velocities[i]*dt;
    }
}

double compute_manipulability(Eigen::MatrixXd& jacobian){
    double manipulability=0.0;
    Eigen::MatrixXd matrix_A=jacobian*(jacobian.transpose());
    manipulability=std::sqrt(std::abs(matrix_A.determinant()));
    //std::cout<<manipulability<<" "<<matrix_A.determinant()<<std::endl;
    return manipulability;
}

double compute_joint_coffe(std::vector<double> joint_values){
    double res=0;
    for(int i=0;i<joint_values.size();i++){
        double t=std::pow(max_joint_values[i]-min_joint_values[i],2)/(max_joint_values[i]-joint_values[i])/(joint_values[i]-min_joint_values[i])/4-1;
        if(i==1 || i==2){
            t=0;
        }
        res+=t;
    }
    return res;
}

std::vector<double> forward_kinematics(std::vector<double> joint_values,
                                    robot_state::RobotStatePtr kinematic_state,
                                    const robot_state::JointModelGroup* joint_model_group){
    std::vector<double> cur_joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, cur_joint_values);

    kinematic_state->setJointGroupPositions(joint_model_group,joint_values);
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("link_7");
    kinematic_state->setJointGroupPositions(joint_model_group,cur_joint_values);

    geometry_msgs::Pose end_pose=Isometry_to_pose(end_effector_state);
    std::vector<double> res;
    res.push_back(end_pose.position.x);
    res.push_back(end_pose.position.y);
    res.push_back(end_pose.position.z);
    res.push_back(end_pose.orientation.x);
    res.push_back(end_pose.orientation.y);
    res.push_back(end_pose.orientation.z);
    res.push_back(end_pose.orientation.w);
    
    return res;
}



#endif