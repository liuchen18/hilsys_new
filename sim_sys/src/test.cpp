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

std::vector<double> generate_way_points(int point_index){
    geometry_msgs::Quaternion start_o,mid_o,end_o;
    initialize_quaternion(start_o,0.0,0.0,0.0,1.0);
    initialize_quaternion(mid_o,0.0,0.0,0.707,0.707);
    initialize_quaternion(end_o,0.0,0.0,1.0,0.0);

    std::vector<double> res;
    if(point_index<=10){
        res.emplace_back(5-0.3*point_index);
        res.emplace_back(0);
        res.emplace_back(0);
        geometry_msgs::Quaternion q=slerp(start_o,mid_o,(double)(point_index/10));
        res.push_back(q.x);
        res.push_back(q.y);
        res.push_back(q.z);
        res.push_back(q.w);
        res.emplace_back(-0.3);
        res.emplace_back(0);
        res.emplace_back(0);
        res.emplace_back(0);
        res.emplace_back(0);
        res.emplace_back(0);
    }
    else{
        res.emplace_back(2*std::cos(M_PI*(point_index-10)/20.0));
        res.emplace_back(2*std::sin(M_PI*(point_index-10)/20.0));
        res.emplace_back(0);
        geometry_msgs::Quaternion q=slerp(mid_o,end_o,(double)((point_index-10)/10));
        res.emplace_back(q.x);
        res.emplace_back(q.y);
        res.emplace_back(q.z);
        res.emplace_back(q.w);
        res.emplace_back(-M_PI*2/20*std::sin(M_PI*(point_index-10)/20.0));
        res.emplace_back(M_PI*2/20*std::cos(M_PI*(point_index-10)/20.0));
        res.emplace_back(0);
        res.emplace_back(-M_PI*M_PI*2/400*std::cos(M_PI*(point_index-10)/20.0));
        res.emplace_back(-M_PI*M_PI*2/400*std::sin(M_PI*(point_index-10)/20.0));
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

std::vector<double> compute_next_cartisian_velocities(bool &done){
    static int index=(int)(T/dt);
    static int index_point=0;
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
        std::cout<<"x: "<<ax<<" "<<bx<<" "<<cx<<" "<<dx<<" "<<ex<<" "<<fx<<std::endl;
        std::cout<<"y: "<<ay<<" "<<by<<" "<<cy<<" "<<dy<<" "<<ey<<" "<<fy<<std::endl;

        initialize_quaternion(current_point_orientation,current_point[3],current_point[4],current_point[5],current_point[6]);
        initialize_quaternion(next_point_orientation,next_point[3],next_point[4],next_point[5],next_point[6]);

        index_point+=1;
        if(index_point==20){
            done=true;
        }
    }
    //compute translate velocity
    double vx=get_next_position_velocities(ax,bx,cx,dx,ex,index);
    double vy=get_next_position_velocities(ay,by,cy,dy,ey,index);
    double vz=get_next_position_velocities(az,bz,cz,dz,ez,index);
    velocities.push_back(vx);
    velocities.push_back(vy);
    velocities.push_back(vz);
    ROS_INFO("translate velocity conpelited");

    ROS_INFO(" got way point");
    double coff=2.0;
    geometry_msgs::Quaternion current_inter_p=slerp(current_point_orientation,next_point_orientation,index*dt);
    ROS_INFO("got current q");
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
    ROS_INFO("got last and next q");
    geometry_msgs::Quaternion d_q=get_derivate(next_inter_p,last_inter_p,coff,dt);
    ROS_INFO("got q derivate! ");

    geometry_msgs::Quaternion q_star(current_inter_p);
    q_star.x*=-1.0;
    q_star.y*=-1.0;
    q_star.z*=-1.0;
    std::vector<double> w=quaternion_dot(d_q,q_star);

    velocities.emplace_back(2*w[1]);
    velocities.emplace_back(2*w[2]);
    velocities.emplace_back(2*w[3]);
    
    index+=1;
    std::cout<<"point index: "<<index_point<<std::endl;
    return velocities;

}

void print(std::vector<double> &v){
    for(int i=0;i<v.size();i++){
        std::cout<<" "<<v[i];
    }
    std::cout<<std::endl;
}


int main(){
    bool done=false;
    
    while(!done){
        std::vector<double> cartisian_velocities=compute_next_cartisian_velocities(done);
        std::cout<<cartisian_velocities[0]<<std::endl;
    }
    /*
    std::vector<double> current_point,next_point;
    static double ax,ay,az,bx,by,bz,cx,cy,cz,dx,dy,dz,ex,ey,ez,fx,fy,fz;
    current_point=generate_way_points(11);
    next_point=generate_way_points(12);
    ROS_INFO("current point");
    print(current_point);
    ROS_INFO("next point");
    print(next_point);
    compute_x_coffe(ax,bx,cx,dx,ex,fx,11);
    compute_y_coffe(ay,by,cy,dy,ey,fy,11);
    std::cout<<"x: "<<ax<<" "<<bx<<" "<<cx<<" "<<dx<<" "<<ex<<" "<<fx<<std::endl;
    std::cout<<"y: "<<ay<<" "<<by<<" "<<cy<<" "<<dy<<" "<<ey<<" "<<fy<<std::endl;
    double vx=get_next_position_velocities(ax,bx,cx,dx,ex,100);
    std::cout<<"x velocity: "<<vx<<std::endl;
    double x=ax+bx+cx+dx+ex+fx;
    std::cout<<x<<std::endl;
    */
}