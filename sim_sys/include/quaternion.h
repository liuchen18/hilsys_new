#ifndef _QUATERNION
#define _QUATERNION

#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"


geometry_msgs::Quaternion slerp(geometry_msgs::Quaternion start_orientation,geometry_msgs::Quaternion end_orientation,const double t){
    double cosa = start_orientation.x*end_orientation.x + start_orientation.y*end_orientation.y + start_orientation.z*end_orientation.z + start_orientation.w*end_orientation.w;
    geometry_msgs::Quaternion end_o;
    if (cosa < 0){
        end_o.x= -end_orientation.x;
        end_o.y= -end_orientation.y;
        end_o.z= -end_orientation.z;
        end_o.w= -end_orientation.w;
        cosa = -cosa;
    }
    else{
        end_o=end_orientation;
    }
    geometry_msgs::Quaternion start_o=start_orientation;
    double k0=0.0;
    double k1=0.0;
    if (cosa >0.999){
        k0=1.0-t;
        k1=t;
    }
    else{
        double sina=std::sqrt(1.0-cosa*cosa);
        double a=std::atan2(sina,cosa);
        k0=std::sin((1.0-t)*a)/sina;
        k1=std::sin(t*a)/sina;
    }
    geometry_msgs::Quaternion res;
    res.x=start_o.x*k0+end_o.x*k1;
    res.y=start_o.y*k0+end_o.y*k1;
    res.z=start_o.z*k0+end_o.z*k1;
    res.w=start_o.w*k0+end_o.w*k1;
    return res;
}

/*w,x,y,z*/
std::vector<double> quaternion_dot(geometry_msgs::Quaternion first,geometry_msgs::Quaternion second){
    std::vector<double> res(4,0.0);
    res[0]=first.w*second.w-first.x*second.x-first.y*second.y-first.z*second.z;
    res[1]=first.w*second.x+first.x*second.w+first.y*second.z-first.z*second.y;
    res[2]=first.w*second.y-first.x*second.z+first.y*second.w+first.z*second.x;
    res[3]=first.w*second.z+first.x*second.y-first.y*second.x+first.z*second.w;
    return res;
}

geometry_msgs::Quaternion get_derivate(geometry_msgs::Quaternion next,geometry_msgs::Quaternion last,double coff,double dt){
    geometry_msgs::Quaternion res;
    res.x=(next.x-last.x)/coff/dt;
    res.y=(next.y-last.y)/coff/dt;
    res.z=(next.z-last.z)/coff/dt;
    res.w=(next.w-last.w)/coff/dt;
    return res;
}
void initialize_quaternion(geometry_msgs::Quaternion &q,const double x,const double y,const double z,const double w){
    q.x=x;
    q.y=y;
    q.z=z;
    q.w=w;
}

geometry_msgs::Pose Isometry_to_pose(const Eigen::Isometry3d& state){
    geometry_msgs::Pose pose;
    pose.position.x=state.translation()[0];
    pose.position.y=state.translation()[1];
    pose.position.z=state.translation()[2];
    Eigen::Quaterniond quaternion(state.rotation());
    Eigen::Vector4d nums=quaternion.coeffs();
    pose.orientation.x=nums[0];
    pose.orientation.y=nums[1];
    pose.orientation.z=nums[2];
    pose.orientation.w=nums[3];
    
    return pose;
}


#endif