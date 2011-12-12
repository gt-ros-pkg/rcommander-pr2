#ifndef MINTIMENODE_
#define MINTIMENODE_
#include "ros/ros.h"
#include "kinematics_msgs/GetKinematicSolverInfo.h"
#include "spline_smoother/cubic_trajectory.h"
#include "rcommander_pr2_gui/MinTime.h"

class MinTimeNode
{
    protected:
        ros::NodeHandle n_;
        ros::ServiceServer min_time_service;
        kinematics_msgs::GetKinematicSolverInfo left_info;
        kinematics_msgs::GetKinematicSolverInfo right_info;
        bool has_left;
        bool has_right;

    public:
        MinTimeNode(ros::NodeHandle &);
        bool min_time_srv_cb(rcommander_pr2_gui::MinTime::Request &, rcommander_pr2_gui::MinTime::Response &);
};

#endif //MINTIMENODE_

