#include "rcommander_pr2_gui/min_time_node.h"
#include <spline_smoother/spline_smoother_utils.h>

bool quadSolve(const double &a, 
               const double &b, 
               const double &c, 
               double &solution)
{
  double t1(0.0), t2(0.0);
  //  double eps = 2.2e-16;
  if (fabs(a) > 0.0)
  {
    double discriminant = b*b-4*a*c;
    if (discriminant >= 0)
    {
      t1 = (-b + sqrt(discriminant))/(2*a);
      t2 = (-b - sqrt(discriminant))/(2*a);
      ROS_DEBUG("t1:%f t2:%f",t1,t2);
      solution = std::max(t1,t2);
      ROS_DEBUG("Solution: %f",solution);
      return true;
    }
    else
      return false;
  }
  else
  {
    if(fabs(b) == 0.0)
      return false;
    t1 = -c/b;
    t2 = t1;
    solution = t1;
    ROS_DEBUG("Solution: %f",solution);
    return true;
  }
}

bool quadSolve(const double &a, 
               const double &b, 
               const double &c, 
               std::vector<double> &solution)
{
  double t1(0.0), t2(0.0);
  double eps = 2.2e-16;
  if (fabs(a) > eps)
  {
    double discriminant = b*b-4*a*c;
    if (discriminant >= 0)
    {
      t1 = (-b + sqrt(discriminant))/(2*a);
      t2 = (-b - sqrt(discriminant))/(2*a);
      ROS_DEBUG("t1:%f t2:%f",t1,t2);
      solution.push_back(t1);
      solution.push_back(t2);
      return true;
    }
    else
    {
      ROS_DEBUG("Discriminant: %f",discriminant);
      return false;
    }
  }
  else
  {
    if(fabs(b) == 0.0)
      return false;
    t1 = -c/b;
    t2 = t1;
    solution.push_back(t1);
    solution.push_back(t2);
    return true;
  }
}

bool validSolution(const double &q0, 
                   const double &q1,
                   const double &v0,
                   const double &v1,
                   const double &dT,
                   const double &vmax,
                   const double &amax)
{
  if (dT == 0.0)
    return false;
  //  double a0 = q0;
  double a1 = v0;
  double a2 = (3*(q1-q0)-(2*v0+v1)*dT)/(dT*dT);
  double a3 = (2*(q0-q1)+(v0+v1)*dT)/(dT*dT*dT);

  double max_accn = fabs(2*a2);
  if(fabs(2*a2+6*a3*dT) > max_accn)
    max_accn = fabs(2*a2+6*a3*dT);

  bool max_vel_exists = false;
  double max_vel = 0.0;

  if(fabs(a3) > 0.0)
  {
    double max_vel_time = (-2*a2)/(6*a3); 
    if (max_vel_time >= 0 && max_vel_time < dT)
    {
      max_vel_exists = true;
      max_vel = a1-(a2*a2)/(a3*3.0);
    }
  }

  if(amax > 0 && max_accn-amax > 1e-2)
  {
    ROS_DEBUG("amax allowed: %f, max_accn: %f",amax,max_accn);
    return false;
  }
  if(max_vel_exists)
    if(fabs(max_vel)-vmax > 1e-2)
    {
      ROS_DEBUG("vmax allowed: %f, max_vel: %f",vmax,max_vel);
      return false;
    }
  return true;
}


double minSegmentTime(const double &q0, 
                      const double &q1, 
                      const double &v0, 
                      const double &v1, 
                      const arm_navigation_msgs::JointLimits &limit)
{
   //    double dq = jointDiff(q0,q1,limit);
   double dq = q1-q0;
   double vmax = limit.max_velocity;
   if( q0 == q1 && fabs(v0-v1) == 0.0)
   {
     return 0.0;
   }
   dq = q1-q0;
   double v = vmax;
   double solution;
   std::vector<double> solution_vec;

   double a1 = 3*(v0+v1)*v - 3* (v0+v1)*v0 + (2*v0+v1)*(2*v0+v1);
   double b1 = -6*dq*v + 6 * v0 *dq - 6*dq*(2*v0+v1);
   double c1 = 9*dq*dq;

   double a2 = 3*(v0+v1)*v + 3* (v0+v1)*v0 - (2*v0+v1)*(2*v0+v1);
   double b2 = -6*dq*v - 6 * v0 *dq + 6*dq*(2*v0+v1);
   double c2 = -9*dq*dq;

   std::vector<double> t1,t2,t3,t4,t5,t6;

   if(quadSolve(a1,b1,c1,t1))
     for(unsigned int i=0; i < t1.size(); i++)
       solution_vec.push_back(t1[i]);
   
   if(quadSolve(a2,b2,c2,t2))
     for(unsigned int i=0; i < t2.size(); i++)
       solution_vec.push_back(t2[i]);
   double amax = -1.0;
   
   if(limit.has_acceleration_limits)
   {
     amax = limit.max_acceleration;
     double a3 = amax/2.0;
     double b3 = 2*v0+v1;
     double c3 = -3*dq;
     if(quadSolve(a3,b3,c3,t3))
       for(unsigned int i=0; i < t3.size(); i++)
         solution_vec.push_back(t3[i]);

     double a4 = amax/2.0;
     double b4 = -(2*v0+v1);
     double c4 = 3*dq;
     if(quadSolve(a4,b4,c4,t4))
       for(unsigned int i=0; i < t4.size(); i++)
         solution_vec.push_back(t4[i]);


     double a5 = amax;
     double b5 = (-2*v0-4*v1);
     double c5 = 6*dq;
     if(quadSolve(a5,b5,c5,t5))
       for(unsigned int i=0; i < t5.size(); i++)
         solution_vec.push_back(t5[i]);

     double a6 = amax;
     double b6 = (2*v0+4*v1);
     double c6 = -6*dq;
     if(quadSolve(a6,b6,c6,t6))
       for(unsigned int i=0; i < t6.size(); i++)
         solution_vec.push_back(t6[i]);
   }
   std::vector<double> positive_durations, valid_durations;
   for(unsigned int i=0; i < solution_vec.size(); i++)
   {
     if(solution_vec[i] > 0)
       positive_durations.push_back(solution_vec[i]);        
   }

   for(unsigned int i=0; i < positive_durations.size(); i++)
   {
     ROS_DEBUG("Positive duration: %f",positive_durations[i]);
     if(validSolution(q0,q1,v0,v1,positive_durations[i],vmax,amax))
       valid_durations.push_back(positive_durations[i]);        
   }

   ROS_DEBUG("valid size: %d",(int)valid_durations.size());       
   std::sort(valid_durations.begin(),valid_durations.end());
   if(!valid_durations.empty())
     solution = valid_durations.front();
   else
     solution = 0.025;

   ROS_DEBUG(" ");
   ROS_DEBUG(" ");
   ROS_DEBUG(" ");
   return solution;
}

double calculateMinimumTime(const trajectory_msgs::JointTrajectoryPoint &start, 
                            const trajectory_msgs::JointTrajectoryPoint &end, 
                            const std::vector<arm_navigation_msgs::JointLimits> &limits)
{
    double minJointTime(FLT_MAX);
    double segmentTime(0);
    int num_joints = (int) start.positions.size();

    for(int i = 0; i < num_joints; i++)
    {
        minJointTime = minSegmentTime(start.positions[i],end.positions[i],start.velocities[i],end.velocities[i],limits[i]);
        if(segmentTime < minJointTime)
            segmentTime = minJointTime;
    }
    return segmentTime;
}


MinTimeNode::MinTimeNode(ros::NodeHandle &n): n_(n)
{
    min_time_service = n_.advertiseService("min_time_to_move", &MinTimeNode::min_time_srv_cb, this);
    ros::ServiceClient left_ik_client = n.serviceClient<kinematics_msgs::GetKinematicSolverInfo>(
            "pr2_left_arm_kinematics/get_ik_solver_info");
    ros::ServiceClient right_ik_client = n.serviceClient<kinematics_msgs::GetKinematicSolverInfo>(
            "pr2_right_arm_kinematics/get_ik_solver_info");
    has_left = false;
    has_right = false;

    if (left_ik_client.call(left_info))
    {
        ROS_INFO("Has left arm.");
        has_left = true;
    } 

    if (right_ik_client.call(right_info))
    {
        ROS_INFO("Has right arm.");
        has_right = true;
    } 
}

bool MinTimeNode::min_time_srv_cb(rcommander_pr2_gui::MinTime::Request &req, 
                                  rcommander_pr2_gui::MinTime::Response &res)
{
    //spline_smoother::CubicTrajectory ct = spline_smoother::CubicTrajectory();
    double mtime = 0.0;
    if (req.left == true)
    {
        mtime = calculateMinimumTime(req.start_point, req.end_point, left_info.response.kinematic_solver_info.limits);
    } else
    {
        mtime = calculateMinimumTime(req.start_point, req.end_point, right_info.response.kinematic_solver_info.limits);
    }
    res.time = mtime;

    //req.start_point
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "min_time_client");
    ros::NodeHandle n;

    //Setup service
    MinTimeNode min_time_node_(n);
    ROS_INFO("min_time_to_move ready.");
    ros::spin();
    return 0;
}


