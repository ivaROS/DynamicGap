#pragma once

#include <vector>
#include <Eigen/Core>
#include <geometry_msgs/PoseArray.h>

struct CbfLinConstraint
{
  bool valid = false;

  float h0 = std::numeric_limits<float>::quiet_NaN();     // h(u_nom)
  Eigen::Vector2f a = Eigen::Vector2f::Zero();            // grad_h  (call it a)
  float b = std::numeric_limits<float>::quiet_NaN();      // a^T u_nom - h0

  Eigen::Vector2f u_proj = Eigen::Vector2f::Zero();       // single-constraint projection
//   Eigen::Vector2f cmdVel  = Eigen::Vector2f::Zero(); 
  geometry_msgs::Twist beforeCBFHoloCmdVel;  // initilized to all zeros
  geometry_msgs::Twist beforeCBFNonHoloCmdVel;  // all zeros
  Eigen::Vector2f u_nom = Eigen::Vector2f::Zero();     


};

