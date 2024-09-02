/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
// system c++ lib
#include <algorithm>
#include <string>
// ros lib
#include <ros/ros.h>
#include <pluginlib/class_list_macros.hpp>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <geometry_msgs/Twist.h>
// local module include file
#include <stop_recovery/stop_recovery.h>

// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(stop_recovery::StopRecovery, nav_core::RecoveryBehavior)

namespace stop_recovery
{
StopRecovery::StopRecovery(): local_costmap_(NULL), initialized_(false), world_model_(NULL)
{
}

void StopRecovery::initialize(std::string name, tf2_ros::Buffer*,
                              costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialized_)
  {
    local_costmap_ = local_costmap;

    // get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    private_nh.param("frequency",      frequency_,      10.0);
    private_nh.param("stop_duration",  stop_duration_,  2.0);
    ROS_INFO("[Stop Recovery] frequency: %.1f, duration: %.1f", frequency_, stop_duration_);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else
  {
    ROS_ERROR("[Stop Recovery] You should not call initialize twice on this object, doing nothing");
  }
}

StopRecovery::~StopRecovery()
{
  delete world_model_;
}

void StopRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if (local_costmap_ == NULL)
  {
    ROS_ERROR("[Stop Recovery] The costmap passed to the StopRecovery object cannot be NULL. Doing nothing.");
    return;
  }

  ROS_ERROR("[Stop Recovery] Stop recovery behavior started ...");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ros::Duration stop_duration(stop_duration_);
  ros::Time init_time = ros::Time::now();

  // ROS_INFO("[Stop Recovery] stop robot triggered");
  while (n.ok() && (ros::Time::now() - init_time <= stop_duration)) {
    // from current speed to stop quickly
    // TODO: reduce from current robot speed to zero
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x  = 0.0;
    cmd_vel.linear.y  = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub.publish(cmd_vel);
    r.sleep();
  }

  ROS_ERROR("[Stop Recovery] End of Stop Recovery");
}
};  // namespace stop_recovery
