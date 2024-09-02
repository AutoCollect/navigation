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
#include <move_back_recovery/move_back_recovery.h>

// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(move_back_recovery::MoveBackRecovery, nav_core::RecoveryBehavior)

namespace move_back_recovery
{

MoveBackRecovery::MoveBackRecovery(): local_costmap_(NULL), initialized_(false), world_model_(NULL) {}

MoveBackRecovery::~MoveBackRecovery()
{
  delete world_model_;
}

void MoveBackRecovery::initialize(std::string name, tf2_ros::Buffer*,
                                  costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialized_)
  {
    local_costmap_ = local_costmap;

    // get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    private_nh.param("frequency",                     frequency_,                    10.0);
    private_nh.param("move_back_duration_threshold",  move_back_duration_threshold_,  5.0);
    private_nh.param("move_back_distance_threshold",  move_back_distance_threshold_,  1.0);

    ROS_INFO("[MoveBack Recovery] frequency: %.1f, duration_thresh: %.2f (sec), distance_thresh: %.2f (m)", frequency_, move_back_duration_threshold_, move_back_distance_threshold_);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else
  {
    ROS_ERROR("[MoveBack Recovery] You should not call initialize twice on this object, doing nothing");
  }
}

void MoveBackRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if (local_costmap_ == NULL)
  {
    ROS_ERROR("[MoveBack Recovery] The costmap passed to the MoveBack Recovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_ERROR("[MoveBack recovery] move back started ...");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // get robot pose
  geometry_msgs::PoseStamped prev_robot_pose;
  local_costmap_->getRobotPose(prev_robot_pose);
  double distance_acc = 0;

  ros::Duration stop_duration(move_back_duration_threshold_);
  ros::Time init_time = ros::Time::now();

  // ROS_INFO("[MoveBack Recovery] move back robot triggered");
  while (n.ok() && ((distance_acc <= move_back_distance_threshold_) || (ros::Time::now() - init_time <= stop_duration))) {

    geometry_msgs::Twist cmd_vel;

    // move back with linear velocity vx = -0.25m/s
    cmd_vel.linear.x  = -0.25;
    cmd_vel.linear.y  =  0.0;
    cmd_vel.angular.z =  0.0;
    vel_pub.publish(cmd_vel);

    geometry_msgs::PoseStamped curr_robot_pose;
    local_costmap_->getRobotPose(curr_robot_pose);

    double x_diff = prev_robot_pose.pose.position.x - curr_robot_pose.pose.position.x;
    double y_diff = prev_robot_pose.pose.position.y - curr_robot_pose.pose.position.y;
    distance_acc += hypot(x_diff, y_diff);
    
    // ROS_INFO("acc dist = %f", distance_acc);
    prev_robot_pose = curr_robot_pose;

    r.sleep();
  }

  ROS_INFO("[MoveBack Recovery] end of Move Back Recovery");
}
};  // namespace move_back_recovery
