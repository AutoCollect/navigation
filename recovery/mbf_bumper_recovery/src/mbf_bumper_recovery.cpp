/*
 *  Copyright 2020, Sebastian Pütz
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

 /**
 * @file mbf_bumper_recovery.cpp
 * @author Patrick (patrick.dong@metalform.co.nz)
 *         Yohan Borda (yohan.borda@metalform.co.nz)
 * @brief front bumper trigger recovery behavior logic
 * @version 0.2
 * @date 2024-09-03
 * @copyright METALFORM (c) 2024
 */

#include <mbf_bumper_recovery/mbf_bumper_recovery.h>

#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mbf_msgs/RecoveryResult.h>
#include <base_local_planner/footprint_helper.h>
#include <tf2/LinearMath/Quaternion.h>

// register as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(mbf_bumper_recovery::BumperRecovery, mbf_costmap_core::CostmapRecovery)

namespace mbf_bumper_recovery
{

BumperRecovery::BumperRecovery() {}

BumperRecovery::~BumperRecovery(){}

void BumperRecovery::initialize(std::string name, tf2_ros::Buffer*, costmap_2d::Costmap2DROS*,
                                costmap_2d::Costmap2DROS* local_costmap) {

  //------------------------------------------------
  // parameter configuration for mbf_bumper_recovery
  //------------------------------------------------

  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("control_frequency",  control_frequency_,   20.0f); // The cycle frequency executing the break checks, (default: 20 Hz)
  private_nh.param("linear_vel_back",    linear_vel_back_,     -0.3f); // The velocity for driving the robot backwards, (default: -0.3 m/sec)
  // firstly the velocity for driving the robot backwards with slow velocity to avoid the strong vibration of bumper in carpark flat terrain condition
  // secondly we keep the same with linear_vel_back_ in darryl's hilly paddock
  private_nh.param("linear_vel_min_back",linear_vel_min_back_, -0.3f);

  // force the linear velocity to be negative, since the robot should drive backwards.
  if(linear_vel_back_ > 0)
    linear_vel_back_ = -linear_vel_back_;

  if(linear_vel_min_back_ > 0)
    linear_vel_min_back_ = -linear_vel_min_back_;

  private_nh.param("step_back_length",    step_back_length_,    1.0f); // The distance to move the robot backwards, (default: 1 m)
  private_nh.param("step_back_timeout",   step_back_timeout_,  15.0f); // The timeout before stopping the robot, (default: 15 sec)

  //------------------------------------------------
  // subscribe & publisher
  //------------------------------------------------

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  //------------------------------------------------
  // costmap variables init
  //------------------------------------------------

  local_costmap_ = local_costmap;
  initialized_   = true;
}

uint32_t BumperRecovery::runBehavior(std::string &message) {

  ROS_INFO("[Bumper Recovery] runBehavior trigger");

  canceled_ = false;

  //------------------------------------------------
  // robot position
  //------------------------------------------------

  geometry_msgs::PoseStamped initial_pose;
  local_costmap_->getRobotPose(initial_pose);
  tf2::Vector3 initial_position;
  tf2::fromMsg(initial_pose.pose.position, initial_position);

  //------------------------------------------------
  // robot velocity
  //------------------------------------------------

  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = linear_vel_back_;

  // real robot bumper slow release speed
  // avoid produce metalic noise 
  geometry_msgs::Twist slow_vel;
  slow_vel.linear.x = linear_vel_min_back_;

  //------------------------------------------------
  // timing
  //------------------------------------------------

  ros::Duration timeout(step_back_timeout_);
  ros::Time time_begin = ros::Time::now();

  //------------------------------------------------
  // hehavior cycle
  //------------------------------------------------

  ros::Rate rate(control_frequency_);

  while (ros::ok())
  {
    // check for timeout
    if(!timeout.isZero() && time_begin + timeout < ros::Time::now()) {
      // stop the robot
      publishStop();
      message = "Time out, moving backwards";
      ROS_WARN("[Bumper Recovery] Time out, moving backwards, %.2f [sec] elapsed.", timeout.toSec());
      return mbf_msgs::RecoveryResult::SUCCESS;
    }

    // check for cancel request
    if(canceled_) {
      // stop the robot
      publishStop();
      message = "Cancel has been requested, stopping robot.";
      ROS_WARN("[Bumper Recovery] Cancel has been requested, stopping robot.");
      return mbf_msgs::RecoveryResult::CANCELED;
    }

    geometry_msgs::PoseStamped robot_pose;
    local_costmap_->getRobotPose(robot_pose);
    tf2::Vector3 robot_position;
    tf2::fromMsg(robot_pose.pose.position, robot_position);
    double dist = (robot_position - initial_position).length();

    // check if the robot moved back the specified distance
    if(step_back_length_ < dist)
    {
      // stop the robot
      publishStop();
      message = "Successfully moved backwards.";
      ROS_INFO("[Bumper Recovery] Successfully moved backwards.");
      return mbf_msgs::RecoveryResult::SUCCESS;
    }

    if (dist > step_back_length_ * 0.333) {
      cmd_vel_pub_.publish(vel_msg);
    }
    else {
      cmd_vel_pub_.publish(slow_vel);
    }

    rate.sleep();
  }

  // stop the robot
  publishStop();

  return mbf_msgs::RecoveryResult::SUCCESS;
}

void BumperRecovery::publishStop() const {
  geometry_msgs::Twist zero_vel;
  zero_vel.linear.x  = zero_vel.linear.y  = zero_vel.linear.z  = 0.0;
  zero_vel.angular.x = zero_vel.angular.y = zero_vel.angular.z = 0.0;
  cmd_vel_pub_.publish(zero_vel);
}

bool BumperRecovery::cancel() {
  canceled_ = true;
  return true;
}

} // namespace mbf_bumper_recovery
