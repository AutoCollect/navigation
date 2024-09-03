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

#ifndef MBF_BUMPER_RECOVERY_H_
#define MBF_BUMPER_RECOVERY_H_

#include <mbf_costmap_core/costmap_recovery.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/Bool.h>

namespace mbf_bumper_recovery
{

class BumperRecovery : public mbf_costmap_core::CostmapRecovery
{
public:

  BumperRecovery();

  virtual ~BumperRecovery();

  virtual void initialize (std::string name, tf2_ros::Buffer*, costmap_2d::Costmap2DROS*,
                           costmap_2d::Costmap2DROS* local_costmap);

  virtual uint32_t runBehavior(std::string& message);

  virtual bool cancel();

private:

  void bumperCallback(const std_msgs::Bool::ConstPtr& msg);

  void publishStop() const;

  ros::NodeHandle nh_;

  ros::Subscriber bumper_sub_;
  ros::Publisher cmd_vel_pub_;

  // The cycle frequency executing the break checks, (default: 20 Hz)
  float control_frequency_;
  // The velocity for driving the robot backwards, (default: -0.3 m/sec)
  float linear_vel_back_;
  
  // The distance to move the robot backwards, (default: 1 m)
  float step_back_length_;
  // The timeout before stopping the robot, (default: 15 sec)
  float step_back_timeout_;

  costmap_2d::Costmap2DROS* local_costmap_;

  bool bumper_triggered_;

  bool initialized_;
  bool canceled_;
};

} // namespace mbf_bumper_recovery

#endif // MBF_BUMPER_RECOVERY_H_