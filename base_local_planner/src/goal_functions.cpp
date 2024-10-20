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
#include <base_local_planner/goal_functions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <costmap_2d/cost_values.h>
#ifdef _MSC_VER
#define GOAL_ATTRIBUTE_UNUSED
#else
#define GOAL_ATTRIBUTE_UNUSED __attribute__ ((unused))
#endif

namespace base_local_planner {

  double getGoalPositionDistance(const geometry_msgs::PoseStamped& global_pose, double goal_x, double goal_y) {
    return hypot(goal_x - global_pose.pose.position.x, goal_y - global_pose.pose.position.y);
  }

  double getGoalOrientationAngleDifference(const geometry_msgs::PoseStamped& global_pose, double goal_th) {
    double yaw = tf2::getYaw(global_pose.pose.orientation);
    return angles::shortest_angular_distance(yaw, goal_th);
  }

  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub) {
    //given an empty path we won't do anything
    if(path.empty())
      return;

    //create a path message
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    pub.publish(gui_path);
  }

  void prunePlan(const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& plan, std::vector<geometry_msgs::PoseStamped>& global_plan){
    ROS_ASSERT(global_plan.size() >= plan.size());
    std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
    const geometry_msgs::PoseStamped& init_waypoint = *it;

    double x_diff = global_pose.pose.position.x - init_waypoint.pose.position.x;
    double y_diff = global_pose.pose.position.y - init_waypoint.pose.position.y;
    double min_distance = hypot(x_diff, y_diff);

    unsigned int erase_index = 0;
    ++ it;

    while(it != plan.end()){

      const geometry_msgs::PoseStamped& waypoint = *it;
      double x_diff = global_pose.pose.position.x - waypoint.pose.position.x;
      double y_diff = global_pose.pose.position.y - waypoint.pose.position.y;
      double distance = hypot(x_diff, y_diff);

      if (distance < min_distance) {
        min_distance = distance;
        ++ erase_index;
      }
      else {
        break;
      }

      ++ it;
    }

    if (erase_index > 0) {
      plan.erase(plan.begin(), plan.begin() + erase_index);
      global_plan.erase(global_plan.begin(), global_plan.begin() + erase_index);
    }
  }

  void prunePlan(const geometry_msgs::PoseStamped& global_pose,
                 const geometry_msgs::PoseStamped& robot_vel,
                 std::vector<geometry_msgs::PoseStamped>& plan,
                 std::vector<geometry_msgs::PoseStamped>& global_plan) {

    ROS_ASSERT(global_plan.size() >= plan.size());
    auto it = plan.begin();
    auto global_it = global_plan.begin();

    double min_distance_threshold = std::numeric_limits<double>::max();

    while (it != plan.end()) {

      const geometry_msgs::PoseStamped& wpt = *it;
      double distance = hypot((global_pose.pose.position.x - wpt.pose.position.x),
                              (global_pose.pose.position.y - wpt.pose.position.y));

      if(distance < 0.25) {
        break;
      }

      if (distance < min_distance_threshold) {
        min_distance_threshold = distance;
        it = plan.erase(it);
        global_it = global_plan.erase(global_it);
      }
      else {
        break;
      }
    }
  }

  double curvatureFrom3Points (const geometry_msgs::PoseStamped& p1, 
                               const geometry_msgs::PoseStamped& p2, 
                               const geometry_msgs::PoseStamped& p3) {

    double fAreaOfTriangle = fabs((p1.pose.position.x * (p2.pose.position.y - p3.pose.position.y) + 
                                   p2.pose.position.x * (p3.pose.position.y - p1.pose.position.y) + 
                                   p3.pose.position.x * (p1.pose.position.y - p2.pose.position.y)) * 0.5);
        
    double fDist12 = hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
    double fDist23 = hypot(p2.pose.position.x - p3.pose.position.x, p2.pose.position.y - p3.pose.position.y);
    double fDist13 = hypot(p1.pose.position.x - p3.pose.position.x, p1.pose.position.y - p3.pose.position.y);
    double fKappa  = 4 * fAreaOfTriangle / (fDist12 * fDist23 * fDist13);
    return fKappa;
  }

  bool transformGlobalPlan(
      const tf2_ros::Buffer& tf,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const geometry_msgs::PoseStamped& global_pose,
      const costmap_2d::Costmap2D& costmap,
      const std::string& global_frame,
      std::vector<geometry_msgs::PoseStamped>& transformed_plan,
      bool& flag) {

    flag = false;
    transformed_plan.clear();

    if (global_plan.empty()) {
      ROS_ERROR("Received plan with zero length");
      return false;
    }

    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];
    try {
      // get plan_to_global_transform from plan frame to global_frame
      geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(),
          plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));

      //let's get the pose of the robot in the frame of the plan
      geometry_msgs::PoseStamped robot_pose;
      tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

      //we'll discard points on the plan that are outside the local costmap
      double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                       costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);

      unsigned int i = 0;
      double sq_dist_threshold = dist_threshold * dist_threshold;
      double sq_dist = 0;

      geometry_msgs::PoseStamped newer_pose;
      double current_curvature  = -1000.0;
      double previous_curvature = -1000.0;
      bool   init_flag          = false;
      double previous_delta     = 0.0;

      //now we'll transform until points are outside of our distance threshold
      while(i < (unsigned int)global_plan.size() && sq_dist <= sq_dist_threshold) {
        const geometry_msgs::PoseStamped& pose = global_plan[i];
        tf2::doTransform(pose, newer_pose, plan_to_global_transform);

        if (i > 0 && i < (global_plan.size() - 2)) {
          current_curvature = curvatureFrom3Points(global_plan[i], global_plan[i+1], global_plan[i+2]);

          if (!init_flag) {
            previous_curvature = current_curvature;
            init_flag = true;
          }

          double current_delta = abs(previous_curvature - current_curvature);

          if (!std::isnan(current_curvature)  &&
              !std::isnan(previous_curvature) &&
              current_delta > 0.15 &&
              previous_delta > 0.1 &&
              previous_curvature > 0.1) {
            sq_dist_threshold = 5.6025;
            flag = true;
          }

          previous_curvature = current_curvature;
          previous_delta = current_delta;
        }

        transformed_plan.push_back(newer_pose);

        double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
        double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;

        ++i;
      }
      unsigned int temp_mx, temp_my;
      if (!transformed_plan.empty() && 
        costmap.worldToMap(transformed_plan.back().pose.position.x, transformed_plan.back().pose.position.y, temp_mx, temp_my)) {
        unsigned char temp_cost = costmap.getCost(temp_mx, temp_my);

        if(temp_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          int global_plan_size = global_plan.size() - 1;
          for (int test_idx = 0; test_idx <= 200; test_idx++) {
            if (i >= global_plan_size) {
              break;
            }
            const geometry_msgs::PoseStamped& pose = global_plan[i];
            geometry_msgs::PoseStamped newer_pose;
            tf2::doTransform(pose, newer_pose, plan_to_global_transform);
            transformed_plan.push_back(newer_pose);
            ++i;
          }
        }
      }
    }
    catch(tf2::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf2::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf2::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (!global_plan.empty())
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }

    return true;
  }

  bool getGoalPose(const tf2_ros::Buffer& tf,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const std::string& global_frame, geometry_msgs::PoseStamped &goal_pose) {
    if (global_plan.empty())
    {
      ROS_ERROR("Received plan with zero length");
      return false;
    }

    const geometry_msgs::PoseStamped& plan_goal_pose = global_plan.back();
    try{
      geometry_msgs::TransformStamped transform = tf.lookupTransform(global_frame, ros::Time(),
                         plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
                         plan_goal_pose.header.frame_id, ros::Duration(0.5));

      tf2::doTransform(plan_goal_pose, goal_pose, transform);
    }
    catch(tf2::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf2::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf2::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (global_plan.size() > 0)
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }
    return true;
  }

  bool isGoalReached(const tf2_ros::Buffer& tf,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const costmap_2d::Costmap2D& costmap GOAL_ATTRIBUTE_UNUSED,
      const std::string& global_frame,
      geometry_msgs::PoseStamped& global_pose,
      const nav_msgs::Odometry& base_odom,
      double rot_stopped_vel, double trans_stopped_vel,
      double xy_goal_tolerance, double yaw_goal_tolerance){

    //we assume the global goal is the last point in the global plan
    geometry_msgs::PoseStamped goal_pose;
    getGoalPose(tf, global_plan, global_frame, goal_pose);

    double goal_x = goal_pose.pose.position.x;
    double goal_y = goal_pose.pose.position.y;
    double goal_th = tf2::getYaw(goal_pose.pose.orientation);

    //check to see if we've reached the goal position
    if(getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
      //check to see if the goal orientation has been reached
      if(fabs(getGoalOrientationAngleDifference(global_pose, goal_th)) <= yaw_goal_tolerance) {
        //make sure that we're actually stopped before returning success
        if(stopped(base_odom, rot_stopped_vel, trans_stopped_vel))
          return true;
      }
    }

    return false;
  }

  bool stopped(const nav_msgs::Odometry& base_odom, 
      const double& rot_stopped_velocity, const double& trans_stopped_velocity){
    return fabs(base_odom.twist.twist.angular.z) <= rot_stopped_velocity 
      && fabs(base_odom.twist.twist.linear.x) <= trans_stopped_velocity
      && fabs(base_odom.twist.twist.linear.y) <= trans_stopped_velocity;
  }
};
