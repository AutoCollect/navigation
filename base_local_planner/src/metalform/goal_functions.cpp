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
#include <base_local_planner/metalform/goal_functions.h>
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
    // Combine OpenMP parallelization and SIMD for loop optimization
    #pragma omp parallel for simd
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    pub.publish(gui_path);
  }

  void mf_prunePlan(const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& plan, std::vector<geometry_msgs::PoseStamped>& global_plan){
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

  void mf_prunePlan(const geometry_msgs::PoseStamped& global_pose,
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

    // ROS_WARN("[mf_prunePlan] plan size: %d", int(plan.size()));
    // ROS_WARN("[mf_prunePlan] global_plan size: %d", int(global_plan.size()));
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


  double projectPoseToTrajectory(const geometry_msgs::PoseStamped& robot_pose, 
                                 const std::vector<geometry_msgs::PoseStamped>& trajectory) {

    // TODO FLANN search
    int num_points = trajectory.size();
    double min_dist = std::numeric_limits<double>::max();
    int index = 0;

    // Combine OpenMP parallelization and SIMD for loop optimization
    #pragma omp parallel for simd
    for (int i = 0; i < num_points; ++i) {

      double dist = hypot((trajectory[i].pose.position.x - robot_pose.pose.position.x), 
                          (trajectory[i].pose.position.y - robot_pose.pose.position.y));

      if (dist < min_dist) {
        min_dist = dist;
        index = i;
      }
    }

    return min_dist;
  }

  /** 
   * use local plan (transformed_plan) as a local variable issues:
   * 1. each control cycle, clear, calculate & refill local plan
   * 2. local goal jumps a lot & local goal is not continous (lack of continuity)
   * 3. this makes velocity control is not smooth, sudden fast & sudden slow
   */
  bool transformGlobalPlan(
      const tf2_ros::Buffer& tf,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const geometry_msgs::PoseStamped& global_pose,
      const costmap_2d::Costmap2D& costmap,
      const std::string& global_frame,
      const double& footprint_cost,
      std::vector<geometry_msgs::PoseStamped>& transformed_plan,
      bool& turn_flag,
      bool& has_suspect) {

    turn_flag   = false;
    has_suspect = false;
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
            turn_flag = true;
          }

          previous_curvature = current_curvature;
          previous_delta = current_delta;
        }

        transformed_plan.push_back(newer_pose);

        //========================================
        // project robot pose onto global plan, to see what the close distace between robot pose & global plan
        double min_dist = projectPoseToTrajectory(robot_pose, global_plan);
        //========================================
        // simple line distace between current robot pose and add plan point 
        double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
        double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;
        //========================================
        // check the footprint cost on the trajectory for legality
        //  ROS_ERROR("[transformGlobalPlan] footprint_cost: %f ", footprint_cost);
        //========================================
        // check transformed_plan points cost on trajectory for legality
        // Combine OpenMP parallelization and SIMD for loop optimization
        #pragma omp parallel for simd
        for (int index = 0; index < transformed_plan.size(); index++) {
          unsigned int temp_mx, temp_my;
          if (costmap.worldToMap(transformed_plan[index].pose.position.x, transformed_plan[index].pose.position.y, temp_mx, temp_my) && 
              costmap.getCost(temp_mx, temp_my) == costmap_2d::SUSPECT_OBSTACLE) {
                has_suspect = true;
                break;
              }
        }
        //=========================================
        // verify the SUSPECT_OBSTACLE value
        //=========================================
        if ((sq_dist >= 2.25) && // define the local goal to make sure 0.3 m/s low speed forward
            (min_dist < 0.5) &&
            (has_suspect || (footprint_cost == costmap_2d::SUSPECT_OBSTACLE))) {
          ROS_ERROR("[transformGlobalPlan] SUSPECT_OBSTACLE: reduce plan ");
          break;
        }
        //=========================================
        ++i;
      }
      
      unsigned int temp_mx, temp_my;
      if (!transformed_plan.empty() && 
        costmap.worldToMap(transformed_plan.back().pose.position.x, transformed_plan.back().pose.position.y, temp_mx, temp_my)) {
        unsigned char temp_cost = costmap.getCost(temp_mx, temp_my);

        if(temp_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          // ROS_ERROR("[transformGlobalPlan] INSCRIBED OBSTACLE: extend local goal");
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

  void mf_initLocalPlan(const tf2_ros::Buffer& tf,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const std::string& global_frame,
      std::vector<geometry_msgs::PoseStamped>& transformed_plan) {

    // do nothing for first time initialization or 
    // if transformed_plan is empty
    if (transformed_plan.empty() || transformed_plan.size() == 0) {
      // ROS_ERROR("[mf_initLocalPlan] local plan empty");
      return;
    }

    // check the availability of global_plan
    if (global_plan.empty()) {
      // ROS_ERROR("[mf_initLocalPlan] Received plan with zero length");
      return;
    }

    const geometry_msgs::PoseStamped& plan_pose  = global_plan[0];
    try {
      // get plan_to_global_transform from plan frame to global_frame
      geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(),
          plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));
      //========================================
      // extend 50 waypts to determine the local goal
      // global_plan is "old" plan, not updated one from makePlan()
      geometry_msgs::PoseStamped newer_pose;
      auto global_it = global_plan.begin() + transformed_plan.size();
      int i = 0, max_extend_pints_num = 100;

      while(global_it != global_plan.end() && i < max_extend_pints_num) {
        const geometry_msgs::PoseStamped& pose = *global_it;
        tf2::doTransform(pose, newer_pose, plan_to_global_transform);
        transformed_plan.push_back(newer_pose);
        ++global_it;
        ++i;
      }
      //========================================
    }
    catch(tf2::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return;
    }
    catch(tf2::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return;
    }
    catch(tf2::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (!global_plan.empty())
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());
      return;
    }
  }

  /** 
   * use local plan (transformed_plan) as a global variable
   * 1. each control cycle, local plan only is inserted necessary way points from current global_plan
   * 2. the local goal keeps continuity, especailly in regular obstacle avoidance
   * 3. global plan only updated once when call makePlan function
   * 3. once move base flex frame triggers recovery, need init/clear local plan
   */
  bool mf_transformGlobalPlan(
      const tf2_ros::Buffer& tf,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const geometry_msgs::PoseStamped& global_pose,
      const costmap_2d::Costmap2D& costmap,
      const std::string& global_frame,
      const double& footprint_cost,
      const double& near_field_distance,
      std::vector<geometry_msgs::PoseStamped>& transformed_plan,
      bool& turn_flag,
      bool& has_suspect,
      bool& near_field_flag) {

    //========================================
    // debug print the size of current size of local plan (transformed_plan)
    // ROS_INFO("[mf_transformGlobalPlan] transformed_plan size: %d", int(transformed_plan.size()));
    //========================================
    // init three bool flags
    turn_flag       = false;
    has_suspect     = false;
    near_field_flag = false;
    //========================================
    // check availability of global plan
    if (global_plan.empty()) {
      ROS_ERROR("[mf_transformGlobalPlan] Received plan with zero length");
      return false;
    }
    //========================================
    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];
    try {
      // get plan_to_global_transform from plan frame to global_frame
      geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(),
          plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));

      //let's get the pose of the robot in the frame of the plan
      geometry_msgs::PoseStamped robot_pose;
      tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);
      //========================================
      // refill the local plan if necessary
      if (!transformed_plan.empty()) {
        // get current local goal
        geometry_msgs::PoseStamped local_goal = transformed_plan.back();
        const double goal_th = tf2::getYaw(local_goal.pose.orientation);
        const double epsilon = 1e-9;
        geometry_msgs::PoseStamped newer_pose;
        tf2::doTransform(global_plan[transformed_plan.size()-1], newer_pose, plan_to_global_transform);
        double dist_diff  = hypot(newer_pose.pose.position.x - local_goal.pose.position.x, 
                                  newer_pose.pose.position.y - local_goal.pose.position.y);
        double angle_diff = getGoalOrientationAngleDifference(newer_pose, goal_th);
        if (dist_diff > epsilon || angle_diff > epsilon) {
          // ROS_WARN("[mf_transformGlobalPlan] Clear Local Plan");
          transformed_plan.clear();
          auto global_it = global_plan.begin();   
          while(global_it != global_plan.end()) {
            const geometry_msgs::PoseStamped& pose = *global_it;
            tf2::doTransform(pose, newer_pose, plan_to_global_transform);
            transformed_plan.push_back(newer_pose);
            dist_diff  = hypot(newer_pose.pose.position.x - local_goal.pose.position.x, 
                                      newer_pose.pose.position.y - local_goal.pose.position.y);
            angle_diff = getGoalOrientationAngleDifference(newer_pose, goal_th);
            if (dist_diff < epsilon && angle_diff < epsilon) {
              break;
            }
            ++global_it;
          }
        }
      }
      //========================================
      // calculate max local goal boundary
      // we'll discard points on the plan that are outside the local costmap
      double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                       costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);

      double sq_dist_threshold = dist_threshold * dist_threshold;
      double sq_dist = 0;
      //========================================
      // we calculate from beginning, just add push_back condition
      unsigned int i = 0;
      // unsigned int i = transformed_plan.size(); // from index [local goal + 1]
      int previous_local_plan_size = transformed_plan.size();
      //========================================
      geometry_msgs::PoseStamped newer_pose;

      double current_curvature  = -1000.0;
      double previous_curvature = -1000.0;
      bool   init_flag          = false;
      double previous_delta     = 0.0;

      // now we'll transform until points are outside of our distance threshold
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
            turn_flag = true;
          }

          previous_curvature = current_curvature;
          previous_delta = current_delta;
        }
        //========================================
        if (i >= previous_local_plan_size) {
          transformed_plan.push_back(newer_pose);
        }
        //========================================
        // project robot pose onto global plan, to see what the close distace between robot pose & global plan
        double min_dist = projectPoseToTrajectory(robot_pose, global_plan);
        //========================================
        // simple line distace between current robot pose and add plan point 
        double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
        double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;
        //========================================
        // check the footprint cost on the trajectory for legality
        // ROS_ERROR("[transformGlobalPlan] footprint_cost: %f ", footprint_cost);
        //========================================
        // check transformed_plan points cost on trajectory for legality
        // has_suspect based on all local plan way points
        //========================================
        // Combine OpenMP parallelization and SIMD for loop optimization
        // #pragma omp parallel for simd
        // for (int index = 0; index < transformed_plan.size(); index++) {
        //   unsigned int temp_mx, temp_my;
        //   if (costmap.worldToMap(transformed_plan[index].pose.position.x, transformed_plan[index].pose.position.y, temp_mx, temp_my) && 
        //       // costmap.getCost(temp_mx, temp_my) == costmap_2d::SUSPECT_OBSTACLE) {
        //       costmap.getCost(temp_mx, temp_my) >= costmap_2d::SUSPECT_OBSTACLE) {
        //         has_suspect = true;
        //         // ROS_ERROR("[transformGlobalPlan] SUSPECT_OBSTACLE: has_suspect, sq_dist: %f, min_dist: %f", sq_dist, min_dist);
        //         break;
        //       }
        // }
        //========================================
        // calculate the local goal cost value
        unsigned int temp_mx, temp_my;
        if (!transformed_plan.empty() &&
            costmap.worldToMap(transformed_plan[transformed_plan.size()-1].pose.position.x, 
                               transformed_plan[transformed_plan.size()-1].pose.position.y, 
                               temp_mx, temp_my) && 
          costmap.getCost(temp_mx, temp_my) >= costmap_2d::SUSPECT_OBSTACLE) {
          has_suspect = true;
        }
        //=========================================
        // verify the SUSPECT_OBSTACLE value
        //=========================================
        // if ((sq_dist >= 2.25) && // define the local goal to make sure 0.3 m/s low speed forward
        if ((sq_dist >= 1.0) && // define the local goal to make sure 0.3 m/s low speed forward
            (min_dist < 0.5)  &&
            (has_suspect || (footprint_cost == costmap_2d::SUSPECT_OBSTACLE))) {
          // ROS_ERROR("[transformGlobalPlan] SUSPECT_OBSTACLE: reduce plan");
          //=========================================
          // calculate the distace between local goal and robot pose
          double robot_dist = getGoalPositionDistance(robot_pose,
                                                      transformed_plan[transformed_plan.size()-1].pose.position.x,
                                                      transformed_plan[transformed_plan.size()-1].pose.position.y);
          if (robot_dist <= near_field_distance) { near_field_flag = true; }
          break;
        }
        //=========================================
        ++i;
      }
      
      unsigned int temp_mx, temp_my;
      if (!transformed_plan.empty() && 
        costmap.worldToMap(transformed_plan.back().pose.position.x, transformed_plan.back().pose.position.y, temp_mx, temp_my)) {
        unsigned char temp_cost = costmap.getCost(temp_mx, temp_my);

        if(temp_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          ROS_ERROR("[transformGlobalPlan] INSCRIBED OBSTACLE: extend local goal");
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
