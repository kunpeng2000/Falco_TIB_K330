/**
 * @file pathFollow.h
 * @brief ROS 1 follow path for MobiRo @ tib_k331
 * @author Alex Liu
 * @author kunpeng fan (Modified)
 * @copyright Copyright (c) 2026. Licensed under the MIT License.
 * * @acknowledgement 
 * this file is referenced and adapted from the following open-source project:
 * Repository: https://github.com/HongbiaoZ/autonomous_exploration_development_environment
 */

#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

class PathFollower {
public:
    PathFollower(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    void odomHandler(const nav_msgs::Odometry::ConstPtr& msg);
    void pathHandler(const nav_msgs::Path::ConstPtr& msg);
    void stopHandler(const std_msgs::Int8::ConstPtr& msg);
    void controlLoop(const ros::TimerEvent& event);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_path_;
    ros::Subscriber sub_stop_;
    ros::Publisher pub_speed_;
    ros::Timer timer_;

    // Parameters
    double sensor_offset_x_;
    double sensor_offset_y_;
    double look_ahead_dis_;
    double yaw_rate_gain_;
    double stop_yaw_rate_gain_;
    double max_yaw_rate_;
    double max_speed_;
    double max_accel_;
    bool two_way_drive_;
    double stop_dis_thre_;
    double slow_down_dis_thre_;
    double dir_diff_thre_;
    bool align_yaw_;
    
    std::string odom_topic_;
    std::string path_topic_;
    std::string cmd_topic_;

    // State variables
    double vehicle_x_ = 0.0;
    double vehicle_y_ = 0.0;
    double vehicle_yaw_ = 0.0;
    double odom_time_ = 0.0;

    double vehicle_x_rec_ = 0.0;
    double vehicle_y_rec_ = 0.0;
    double vehicle_yaw_rec_ = 0.0;

    nav_msgs::Path path_;
    int path_point_id_ = 0;
    bool path_init_ = false;

    int safety_stop_ = 0;
    double current_v_ = 0.0;
    double current_w_ = 0.0;
};