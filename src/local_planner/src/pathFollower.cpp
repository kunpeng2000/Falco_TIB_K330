/**
 * @file pathFollow.cpp
 * @brief ROS 1 follow path for MobiRo @ tib_k331
 * @author Alex Liu
 * @author kunpeng fan (Modified)
 * @copyright Copyright (c) 2026. Licensed under the MIT License.
 * * @acknowledgement 
 * this file is referenced and adapted from the following open-source project:
 * Repository: https://github.com/HongbiaoZ/autonomous_exploration_development_environment
 */

#include "pathFollow.h"

PathFollower::PathFollower(ros::NodeHandle& nh, ros::NodeHandle& pnh) 
    : nh_(nh), pnh_(pnh) {
    pnh_.param("sensorOffsetX", sensor_offset_x_, 0.0); // offset of the sensor center relative to the robot's center
    pnh_.param("sensorOffsetY", sensor_offset_y_, 0.0);
    pnh_.param("lookAheadDis", look_ahead_dis_, 0.5); // look-ahead distance
    pnh_.param("yawRateGain", yaw_rate_gain_, 7.5); // angular velocity gain during normal driving
    pnh_.param("stopYawRateGain", stop_yaw_rate_gain_, 7.5); // angular velocity gain when stopping/rotating in place
    pnh_.param("maxYawRate", max_yaw_rate_, 45.0);
    pnh_.param("maxSpeed", max_speed_, 1.0);
    pnh_.param("maxAccel", max_accel_, 1.0);
    pnh_.param("twoWayDrive", two_way_drive_, true); // two-way drive enable
    pnh_.param("stopDisThre", stop_dis_thre_, 0.2); // stop distance threshold
    pnh_.param("slowDwnDisThre", slow_down_dis_thre_, 1.0); // slow-down distance threshold 
    pnh_.param("dirDiffThre", dir_diff_thre_, 0.1); // direction difference threshold, used to determine if an "in-place rotation" is needed.
    pnh_.param("alignYaw", align_yaw_, false); // align heading enable. If true, after reaching the target position, the robot will continue to rotate in place to align with the final Goal Yaw.
    
    pnh_.param<std::string>("odomTopic", odom_topic_, "/odometry/filtered");
    pnh_.param<std::string>("pathTopic", path_topic_, "/local_planner/path");
    pnh_.param<std::string>("cmdTopic", cmd_topic_, "/cmd_vel");

    sub_odom_ = nh_.subscribe(odom_topic_, 10, &PathFollower::odomHandler, this);
    sub_path_ = nh_.subscribe(path_topic_, 10, &PathFollower::pathHandler, this);
    sub_stop_ = nh_.subscribe("/stop", 10, &PathFollower::stopHandler, this);

    pub_speed_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 10);

    timer_ = nh_.createTimer(ros::Duration(0.01), &PathFollower::controlLoop, this);

    ROS_INFO("ROS 1 Path Follower Node Initialized.");
}

void PathFollower::odomHandler(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch, yaw;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    vehicle_yaw_ = yaw;
    vehicle_x_ = msg->pose.pose.position.x - cos(yaw) * sensor_offset_x_ + sin(yaw) * sensor_offset_y_;
    vehicle_y_ = msg->pose.pose.position.y - sin(yaw) * sensor_offset_x_ - cos(yaw) * sensor_offset_y_;
    
    odom_time_ = msg->header.stamp.toSec();
}

void PathFollower::pathHandler(const nav_msgs::Path::ConstPtr& msg) {
    if (msg->poses.empty()) return;
    path_ = *msg;
    
    vehicle_x_rec_ = vehicle_x_;
    vehicle_y_rec_ = vehicle_y_;
    vehicle_yaw_rec_ = vehicle_yaw_;

    path_point_id_ = 0;
    path_init_ = true;
}

void PathFollower::stopHandler(const std_msgs::Int8::ConstPtr& msg) {
    safety_stop_ = msg->data;
}

void PathFollower::controlLoop(const ros::TimerEvent& event) {
    if (!path_init_){
        ROS_WARN_THROTTLE(1.0, "No path set.");
        return;
    }

    if (path_.poses.empty()){
        ROS_WARN_THROTTLE(1.0, "Path is empty.");
        return;
    }

    // current pos in path's coor
    float dx = vehicle_x_ - vehicle_x_rec_;
    float dy = vehicle_y_ - vehicle_y_rec_;
    float vehicle_x_rel = cos(vehicle_yaw_rec_) * dx + sin(vehicle_yaw_rec_) * dy;
    float vehicle_y_rel = -sin(vehicle_yaw_rec_) * dx + cos(vehicle_yaw_rec_) * dy;

    // distance of the path's last point
    int path_size = path_.poses.size();
    float end_dis_x = path_.poses[path_size - 1].pose.position.x - vehicle_x_rel;
    float end_dis_y = path_.poses[path_size - 1].pose.position.y - vehicle_y_rel;
    float end_dis = sqrt(end_dis_x * end_dis_x + end_dis_y * end_dis_y);

    // local target point more than look_ahead_dis_ away from the current pos
    float p_x, p_y, dis;
    while (path_point_id_ < path_size - 1) {
        p_x = path_.poses[path_point_id_].pose.position.x - vehicle_x_rel;
        p_y = path_.poses[path_point_id_].pose.position.y - vehicle_y_rel;
        dis = sqrt(p_x * p_x + p_y * p_y);
        if (dis < look_ahead_dis_) {
            path_point_id_++;
        } 
        else {
            break;
        }
    }

    // x, y, yaw between current pos and local target point
    p_x = path_.poses[path_point_id_].pose.position.x - vehicle_x_rel;
    p_y = path_.poses[path_point_id_].pose.position.y - vehicle_y_rel;
    dis = sqrt(p_x * p_x + p_y * p_y);
    
    float path_dir = 0.0;
    if (align_yaw_ && end_dis < stop_dis_thre_) {
        tf2::Quaternion q_target(
            path_.poses[path_size - 1].pose.orientation.x,
            path_.poses[path_size - 1].pose.orientation.y,
            path_.poses[path_size - 1].pose.orientation.z,
            path_.poses[path_size - 1].pose.orientation.w);
        double roll, pitch, target_yaw_global;
        tf2::Matrix3x3(q_target).getRPY(roll, pitch, target_yaw_global);
        
        path_dir = target_yaw_global - vehicle_yaw_rec_;
        
    } else {
        path_dir = atan2(p_y, p_x);
    }

    float dir_diff = vehicle_yaw_ - vehicle_yaw_rec_ - path_dir;
    while (dir_diff > M_PI) dir_diff -= 2 * M_PI;
    while (dir_diff < -M_PI) dir_diff += 2 * M_PI;

    bool nav_fwd = true;
    if (two_way_drive_ && fabs(dir_diff) > M_PI / 2) {
        if ((align_yaw_ && end_dis >= stop_dis_thre_) || (!align_yaw_)){
            nav_fwd = false;
            dir_diff += (dir_diff > 0) ? -M_PI : M_PI;
        }
    }

    // target speed
    float target_speed = max_speed_;
    if (path_size <= 1) {
        target_speed = 0;
    } 
    else if (end_dis < slow_down_dis_thre_) {
        target_speed *= (end_dis / slow_down_dis_thre_);
    }
    if (!nav_fwd) {
        target_speed *= -1.0;
    }

    // speed ramp
    if (fabs(dir_diff) < dir_diff_thre_ && dis > stop_dis_thre_) {
        if (current_v_ < target_speed) {
            current_v_ = std::min(current_v_ + max_accel_ / 100.0, (double)target_speed);
        }
         else if (current_v_ > target_speed) {
            current_v_ = std::max(current_v_ - max_accel_ / 100.0, (double)target_speed);
        }
    } 
    else {
        if (current_v_ > 0) {
            current_v_ = std::max(0.0, current_v_ - max_accel_ / 100.0);
        }
        else if (current_v_ < 0) {
            current_v_ = std::min(0.0, current_v_ + max_accel_ / 100.0);
        }
    }

    // overcome the large static friction during on-the-spot turning and ensure lateral stability at high speeds
    float stop_speed_threshold = 2.0 * max_accel_ / 100.0;
    if (fabs(current_v_) < stop_speed_threshold) {
        current_w_ = -stop_yaw_rate_gain_ * dir_diff;
    } 
    else {
        current_w_ = -yaw_rate_gain_ * dir_diff;
    }

    // Limit acceleration between [min_w, max_w]
    double max_w_rad = max_yaw_rate_ * M_PI / 180.0;
    current_w_ = std::max(-max_w_rad, std::min(current_w_, max_w_rad));

    // safe stop
    if (safety_stop_ >= 1) current_v_ = 0;
    if (safety_stop_ >= 2) current_w_ = 0;

    // pub the msg
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = current_v_;
    cmd_msg.angular.z = current_w_;
    pub_speed_.publish(cmd_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    PathFollower follower(nh, pnh);

    ros::spin();
    return 0;
}