/**
 * @file online_reloc.h
 * @brief Online relocalization at the start.
 * @author kunpeng fan
 * * @copyright Copyright (c) 2026. Licensed under the MIT License.
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "global_reloc.h"

Eigen::Matrix4f poseToMatrix(double tx, double ty, double tz, double qx, double qy, double qz, double qw);

class OnlineRelocalizationNode {
public:
    OnlineRelocalizationNode(ros::NodeHandle& nh);

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher map_pub_, odom_pub_;  
    ros::Timer map_pub_timer_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    
    std::string cloud_topic_, data_dir_, global_map_file_, map_frame_, base_frame_;
    int n_avg_;
    double global_map_pub_freq_;
    bool is_processing_, init_done_;
    std::vector<Eigen::Matrix4f> pose_buffer_;

    UnifiedGlobalRelocalizer relocalizer_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
    sensor_msgs::PointCloud2 global_map_msg_;
    
    bool loadDatabaseAndMap();
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    Eigen::Matrix4f computeAveragePose(const std::vector<Eigen::Matrix4f>& poses);
    void publishTfAndOdom(const Eigen::Matrix4f& T, const ros::Time& stamp);
    void mapTimerCallback(const ros::TimerEvent& event);
};