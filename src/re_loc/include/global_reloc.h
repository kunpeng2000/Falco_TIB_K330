/**
 * @file global_reloc.h
 * @brief Global relocalization
 * @author kunpeng fan
 * * @copyright Copyright (c) 2026. Licensed under the MIT License.
 * * @acknowledgement 
 * this file is referenced and adapted from the following open-source project:
 * Repository: https://github.com/gisbi-kim/SC-LIO-SAM.git
 */

#pragma once

#include <vector>
#include <map>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "scan_context.h" 

class UnifiedGlobalRelocalizer {
public:
    UnifiedGlobalRelocalizer(double submap_radius, double icp_thresh, int icp_iters, double icp_score_thres);

    void addKeyframeToDatabase(pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe, 
                               int node_id, 
                               Eigen::Matrix4f global_pose);

    bool localize(pcl::PointCloud<pcl::PointXYZ>::Ptr global_map,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr local_frame,
                  Eigen::Matrix4f& final_pose);

private:
    ScanContextLocalizer sc_localizer_;
    std::map<int, Eigen::Matrix4f> node_poses_;
    int icp_iters_;
    double submap_radius_, icp_thresh_, icp_score_thres_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr extractSubmap(pcl::PointCloud<pcl::PointXYZ>::Ptr global_map, 
                                                      Eigen::Vector3f center);
};