/**
 * @file global_reloc.cpp
 * @brief Global relocalization
 * @author kunpeng fan
 * * @copyright Copyright (c) 2026. Licensed under the MIT License.
 * * @acknowledgement 
 * this file is referenced and adapted from the following open-source project:
 * Repository: https://github.com/gisbi-kim/SC-LIO-SAM.git
 */

#include "global_reloc.h"

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

UnifiedGlobalRelocalizer::UnifiedGlobalRelocalizer(double submap_radius, double icp_thresh, int icp_iters, double icp_score_thres)
    : submap_radius_(submap_radius), icp_thresh_(icp_thresh), icp_iters_(icp_iters), icp_score_thres_(icp_score_thres) {}

void UnifiedGlobalRelocalizer::addKeyframeToDatabase(pcl::PointCloud<pcl::PointXYZ>::Ptr keyframe, 
                                                     int node_id, 
                                                     Eigen::Matrix4f global_pose) {
    sc_localizer_.addGlobalMapNode(keyframe, node_id);
    node_poses_[node_id] = global_pose; 
}

bool UnifiedGlobalRelocalizer::localize(pcl::PointCloud<pcl::PointXYZ>::Ptr global_map,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr local_frame,
                                        Eigen::Matrix4f& final_pose) {
    // ---------------------------------------------------------
    // Scan Context Global Place Recognition
    // ---------------------------------------------------------
    std::cout << "Running Scan Context global search..." << std::endl;
    int best_node_id = -1;
    float sc_yaw_diff_rad = 0.0;
    double sc_score = 0.0;        
    
    if (!sc_localizer_.recognize(local_frame, best_node_id, sc_yaw_diff_rad, sc_score)) {
        std::cout << "=> Scan Context failed to find a valid candidate in the global map." << std::endl;
        return false;
    }

    // candidate global pose
    Eigen::Matrix4f matched_node_pose = node_poses_[best_node_id];
    std::cout << "=> Found candidate! Node ID: " << best_node_id 
              << " at Center(" << matched_node_pose << ")" << std::endl;

    // SC yaw matrix
    Eigen::AngleAxisf yaw_rotation(sc_yaw_diff_rad, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f sc_yaw_transform = Eigen::Matrix4f::Identity();
    sc_yaw_transform.block<3,3>(0,0) = yaw_rotation.toRotationMatrix();

    // initial guess
    Eigen::Matrix4f initial_guess_pose = matched_node_pose * sc_yaw_transform;

    // submap crop
    std::cout << "Extracting local submap from global map..." << std::endl;
    Eigen::Vector3f submap_center = initial_guess_pose.block<3,1>(0,3);
    pcl::PointCloud<pcl::PointXYZ>::Ptr submap = extractSubmap(global_map, submap_center);
    if (submap->empty()) {
        std::cout << "Submap extraction failed (empty cloud). Skip ICP." << std::endl;
        return false;
    }

    // ---------------------------------------------------------
    // ICP Fine Alignment
    // ---------------------------------------------------------
    std::cout << "Running ICP fine alignment..." << std::endl;
    
    // downsample
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_initial_aligned(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*local_frame, *local_initial_aligned, initial_guess_pose);

    pcl::PointCloud<pcl::PointXYZ>::Ptr local_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    float leaf_size = 0.1f; 
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

    // voxel_filter.setInputCloud(local_initial_aligned);
    // voxel_filter.filter(*local_downsampled);

    voxel_filter.setInputCloud(submap); 
    voxel_filter.filter(*target_downsampled);

    // std::cout << "Source points before/after: " << local_initial_aligned->size() << " -> " << local_downsampled->size() << std::endl;
    std::cout << "Target points before/after: " << submap->size() << " -> " << target_downsampled->size() << std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(local_initial_aligned);
    icp.setInputTarget(target_downsampled);               
    icp.setMaxCorrespondenceDistance(icp_thresh_); 
    icp.setMaximumIterations(icp_iters_);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    if (icp.hasConverged() && icp.getFitnessScore() < icp_score_thres_) {
        final_pose = icp.getFinalTransformation() * initial_guess_pose;
        std::cout << "============================================" << std::endl;
        std::cout << "✅ Relocalization SUCCESS!" << std::endl;
        std::cout << "ICP Fitness Score: " << icp.getFitnessScore() << std::endl;
        std::cout << "Final Pose:\n" << final_pose << std::endl;
        std::cout << "============================================" << std::endl;
        return true;
    } 
    else {
        std::cout << "=> ICP did not converge: " << icp.getFitnessScore() << std::endl;
        return false;
    }     
}

pcl::PointCloud<pcl::PointXYZ>::Ptr UnifiedGlobalRelocalizer::extractSubmap(
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map, Eigen::Vector3f center) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr submap(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
    kdtree->setInputCloud(global_map);

    pcl::PointXYZ pt(center.x(), center.y(), center.z());
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (kdtree->radiusSearch(pt, submap_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            submap->points.push_back(global_map->points[pointIdxRadiusSearch[i]]);
        }
    }
    // std::cout << submap->points.size() << submap_radius_ << std::endl;
    submap->width = submap->points.size();
    submap->height = 1;
    submap->is_dense = true;
    return submap;
}