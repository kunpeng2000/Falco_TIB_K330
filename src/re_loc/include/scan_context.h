/**
 * @file scan_context.h
 * @brief Scan Context
 * @author kunpeng fan
 * * @copyright Copyright (c) 2026. Licensed under the MIT License.
 * * @acknowledgement 
 * this file is referenced and adapted from the following open-source project:
 * Repository: https://github.com/gisbi-kim/SC-LIO-SAM.git
 */

#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ SCPointType;

class ScanContextLocalizer {
public:
    const int PC_NUM_RING = 20;        // 行数 (半径划分)
    const int PC_NUM_SECTOR = 60;      // 列数 (角度划分)
    const double PC_MAX_RADIUS = 10.0; // 雷达最大有效探测半径
    const double LIDAR_HEIGHT = 0.0;   // 传感器高度补偿
    const double SC_DIST_THRES = 0.2;  // 匹配成功的距离阈值
    const double SEARCH_RATIO = 0.1;   // 快速比对时的搜索范围比例
    const double PC_UNIT_SECTORANGLE = 360.0 / PC_NUM_SECTOR;

    ScanContextLocalizer();

    void addGlobalMapNode(pcl::PointCloud<SCPointType>::Ptr map_frame, int node_id);

    bool recognize(pcl::PointCloud<SCPointType>::Ptr local_frame, 
                   int& best_node_id, 
                   float& yaw_diff_rad, 
                   double& best_score);

private:
    std::vector<Eigen::MatrixXd> global_contexts_;
    std::vector<Eigen::MatrixXd> global_ringkeys_;
    std::vector<int> node_ids_;

    float xy2theta(const float& _x, const float& _y);
    Eigen::MatrixXd circshift(Eigen::MatrixXd& _mat, int _num_shift);
    double distDirectSC(Eigen::MatrixXd& _sc1, Eigen::MatrixXd& _sc2);
    int fastAlignUsingVkey(Eigen::MatrixXd& _vkey1, Eigen::MatrixXd& _vkey2);
    std::pair<double, int> distanceBtnScanContext(Eigen::MatrixXd& _sc1, Eigen::MatrixXd& _sc2);
    Eigen::MatrixXd makeScancontext(pcl::PointCloud<SCPointType>& _scan_down);
    Eigen::MatrixXd makeRingkeyFromScancontext(Eigen::MatrixXd& _desc);
    Eigen::MatrixXd makeSectorkeyFromScancontext(Eigen::MatrixXd& _desc);
};