/**
 * @file scan_context.cpp
 * @brief Scan Context
 * @author kunpeng fan
 * * @copyright Copyright (c) 2026. Licensed under the MIT License.
 * * @acknowledgement 
 * this file is referenced and adapted from the following open-source project:
 * Repository: https://github.com/gisbi-kim/SC-LIO-SAM.git
 */

#include "scan_context.h"

ScanContextLocalizer::ScanContextLocalizer() {}

void ScanContextLocalizer::addGlobalMapNode(pcl::PointCloud<SCPointType>::Ptr map_frame, int node_id) {
    Eigen::MatrixXd sc = makeScancontext(*map_frame);
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc);
    
    global_contexts_.push_back(sc);
    global_ringkeys_.push_back(ringkey);
    node_ids_.push_back(node_id);
    
    // nanoflann KD-Tree
}

bool ScanContextLocalizer::recognize(pcl::PointCloud<SCPointType>::Ptr local_frame, 
               int& best_node_id, 
               float& yaw_diff_rad, 
               double& best_score) {
    if (global_contexts_.empty()) {
        std::cout << "Error: Global map database is empty!" << std::endl;
        return false;
    }

    Eigen::MatrixXd curr_sc = makeScancontext(*local_frame);
    Eigen::MatrixXd curr_ringkey = makeRingkeyFromScancontext(curr_sc);

    std::vector<std::pair<double, int>> candidate_distances;
    for (size_t i = 0; i < global_ringkeys_.size(); i++) {
        double dist = (curr_ringkey - global_ringkeys_[i]).norm();
        candidate_distances.push_back({dist, (int)i});
    }
    int num_candidates = std::min(10, (int)candidate_distances.size());
    std::sort(candidate_distances.begin(), candidate_distances.end());

    double min_dist = 10000000.0;
    int best_align_shift = 0;
    int best_idx = -1;

    for (int i = 0; i < num_candidates; i++) {
        int candidate_idx = candidate_distances[i].second;
        Eigen::MatrixXd candidate_sc = global_contexts_[candidate_idx];

        std::pair<double, int> sc_dist_result = distanceBtnScanContext(curr_sc, candidate_sc);
        
        if (sc_dist_result.first < min_dist) {
            min_dist = sc_dist_result.first;
            best_align_shift = sc_dist_result.second;
            best_idx = candidate_idx;
        }
    }

    best_score = min_dist;

    if (min_dist < SC_DIST_THRES) {
        best_node_id = node_ids_[best_idx];
        yaw_diff_rad = (best_align_shift * PC_UNIT_SECTORANGLE) * M_PI / 180.0; 
        std::cout << "[Relocalization Success] Matched Map Node: " << best_node_id 
             << ", Score: " << min_dist << ", Yaw Diff: " << yaw_diff_rad * 180.0 / M_PI << " deg." << std::endl;
        return true;
    } 
    else {
        std::cout << "[Relocalization Failed] Nearest Score: " << min_dist << " (Threshold: " << SC_DIST_THRES << ")" << std::endl;
        return false;
    }
}

float ScanContextLocalizer::xy2theta(const float& _x, const float& _y) {
    if ((_x >= 0) & (_y >= 0)) return (180 / M_PI) * std::atan(_y / _x);
    if ((_x < 0) & (_y >= 0)) return 180 - ((180 / M_PI) * std::atan(_y / (-_x)));
    if ((_x < 0) & (_y < 0)) return 180 + ((180 / M_PI) * std::atan(_y / _x));
    if ((_x >= 0) & (_y < 0)) return 360 - ((180 / M_PI) * std::atan((-_y) / _x));
    return 0;
}

Eigen::MatrixXd ScanContextLocalizer::circshift(Eigen::MatrixXd& _mat, int _num_shift) {
    if (_num_shift == 0) return _mat;
    Eigen::MatrixXd shifted_mat = Eigen::MatrixXd::Zero(_mat.rows(), _mat.cols());
    for (int col_idx = 0; col_idx < _mat.cols(); col_idx++) {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }
    return shifted_mat;
}

double ScanContextLocalizer::distDirectSC(Eigen::MatrixXd& _sc1, Eigen::MatrixXd& _sc2) {
    int num_eff_cols = 0;
    double sum_sector_similarity = 0;
    for (int col_idx = 0; col_idx < _sc1.cols(); col_idx++) {
        Eigen::VectorXd col_sc1 = _sc1.col(col_idx);
        Eigen::VectorXd col_sc2 = _sc2.col(col_idx);
        if ((col_sc1.norm() == 0) || (col_sc2.norm() == 0)) continue;
        double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());
        sum_sector_similarity += sector_similarity;
        num_eff_cols++;
    }
    return num_eff_cols > 0 ? (1.0 - (sum_sector_similarity / num_eff_cols)) : 1.0;
}

int ScanContextLocalizer::fastAlignUsingVkey(Eigen::MatrixXd& _vkey1, Eigen::MatrixXd& _vkey2) {
    int argmin_vkey_shift = 0;
    double min_veky_diff_norm = 10000000;
    for (int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++) {
        Eigen::MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);
        Eigen::MatrixXd vkey_diff = _vkey1 - vkey2_shifted;
        double cur_diff_norm = vkey_diff.norm();
        if (cur_diff_norm < min_veky_diff_norm) {
            argmin_vkey_shift = shift_idx;
            min_veky_diff_norm = cur_diff_norm;
        }
    }
    return argmin_vkey_shift;
}

std::pair<double, int> ScanContextLocalizer::distanceBtnScanContext(Eigen::MatrixXd& _sc1, Eigen::MatrixXd& _sc2) {
    Eigen::MatrixXd vkey_sc1 = makeSectorkeyFromScancontext(_sc1);
    Eigen::MatrixXd vkey_sc2 = makeSectorkeyFromScancontext(_sc2);
    int argmin_vkey_shift = fastAlignUsingVkey(vkey_sc1, vkey_sc2);

    const int SEARCH_RADIUS = std::round(0.5 * SEARCH_RATIO * _sc1.cols());
    std::vector<int> shift_idx_search_space{argmin_vkey_shift};
    for (int ii = 1; ii < SEARCH_RADIUS + 1; ii++) {
        shift_idx_search_space.push_back((argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols());
        shift_idx_search_space.push_back((argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols());
    }

    int argmin_shift = 0;
    double min_sc_dist = 10000000;
    for (int num_shift : shift_idx_search_space) {
        Eigen::MatrixXd sc2_shifted = circshift(_sc2, num_shift);
        double cur_sc_dist = distDirectSC(_sc1, sc2_shifted);
        if (cur_sc_dist < min_sc_dist) {
            argmin_shift = num_shift;
            min_sc_dist = cur_sc_dist;
        }
    }
    return std::make_pair(min_sc_dist, argmin_shift);
}

Eigen::MatrixXd ScanContextLocalizer::makeScancontext(pcl::PointCloud<SCPointType>& _scan_down) {
    const int NO_POINT = -1000;
    Eigen::MatrixXd desc = NO_POINT * Eigen::MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);
    for (size_t pt_idx = 0; pt_idx < _scan_down.points.size(); pt_idx++) {
        SCPointType pt = _scan_down.points[pt_idx];
        pt.z += LIDAR_HEIGHT;
        float azim_range = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        float azim_angle = xy2theta(pt.x, pt.y);

        if (azim_range > PC_MAX_RADIUS) continue;

        int ring_idx = std::max(std::min(PC_NUM_RING, int(std::ceil((azim_range / PC_MAX_RADIUS) * PC_NUM_RING))), 1);
        int sctor_idx = std::max(std::min(PC_NUM_SECTOR, int(std::ceil((azim_angle / 360.0) * PC_NUM_SECTOR))), 1);

        if (desc(ring_idx - 1, sctor_idx - 1) < pt.z) 
            desc(ring_idx - 1, sctor_idx - 1) = pt.z; 
    }

    for (int row_idx = 0; row_idx < desc.rows(); row_idx++)
        for (int col_idx = 0; col_idx < desc.cols(); col_idx++)
            if (desc(row_idx, col_idx) == NO_POINT) desc(row_idx, col_idx) = 0;
    return desc;
}

Eigen::MatrixXd ScanContextLocalizer::makeRingkeyFromScancontext(Eigen::MatrixXd& _desc) {
    Eigen::MatrixXd invariant_key(_desc.rows(), 1);
    for (int row_idx = 0; row_idx < _desc.rows(); row_idx++) {
        invariant_key(row_idx, 0) = _desc.row(row_idx).mean();
    }
    return invariant_key;
}

Eigen::MatrixXd ScanContextLocalizer::makeSectorkeyFromScancontext(Eigen::MatrixXd& _desc) {
    Eigen::MatrixXd variant_key(1, _desc.cols());
    for (int col_idx = 0; col_idx < _desc.cols(); col_idx++) {
        variant_key(0, col_idx) = _desc.col(col_idx).mean();
    }
    return variant_key;
}