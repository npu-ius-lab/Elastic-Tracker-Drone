// [depth_processing.h]
#ifndef DEPTH_PROCESSING_H
#define DEPTH_PROCESSING_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <limits>

/**
 * @brief 使用RANSAC算法从深度图像中鲁棒估计深度值
 * @param depth_image 输入的深度图像（16UC1格式）
 * @param max_iterations RANSAC最大迭代次数
 * @param inlier_threshold 内点判断阈值（单位：米）
 * @return 估计的深度值（米），无效时返回NaN
 */
double computeRansacDepth(const cv::Mat& depth_image,
                         int max_iterations = 100,
                         double inlier_threshold = 0.05) {
    // 提取有效深度值（遍历全图）
    std::vector<double> valid_depths;
    for (int y = 0; y < depth_image.rows; ++y) {
        for (int x = 0; x < depth_image.cols; ++x) {
            uint16_t raw_val = depth_image.at<uint16_t>(y, x);
            if (raw_val == 0) continue; // 过滤无效值
            
            double depth = raw_val / 1000.0;
            valid_depths.push_back(depth);
        }
    }
    
    if (valid_depths.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    // RANSAC算法
    int best_inliers = 0;
    double best_depth = 0;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        size_t idx = rand() % valid_depths.size();
        double model = valid_depths[idx];
        
        int inliers = 0;
        for (const auto& d : valid_depths) {
            if (std::abs(d - model) <= inlier_threshold) {
                ++inliers;
            }
        }
        
        if (inliers > best_inliers) {
            best_inliers = inliers;
            best_depth = model;
        }
    }
    
    std::vector<double> inliers;
    for (const auto& d : valid_depths) {
        if (std::abs(d - best_depth) <= inlier_threshold) {
            inliers.push_back(d);
        }
    }
    
    if (inliers.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    std::nth_element(inliers.begin(), 
                    inliers.begin() + inliers.size()/2,
                    inliers.end());
    return inliers[inliers.size()/2];
}

#endif // DEPTH_PROCESSING_H