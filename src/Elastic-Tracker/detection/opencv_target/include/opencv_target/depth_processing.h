// [depth_processing.h]
#ifndef DEPTH_PROCESSING_H
#define DEPTH_PROCESSING_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <limits>

/**
 * @brief 计算目标区域深度的中位数
 * @param depth_image 输入的深度图像（16UC1格式）
 * @param rect 目标边界框区域
 * @return 估计的深度值（米），无效时返回NaN
 */
double computeMedianDepth(const cv::Mat& depth_image, const cv::Rect& rect) {
    // 非法坐标处理
    if (rect.x < 0 || rect.y < 0 || 
    rect.x + rect.width > depth_image.cols || 
    rect.y + rect.height > depth_image.rows) 
    {
        return std::numeric_limits<double>::quiet_NaN();
    }   

    std::vector<double> valid_depths;
    for (int y = rect.y; y < rect.y + rect.height; ++y) {
        if (y >= depth_image.rows) continue;
        for (int x = rect.x; x < rect.x + rect.width; ++x) {
            if (x >= depth_image.cols) continue;
            
            uint16_t raw_val = depth_image.at<uint16_t>(y, x);
            if (raw_val == 0) continue; // 过滤无效值
            
            valid_depths.push_back(raw_val / 1000.0);
        }
    }
    
    if (valid_depths.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    // 计算中位数
    size_t n = valid_depths.size() / 2;
    std::nth_element(valid_depths.begin(), valid_depths.begin() + n, valid_depths.end());
    return valid_depths[n];
}

#endif // DEPTH_PROCESSING_Hc
