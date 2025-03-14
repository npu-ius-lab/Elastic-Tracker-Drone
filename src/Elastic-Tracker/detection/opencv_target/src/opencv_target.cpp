#include <opencv_target/opencv_target.h>
#include <opencv_target/depth_processing.h>

// 图像发布函数
void TargetTracker::pubProcessedImage(cv::Mat& bgr_image)
{
    sensor_msgs::ImagePtr msg;
    msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, bgr_image).toImageMsg();
    TargetTracker::image_pub_.publish(msg);

    return;
}

// 特征提取函数
TargetTracker::TargetFeature TargetTracker::extractFeature(const cv::Mat& hsv_roi, const cv::Mat& depth_roi, const cv::Rect& rect) {
    
    TargetFeature feat;
    feat.rect = rect;
    
    // 直方图均衡化增强鲁棒性
    std::vector<cv::Mat> channels;
    cv::split(hsv_roi, channels);
    clahe_->apply(channels[0], channels[0]);
    clahe_->apply(channels[1], channels[1]);
    
    // 计算2D直方图（H:0-180, S:0-256）
    int histSize[] = {30, 32};    // 降低维度提升速度
    float hRanges[] = {0, 180};
    float sRanges[] = {0, 256};
    const float* ranges[] = {hRanges, sRanges};
    int channels_num[] = {0, 1};
    
    cv::calcHist(&hsv_roi, 1, channels_num, cv::Mat(),
                feat.color_hist, 2, histSize, ranges);
    cv::normalize(feat.color_hist, feat.color_hist);

    // 计算平均深度
    feat.avg_depth = computeRansacDepth(depth_roi, 100, 0.05);

    return feat;
}

// 特征相似度计算函数
double TargetTracker::compareFeatures(const TargetFeature& curr, const TargetFeature& ref) {
    // 颜色相似度（直方图交集）
    double color_sim = cv::compareHist(curr.color_hist, 
                                      ref.color_hist, 
                                      cv::HISTCMP_INTERSECT);
    
    // 深度相似度（高斯加权）
    double depth_diff = fabs(curr.avg_depth - ref.avg_depth);
    double depth_sim = exp(-pow(depth_diff, 2) / (2 * pow(0.5, 2))); // σ=0.5m
    
    // 综合评分（加权求和）
    return depth_weight_ * depth_sim + color_weight_ * color_sim;
}

// RGB与深度图像同步订阅
void TargetTracker::syncCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg)
{
    ros::Time current_time = ros::Time::now();
    
    // 丢失目标超过一定时间，重置
    if (lost_target_ && 
       (current_time - last_valid_time_).toSec() > 5.0) {
        is_tracking_ = false;
        lost_target_ = false;
        ROS_WARN("Reset tracking state after timeout");
    }
    
    /* -------- 数据传入 -------- */

    // 传入RGB数据，并转为BGR格式   
    auto bgr_cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
    // 传入深度数据，默认为16UC1类型  
    auto depth_cv_ptr = cv_bridge::toCvShare(depth_msg);
    
    /* -------- 图像处理 -------- */

    // HSV颜色检测
    bgr_image_ = bgr_cv_ptr->image;
    cv::cvtColor(bgr_image_, hsv_image_, cv::COLOR_BGR2HSV);
        
    // 红色阈值范围
    cv::Scalar lower_red1(0, 120, 70);
    cv::Scalar upper_red1(10, 255, 255);
    cv::Scalar lower_red2(170, 120, 70);
    cv::Scalar upper_red2(180, 255, 255);
    
	// 创建红色掩膜
    cv::Mat mask1, mask2;
    cv::inRange(hsv_image_, lower_red1, upper_red1, mask1);
    cv::inRange(hsv_image_, lower_red2, upper_red2, mask2);
    cv::bitwise_or(mask1, mask2, mask_);

    // 形态学操作
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
    cv::morphologyEx(mask_, mask_, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask_, mask_, cv::MORPH_CLOSE, kernel);

    // 查找轮廓
    cv::Mat labels, stats, centroids;
    int nLabels = cv::connectedComponentsWithStats(mask_, labels, stats, centroids, 8);
    std::vector<cv::Rect> valid_rects;
    for (int i = 1; i < nLabels; ++i) { // 跳过背景
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area < 100) continue;
        int x = stats.at<int>(i, cv::CC_STAT_LEFT);
        int y = stats.at<int>(i, cv::CC_STAT_TOP);
        int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        valid_rects.emplace_back(x, y, w, h);
    }

    // 如果追踪到有效目标，处理检测结果并发送
    if (!valid_rects.empty())
	{
        depth_image = depth_cv_ptr->image;
        cv::cvtColor(bgr_image_, hsv_image_, cv::COLOR_BGR2HSV);

        int selected_idx = 0;
        std::vector<TargetFeature> features(valid_rects.size());
        std::vector<double> scores(valid_rects.size());
            
        // 提取所有候选特征
        for (size_t i = 0; i < valid_rects.size(); ++i) {
            const auto& rect = valid_rects[i];
            cv::Mat hsv_roi = hsv_image_(rect);
            cv::Mat depth_roi = depth_image(rect);
            features[i] = extractFeature(hsv_roi, depth_roi, rect);
        }
            
        // 首次选择：最大面积+深度合法值
        if (!is_tracking_) {
            auto it = std::max_element(features.begin(), features.end(),
                [](const TargetFeature& a, const TargetFeature& b) {
                    return (a.rect.area() < b.rect.area()) || 
                           (std::isnan(a.avg_depth) && !std::isnan(b.avg_depth));
                });
            last_feature_ = *it;
            is_tracking_ = true;
        } 
        // 跟踪状态：综合评分选择
        else {
            // 计算每个候选与上一次特征的相似度
            for (size_t i = 0; i < features.size(); ++i) {
                scores[i] = compareFeatures(features[i], last_feature_);
            }
            
            // 选择最高分且超过阈值的目标
            auto max_it = std::max_element(scores.begin(), scores.end());
            if (*max_it > 0.5) { // 相似度阈值
                selected_idx = std::distance(scores.begin(), max_it);
                last_feature_ = features[selected_idx];
            } else {
            // 无相似目标，认为找不到目标
                ROS_WARN("No similar target, keep last feature");
                lost_target_ = true;
                pubProcessedImage(bgr_image_);
                return;
            }
        }

        // 更新最后已知位置
        cv::Rect rect = valid_rects[selected_idx];
        double depth = features[selected_idx].avg_depth;
        // 绘制目标框
        cv::rectangle(bgr_image_, rect, cv::Scalar(0,255,0), 2);
            
        // 深度值合法则允许发布
        if (std::isnan(depth) || depth <= 0.0) {
            ROS_WARN_STREAM_THROTTLE(1.0,"Invalid depth value: " << depth);
        }
        else{
            ROS_INFO_THROTTLE(1.0,"Target at Pixel: (%.1f, %.1f), depth = %.2f", 
                              rect.x + rect.width/2.0 , rect.y + rect.height/2.0, depth);
            // 赋值bbox
            object_detection_msgs::BoundingBox opencv_bbox_;
            opencv_bbox_.xmin = rect.x;
            opencv_bbox_.xmax = rect.x + rect.width;
            opencv_bbox_.ymin = rect.y;
            opencv_bbox_.ymax = rect.y + rect.height;
            opencv_bbox_.depth = depth;
            opencv_bbox_.probability = 0.9;
            opencv_bbox_.id = 0;
            opencv_bbox_.Class = "red";
            // 发布消息
            object_detection_msgs::BoundingBoxes opencv_bboxes_;
            opencv_bboxes_.bounding_boxes.push_back(opencv_bbox_);
            opencv_bboxes_.header.stamp = ros::Time::now();
            bbox_pub_.publish(opencv_bboxes_);
        }
    }
    // 未追踪到目标
    else
    {
        is_tracking_ = false;  // 重置跟踪状态
        ROS_INFO_THROTTLE(1.0,"NO Target.");
    }
    
    pubProcessedImage(bgr_image_);
    last_valid_time_ = current_time;
    return;
}

// 主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "opencv_target");
    ros::NodeHandle nh;

    TargetTracker opencv_target(nh);
    ros::spin();

    return 0;
}
