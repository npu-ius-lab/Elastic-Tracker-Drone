#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <object_detection_msgs/BoundingBoxes.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;

class TargetTracker {
private:
    // RGB与深度同步回调
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    boost::shared_ptr<Synchronizer<SyncPolicy>> sync_;
    
    // 目标框发布
    ros::Publisher bbox_pub_;
    ros::Publisher image_pub_;

    // 多目标特征处理
    struct TargetFeature {
        cv::Mat color_hist;   // HSV颜色直方图（H+S）
        double avg_depth;     // 平均深度（米）
        cv::Rect rect;        // 目标框
    };
    TargetFeature last_feature_;            // 上一次目标特征
    cv::Ptr<cv::CLAHE> clahe_;              // 直方图均衡化（一个暂存变量）
    double depth_weight_ = 0.6;             // 深度特征权重
    double color_weight_ = 0.4;             // 颜色特征权重
    double depth_fix_ = 0.0;

    bool is_tracking_ = false;              // 是否处于跟踪状态
    bool lost_target_ = false;              // 是否找不到目标
    ros::Time last_valid_time_;             // 最后一次有效跟踪时间
    // 注意：这里lost_target指的是出现多个目标时没有和上一个识别目标相似的目标
    // 未识别到目标/识别目标不满足面积阈值的情况不算

    // 同步回调函数
    void syncCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg);
    // 特征提取函数
    TargetFeature extractFeature(const cv::Mat& bgr, const cv::Mat& depth, const cv::Rect& rect);
    // 特征相似度计算函数
    double compareFeatures(const TargetFeature& curr, const TargetFeature& ref);
    // 图像发布函数
    void pubProcessedImage(cv::Mat& bgr_image);

public:
    // 构造函数（初始化）
    TargetTracker(ros::NodeHandle& nh_)
    {
        // 初始化订阅者
        rgb_sub_.subscribe(nh_, "/color/image_raw", 10);
        depth_sub_.subscribe(nh_, "/depth/image_raw", 10);
        sync_.reset(new Synchronizer<SyncPolicy>(SyncPolicy(10), rgb_sub_, depth_sub_));
        sync_->registerCallback(boost::bind(&TargetTracker::syncCallback, this, _1, _2));

        // 初始化发布者
        bbox_pub_ = nh_.advertise<object_detection_msgs::BoundingBoxes>("/target/bbox", 10);
        image_pub_ = nh_.advertise<sensor_msgs::Image>("color_image_processed", 10);

        clahe_ = cv::createCLAHE(2.0, cv::Size(8,8));
        nh_.getParam("depth_fix", depth_fix_);
    }
};
