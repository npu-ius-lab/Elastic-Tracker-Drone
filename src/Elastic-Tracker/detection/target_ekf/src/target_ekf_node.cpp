#include <ros/ros.h>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <object_detection_msgs/BoundingBoxes.h>

typedef message_filters::sync_policies::ApproximateTime<object_detection_msgs::BoundingBoxes, nav_msgs::Odometry>
    YoloOdomSyncPolicy;
typedef message_filters::Synchronizer<YoloOdomSyncPolicy>
    YoloOdomSynchronizer;

ros::Publisher target_odom_pub_, yolo_odom_pub_;

Eigen::Vector3d  extrinsic_t_;
Eigen::Quaterniond  extrinsic_r_;
bool have_extrinstic_ = false;

double fx_, fy_, cx_, cy_;
ros::Time last_update_stamp_;
double pitch_thr_ = 30;

struct Ekf {
  // 成员变量
  double dt;
  Eigen::MatrixXd A, B, C;
  Eigen::MatrixXd Qt, Rt;
  Eigen::MatrixXd Sigma, K;
  Eigen::VectorXd x;
  // 构造函数
  Ekf(double _dt) : dt(_dt) {
    A.setIdentity(6, 6);
    Sigma.setZero(6, 6);
    B.setZero(6, 3);
    C.setZero(3, 6);
    A(0, 3) = dt;
    A(1, 4) = dt;
    A(2, 5) = dt;
    double t2 = dt * dt / 2;
    B(0, 0) = t2;
    B(1, 1) = t2;
    B(2, 2) = t2;
    B(3, 0) = dt;
    B(4, 1) = dt;
    B(5, 2) = dt;
    C(0, 0) = 1;
    C(1, 1) = 1;
    C(2, 2) = 1;
    K = C;
    Qt.setIdentity(3, 3);
    Rt.setIdentity(3, 3);
    Qt(0, 0) = 4;
    Qt(1, 1) = 4;
    Qt(2, 2) = 1;
    Rt(0, 0) = 0.1;
    Rt(1, 1) = 0.1;
    Rt(2, 2) = 0.1;
    x.setZero(6);
  }
  // 成员函数
  inline void predict() {
    x = A * x;
    Sigma = A * Sigma * A.transpose() + B * Qt * B.transpose();
    return;
  }
  inline void reset(const Eigen::Vector3d& z) {
    x.head(3) = z;
    x.tail(3).setZero();
    Sigma.setZero();
  }
  inline bool checkValid(const Eigen::Vector3d& z) const {
    Eigen::MatrixXd K_tmp = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    Eigen::VectorXd x_tmp = x + K_tmp * (z - C * x);
    const double vmax = 4;
    if (x_tmp.tail(3).norm() > vmax) {
      return false;
    } else {
      return true;
    }
  }
  inline void update(const Eigen::Vector3d& z) {
    K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    x = x + K * (z - C * x);
    Sigma = Sigma - K * C * Sigma;
  }
  inline const Eigen::Vector3d pos() const {
    return x.head(3);
  }
  inline const Eigen::Vector3d vel() const {
    return x.tail(3);
  }
};

std::shared_ptr<Ekf> ekfPtr_;

void predict_state_callback(const ros::TimerEvent& event) {
  double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
  if (update_dt < 2.0) {
    ekfPtr_->predict();
  } else {
    //ROS_WARN("too long time no update!");
    return;
  }
  // publish target odom
  nav_msgs::Odometry target_odom;
  target_odom.header.stamp = ros::Time::now();
  target_odom.header.frame_id = "world";
  target_odom.pose.pose.position.x = ekfPtr_->pos().x();
  target_odom.pose.pose.position.y = ekfPtr_->pos().y();
  target_odom.pose.pose.position.z = ekfPtr_->pos().z();
  target_odom.twist.twist.linear.x = ekfPtr_->vel().x();
  target_odom.twist.twist.linear.y = ekfPtr_->vel().y();
  target_odom.twist.twist.linear.z = ekfPtr_->vel().z();
  target_odom.pose.pose.orientation.w = 1.0;
  target_odom_pub_.publish(target_odom);
}

// 获取外参并存储
void extrinsic_callback(const nav_msgs::Odometry::ConstPtr& msg){
  extrinsic_t_ = Eigen::Vector3d(msg->pose.pose.position.x, 
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z);
  extrinsic_r_ = Eigen::Quaterniond(msg->pose.pose.orientation.w, 
                                   msg->pose.pose.orientation.x, 
                                   msg->pose.pose.orientation.y, 
                                   msg->pose.pose.orientation.z).normalized();
  have_extrinstic_ = true;
}

// 目标与里程计同步回调
void update_state_callback(const object_detection_msgs::BoundingBoxesConstPtr &bboxes_msg, const nav_msgs::OdometryConstPtr &odom_msg) {
  // std::cout << "yolo stamp: " << bboxes_msg->header.stamp << std::endl;
  // std::cout << "odom stamp: " << odom_msg->header.stamp << std::endl;
  if (!have_extrinstic_){
    ROS_WARN_THROTTLE(1.0, "[ekf]Waiting for camera extrinsic...");
    return;
  }
  // 获取无人机位姿
  Eigen::Vector3d odom_p(odom_msg->pose.pose.position.x, 
                         odom_msg->pose.pose.position.y, 
                         odom_msg->pose.pose.position.z);
  Eigen::Quaterniond odom_q(odom_msg->pose.pose.orientation.w, 
                            odom_msg->pose.pose.orientation.x, 
                            odom_msg->pose.pose.orientation.y, 
                            odom_msg->pose.pose.orientation.z);
  odom_q.normalize();

  // NOTE check pitch
  // Eigen::Vector3d eulerAngle = odom_q.matrix().eulerAngles(2, 1, 0);
  // double pitch = fabs( eulerAngle[1] / M_PI * 180 );
  // pitch = pitch > 90 ? 180 - pitch : pitch;
  // if (pitch > pitch_thr_) {
  //   ROS_ERROR("pitch too large!");
  //   return;
  // }

  // 求解相机位姿
  Eigen::Vector3d cam_p = odom_q * extrinsic_t_ + odom_p;
  Eigen::Quaterniond cam_q = odom_q * extrinsic_r_;
  cam_q.normalize();

  // 处理目标消息
  auto yolo_bbox = bboxes_msg->bounding_boxes.front();
  double xmin, ymin, xmax, ymax;
  double depth;
  xmin = yolo_bbox.xmin;
  xmax = yolo_bbox.xmax;
  ymin = yolo_bbox.ymin;
  ymax = yolo_bbox.ymax;
  depth = yolo_bbox.depth;  //使用RGBD相机，获取深度与RGB图像对齐后的深度数据
  //ROS_INFO("Depth:%d",RGB_depth);

  // 检查目标框
  //int pixel_thr = 30;
  //if (ymin < pixel_thr || ymax > 480 - pixel_thr) {
  //  ROS_ERROR_THROTTLE(1.0,"pixel out of range!");
  //  return;
  //}

  // 计算目标在相机坐标系下的位置
  double y = ((ymin + ymax) * 0.5 - cy_) * depth / fy_;
  double x = ((xmin + xmax) * 0.5 - cx_) * depth / fx_;
  Eigen::Vector3d point(x, y, depth);
  
  // 计算目标在世界坐标系下的位置并发布
  Eigen::Vector3d p = cam_q * point + cam_p;
  // publish yolo odom
  nav_msgs::Odometry yolo_odom;
  yolo_odom.header.stamp = bboxes_msg->header.stamp;
  yolo_odom.header.frame_id = "world";
  yolo_odom.pose.pose.orientation.w = 1.0;
  yolo_odom.pose.pose.position.x = p.x();
  yolo_odom.pose.pose.position.y = p.y();
  yolo_odom.pose.pose.position.z = p.z();
  yolo_odom_pub_.publish(yolo_odom);
  // update target odom
  double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
  if (update_dt > 3.0) {
    ekfPtr_->reset(p);
    ROS_WARN("ekf reset!");
  } else if (ekfPtr_->checkValid(p)) {
    ekfPtr_->update(p);
  } else {
    ROS_ERROR("update invalid!");
    return;
  }
  last_update_stamp_ = ros::Time::now();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_ekf");
  ros::NodeHandle nh("~");
  last_update_stamp_ = ros::Time::now();

  std::vector<double> tmp;

  nh.getParam("cam_fx", fx_);
  nh.getParam("cam_fy", fy_);
  nh.getParam("cam_cx", cx_);
  nh.getParam("cam_cy", cy_);
  nh.getParam("pitch_thr", pitch_thr_);

  // 同步订阅里程计与目标框
  message_filters::Subscriber<object_detection_msgs::BoundingBoxes> yolo_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  std::shared_ptr<YoloOdomSynchronizer> yolo_campose_sync_Ptr_;
  // 订阅相机外参
  ros::Subscriber extrinsic_sub_;
  // 创建计时器
  ros::Timer ekf_predict_timer_; 
  // 发布目标位姿
  target_odom_pub_ = nh.advertise<nav_msgs::Odometry>("target_odom", 1);
  yolo_odom_pub_ = nh.advertise<nav_msgs::Odometry>("yolo_odom", 1);

  int ekf_rate = 30;
  nh.getParam("ekf_rate", ekf_rate);
  ekfPtr_ = std::make_shared<Ekf>(1.0/ekf_rate);

  yolo_sub_.subscribe(nh, "yolo", 1, ros::TransportHints().tcpNoDelay());
  odom_sub_.subscribe(nh, "odom", 100, ros::TransportHints().tcpNoDelay());
  extrinsic_sub_ = nh.subscribe("extrinsic", 1, extrinsic_callback);
  
  yolo_campose_sync_Ptr_ = std::make_shared<YoloOdomSynchronizer>(YoloOdomSyncPolicy(200), yolo_sub_, odom_sub_);
  yolo_campose_sync_Ptr_->registerCallback(boost::bind(&update_state_callback, _1, _2));
  ekf_predict_timer_ = nh.createTimer(ros::Duration(1.0 / ekf_rate), &predict_state_callback);
  
  ROS_WARN("Target_EKF node initialized!");
  ros::spin();
  return 0;
}


