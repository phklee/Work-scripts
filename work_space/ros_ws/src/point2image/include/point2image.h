/******************************************************************************
 * Copyright (C) 2015-2020, siqi Li
 *
 * NodeName: point2image
 * FileName: point2image.cpp
 *
 * Description: 将3D点云投影到2D图像上
 *
 * History:
 * lisiqi         2022/07/15    1.0.0    build this module.
 ******************************************************************************/

#ifndef __POINT_TO_IMAGE_H__
#define __POINT_TO_IMAGE_H__

#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#define __APP_NAME__ "point_to_image"

namespace point2image {
class Point2Image {
 public:
  Point2Image();
  ~Point2Image();
  void mainLoop();

 private:
  // ROS句柄
  ros::NodeHandle nh_;          //(全局空间)
  ros::NodeHandle nh_private_;  //(局部空间)
  // ROS订阅者
  ros::Subscriber lidar_sub_;
  ros::Subscriber cam_sub_;
  // ROS发布者
  ros::Publisher lidar2cam_pub_;

  bool is_load_int_;
  bool is_load_ext_;
  bool is_lidar2cam_;

  std::string str_input_topic_lidar_;       //订阅话题名lidar
  std::string str_input_topic_cam_;         //订阅话题名camera
  std::string str_output_topic_lidar2cam_;  //发布话题名lidar2cam
  float frequency_;

  Eigen::Matrix<float, 3, 3> intr_K_;
  Eigen::Matrix<float, 1, 5> intr_D_;
  Eigen::Matrix<float, 3, 3> extr_R_init_;
  Eigen::Matrix<float, 3, 3> extr_R_;
  Eigen::Matrix<float, 3, 1> extr_t_;
  float camera_width_ = 0.0;
  float camera_height_ = 0.0;

  void InitROSIO(ros::NodeHandle &in_nh_private);
  bool LoadCameraIntrinsics(const std::string &yaml_file);
  bool LoadCameraExtrinsics(const std::string &yaml_file);
  void PixelDenormalize(cv::Point3f &pt);
  bool ConvertPointVCS2IMG(const cv::Point3f &p3fVcs, cv::Point3f &p3fImg);
  cv::Scalar GetRainbowColor(const int i);
  cv::Scalar ConvertDistance2Scalar(const float dis);

  void PointToImage(const std::string img_file, const std::string pcd_file,
                    const std::string ext_file, const std::string int_file,
                    const std::string save_file);

  void LidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void CamCallback(const sensor_msgs::Image::ConstPtr &msg);
};
}  // namespace point2image

#endif  // __POINT_TO_IMAGE_H__