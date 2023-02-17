/******************************************************************************
 * Copyright (C) 2015-2020, siqi Li
 *
 * NodeName: point2image
 * FileName: point2image.h
 *
 * Description: 将3D点云投影到2D图像上
 *
 * History:
 * lisiqi         2022/07/15    1.0.0    build this module.
 ******************************************************************************/

#ifndef __POINT_TO_IMAGE_H__
#define __POINT_TO_IMAGE_H__

#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Point2Image {
 public:
  Point2Image();
  ~Point2Image();

  bool LoadCameraIntrinsics(const std::string &yaml_file);
  bool LoadCameraExtrinsics(const std::string &yaml_file);
  void PixelDenormalize(cv::Point3f &pt);
  bool ConvertPointVCS2IMG(const cv::Point3f &p3fVcs, cv::Point3f &p3fImg);
  cv::Scalar GetRainbowColor(const int i);
  cv::Scalar ConvertDistance2Scalar(const float dis);
  void PointToImage(const std::string img_file, const std::string pcd_file,
                    const std::string ext_file, const std::string int_file,
                    const std::string save_file);

  Eigen::Matrix<float, 3, 3> intr_K_;
  Eigen::Matrix<float, 1, 5> intr_D_;
  Eigen::Matrix<float, 3, 3> extr_R_init_;
  Eigen::Matrix<float, 3, 3> extr_R_;
  Eigen::Matrix<float, 3, 1> extr_t_;
  float camera_width_ = 0.0;
  float camera_height_ = 0.0;
};

#endif  // __POINT_TO_IMAGE_H__