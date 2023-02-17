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

#include "point2image.h"

Point2Image::Point2Image() {}
Point2Image::~Point2Image() {}

inline bool Point2Image::LoadCameraIntrinsics(const std::string &yaml_file) {
  YAML::Node node = YAML::LoadFile(yaml_file);

  Eigen::VectorXf params(9 + 5);
  try {
    camera_width_ = node["width"].as<float>();
    camera_height_ = node["height"].as<float>();
    for (size_t i = 0; i < 9; ++i) {
      params(i) = node["K"][i].as<float>();
    }
    for (size_t i = 0; i < 5; ++i) {
      params(9 + i) = node["D"][i].as<float>();
    }
  } catch (YAML::Exception &e) {
    ROS_ERROR_STREAM("load camera intrisic file "
                     << yaml_file
                     << " with error, YAML exception: " << e.what());
    return false;
  }
  // fill in camera k and d
  intr_K_.row(0) << params(0), params(1), params(2);
  intr_K_.row(1) << params(3), params(4), params(5);
  intr_K_.row(2) << params(6), params(7), params(8);
  intr_D_.row(0) << params(9), params(10), params(11), params(12), params(13);
  return true;
}

inline bool Point2Image::LoadCameraExtrinsics(const std::string &yaml_file) {
  YAML::Node config = YAML::LoadFile(yaml_file);
  if (!config) {
    ROS_ERROR_STREAM("Open TransformationMatrix File:" << yaml_file
                                                       << " failed.");
    return false;
  }

  float x, y, z;
  float qx, qy, qz, qw;
  try {
    if (!config["transform"]) {
      ROS_ERROR_STREAM("Open TransformationMatrix File:"
                       << yaml_file << " has no transform.");
      return false;
    }
    if (config["transform"]["translation"]) {
      x = config["transform"]["translation"]["x"].as<float>();
      y = config["transform"]["translation"]["y"].as<float>();
      z = config["transform"]["translation"]["z"].as<float>();
    } else {
      ROS_ERROR_STREAM("TransformationMatrix File:"
                       << yaml_file << " has no transform:translation.");
      return false;
    }
    // fill rotation
    if (config["transform"]["rotation"]) {
      qx = config["transform"]["rotation"]["x"].as<float>();
      qy = config["transform"]["rotation"]["y"].as<float>();
      qz = config["transform"]["rotation"]["z"].as<float>();
      qw = config["transform"]["rotation"]["w"].as<float>();
    } else {
      ROS_ERROR_STREAM("TransformationMatrix File:"
                       << yaml_file << " has no transform:rotation.");
      return false;
    }
  } catch (const YAML::Exception &e) {
    ROS_ERROR_STREAM(yaml_file << " load failed. error:" << e.what());
    ROS_ERROR_STREAM("Please ensure param file is exist or format is correct");
    return false;
  }

  // fill rotation and translation
  Eigen::Quaternion<float> rotation(qw, qx, qy, qz);
  extr_R_ = rotation.toRotationMatrix();
  extr_t_.col(0) << x, y, z;

  float roll = 90 * M_PI / 180;    // 3-x-roll
  float pitch = -90 * M_PI / 180;  // 2-y-pitch
  float yaw = 0 * M_PI / 180;      // 1-z-yaw
  extr_R_init_ = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                 Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                 Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
  return true;
}

inline void Point2Image::PixelDenormalize(cv::Point3f &pt) {
  // add distortion
  Eigen::Matrix<double, 2, 1> pt2d(pt.x, pt.y);
  double r_sq = pt2d[0] * pt2d[0] + pt2d[1] * pt2d[1];

  Eigen::Matrix<double, 2, 1> pt2d_radial =
      pt2d * (1 + intr_D_[0] * r_sq + intr_D_[1] * r_sq * r_sq +
              intr_D_[4] * r_sq * r_sq * r_sq);
  Eigen::Matrix<double, 2, 1> dpt2d;
  dpt2d[0] = 2 * intr_D_[2] * pt2d[0] * pt2d[1] +
             intr_D_[3] * (r_sq + 2 * pt2d[0] * pt2d[0]);
  dpt2d[1] = intr_D_[2] * (r_sq + 2 * pt2d[1] * pt2d[1]) +
             2 * intr_D_[3] * pt2d[0] * pt2d[1];

  Eigen::Matrix<double, 2, 1> pt2d_distort;
  pt2d_distort[0] = pt2d_radial[0] + dpt2d[0];
  pt2d_distort[1] = pt2d_radial[1] + dpt2d[1];

  // add intrinsic K
  double focal_length_x = intr_K_(0, 0);
  double focal_length_y = intr_K_(1, 1);
  double center_x = intr_K_(0, 2);
  double center_y = intr_K_(1, 2);

  pt.x = pt2d_distort[0] * focal_length_x + center_x;
  pt.y = pt2d_distort[1] * focal_length_y + center_y;
}

inline bool Point2Image::ConvertPointVCS2IMG(const cv::Point3f &p3fVcs,
                                             cv::Point3f &p3fImg) {
  Eigen::Matrix<float, 3, 1> point_vcs;
  Eigen::Matrix<float, 3, 1> point_camera;
  point_vcs << p3fVcs.x, p3fVcs.y, p3fVcs.z;
  point_camera = extr_R_ * (extr_R_init_ * point_vcs + extr_t_);
  if (point_camera[2] <= 0) {
    return false;
  }
  float x = point_camera[0] / point_camera[2];
  float y = point_camera[1] / point_camera[2];
  float depth = point_camera[2];
  p3fImg = cv::Point3f(x, y, depth);
  PixelDenormalize(p3fImg);
  if (p3fImg.x >= 0 && p3fImg.x < camera_width_ && p3fImg.y >= 0 &&
      p3fImg.y < camera_height_) {
    return true;
  }
  return false;
}

inline cv::Scalar Point2Image::GetRainbowColor(const int i) {
  static std::vector<cv::Scalar> i_color;
  static bool init_flag = false;
  if (!init_flag) {
    for (int i = 0; i < 256; i++) {
      cv::Scalar p;
      p[0] = 0;
      p[1] = i;
      p[2] = 255;
      i_color.push_back(p);
    }
    for (int i = 0; i < 256; i++) {
      cv::Scalar p;
      p[0] = 0;
      p[1] = 255;
      p[2] = 255 - i;
      i_color.push_back(p);
    }
    for (int i = 0; i < 256; i++) {
      cv::Scalar p;
      p[0] = i;
      p[1] = 255;
      p[2] = 0;
      i_color.push_back(p);
    }
    for (int i = 0; i < 256; i++) {
      cv::Scalar p;
      p[0] = 255;
      p[1] = 255 - i;
      p[2] = 0;
      i_color.push_back(p);
    }
    for (int i = 0; i < 256; i++) {
      cv::Scalar p;
      p[0] = 255;
      p[1] = 0;
      p[2] = i;
      i_color.push_back(p);
    }
    init_flag = true;
  }
  return i_color[i * 5];
}

inline cv::Scalar Point2Image::ConvertDistance2Scalar(const float dis) {
  int i = dis * 5;
  if (i < 0) i = 0;
  if (i > 255) i = 255;
  return GetRainbowColor(i);
}

void Point2Image::PointToImage(const std::string img_file,
                               const std::string pcd_file,
                               const std::string ext_file,
                               const std::string int_file,
                               const std::string save_file) {
  bool success1 = LoadCameraIntrinsics(int_file);
  bool success2 = LoadCameraExtrinsics(ext_file);

  if (success1 && success2) {
    std::cout << "load yaml files success!" << std::endl;
  }

  cv::Mat image_origin = cv::imread(img_file);

  cv::Mat imgLeft90_ = image_origin.clone();

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_origin(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_withoutNAN(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *cloud_origin);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_origin, *cloud_withoutNAN, indices);

  std::vector<cv::Point3f> pts_3d;
  for (size_t i = 0; i < cloud_withoutNAN->size(); ++i) {
    pcl::PointXYZI point_3d = cloud_withoutNAN->points[i];
    if (point_3d.x > 2 && point_3d.x < 3 && point_3d.y > -10 &&
        point_3d.y < 10) {
      pts_3d.emplace_back(cv::Point3f(point_3d.x, point_3d.y, point_3d.z));
    }
    cv::Point3f p3fVcs(point_3d.x, point_3d.y, point_3d.z);
    cv::Point3f p3fImg;
    if (ConvertPointVCS2IMG(p3fVcs, p3fImg)) {
      cv::Point2f pt(p3fImg.x, p3fImg.y);
      float depth = p3fImg.z;
      // const float r = 10;
      // cv::Point2f pt1, pt2;
      // pt1.x = pt.x - r;
      // pt1.y = pt.y - r;
      // pt2.x = pt.x + r;
      // pt2.y = pt.y + r;
      // cv::rectangle(imgLeft90_, pt1, pt2, cv::Scalar(255, 255, 255), 1, 8,
      // 0);
      cv::circle(imgLeft90_, pt, 1, ConvertDistance2Scalar(depth), -1, 8);
    }
  }

  cv::imshow("Point2Image", imgLeft90_);
  cv::imwrite(save_file, imgLeft90_);
}