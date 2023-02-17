/******************************************************************************
 * Copyright (C) 2015-2020, siqi Li
 *
 * NodeName: main
 * FileName: main.cpp
 *
 * Description: 将3D点云投影到2D图像上
 *
 * History:
 * lisiqi         2022/07/15    1.0.0    build this module.
 ******************************************************************************/

#include <dirent.h>
#include <sys/types.h>
#include "point2image.h"


int main(int argc, char **argv)
{
  // **************************************修改文件路径**************************************************
  // 输入：点云、图像、相机内外参所在路径、point2image结果保存路径
  std::string image_file = "../../test_data/Camera_images/h60/1_H60_1657079195300.jpg";
  std::string pcd_file = "../../test_data/LiDAR_pcd/1657079195300.pcd";
  std::string int_yaml_file = "../../test_data/BC25_camera_params_latest_0715/front_h60_intrinsics.yaml";
  std::string ext_yaml_file = "../../test_data/BC25_camera_params_latest_0715/front_h60_extrinsics.yaml";
  std::string save_file = "../../test_data/Result_point2image/h60/h60_1657079195300_point2image.jpg";
  // **************************************修改文件路径*************************************************

  Point2Image p;
  p.PointToImage(image_file, pcd_file, ext_yaml_file, int_yaml_file, save_file);

  return 0;
}