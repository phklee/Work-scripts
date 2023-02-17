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

int main(int argc, char** argv) {
  // ros程序正常运行必须得初始化
  ros::init(argc, argv, __APP_NAME__);

  point2image::Point2Image lidar2cam;

  lidar2cam.mainLoop();

  return 0;
}