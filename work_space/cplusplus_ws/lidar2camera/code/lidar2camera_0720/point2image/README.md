## 点云投影到图像上
1、输入说明:
- 图像 (.jpg)路径
- 点云 (.pcd)路径
- 相机内参 (.yaml)路径，用到了D(畸变系数1x5)、K(内参矩阵3x3)
- 相机外参 (.yaml)路径, 用到了旋转矩阵的四元素(x, y, z, w)、Lidar和Camera之间的位移距离(x, y, z)
- 点云投影到图像上的结果(.jpg)路径

2、进入容器
```shell
$ cd ${lidar2camera}/docker/x86
$ ./start_docker.sh
$ ./into_docker.sh
```

3、编译执行程序
在`${lidar2camera}/point2image/src/main.cpp`修改输入文件的路径，然后编译执行:
```shell
$ cd ${lidar2camera}/point2image/
$ mkdir build
$ cd build
$ cmake ..
$ make  # 编译
$ ./bin/point2image  # 运行
```