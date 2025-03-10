#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.
 
int main (int argc, char **argv)
{
  ros::init (argc, argv, "UandBdetect");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;
  pcl::io::loadPCDFile ("/home/user/lidar2camera/1656648495300.pcd", cloud);   
// 修改自己pcd文件所在路径
  //Convert the cloud to ROS message
  pcl::toROSMsg(cloud, output);
  output.header.frame_id = "base_link";
  output.header.stamp=ros::Time(std::stoi("1656648495"),std::stoi("299866000"));
  
  
  //this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer    
    //！！！这一步需要注意，是后面rviz的 fixed_frame 重点！！
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}