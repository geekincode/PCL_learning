/*
 * @Description: 使用statisticalOutlierRemoval滤波器移除离群点
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-20 11:37:27
 * @LastEditTime: 2020-10-20 11:56:46
 * @FilePath: /pcl-learning/09filters滤波/3滤波器移除离群点/statistical_removal.cpp
 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers (new pcl::PointCloud<pcl::PointXYZ>);

  // 定义读取对象
  pcl::PCDReader reader;
  // 读取点云文件
  reader.read<pcl::PointXYZ> ("./src/chapter09/src/3滤波器移除离群点/table_scene_lms400.pcd", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
   //个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //创建滤波器对象
  sor.setInputCloud (cloud);                           //设置待滤波的点云
  sor.setMeanK (50);                               //设置在进行统计时考虑查询点临近点数
  sor.setStddevMulThresh (1.0);                      //设置判断是否为离群点的阀值
  sor.filter (*cloud_filtered);                    //存储

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("./src/chapter09/src/3滤波器移除离群点/table_scene_lms400_inliers.pcd", *cloud_filtered, false);
//true：滤波结果取反，被过滤的点
  sor.setNegative (true);
  sor.filter (*cloud_outliers);
  writer.write<pcl::PointXYZ> ("./src/chapter09/src/3滤波器移除离群点/table_scene_lms400_outliers.pcd", *cloud_outliers, false);

  // 创建三个可视化窗口
  // 原始点云窗口
  pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("Original Point Cloud"));
  viewer1->setBackgroundColor(0, 0, 0);
  viewer1->addPointCloud<pcl::PointXYZ>(cloud, "original_cloud");
  viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");
  viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "original_cloud");
  viewer1->addCoordinateSystem(1.0);
  viewer1->initCameraParameters();

  // 内点窗口
  pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer("Inliers"));
  viewer2->setBackgroundColor(0, 0, 0);
  viewer2->addPointCloud<pcl::PointXYZ>(cloud_filtered, "inliers_cloud");
  viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "inliers_cloud");
  viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "inliers_cloud");
  viewer2->addCoordinateSystem(1.0);
  viewer2->initCameraParameters();

  // 外点窗口
  pcl::visualization::PCLVisualizer::Ptr viewer3(new pcl::visualization::PCLVisualizer("Outliers"));
  viewer3->setBackgroundColor(0, 0, 0);
  viewer3->addPointCloud<pcl::PointXYZ>(cloud_outliers, "outliers_cloud");
  viewer3->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "outliers_cloud");
  viewer3->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "outliers_cloud");
  viewer3->addCoordinateSystem(1.0);
  viewer3->initCameraParameters();

  // 主循环
  while (!viewer1->wasStopped() && !viewer2->wasStopped() && !viewer3->wasStopped())
  {
    viewer1->spin();
    viewer2->spin();
    viewer3->spin();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return (0);
}