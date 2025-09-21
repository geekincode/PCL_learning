/*
 * @Description: 从一个点云中提取索引        https://www.cnblogs.com/li-yao7758258/p/6473304.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-20 15:00:12
 * @LastEditTime: 2020-10-20 15:19:31
 * @FilePath: /pcl-learning/09filters滤波/5从一个点云中提取索引/extract_indices.cpp
 */
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>  // 从一个点云中提取索引 
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <chrono>

int
main (int argc, char** argv)
{  

  /**********************************************************************************************************
   从输入的.PCD 文件载入数据后，创建一个VoxelGrid滤波器对数据进行下采样，在这里进行下才样是为了加速处理过程，
   越少的点意味着分割循环中处理起来越快
   **********************************************************************************************************/

  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);//申明滤波前后的点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blob_xyz (new pcl::PointCloud<pcl::PointXYZ>); // 可视化用
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p_1 (new pcl::PointCloud<pcl::PointXYZ>);  // 第一个平面
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p_2 (new pcl::PointCloud<pcl::PointXYZ>);  // 第二个平面

  // 读取PCD文件
  pcl::PCDReader reader;
  reader.read ("./src/chapter09/src/5从一个点云中提取索引/table_scene_lms400.pcd", *cloud_blob);
   //统计滤波前的点云个数
  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // 转换为模板点云，用于可视化
  pcl::fromPCLPointCloud2 (*cloud_blob, *cloud_blob_xyz);

  // 创建体素栅格下采样: 下采样的大小为1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;  //体素栅格下采样对象
  sor.setInputCloud (cloud_blob);             //原始点云
  sor.setLeafSize (0.01f, 0.01f, 0.01f);    // 设置采样体素大小
  sor.filter (*cloud_filtered_blob);        //保存

  // 转换为模板点云
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // 保存下采样后的点云
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("./src/chapter09/src/5从一个点云中提取索引/table_scene_lms400_downsampled.pcd", *cloud_filtered, false);
  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());   
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  pcl::SACSegmentation<pcl::PointXYZ> seg;               //创建分割对象
  seg.setOptimizeCoefficients (true);                    //设置对估计模型参数进行优化处理
  seg.setModelType (pcl::SACMODEL_PLANE);                //设置分割模型类别
  seg.setMethodType (pcl::SAC_RANSAC);                   //设置用哪个随机参数估计方法
  seg.setMaxIterations (1000);                            //设置最大迭代次数
  seg.setDistanceThreshold (0.01);                      //判断是否为模型内点的距离阀值

  // 设置ExtractIndices的实际参数
  pcl::ExtractIndices<pcl::PointXYZ> extract;        //创建点云提取对象

  int i = 0, nr_points = (int) cloud_filtered->points.size (); // 点云总数
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // 为了处理点云包含的多个模型，在一个循环中执行该过程并在每次模型被提取后，保存剩余的点进行迭代
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);  // 
    extract.setNegative (false);
    extract.filter (*cloud_p);
    
    // 保存前两个平面
    if (i == 0) {
      *cloud_p_1 = *cloud_p;
    } else if (i == 1) {
      *cloud_p_2 = *cloud_p;
    }
    
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    std::stringstream ss;
    ss << "./src/chapter09/src/5从一个点云中提取索引/table_scene_lms400_plane_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
    
    // 处理两个平面后退出
    if (i >= 2) {
      break;
    }
  }
  
  // 创建四个可视化窗口
  // 窗口1: 原始点云
  pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("Original Point Cloud"));
  viewer1->setBackgroundColor(0, 0, 0);
  viewer1->addPointCloud<pcl::PointXYZ>(cloud_blob_xyz, "original_cloud");
  viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");
  viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "original_cloud");
  viewer1->addCoordinateSystem(1.0);
  viewer1->initCameraParameters();

  // 窗口2: 下采样点云
  pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer("Downsampled Point Cloud"));
  viewer2->setBackgroundColor(0, 0, 0);
  viewer2->addPointCloud<pcl::PointXYZ>(cloud_filtered, "downsampled_cloud");
  viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "downsampled_cloud");
  viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "downsampled_cloud");
  viewer2->addCoordinateSystem(1.0);
  viewer2->initCameraParameters();

  // 窗口3: 第一个平面点云
  pcl::visualization::PCLVisualizer::Ptr viewer3(new pcl::visualization::PCLVisualizer("First Planar Component"));
  viewer3->setBackgroundColor(0, 0, 0);
  viewer3->addPointCloud<pcl::PointXYZ>(cloud_p_1, "planar_cloud_1");
  viewer3->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "planar_cloud_1");
  viewer3->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "planar_cloud_1");
  viewer3->addCoordinateSystem(1.0);
  viewer3->initCameraParameters();

  // 窗口4: 第二个平面点云
  pcl::visualization::PCLVisualizer::Ptr viewer4(new pcl::visualization::PCLVisualizer("Second Planar Component"));
  viewer4->setBackgroundColor(0, 0, 0);
  viewer4->addPointCloud<pcl::PointXYZ>(cloud_p_2, "planar_cloud_2");
  viewer4->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "planar_cloud_2");
  viewer4->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "planar_cloud_2");
  viewer4->addCoordinateSystem(1.0);
  viewer4->initCameraParameters();

  // 主循环
  while (!viewer1->wasStopped() && !viewer2->wasStopped() && 
         !viewer3->wasStopped() && !viewer4->wasStopped())
  {
    viewer1->spin();
    viewer2->spin();
    viewer3->spin();
    viewer4->spin();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return (0);
}