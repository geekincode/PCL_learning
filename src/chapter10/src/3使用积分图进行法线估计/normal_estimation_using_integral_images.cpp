/*
 * @Description: 使用积分图计算一个有序的点云的法线，注意此方法只适用有序点云 : https://www.cnblogs.com/li-yao7758258/p/6479255.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-19 16:08:38
 * @LastEditTime: 2020-10-19 16:11:36
 * @FilePath: /pcl-learning/10features特征/3使用积分图进行法线估计/normal_estimation_using_integral_images.cpp
 */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>  // 积分图法线估计类头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <thread>
#include <chrono>

int main() {
    //打开点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("./src/chapter10/src/3使用积分图进行法线估计/table_scene_mug_stereo_textured.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file table_scene_mug_stereo_textured.pcd\n");
        return -1;
    }
    
    //创建法线估计向量
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    
    /****************************************************************************************
    三种法线估计方法
     COVARIANCE_MATRIX 模式从具体某个点的局部邻域的协方差矩阵创建9个积分，来计算这个点的法线
    AVERAGE_3D_GRADIENT   模式创建6个积分图来计算水平方向和垂直方向的平滑后的三维梯度并使用两个梯度间的向量积计算法线
    AVERAGE_DEPTH_CHANGE  模式只创建了一个单一的积分图，从而平均深度变化计算法线
    ********************************************************************************************/
    
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);  //设置法线估计的方式AVERAGE_3D_GRADIENT
    ne.setMaxDepthChangeFactor(0.02f);   //设置深度变化系数
    ne.setNormalSmoothingSize(10.0f);   //设置法线优化时考虑的邻域的大小
    ne.setInputCloud(cloud);               //输入的点云
    ne.compute(*normals);                    //计算法线
    
    //可视化
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCL Viewer"));   //视口的名称
    viewer->setBackgroundColor (0.0, 0.0, 0.5);    //背景颜色的设置
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals, 50, 0.05, "normals");  //将法线加入到点云中
    
    while (!viewer->wasStopped ()) {
        viewer->spin();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return 0;
}