/*
 * @Description: 双边滤波 （耗时较长）   https://github.com/MNewBie/PCL-Notes/blob/master/chapter8.md
 * 运行：（别忘了携带参数 pcd）
 *  ./bilateral_filter ../table_scene_lms400.pcd
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-20 15:50:44
 * @LastEditTime: 2020-10-20 15:55:27
 * @FilePath: /pcl-learning/09filters滤波/7双边滤波/bilateral_filter.cpp
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/impl/bilateral.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

void bilateralFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr &input, pcl::PointCloud<pcl::PointXYZI>::Ptr& output)
{
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::BilateralFilter<pcl::PointXYZI> fbf;
    fbf.setInputCloud(input);
    fbf.setSearchMethod(tree1);
    fbf.setStdDev(0.1);
    fbf.setHalfSize(0.1);
    
    #ifdef _OPENMP
    omp_set_num_threads(4); // 设置使用4个线程，可根据CPU核心数调整
    #endif

    std::cout << "Starting bilateral filter on " << input->size() << " points..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    
    fbf.filter(*output);
    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;
    std::cout << "Bilateral filter completed in " << diff.count() << " seconds" << std::endl;
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>); // 需要PointXYZI 
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read<pcl::PointXYZI>(argv[1], *cloud);
	
	bilateralFilter(cloud, cloud_filtered);

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->addPointCloud<pcl::PointXYZI>(cloud_filtered, "sample cloud");
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spin();
		pcl_sleep(0.1);
	}

	return (0);
}