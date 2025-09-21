/*
 * @Description: 快速点特征直方图(FPFH)描述子 https://www.cnblogs.com/li-yao7758258/p/6481738.html
 * // 计算fpfh特征
 * // input: keypoints ,cloud , normals
 * // output: FPFH descriptors
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-19 18:36:27
 * @LastEditTime: 2020-10-19 19:00:53
 * @FilePath: /pcl-learning/10features特征/5快速点特征直方图(FPFH)描述子/FPFH.cpp
 */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>           // FPFH特征估计类头文件
#include <pcl/features/normal_3d.h>      // 法线估计
#include <pcl/search/kdtree.h>           // KdTree搜索方法
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_plotter.h> // 直方图可视化

int main(int argc, char** argv)
{
    // 其他相关操作
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 打开点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("src/chapter10/src/4点特征直方图（PFH）描述子/ism_test_cat.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file ism_test_cat.pcd\n");
        return -1;
    }
    
    // 估计法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    ne.setRadiusSearch(0.03);
    ne.compute(*normals);

    // 创建FPFH估计对象fpfh，并把输入数据集cloud和法线normals传递给它。
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);

    // 如果点云是类型为PointNormal，则执行fpfh.setInputNormals (cloud);
    // 创建一个空的kd树对象tree，并把它传递给FPFH估计对象。
    // 基于已知的输入数据集，建立kdtree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>());
    fpfh.setSearchMethod(tree2);

    // 输出数据集
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());

    // 使用所有半径在5厘米范围内的邻元素
    // 注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
    fpfh.setRadiusSearch(0.05);

    // 计算获取特征向量
    fpfh.compute(*fpfhs);

    // fpfhs->points.size ()应该和input cloud->points.size ()有相同的大小，即每个点有一个特征向量
    
    // ========直方图可视化=============================
    pcl::visualization::PCLPlotter plotter;
    plotter.addFeatureHistogram(*fpfhs, 300); //设置的很坐标长度，该值越大，则显示的越细致
    plotter.plot();
    
    return 0;
}