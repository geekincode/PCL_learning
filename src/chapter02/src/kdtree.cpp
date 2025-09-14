/*
 * @Description: https://www.cnblogs.com/li-yao7758258/p/6437440.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-08 21:46:40
 * @LastEditTime: 2020-10-27 16:03:16
 * @FilePath: /pcl-learning/02kdtree/1kdtree_search/kdtree_search.cpp
 */


#include <pcl/point_cloud.h>        //点类型定义头文件
#include <pcl/kdtree/kdtree_flann.h> //kdtree类定义头文件
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <vector>
#include <ctime>

int
main (int argc, char** argv)
{
  srand (time (NULL));   //用系统时间初始化随机种子
  //创建一个PointCloud<pcl::PointXYZ>
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // 随机点云生成
  cloud->width = 1000;             //此处点云数量
  cloud->height = 1;                //表示点云为无序点云
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)   //循环填充点云数据
  {
    cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f); // // 产生数值为0-1024的浮点数
    cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
  }
    std::string file_path = "./src/chapter02/pcd/test.pcd";
    if (pcl::io::savePCDFileASCII(file_path, *cloud) == 0) {
        std::cout << "成功生成点云: " << file_path << " (" << cloud->size() << " 个点)\n";
    } else {
        std::cerr << "保存失败! 错误原因:\n"
                  << "1. 目录不存在: " << file_path << "\n"
                  << "2. 写入权限不足\n"
                  << "3. 路径无效\n";
    }

 //创建KdTreeFLANN对象，并把创建的点云设置为输入,创建一个searchPoint变量作为查询点
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree; // pcl::KdTreeFLANN<PointT, Dist>::setInputCloud (const PointCloudConstPtr &cloud, const IndicesConstPtr &indices)
  //设置搜索空间
  kdtree.setInputCloud (cloud);
  //设置查询点并赋随机值
  pcl::PointXYZ searchPoint;
  searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

  // K 临近搜索
  //创建一个整数（设置为10）和两个向量来存储搜索到的K近邻，两个向量中，一个存储搜索到查询点近邻的索引，另一个存储对应近邻的距离平方
  int K = 5;   // 与查询点的欧式距离最近的K个点

  std::vector<int> pointIdxNKNSearch(K);      //存储查询点近邻索引
  std::vector<float> pointNKNSquaredDistance(K); //存储近邻点对应距离平方
  //打印相关信息
  std::cout << "K nearest neighbor search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;

  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )  //执行K近邻搜索
  {
     //打印所有近邻坐标
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
  }
  /**********************************************************************************
   下面的代码展示查找到给定的searchPoint的某一半径（随机产生）内所有近邻，重新定义两个向量
   pointIdxRadiusSearch  pointRadiusSquaredDistance来存储关于近邻的信息
   ********************************************************************************/
  // 半径 R内近邻搜索方法

  std::vector<int> pointIdxRadiusSearch;           //存储近邻索引
  std::vector<float> pointRadiusSquaredDistance;   //存储近邻对应距离的平方

  float radius = 256.0f * rand () / (RAND_MAX + 1.0f);   //随机的生成某一半径
  //打印输出
  std::cout << "Neighbors within radius search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with radius=" << radius << std::endl;

  // 假设我们的kdtree返回了大于0个近邻。那么它将打印出在我们"searchPoint"附近的10个最近的邻居并把它们存到先前创立的向量中。
  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )  //执行半径R内近邻搜索方法
  {
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
      std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
  }


  return 0;
}

