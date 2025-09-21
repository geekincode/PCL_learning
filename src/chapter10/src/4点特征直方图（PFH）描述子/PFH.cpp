#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>        // PFH 特征估计
#include <pcl/features/normal_3d.h>  // 法线估计
// #include <pcl/visualization/histogram_visualizer.h> // 直方图窗口可视化
#include <pcl/visualization/pcl_plotter.h> // 使用PCLPlotter解决VTK兼容性问题
#include <pcl/search/kdtree.h>       // KdTree搜索
#include <iostream>

/**
 * @brief 主函数：计算并可视化点云的PFH特征
 * 
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 *              argv[1]: 输入的PCD文件路径
 * 
 * @return int 程序执行状态码，0表示成功，-1表示失败
 */
int main(int argc, char** argv)
{
    // 检查命令行参数
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " input.pcd\n";
        return -1;
    }

    // 1. 读取输入点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(argv[1], *cloud) == -1)
    {
        std::cerr << "Cannot read PCD file!\n";
        return -1;
    }
    
    if (cloud->empty())
    {
        std::cerr << "错误：点云为空！\n";
        return -1;
    }
    
    std::cout << "Loaded " << cloud->size() << " points.\n";

    // 2. 估计点云法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    
    // 创建KdTree用于邻域搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    
    // 输出法线点云
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // 设置法线估计的搜索半径
    ne.setRadiusSearch(0.03);  // 可根据具体数据调整
    
    // 执行法线估计
    ne.compute(*normals);
    
    std::cout << "Normal estimation done.\n";

    // 3. 计算PFH特征描述子
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(cloud);
    pfh.setInputNormals(normals);
    
    // 为PFH估计器创建新的KdTree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
    pfh.setSearchMethod(tree2);
    
    // 设置PFH特征的搜索半径
    pfh.setRadiusSearch(0.05);  // 可根据具体数据调整
    
    // 存储PFH特征的点云
    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_features(new pcl::PointCloud<pcl::PFHSignature125>);
    
    // 计算PFH特征
    pfh.compute(*pfh_features);
    
    std::cout << "Computed PFH for " << pfh_features->size() << " points.\n";

    // 4. 可视化第一个点的PFH直方图
    // 根据经验教训和项目规范，使用PCLPlotter解决VTK兼容性问题
    pcl::visualization::PCLPlotter plotter;
    plotter.addFeatureHistogram(*pfh_features, "pfh", 0);
    
    std::cout << "正在显示第一个点的PFH特征直方图...\n";
    std::cout << "提示：关闭可视化窗口以退出程序。\n";
    
    // 显示直方图
    plotter.plot();

    return 0;
}