#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/StdVector>
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include <limits>

int main(int argc, char** argv)
{
    // 检查文件是否存在
    std::string filename = "/home/rm/PCL/pcl_ws/src/chapter03/pcd/pyramid_color.pcd";
    std::ifstream file_check(filename);
    if (!file_check.good()) {
        std::cerr << "Error: PCD file '" << filename << "' does not exist or cannot be opened." << std::endl;
        return -1;
    }
    file_check.close();

    // Load point cloud from PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
        PCL_ERROR("Couldn't read file input.pcd\n");
        return (-1);
    }
    
    std::cout << "Loaded " << cloud->points.size() << " points from " << filename << std::endl;

    // Create octree structure
    float resolution = 0.05f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    
    std::cout << "Octree created with resolution: " << resolution << std::endl;

    // Create visualizer
    pcl::visualization::PCLVisualizer viewer("Octree Visualization");
    viewer.setBackgroundColor(0, 0, 0);
    
    // Add original point cloud
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "original_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    
    std::cout << "Original point cloud added to viewer" << std::endl;

    // Visualize octree leaf nodes
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_centers(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto leaf_it = octree.leaf_depth_begin(); leaf_it != octree.leaf_depth_end(); ++leaf_it) {
        Eigen::Vector3f min_pt, max_pt;
        octree.getVoxelBounds(leaf_it, min_pt, max_pt); // 获取体素边界

        // 验证边界有效性（非 NaN 且 min < max）
        if (std::isnan(min_pt.x()) || std::isnan(min_pt.y()) || std::isnan(min_pt.z()) ||
            std::isnan(max_pt.x()) || std::isnan(max_pt.y()) || std::isnan(max_pt.z()) ||
            min_pt.x() >= max_pt.x() || min_pt.y() >= max_pt.y() || min_pt.z() >= max_pt.z()) {
            continue; // 跳过无效体素
        }

        // 计算中心点（转换为 pcl::PointXYZ）
        pcl::PointXYZ center;
        center.x = (min_pt.x() + max_pt.x()) / 2.0f;
        center.y = (min_pt.y() + max_pt.y()) / 2.0f;
        center.z = (min_pt.z() + max_pt.z()) / 2.0f;
        voxel_centers->points.push_back(center);
    }
    
    std::cout << "Found " << voxel_centers->points.size() << " valid voxels" << std::endl;

    // Add voxel centers as cubes
    for (size_t i = 0; i < voxel_centers->points.size() && i < 10; ++i) { // 限制调试数量为 10 个
        pcl::PointXYZ voxel = voxel_centers->points[i];
        
        // 计算立方体边界
        float x_min = voxel.x - resolution/2;
        float x_max = voxel.x + resolution/2;
        float y_min = voxel.y - resolution/2;
        float y_max = voxel.y + resolution/2;
        float z_min = voxel.z - resolution/2;
        float z_max = voxel.z + resolution/2;

        // 验证边界有效性（min < max）
        if (x_min >= x_max || y_min >= y_max || z_min >= z_max) {
            std::cerr << "Invalid cube bounds for voxel " << i << ". Skipping." << std::endl;
            continue;
        }

        // 使用 double 类型的颜色参数
        double r = 1.0, g = 0.0, b = 0.0; // 红色
        std::string cube_id = "voxel_" + std::to_string(i);
        viewer.addCube(x_min, x_max, y_min, y_max, z_min, z_max, r, g, b, cube_id);
        
        // 设置立方体为线框模式
        viewer.setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
            cube_id
        );
    }
    
    std::cout << "Voxels added to viewer. Starting visualization loop..." << std::endl;

    // Main visualization loop
    while (!viewer.wasStopped()) {
        viewer.spin();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "Visualization ended." << std::endl;

    return 0;
}