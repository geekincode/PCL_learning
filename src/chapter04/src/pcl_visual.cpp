#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

int main(int argc, char** argv)
{
    // Check command line arguments
    // if (argc != 2) {
    //     std::cerr << "Usage: " << argv[0] << " <pcd_file>" << std::endl;
    //     return -1;
    // }

    // Create point cloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string file_pcd = "/home/rm/PCL/pcl_ws/src/chapter04/pcd/ism_test_wolf.pcd";
    
    // Load PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_pcd, *cloud) == -1) {
        std::cerr << "Couldn't read file " << file_pcd << std::endl;
        return -1;
    }

    // Initialize visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    
    // Add point cloud to visualizer
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "sample cloud");

    // Visualization loop
    while (!viewer->wasStopped()) {
        viewer->spin();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}