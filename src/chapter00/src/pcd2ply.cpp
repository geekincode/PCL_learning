#include <iostream>          
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h> 
#include <pcl/io/ply_io.h>  

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("./src/chapter01/pcd/ism_test_wolf.pcd", *cloud);
	pcl::io::savePLYFileBinary("./src/chapter01/output/ism_test_wolf.ply", *cloud);

	return 0;
}
