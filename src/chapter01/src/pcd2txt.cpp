#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>   

int main(int argc, char *argv[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("./src/chapter01/pcd/ism_test_wolf.pcd", *cloud);

	std::ofstream outfile;
	outfile.open("./src/chapter01/output/ism_test_wolf.txt");

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		outfile << cloud->points[i].x << "\t" << cloud->points[i].y << "\t" << cloud->points[i].z << "\n";
	}

	return 0;
}
