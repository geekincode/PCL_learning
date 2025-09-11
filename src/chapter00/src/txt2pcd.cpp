#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>   

int main(int argc, char *argv[])
{
	std::ifstream infile;
	infile.open("./src/chapter01/pcd/ism_test_wolf.txt");
	
	float x, y, z;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	while (infile >> x >> y >> z)
	{
		cloud->push_back(pcl::PointXYZ(x, y, z));
	}
	pcl::io::savePCDFileBinary("./src/chapter01/output/ism_test_wolf.pcd", *cloud);

	return 0;
}
