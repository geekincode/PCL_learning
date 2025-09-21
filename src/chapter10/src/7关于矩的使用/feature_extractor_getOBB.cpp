#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile("src/chapter10/src/7关于矩的使用/ism_test_cat.pcd", *cloud) == -1)
        return (-1);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass(new pcl::PointCloud<pcl::PointXYZ>());
    //if (pcl::io::loadPCDFile("cloud_filtered.pcd",*cloud_pass)==-1)
    //	return (-1);


    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;//实例化一个对象
    feature_extractor.setInputCloud(cloud);//设置输入点云
    feature_extractor.compute();//开始特征计算

    std::vector <float> moment_of_inertia;//存放惯性距的特征向量
    std::vector <float> eccentricity;//存放偏心率的特征向量
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia(moment_of_inertia);//计算出的惯性矩
    feature_extractor.getEccentricity(eccentricity);//计算出的偏心率
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);//OBB对应的相关参数
    feature_extractor.getEigenValues(major_value, middle_value, minor_value);//三个特征值
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);//三个特征向量
    feature_extractor.getMassCenter(mass_center);//计算质心

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("rect")); 
    viewer->setBackgroundColor(1, 1, 1);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->addPointCloud<pcl::PointXYZ>(cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 0, 255, 0), "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
    Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
    std::cout << "position_OBB: " << position_OBB << endl;
    std::cout << "mass_center: " << mass_center << endl;//中心坐标
    Eigen::Quaternionf quat(rotational_matrix_OBB);
    viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "OBB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "OBB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "OBB");
    viewer->setRepresentationToWireframeForAllActors();//将所有actor的可视化表示更改为线框表示
    pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
    pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
    pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
std::cout << "size of cloud :" << cloud->points.size() << endl;
std::cout << "moment_of_inertia :" << moment_of_inertia.size() << endl;
std::cout << "eccentricity :" << eccentricity.size() << endl;
float height = max_point_OBB.z - min_point_OBB.z;
float width = max_point_OBB.y - min_point_OBB.y;
float depth = max_point_OBB.x - min_point_OBB.x;
cout << "长：" << depth << endl;
cout << "宽：" << width << endl;
cout << "高：" << height << endl;

while (!viewer->wasStopped())
{
viewer->spin();
boost::this_thread::sleep(boost::posix_time::microseconds(100000));
}

system("pause");
return (0);
}
