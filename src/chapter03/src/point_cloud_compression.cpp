/*
 * @Description: 点云压缩：https://www.cnblogs.com/li-yao7758258/p/6436117.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-13 10:10:33
 * @LastEditTime: 2020-10-27 16:02:18
 * @FilePath: /pcl-learning/03octree/1point_cloud_compression/point_cloud_compression.cpp
 */


#include <pcl/point_cloud.h>                // 点云类型
#include <pcl/point_types.h>                //点数据类型
#include <pcl/io/openni_grabber.h>          //点云获取接口类
#include <pcl/visualization/cloud_viewer.h> //点云可视化类

#include <pcl/compression/octree_pointcloud_compression.h> //点云压缩类

#include <stdio.h>
#include <sstream>
#include <stdlib.h>

#ifdef WIN32
#define sleep(x) Sleep((x)*1000)
#endif
#include <functional>
#include <pcl/io/pcd_io.h>

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer() : viewer(" Point Cloud Compression Example")
    {
    }
    /************************************************************************************************
  在OpenNIGrabber采集循环执行的回调函数cloud_cb_中，首先把获取的点云压缩到stringstream缓冲区，下一步就是解压缩，
  它对压缩了的二进制数据进行解码，存储在新的点云中解码了点云被发送到点云可视化对象中进行实时可视化
*************************************************************************************************/

    void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
        if (!viewer.wasStopped())
        {
            // 存储压缩点云的字节流对象
            std::stringstream compressedData;
            // 存储输出点云
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>());

            // 压缩点云
            PointCloudEncoder->encodePointCloud(cloud, compressedData);

            // 解压缩点云
            PointCloudDecoder->decodePointCloud(compressedData, cloudOut);

            // 可视化解压缩的点云
            viewer.showCloud(cloudOut);
        }
    }
    /**************************************************************************************************************
 在函数中创建PointCloudCompression类的对象来编码和解码，这些对象把压缩配置文件作为配置压缩算法的参数
 所提供的压缩配置文件为OpenNI兼容设备采集到的点云预先确定的通用参数集，本例中使用MED_RES_ONLINE_COMPRESSION_WITH_COLOR
 配置参数集，用于快速在线的压缩，压缩配置方法可以在文件/io/include/pcl/compression/compression_profiles.h中找到，
  在PointCloudCompression构造函数中使用MANUAL——CONFIGURATION属性就可以手动的配置压缩算法的全部参数
******************************************************************************************************************/
    void run()
    {

        bool showStatistics = true; //设置在标准设备上输出打印出压缩结果信息

        // 压缩选项详情在: /io/include/pcl/compression/compression_profiles.h
        pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

        // 初始化压缩和解压缩对象  其中压缩对象需要设定压缩参数选项，解压缩按照数据源自行判断
        PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, showStatistics);
        PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();
        /***********************************************************************************************************
    下面的代码为OpenNI兼容设备实例化一个新的采样器，并且启动循环回调接口，每从设备获取一帧数据就回调函数一次，，这里的
    回调函数就是实现数据压缩和可视化解压缩结果。
   ************************************************************************************************************/
        // //创建从OpenNI获取点云的抓取对象
        // pcl::OpenNIGrabber* interface = new pcl::OpenNIGrabber();
        // if (!interface->isRunning()) { // [1]
        //     std::cerr << "设备连接失败！" << std::endl;
        //     return; // 或进行错误处理
        // }

        // // 在类定义中使用 std::function
        // std::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = 
        //     std::bind(&SimpleOpenNIViewer::cloud_cb_, this, std::placeholders::_1);

        // // 正确注册回调
        // boost::signals2::connection c = interface->registerCallback(f);

        // // 开始接受点云的数据流
        // interface->start();

        pcl::PCDReader reader;
        std::string pcd_file = "./src/chapter03/pcd/pyramid_color.pcd"; 
        // 创建点云对象
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        
        // 先读取一次文件获取点云结构
        if (reader.read(pcd_file, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read file. Stopping.\n");
            delete PointCloudEncoder;
            delete PointCloudDecoder;
            return;
        }

        while (!viewer.wasStopped())
        {
            // 每次循环都重新读取文件
            if (reader.read(pcd_file, *cloud) == -1)
            {
                PCL_ERROR("Couldn't read file. Stopping.\n");
                break;
            }
            
            cloud_cb_(cloud); // 处理并显示点云
            sleep(1); // 每秒一帧
        }

        // interface->stop();

        // 删除压缩与解压缩的实例
        delete (PointCloudEncoder);
        delete (PointCloudDecoder);
    }

    pcl::visualization::CloudViewer viewer;

    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *PointCloudEncoder;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *PointCloudDecoder;
};

int main(int argc, char **argv)
{
    SimpleOpenNIViewer v; //创建一个新的SimpleOpenNIViewer  实例并调用他的run方法
    v.run();

    return (0);
}