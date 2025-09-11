#include <pcl/io/pcd_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
 
using namespace std;
typedef pcl::PointXYZ point;
typedef pcl::PointXYZRGBA pointcolor;
 
int main(int argc,char **argv)
{
        pcl::PointCloud<pointcolor>::Ptr input (new pcl::PointCloud<pointcolor>);
        pcl::io::loadPCDFile(argv[1],*input);           // ros2 run chapter01 point_convert src/chapter01/pcd/pyramid_color.pcd
        
 
        pcl::PointCloud<point>::Ptr output (new pcl::PointCloud<point>);
        int M = input->points.size();
        cout<<"input size is:"<<M<<endl;
 
        for (int i = 0;i <M;i++)
        {
                point p;
                p.x = input->points[i].x;
                p.y = input->points[i].y;
                p.z = input->points[i].z; 
                output->points.push_back(p);
        }
        output->width = 1;
        output->height = M;
        
        cout<< "size is"<<output->size()<<endl;
        pcl::io::savePCDFile("./src/chapter01/pcd/convert.pcd",*output);
 
        // 查看转换后的pcd文件
        // pcl_viewer src/chapter01/pcd/convert.pcd
}
