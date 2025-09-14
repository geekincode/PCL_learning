#include "pcl/common/common.h"
#include "pcl/common/angles.h"
#include "pcl/common/centroid.h"
#include "pcl/common/geometry.h"
#include "pcl/common/random.h"
#include "pcl/common/file_io.h"
#include <pcl/common/distances.h>
#include <pcl/common/eigen.h>
#include <pcl/common/intersections.h>
#include <pcl/common/io.h>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

#include "Eigen/Core"
#include "Eigen/Geometry"

using namespace std;

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    
    // 添加示例点数据
    cloud.push_back(pcl::PointXYZ(1.0f, 2.0f, 3.0f));
    cloud.push_back(pcl::PointXYZ(4.0f, 5.0f, 6.0f));

    float rad = 3.14;
    float deg = 180.0;
    cout << "角度弧度转换:" << endl;
    cout << pcl::rad2deg(rad) << endl;
    cout << pcl::deg2rad(deg) << endl;
    cout << pcl::normAngle(rad+0.3) << endl;

    cout << endl << "计算给定一群点的3D中心点,并且返回一个三维向量:" << endl;
    
    Eigen::Vector4f centroid;
    // pcl::compute3DCentroid (const pcl::PointCloud< PointT > &cloud, Eigen::Matrix< Scalar, 4, 1 > &centroid)
    pcl::compute3DCentroid(cloud, centroid);
    cout << "Centroid: " << centroid.head<3>().transpose() << endl;

    // 计算协方差矩阵
    Eigen::Matrix3f covariance_matrix;
    // pcl::computeCovarianceMatrix (const pcl::PointCloud< PointT > &cloud, const Eigen::Matrix< Scalar, 4, 1 > &centroid, Eigen::Matrix< Scalar, 3, 3 > &covariance_matrix)
    pcl::computeCovarianceMatrix(cloud, centroid, covariance_matrix);
    // 归一化协方差矩阵
    Eigen::Matrix3f normalized_covariance = covariance_matrix / cloud.size();
    cout << "Covariance Matrix:\n" << covariance_matrix << endl;
    cout << "Normalized Covariance Matrix(归一化):\n" << normalized_covariance << endl;

    // pcl::demeanPointCloud - 去均值化点云
    pcl::PointCloud<pcl::PointXYZ> cloud_demean;
    pcl::demeanPointCloud(cloud, centroid, cloud_demean);
    cout << "\n去均值化点云示例：" << endl;
    for (auto p : cloud_demean) {
        cout << "(" << p.x << ", " << p.y << ", " << p.z << ")" << endl;
    }

    // pcl::computeNDCentroid - N维中心估计
    Eigen::VectorXf nd_centroid;
    pcl::computeNDCentroid(cloud, nd_centroid);
    cout << "\nN维中心估计：" << endl;
    for (int i = 0; i < nd_centroid.size(); ++i) {
        cout << "维度 " << i << ": " << nd_centroid(i) << endl;
    }
    
    // pcl::getAngle3D - 计算两个向量之间的角度
    Eigen::Vector4f v1(1, 0, 0, 0);
    Eigen::Vector4f v2(0, 1, 0, 0);
    float angle_rad = pcl::getAngle3D(v1, v2);  // 弧度制
    float angle_deg = pcl::getAngle3D(v1, v2, true);  // 角度制
    cout << "\n向量夹角：" << endl;
    cout << "弧度: " << angle_rad << ", 角度: " << angle_deg << endl;

    // pcl::getMeanStd - 计算均值和标准差
    vector<float> values = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    double mean, stddev;
    pcl::getMeanStd(values, mean, stddev);
    cout << "\n均值和标准差：" << endl;
    cout << "均值: " << mean << ", 标准差: " << stddev << endl;

    // pcl::getPointsInBox - 获取边界框内的点
    Eigen::Vector4f min_pt(1.5f, 2.5f, 3.5f, 0);
    Eigen::Vector4f max_pt(4.5f, 5.5f, 6.5f, 0);
    vector<int> indices;
    pcl::getPointsInBox(cloud, min_pt, max_pt, indices);
    cout << "\n边界框内的点索引：" << endl;
    for (int idx : indices) {
        cout << idx << " ";
    }

    // pcl::getMaxDistance - 获取最大距离
    Eigen::Vector4f pivot(0, 0, 0, 0);
    Eigen::Vector4f max_point;
    pcl::getMaxDistance(cloud, pivot, max_point);
    std::cout << "最远点坐标: (" 
            << max_point.x() << ", " 
            << max_point.y() << ", " 
            << max_point.z() << ")" << std::endl;

    // pcl::getMinMax3D - 获取XYZ轴上的最小/最大值
    pcl::PointXYZ min_point, max_point_3d;
    pcl::getMinMax3D(cloud, min_point, max_point_3d);
    cout << "\nXYZ最小/最大值：" << endl;
    cout << "最小点: (" << min_point.x << ", " << min_point.y << ", " << min_point.z << ")" << endl;
    cout << "最大点: (" << max_point_3d.x << ", " << max_point_3d.y << ", " << max_point_3d.z << ")" << endl;


    // pcl::getCircumcircleRadius - 计算外接圆半径
    pcl::PointXYZ pa(0, 0, 0), pb(1, 0, 0), pc(0, 1, 0);
    float radius = pcl::getCircumcircleRadius(pa, pb, pc);
    cout << "\n外接圆半径: " << radius << endl;

    // pcl::getMinMax - 获取直方图的最小/最大值
    float histogram[] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    float min_hist, max_hist;
    pcl::getMinMax(histogram, 5, min_hist, max_hist);
    cout << "\n直方图范围: " << min_hist << " - " << max_hist << endl;

    // pcl::calculatePolygonArea - 计算多边形面积
    pcl::PointCloud<pcl::PointXYZ> polygon;
    polygon.push_back(pcl::PointXYZ(0, 0, 0));
    polygon.push_back(pcl::PointXYZ(1, 0, 0));
    polygon.push_back(pcl::PointXYZ(0, 1, 0));
    float area = pcl::calculatePolygonArea(polygon);
    cout << "\n多边形面积: " << area << endl;

    // 点字段复制（手动实现）
    pcl::PointXYZRGB src_point;
    src_point.x = 1.0f; src_point.y = 2.0f; src_point.z = 3.0f;
    src_point.r = 255; src_point.g = 0; src_point.b = 0;
    
    pcl::PointXYZ dst_point;
    dst_point.x = src_point.x;
    dst_point.y = src_point.y;
    dst_point.z = src_point.z;
    cout << "\n点字段复制: (" 
         << dst_point.x << ", " << dst_point.y << ", " << dst_point.z << ")" << endl;

    // 3. pcl::lineToLineSegment - 三维直线最短线段
    Eigen::VectorXf line_a(6), line_b(6);
    // 直线A: 点(0,0,0) + 方向(1,0,0)
    line_a << 0, 0, 0, 1, 0, 0;
    // 直线B: 点(0,1,0) + 方向(0,0,1)
    line_b << 0, 1, 0, 0, 0, 1;
    
    Eigen::Vector4f pt1_seg, pt2_seg;
    pcl::lineToLineSegment(line_a, line_b, pt1_seg, pt2_seg);
    cout << "\n直线最短线段: (" 
         << pt1_seg.head<3>().transpose() << ") - (" 
         << pt2_seg.head<3>().transpose() << ")" << endl;

    // 4. pcl::sqrPointToLineDistance - 点到直线平方距离
    Eigen::Vector4f pt(1, 1, 1, 0);
    Eigen::Vector4f line_pt(0, 0, 0, 0);
    Eigen::Vector4f line_dir(1, 1, 1, 0);
    double dist = pcl::sqrPointToLineDistance(pt, line_pt, line_dir);
    cout << "\n点到直线平方距离: " << dist << endl;

    // 5. pcl::getMaxSegment - 获取最大分段
    pcl::PointXYZ pmin, pmax;
    pcl::getMaxSegment(cloud, pmin, pmax);
    cout << "\n最大分段: (" 
         << pmin.x << ", " << pmin.y << ", " << pmin.z << ") - ("
         << pmax.x << ", " << pmax.y << ", " << pmax.z << ")" << endl;

    // 6. pcl::eigen22 - 2x2特征值分解
    Eigen::Matrix2f mat22;
    mat22 << 1, 2,
              2, 1;
    float eigenvalue22;
    Eigen::Vector2f eigenvector22;
    pcl::eigen22(mat22, eigenvalue22, eigenvector22);
    cout << "\n2x2特征值: " << eigenvalue22 << "\n特征向量: " << eigenvector22.transpose() << endl;

    // 7. pcl::eigen33 - 3x3特征值分解
    Eigen::Matrix3f mat33;
    mat33 << 1, 2, 3,
              2, 1, 4,
              3, 4, 1;
    float eigenvalue33;
    Eigen::Vector3f eigenvector33;
    pcl::eigen33(mat33, eigenvalue33, eigenvector33);
    cout << "\n3x3特征值: " << eigenvalue33 << "\n特征向量: " << eigenvector33.transpose() << endl;

    // 8. pcl::invert2x2 - 2x2矩阵求逆
    Eigen::Matrix2f mat22_inv;
    mat22 << 1, 2, 3, 4;
    Eigen::Matrix2f inv;
    pcl::invert2x2(mat22, inv);
    cout << "\n2x2矩阵逆:\n" << inv << endl;

    // 9. pcl::determinant3x3Matrix - 3x3行列式
    Eigen::Matrix3f mat33_det;
    mat33_det << 1, 2, 3,
                 4, 5, 6,
                 7, 8, 9;
    float det = pcl::determinant3x3Matrix(mat33_det);
    cout << "\n3x3行列式: " << det << endl;

    // 10. pcl::getTransformation - 创建变换矩阵
    Eigen::Affine3f trans = pcl::getTransformation(1.0f, 2.0f, 3.0f, 
                                                  M_PI/2, 0, 0);
    cout << "\n变换矩阵:\n" << trans.matrix() << endl;

    // 11. pcl::lineWithLineIntersection - 直线交点
    // Eigen::VectorXf line_a(6), line_b(6);
    line_a << 0, 0, 0, 1, 0, 0;  // X轴
    line_b << 0, 1, 0, 0, 0, 1;  // Y轴方向移动的直线
    Eigen::Vector4f intersection;
    bool has_intersect = pcl::lineWithLineIntersection(line_a, line_b, intersection);
    if (has_intersect) {
        cout << "存在(" << intersection.head<3>().transpose() << ")";
    } else {
        cout << "不存在)";
    }

    // 12. pcl::getFieldIndex - 获取字段索引
    pcl::PCLPointCloud2 cloud2;
    cloud2.height = 1;
    cloud2.width = 1;
    cloud2.fields.resize(3);
    cloud2.fields[0].name = "x";
    cloud2.fields[1].name = "y";
    cloud2.fields[2].name = "z";
    int idx = pcl::getFieldIndex(cloud2, "y");
    cout << "\n字段索引(y): " << idx << endl;

    // 13. pcl::getFieldsList - 获取字段列表
    pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
    string fields = pcl::getFieldsList(cloud_rgb);
    cout << "\n字段列表: \n";
    for (const auto& f : fields) cout << f << " ";
    cout << endl;


    return 0;
}

/**
输出：
角度弧度转换:
179.909
3.14159
-2.84319

计算给定一群点的3D中心点,并且返回一个三维向量:
Centroid: 2.5 3.5 4.5
Covariance Matrix:
4.5 4.5 4.5
4.5 4.5 4.5
4.5 4.5 4.5
Normalized Covariance Matrix(归一化):
2.25 2.25 2.25
2.25 2.25 2.25
2.25 2.25 2.25

去均值化点云示例：
(-1.5, -1.5, -1.5)
(1.5, 1.5, 1.5)

N维中心估计：
维度 0: 2.5
维度 1: 3.5
维度 2: 4.5

向量夹角：
弧度: 1.5708, 角度: 90

均值和标准差：
均值: 3, 标准差: 1.58114

边界框内的点索引：
1 最远点坐标: (4, 5, 6)

XYZ最小/最大值：
最小点: (1, 2, 3)
最大点: (4, 5, 6)

外接圆半径: 0.707107

直方图范围: 1 - 5

多边形面积: 0.5

点字段复制: (1, 2, 3)

直线最短线段: (0 0 0) - (0 1 0)

点到直线平方距离: 0

最大分段: (1, 2, 3) - (4, 5, 6)

2x2特征值: -1
特征向量: -0.707107  0.707107

3x3特征值: 0
特征向量: -0.801784  0.534522  0.267261

2x2矩阵逆:
  -2    1
 1.5 -0.5

3x3行列式: 0

变换矩阵:
           1            0            0            1
           0 -4.37114e-08           -1            2
          -0            1 -4.37114e-08            3
           0            0            0            1
不存在)
字段索引(y): 1

字段列表: 
x   y   z   r g b 
*/