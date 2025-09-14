common模块中的头文件

1. angles.h 定义了标准的C接口的角度计算函数
2. centriod.h 定义了中心点的估算以及协方差矩阵的计算
3. commo.h 标准的C以及C++类，是其他common 函数的父类
4. distance.h 定义标准的C接口用于计算距离
5. file_io.h 定义了一些文件帮助写或者读方面的功能。
6. random.h 定义一些随机点云生成的函数
7. geometry.h 定义一些基本的几何功能的函数
8. intersection.h 定义线与线相交的函数
9. norm.h 定义了标准的C方法计算矩阵的正则化
10. time.h 定义了时间计算的函数
11. Point_types.h 定义了所有PCL实现的点云的数据结构的类型
common模块中的基本函数

pcl::rad2deg(float alpha) 
从弧度到角度

pcl::deg2rad(float alpha)
从角度到弧度

pcl::normAngle(float alpha)
正则化角度在（-PI，PI）之间

pcl::compute3DCentroid (const pcl::PointCloud< PointT > &cloud, Eigen::Matrix< Scalar, 4, 1 > &centroid)
计算给定一群点的3D中心点，并且返回一个三维向量

pcl::computeCovarianceMatrix (const pcl::PointCloud< PointT > &cloud, const Eigen::Matrix< Scalar, 4, 1 > &centroid, Eigen::Matrix< Scalar, 3, 3 > &covariance_matrix)
计算给定的三维点云的协方差矩阵。

pcl::computeMeanAndCovarianceMatrix (const pcl::PointCloud< PointT > &cloud, Eigen::Matrix< Scalar, 3, 3 > &covariance_matrix, Eigen::Matrix< Scalar, 4, 1 > &centroid
计算正则化的3*3的协方差矩阵以及给定点云数据的中心点

pcl::demeanPointCloud (const pcl::PointCloud< PointT > &cloud_in, const Eigen::Matrix< Scalar, 4, 1 > &centroid, pcl::PointCloud< PointT > &cloud_out)
pcl::computeNDCentroid (const pcl::PointCloud< PointT > &cloud, Eigen::Matrix< Scalar, Eigen::Dynamic, 1 > &centroid)
利用一组点的指数对其进行一般的、通用的nD中心估计。

pcl::getAngle3D (const Eigen::Vector4f &v1, const Eigen::Vector4f &v2, const bool in_degree=false)
计算两个向量之间的角度

pcl::getMeanStd (const std::vector< float > &values, double &mean, double &stddev)
同时计算给定点云数据的均值和标准方差

pcl::getPointsInBox (const pcl::PointCloud< PointT > &cloud, Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt, std::vector< int > &indices)
在给定边界的情况下，获取一组位于框中的点

pcl::getMaxDistance (const pcl::PointCloud< PointT > &cloud, const Eigen::Vector4f &pivot_pt, Eigen::Vector4f &max_pt)
给定点云数据中点与点之间的最大距离的值

pcl::getMinMax3D (const pcl::PointCloud< PointT > &cloud, PointT &min_pt, PointT &max_pt)
获取给定点云中的在XYZ轴上的最大和最小值

pcl::getCircumcircleRadius (const PointT &pa, const PointT &pb, const PointT &pc)
计算由三个点pa、pb和pc构成的三角形的外接圆半径。

pcl::getMinMax (const PointT &histogram, int len, float &min_p, float &max_p)
获取点直方图上的最小值和最大值。

pcl::calculatePolygonArea (const pcl::PointCloud< PointT > &polygon)
根据给定的多边形的点云计算多边形的面积

pcl::copyPoint (const PointInT &point_in, PointOutT &point_out)
从Point_in把字段数据赋值到Point_out

pcl::lineToLineSegment (const Eigen::VectorXf &line_a, const Eigen::VectorXf &line_b, Eigen::Vector4f &pt1_seg, Eigen::Vector4f &pt2_seg)
获取两条三维直线之间的最短三维线段

pcl::sqrPointToLineDistance (const Eigen::Vector4f &pt, const Eigen::Vector4f &line_pt, const Eigen::Vector4f &line_dir)
获取点到线的平方距离（由点和方向表示）

pcl::getMaxSegment (const pcl::PointCloud< PointT > &cloud, PointT &pmin, PointT &pmax)
在给定的一组点中获得最大分段，并返回最小和最大点。

pcl::eigen22 (const Matrix &mat, typename Matrix::Scalar &eigenvalue, Vector &eigenvector)
确定最小特征值及其对应的特征向量

pcl::computeCorrespondingEigenVector (const Matrix &mat, const typename Matrix::Scalar &eigenvalue, Vector &eigenvector)
确定对称半正定输入矩阵给定特征值对应的特征向量

pcl::eigen33 (const Matrix &mat, typename Matrix::Scalar &eigenvalue, Vector &eigenvector)
确定对称半正定输入矩阵最小特征值的特征向量和特征值

pcl::invert2x2 (const Matrix &matrix, Matrix &inverse)
计算2x2矩阵的逆。

pcl::invert3x3SymMatrix (const Matrix &matrix, Matrix &inverse)
计算3x3对称矩阵的逆。

pcl::determinant3x3Matrix (const Matrix &matrix)
计算3x3矩阵的行列式

pcl::getTransFromUnitVectorsZY (const Eigen::Vector3f &z_axis, const Eigen::Vector3f &y_direction, Eigen::Affine3f &transformation)
获得唯一 的3D旋转，将Z轴旋转成（0，0，1）Y轴旋转成（0,1,0）并且两个轴是正交的。

pcl::getTransformationFromTwoUnitVectorsAndOrigin (const Eigen::Vector3f &y_direction, const Eigen::Vector3f &z_axis, const Eigen::Vector3f &origin, Eigen::Affine3f &transformation)
得到将origin转化为（0，0，0）的变换，并将Z轴旋转成（0，0，1）和Y方向（0，1，0）

pcl::getEulerAngles (const Eigen::Transform< Scalar, 3, Eigen::Affine > &t, Scalar &roll, Scalar &pitch, Scalar &yaw)
从给定的变换矩阵中提取欧拉角

pcl::getTranslationAndEulerAngles (const Eigen::Transform< Scalar, 3, Eigen::Affine > &t, Scalar &x, Scalar &y, Scalar &z, Scalar &roll, Scalar &pitch, Scalar &yaw)
给定的转换中，提取XYZ以及欧拉角

pcl::getTransformation (float x, float y, float z, float roll, float pitch, float yaw)
从给定的平移和欧拉角创建转换矩阵

pcl::saveBinary (const Eigen::MatrixBase< Derived > &matrix, std::ostream &file)
保存或者写矩阵到一个输出流中

pcl::loadBinary (Eigen::MatrixBase< Derived > const &matrix, std::istream &file)
从输入流中读取矩阵

pcl::lineWithLineIntersection (const Eigen::VectorXf &line_a, const Eigen::VectorXf &line_b, Eigen::Vector4f &point, double sqr_eps=1e-4)
获取空间中两条三维直线作为三维点的交点。

pcl::getFieldIndex (const pcl::PCLPointCloud2 &cloud, const std::string &field_name)
获取指定字段的索引（即维度/通道）

pcl::getFieldsList (const pcl::PointCloud< PointT > &cloud)
获取给定点云中所有可用字段的列表

pcl::getFieldSize (const int datatype)
获取特定字段数据类型的大小（字节）。

pcl::concatenatePointCloud (const pcl::PCLPointCloud2 &cloud1, const pcl::PCLPointCloud2 &cloud2, pcl::PCLPointCloud2 &cloud_out)
连接 pcl::PCLPointCloud2类型的点云字段