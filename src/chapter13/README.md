```shell
rm@rm-NUC10i7FNH:~/PCL/pcl_ws$ ros2 run chapter13 correspondence_grouping src/chapter13/src/1基于对应分组的三维物体识别/milk.pcd src/chapter13/src/1基于对应分组的三维物体识别/milk_cartoon_all_small_clorox.pcd -c -k
Failed to find match for field 'rgba'.
Model valid points after normal filtering: 13704
Scene valid points after normal filtering: 241405
Model total points: 13704; Selected Keypoints before filtering: 739
Scene total points: 241405; Selected Keypoints before filtering: 3747
Model valid keypoints: 739; Valid descriptors: 739
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 2262
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 1983
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 838
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 2737
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 1158
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 907
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 1548
Scene valid keypoints: 3747; Valid descriptors: 3747
correspondence_grouping: /usr/include/pcl-1.12/pcl/kdtree/impl/kdtree_flann.hpp:237: int pcl::KdTreeFLANN<PointT, Dist>::nearestKSearch(const PointT&, unsigned int, pcl::Indices&, std::vector<float>&) const [with PointT = pcl::SHOT352; Dist = flann::L2_Simple<float>; pcl::Indices = std::vector<int, std::allocator<int> >]: Assertion `point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!"' failed.
[ros2run]: Aborted
```

## 2隐式形状模型 ISM (Implicit Shape Model)
运行命令(需要复制pcd到工作空间目录)
```shell
ros2 run chapter13 implicit_shape_model      ./ism_train_cat.pcd      0      ./ism_train_horse.pcd    1      ./ism_train_lioness.pc
d  2      ./ism_train_michael.pcd  3      ./ism_train_wolf.pcd     4  ./ism_test_cat.pcd       0
```

## 33D物体识别的假设检验
运行命令
```shell
ros2 run chapter13 global_hypothesis_verification src/chapter13/src/33D物体识别的假设检验/milk.pcd src/chapter13/src/33D物体识别的 假设检验/milk_cartoon_all_small_clorox.pcd
```
报错：
```shell
rm@rm-NUC10i7FNH:~/PCL/pcl_ws$ ros2 run chapter13 global_hypothesis_verification src/chapter13/src/33D物体识别的假设检验/milk.pcd src/chapter13/src/33D物体识别的 假设检验/milk_cartoon_all_small_clorox.pcd
Failed to find match for field 'rgba'.
Model total points: 13704; Selected Keypoints: 206
Scene total points: 307200; Selected Keypoints: 7756
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 3890
[pcl::SHOTEstimation::createBinDistanceShape] Point 1994 has 1 (5.555555%) NaN normals in its neighbourhood
[pcl::SHOTEstimation::createBinDistanceShape] Point 1996 has 1 (6.250000%) NaN normals in its neighbourhood
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 2012
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 3384
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 2751
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 2796
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 3519
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 2199
[pcl::SHOTEstimation::createBinDistanceShape] Point 2953 has 1 (4.761905%) NaN normals in its neighbourhood
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 3648
[pcl::SHOTEstimation::createBinDistanceShape] Point 4947 has 1 (5.263158%) NaN normals in its neighbourhood
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 3079
[pcl::SHOTEstimation::createBinDistanceShape] Point 5027 has 1 (2.857143%) NaN normals in its neighbourhood
[pcl::SHOTEstimation::createBinDistanceShape] Point 5029 has 1 (4.000000%) NaN normals in its neighbourhood
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 2429
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 7646
[pcl::SHOTEstimation::computeFeature] The local reference frame is not valid! Aborting description of point with index 3188
global_hypothesis_verification: /usr/include/pcl-1.12/pcl/kdtree/impl/kdtree_flann.hpp:237: int pcl::KdTreeFLANN<PointT, Dist>::nearestKSearch(const PointT&, unsigned int, pcl::Indices&, std::vector<float>&) const [with PointT = pcl::SHOT352; Dist = flann::L2_Simple<float>; pcl::Indices = std::vector<int, std::allocator<int> >]: Assertion `point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!"' failed.
[ros2run]: Aborted
```