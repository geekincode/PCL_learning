### 点云压缩可视化有问题

### [text](src/pcd2octree.cpp)可视化也有问题
```
rm@rm-NUC10i7FNH:~/PCL/pcl_ws$ ros2 run chapter03 pcd2octree 
Loaded 3400 points from /home/rm/PCL/pcl_ws/src/chapter03/pcd/ism_test_wolf.pcd
Octree created with resolution: 0.05
Original point cloud added to viewer
Found 3400 valid voxels
Voxels added to viewer. Starting visualization loop...
[ros2run]: Segmentation fault
```