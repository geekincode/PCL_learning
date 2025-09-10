# 初识点云

### pcd点云文件的格式
```
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z rgba
SIZE 4 4 4 4
TYPE F F F U
COUNT 1 1 1 1
WIDTH 10208
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 10208
DATA ascii
-2.5 -2.5 0 2021160960
2.5 -2.5 0 2021160960
2.5 2.5 0 2021160960
-2.5 2.5 0 2021160960
-2.4494948 -2.5 0 2021160960
2.5 -2.4494948 0 2021160960
2.4494948 2.5 0 2021160960
-2.5 2.4494948 0 2021160960
-2.3989899 -2.5 0 2021160960
2.5 -2.3989899 0 2021160960
...
```

文件格式：pcl::PointXYZ pcl::PointXYZRGBA
以及文件格式的转换

### 编译指令
```
colcon build --event-handlers console_direct+
```
colcon build 默认使用 console_cohesion 事件处理器，它会将每个包的构建输出缓冲，并在包构建完成后一次性显示。

使用 --event-handlers console_direct+ 后，所有构建日志实时直接输出到控制台，无需等待包构建完成，`特别是包括cmake的message消息也会输出到控制台`。

* --event-handlers：指定事件处理器。
* console_direct+：事件处理器名称，+ 表示启用该处理器，- 表示禁用（例如 console_cohesion- 表示禁用默认的汇总模式）。

