# pcd2pgm

基于 ROS2 和 PCL 库，用于将 `.pcd` 点云文件转换为用于 Navigation 的 `pgm` 栅格地图

|pcd|pgm|
|:-:|:-:|
|![pcd](.docs/pcd.png)|![pgm](.docs/pgm.png)|

## 1. Overview

- 读取指定的 `.pcd` 文件

- 使用 Pass Through 滤波器过滤点云

- 使用 Radius Outlier 滤波器进一步处理点云

- 将处理后的点云转换为占据栅格地图（Occupancy Grid Map）

- 将转换后的地图发布到指定 ROS 话题上

## 2. Quick Start

### 2.1 Setup Environment

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### 2.2 Create Workspace

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
```

```bash
git clone https://github.com/LihanChen2004/pcd2pgm.git
```

### 2.3 Build

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
```

### 2.4 Running

启动 pcd2pgm 节点，可在 RViz 中预览滤波后点云和栅格地图：

```sh
ros2 launch pcd2pgm pcd2pgm_launch.py
```

保存栅格地图：

```sh
ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>
```

## 三. Node Parameters

可以通过修改 `pcd2pgm/pcd2pgm.yaml` 文件来配置节点的参数。

  ```yaml
  pcd2pgm:
    ros__parameters:
      pcd_file: /home/lihanchen/NAVIGATION_WS/pcd2pgm/rmuc_2025.pcd   # pcd 文件所在目录
      odom_to_lidar_odom: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]              # [x, y, z, r, p, y] 里程计到激光雷达的坐标变换（用于变换点云）
      flag_pass_through: false                                        # 是否使用 Pass Through 滤波器
      map_resolution: 0.05                                            # 地图分辨率
      map_topic_name: map                                             # 发布地图的 ROS 话题名
      thre_radius: 0.1                                                # Radius Outlier 滤波器半径
      thre_z_max: 2.0                                                 # Z轴最大值（用于 Pass Through 滤波器）
      thre_z_min: 0.1                                                 # Z轴最小值（用于 Pass Through 滤波器）
      thres_point_count: 10                                           # 最小点数阈值（用于 Radius Outlier 滤波器）
  ```
