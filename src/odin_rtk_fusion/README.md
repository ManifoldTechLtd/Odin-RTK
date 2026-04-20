# Odin RTK Fusion

SLAM 里程计与 GPS (GGA) 数据融合节点，基于 GTSAM 因子图优化。

## 功能特性

- 航向角对齐估计
- 初始时刻时间偏移标定
- GPS 野值剔除
- iSAM2 增量优化

## 依赖安装

### 第三方库

```bash
# 基础依赖
sudo apt install libproj-dev libyaml-cpp-dev libspdlog-dev libeigen3-dev

# GTSAM (推荐从源码安装以获取最新版本)
# 参考: https://gtsam.org/get_started/
sudo apt install libgtsam-dev  # 或从源码编译
```

### ROS1 (Noetic)

```bash
sudo apt install ros-noetic-nmea-msgs ros-noetic-pcl-ros ros-noetic-pcl-conversions \
    ros-noetic-tf2-ros ros-noetic-visualization-msgs
```

### ROS2 (Humble/Iron/Jazzy)

```bash
sudo apt install ros-${ROS_DISTRO}-nmea-msgs ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-visualization-msgs
```

## 编译

### ROS1 (catkin)

```bash
cd ~/catkin_ws
catkin_make -DROS_VERSION=1
source devel/setup.bash
```

### ROS2 (colcon)

```bash
cd ~/ros2_ws
colcon build --packages-select odin_rtk_fusion --cmake-args -DROS_VERSION=2
source install/setup.bash
```

## 测试数据

提供测试 rosbag 数据供验证：

| 文件 | 说明 | 下载链接 |
|------|------|----------|
| `odin_gnss_demo.bag` | 包含里程计和RTK数据的测试包 | [飞书](https://vvcazjv268.feishu.cn/file/OqSPbvXKyoJ0wOxh2hMc7GXSnqc) 

## 启动

### ROS1

使用 launch 文件启动，通过 `play_bag` 参数开启 rosbag 回放，并指定 bag 文件路径：

```bash
roslaunch odin_rtk_fusion fusion.launch play_bag:=true bag_path:=/path/to/odin_gnss_demo.bag
```

**Launch 参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `config_file` | `config/fusion_params.yaml` | 配置文件路径 |
| `play_bag` | `false` | 是否播放 bag 文件 |
| `bag_path` | - | bag 文件路径 |
| `bag_rate` | `3.0` | 播放速率 |
| `bag_start` | `5.0` | 起始时间 (s) |

**使用测试数据示例：**

```bash
# 下载测试数据后，指定路径启动
roslaunch odin_rtk_fusion fusion.launch play_bag:=true bag_path:=~/Downloads/odin_gnss_demo.bag bag_rate:=1.0
```

### ROS2

```bash
ros2 launch odin_rtk_fusion fusion.launch.py 
```

## Odin ROS Driver 配置

使用本融合节点前，需要修改 Odin ROS Driver 的配置：

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| `use_host_ros_time` | `1` | 使用主机接收到的时间作为 Odin 数据时间戳 |

> **注意**：此设置确保 Odin 输出的里程计数据使用主机 ROS 时间，便于与 GPS 数据进行时间同步。

## 输入话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/odin1/odometry` | `nav_msgs/Odometry` | 里程计输入 |
| `/odin1/cloud_slam` | `sensor_msgs/PointCloud2` | SLAM 点云，用于可视化 |
| `/odin1/nmea_sentence` | `nmea_msgs/Sentence` | NMEA 语句，只需包含 GPGGA 信息 |

## RTK 适配

如需适配自己的 RTK 设备，只需满足以下条件：

1. **提供 GGA 原始语句**：RTK 驱动需要输出包含 GGA 原始语句的 `nmea_msgs/Sentence` 消息
2. **话题名称**：发布到 `/odin1/nmea_sentence` 话题
3. **启动顺序**：先启动 RTK 驱动，再启动 fusion 节点

参考驱动示例：`scripts/rtk_string_publisher_ros2.py` (适用于 BG550 型号)

## 输出话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/world_enu/odom` | `nav_msgs/Odometry` | 融合后的 ENU 坐标系位姿 |
| `/world_enu/gnss_marker` | `visualization_msgs/Marker` | GNSS 可视化标记 |
| `/world/odom_projected` | `nav_msgs/Odometry` | 投影坐标系位姿 (可选，需启用 `output_project_enable`) |

## 参数配置

配置文件：`config/fusion_params.yaml`

### 噪声参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `odom_position_sigma` | `0.005` | 里程计位置噪声标准差 (m) |
| `odom_rotation_sigma` | `0.1` | 里程计旋转噪声标准差 (rad) |
| `gps_base_sigma` | `5.0` | GPS 基础噪声标准差 (m)，实际噪声 = `gps_base_sigma × HDOP` |

### 融合参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `initial_lever_arm` | `[-0.1, 0, 0.1]` | body 系下 GPS 天线位置 [x, y, z] (m) |
| `max_gps_match_dt` | `0.5` | GPS 与 odom 时间戳最大允许差值 (s) |
| `max_nodes` | `5000` | 因子图最大节点数，超过后触发边缘化 |

### 初始化参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `init_gps_buffer_size` | `30` | 初始化阶段 GPS 缓冲区大小 |
| `init_min_gps_displacement` | `1.0` | 触发初始化所需的最小 GPS 位移 (m) |
| `init_min_odom_displacement` | `1.0` | 触发初始化所需的最小 odom 位移 (m) |
| `init_yaw_prior_sigma` | `0.2` | 初始 yaw 先验噪声 (rad) |

### 坐标系参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `input_lla_crs` | `EPSG:4326` | 输入经纬高坐标系 (WGS84) |
| `output_project_enable` | `true` | 是否启用投影输出 |
| `output_project_target_crs` | `EPSG:32650` | 目标投影坐标系 |

**常用投影坐标系：**
- `EPSG:32650` - WGS84 / UTM zone 50N (东经 114°~120°)
- `EPSG:4526` - CGCS2000 / 3度带 38 (深圳)
- `EPSG:4527` - CGCS2000 / 3度带 39 (北京)

## License

Apache License 2.0
