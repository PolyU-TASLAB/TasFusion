# Batch Board GNSS/IMU Fusion Workspace

## 概览
Batch Board 是一个面向多传感器导航的 ROS catkin 工作区，核心提供基于 Ceres 的 GNSS/IMU 紧耦合滑动窗口优化，并配套 GNSS 消息定义、NLOS 排除工具、NovAtel 驱动、NMEA 解析与结果分析脚本。核心融合节点支持 IMU 预积分、偏置在线估计、边缘化保持历史信息以及 GPS 位置/速度约束，可通过 launch 参数灵活开启。

## 功能亮点
- **滑动窗口优化与边缘化**：`toyslam` 的 CMake 配置显示依赖 Ceres、Eigen、Boost 和 ROS 核心库，驱动 `gnss_imu_sw_node` 进行非线性优化。【F:src/toyslam/CMakeLists.txt†L16-L99】
- **可配置的传感器输入**：启动文件提供 IMU、GPS、真值话题与消息类型选择，支持 GPS 速度/姿态初始化，并可开关偏置估计、边缘化与速度/姿态约束权重。【F:src/toyslam/launch/batch_board.launch†L4-L118】
- **日志与可视化**：默认启动 RViz 轨迹视图，并可将 GPS/真值/优化结果、指标与偏置写入 CSV 路径，便于后处理。【F:src/toyslam/launch/batch_board.launch†L20-L131】
- **辅助脚本与工具链**：提供 rosbag 话题频率分析、偏置可视化等 Python 工具，便于数据质量评估。【F:src/helper_scripts/analysis_freq.py†L1-L120】

## 仓库结构
- `toyslam/`：核心 GNSS/IMU 融合节点、rviz 配置与启动文件，负责滑动窗口优化与数据记录。【F:src/toyslam/launch/batch_board.launch†L1-L132】【F:src/toyslam/CMakeLists.txt†L16-L109】
- `gnss_comm/`：GNSS 原始测量的定义与工具库，包含依赖说明与 Docker 支持。【F:src/gnss_comm/README.md†L1-L60】
- `nlosexclusion/`：GNSS NLOS 排除相关消息定义与 PCL/TF 工具依赖配置。【F:src/nlosexclusion/CMakeLists.txt†L4-L46】
- `novatel_span_driver/`：连接 NovAtel SPAN 接收机的以太网驱动包。【F:src/novatel_span_driver/README.md†L1-L8】
- `nmea_parser/`：基于 `gnss_comm` 的 NMEA 解析包骨架，声明 ROS/标准消息依赖。【F:src/nmea_parser/package.xml†L3-L63】
- `helper_scripts/`：用于频率分析与偏置绘制的脚本集合。【F:src/helper_scripts/analysis_freq.py†L1-L120】
- `data/rosbag/demo_rosbag.zip`：内置示例数据包，可用于快速回放测试。
- `support_files/`：包含 toySLAM 教程 PDF 与依赖包压缩文件，便于了解算法原理与环境准备。

## 环境与依赖
- ROS（建议 Noetic 或兼容版本）以及 catkin 构建工具。
- 核心库：Eigen、Ceres Solver、Boost；可选 GFlags/Glog；从 CMake 可见必需依赖。【F:src/toyslam/CMakeLists.txt†L16-L99】
- GNSS 工具库 `gnss_comm` 依赖 Eigen 与 Glog，并提供 docker 构建方案。【F:src/gnss_comm/README.md†L15-L54】
- NLOS 处理需要 PCL 1.7 及 TF2 相关库。【F:src/nlosexclusion/CMakeLists.txt†L4-L46】

## 构建步骤
1. 创建工作区并克隆仓库（假设路径 `~/catkin_ws/src`）：
   ```bash
   cd ~/catkin_ws/src
   git clone <repo_url> batch_board_sw
   ```
2. 安装系统依赖（以 Ubuntu/ROS 为例）：
   ```bash
   sudo apt-get install libeigen3-dev libceres-dev libboost-all-dev ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-rviz
   sudo apt-get install libgoogle-glog-dev  # gnss_comm 需要
   ```
3. 在工作区根目录编译并加载环境：
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```
   构建流程与 `gnss_comm` 文档保持一致，使用 catkin_make 生成可执行文件。【F:src/gnss_comm/README.md†L34-L45】

## 快速开始
1. 解压示例数据：`unzip data/rosbag/demo_rosbag.zip -d data/rosbag`。
2. 启动融合节点与可视化（默认启用 RViz）：
   ```bash
   roslaunch toyslam batch_board.launch
   ```
   启动文件允许切换 GPS/IMU 话题、开启偏置估计与滑窗大小等参数。【F:src/toyslam/launch/batch_board.launch†L4-L118】
3. 回放 rosbag（示例文件名以解压后实际为准）：
   ```bash
   rosbag play data/rosbag/<your_demo>.bag
   ```
4. 如需在服务器或无界面环境运行，可将 `rviz:=false` 关闭可视化。【F:src/toyslam/launch/batch_board.launch†L2-L74】

## 关键参数
- **传感器话题**：`imu_topic`、`gps_topic`、`gps_message_type` 用于指定输入数据源。【F:src/toyslam/launch/batch_board.launch†L6-L28】
- **优化配置**：`optimization_window_size`、`optimization_frequency`、`max_iterations` 控制滑窗大小与 Ceres 迭代步数。【F:src/toyslam/launch/batch_board.launch†L34-L40】
- **噪声与偏置**：`imu_acc_noise`、`imu_gyro_noise`、偏置随机游走与初值参数可根据设备调整。【F:src/toyslam/launch/batch_board.launch†L42-L57】
- **约束开关**：`use_gps_velocity`、`enable_velocity_constraint`、`enable_roll_pitch_constraint`、`enable_orientation_smoothness_factor` 用于选择速度/姿态约束及其权重。【F:src/toyslam/launch/batch_board.launch†L59-L71】
- **日志输出**：`gps_log_path`、`gt_log_path`、`optimized_log_path`、`results_log_path`、`metrics_log_path`、`bias_log_path` 指定 CSV 写入位置，请确保目录存在。【F:src/toyslam/launch/batch_board.launch†L20-L25】【F:src/toyslam/launch/batch_board.launch†L126-L131】

## 数据记录与后处理
- 轨迹、真值、优化结果与评估指标会以 CSV 形式输出到 `data/results/` 下的指定文件名，便于进一步对齐或绘图分析。【F:src/toyslam/launch/batch_board.launch†L20-L131】
- `helper_scripts/analysis_freq.py` 可对 rosbag 话题频率、抖动进行统计和可视化，帮助评估传感器时序质量。【F:src/helper_scripts/analysis_freq.py†L1-L120】

## 参考资料
- `support_files/toySLAM_Tutorial.pdf`：包含算法推导与实验说明。
- `gnss_comm` README 提供 GNSS 原始测量处理的背景与依赖说明。【F:src/gnss_comm/README.md†L1-L60】
- NovAtel 官方 Wiki 链接在 `novatel_span_driver` README 中，可获取设备配置详情。【F:src/novatel_span_driver/README.md†L1-L8】