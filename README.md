# ToySLAM GNSS/IMU Fusion Workspace

## Overview

ToySLAM is a ROS1 package designed for multi-sensor navigation. Its core functionality provides a Ceres-based GNSS/IMU loosely coupled sliding-window optimization framework, along with supporting tools including GNSS message definitions, NLOS exclusion utilities, a NovAtel driver, and NMEA ROS parsing scripts.

The central sensor-fusion node supports IMU pre-integration, online bias estimation, marginalization to preserve historical information, and GPS position/velocity constraints. All major functions can be flexibly enabled or disabled through parameters configured in launch files.

## Key Features

* **Sliding-Window Nonlinear Optimization with Marginalization**

  `toyslam` implements a GNSS/IMU loosely coupled sliding-window optimization framework based on  **Ceres Solver** ,  **Eigen** , and core **ROS** libraries. The central node `gnss_imu_sw_node` performs nonlinear least-squares optimization within a fixed-size window, while marginalization is applied to retain historical information and maintain computational efficiency.
* **Highly Configurable Sensor Inputs and Fusion Strategy**

  Launch files provide flexible configuration of IMU, GNSS, and ground-truth topics and message types. The system supports GPS-based position, velocity, and attitude initialization, and allows users to enable or disable online bias estimation, marginalization, and velocity/attitude constraint weighting via parameters, facilitating ablation studies and algorithm evaluation.
* **Logging and Visualization Support**

  RViz-based trajectory visualization is enabled by default. The system can also export GNSS measurements, ground truth, optimized states, performance metrics, and IMU biases to CSV files for offline post-processing, benchmarking, and result reproduction.
* **Auxiliary Scripts and Toolchain**

  A set of Python utilities is provided, including rosbag topic frequency analysis and bias visualization tools, enabling rapid assessment of sensor data quality and system behavior.
* **Support for Multiple GNSS Message Types**

  The fusion node supports multiple GNSS data formats, selectable via the `gnss_message_type` parameter:

  * `novatel_msgs/INSPVAX`
  * `gnss_comm/GnssPVTSolnMsg`
  * `nav_msgs/Odometry`

  This design allows seamless integration with different GNSS receivers and data pipelines.

## Repository Structure

* **`toyslam/`**

  Core GNSS/IMU sensor fusion package, including the sliding-window optimization node, RViz configurations, and launch files. This module is responsible for nonlinear optimization, state estimation, and data logging.
* **`nlosexclusion/`**

  GNSS NLOS (Non-Line-of-Sight) exclusion module, providing message definitions and implementation for NLOS detection and mitigation.
* **[`novatel_span_driver/`](https://github.com/ros-drivers/novatel_span_driver.git)**

  ROS driver for interfacing with NovAtel SPAN GNSS/INS receivers.
* **[`gnss_comm/`](https://github.com/HKUST-Aerial-Robotics/gnss_comm.git)**

  A GNSS utility library providing raw GNSS measurement definitions and processing tools, along with dependency documentation and Docker support.
* **`nmea_parser/`**

  NMEA parsing package built on top of `gnss_comm`, enabling standardized GNSS message decoding and conversion.
* **`helper_scripts/`**

  A collection of scripts for data analysis, visualization, and performance evaluation.
* **`support_files/`**

  Supplementary materials including a ToySLAM tutorial PDF and archived dependency packages for Ceres Solver and Eigen.
* **`data/rosbag/demo_rosbag.zip`**

  Built-in demo rosbag for quick testing and playback.
* **`data/results/`**

  Directory for storing output results, logs, and exported CSV files.

## Enviroment & Dependencies

- This package is developed under Ubuntu20.04 LTSC [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) environment.
- Use [Eigen 3.3.4](https://gitlab.com/libeigen/eigen/-/archive/3.3.4/eigen-3.3.4.zip) for matrix manipulation.
  ```
  cd eigen-3.3.4/
  mkdir build
  cd build
  cmake ..
  sudo make install
  ```
- Use [Ceres 2.1.0 ](https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.1.0.zip)for optimize.
  ```
  cd Ceres-2.1.0
  mkdir build
  cd build
  cmake ..
  make -j4
  sudo make install
  ```
- Use google's glog library for message output
  ```bash
  sudo apt-get install libgoogle-glog-dev
  ```

## 构建步骤

1. 创建工作区并克隆仓库（假设路径 `~/catkin_ws/src`）：

   ```bash
   cd ~/catkin_ws/src
   git clone <repo_url> batch_board_sw
   ```
2. 安装系统依赖（以 Ubuntu20.04/ROS Noetic 为例）：

   ```bash
   sudo apt-get install libeigen3-dev libceres-dev ros-$ROS_DISTRO-rviz
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
