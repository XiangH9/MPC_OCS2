# NMPC 与仿真复现环境说明

本文档用于说明当前仓库中 OCS2-based NMPC 与仿真链路的复现环境。

## 1. 推荐复现基线

当前最推荐的复现基线如下：

- 操作系统：Ubuntu 22.04
- ROS 2 发行版：Humble
- Conda 环境名：`uros`
- 仿真器：Gazebo Harmonic（通过 ROS 2 Humble 适配）
- 工作区结构：
  - `uni_ws/src/quadruped_ros2_control`
  - `uni_ws/src/ocs2_ros2`
  - `uni_ws/src/unitree_sdk2`

说明：

- 部分包在 Ubuntu 24.04 / ROS 2 Jazzy 下也可能可以编译
- 但如果目标是严格复现当前 NMPC 实现，请优先使用 Humble 基线

## 2. 环境分层

这个项目的环境分成两层：

1. `Conda` 用户态环境
   - 用于开发工具和常用 Python 工具
   - 由 [environment.uros.yml](environment.uros.yml) 定义

2. 系统级 ROS 2 / Gazebo 环境
   - 通过 Ubuntu / ROS 官方 apt 源安装
   - 包括 ROS 2、ros2_control、Gazebo、KDL、pluginlib 等依赖

重要说明：

- 不要尝试只靠 conda 安装完整 ROS 2 生态
- ROS 2 与 Gazebo 依赖应优先使用 apt 安装
- conda 在这里主要作为开发 shell 和工具环境使用

## 3. Conda 环境

创建并激活环境：

```bash
conda env create -f environment.uros.yml
conda activate uros
```

## 4. 系统依赖

先安装基础构建工具和 ROS 常用工具：

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  git \
  cmake \
  pkg-config \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

安装 ROS 2 Humble 常用依赖：

```bash
sudo apt install -y \
  ros-humble-controller-manager \
  ros-humble-controller-interface \
  ros-humble-hardware-interface \
  ros-humble-joint-state-broadcaster \
  ros-humble-imu-sensor-broadcaster \
  ros-humble-pluginlib \
  ros-humble-kdl-parser \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-backward-ros \
  ros-humble-yaml-cpp-vendor \
  ros-humble-ament-index-cpp
```

安装 Gazebo Harmonic 与 ROS 2 Humble 适配：

```bash
sudo apt install -y ros-humble-ros-gzharmonic
```

可选项：

- Gazebo Classic：
  ```bash
  sudo apt install -y ros-humble-gazebo-ros ros-humble-gazebo-ros2-control
  ```

- MuJoCo / Unitree 仿真：
  - 需要按对应外部项目说明额外配置

## 5. 工作区中需要存在的源码仓库

当前开发时的工作区结构为：

```text
uni_ws/src/
  ocs2_ros2/
  quadruped_ros2_control/
  unitree_sdk2/
```

说明：

- `quadruped_ros2_control`：当前仓库
- `ocs2_ros2`：提供 OCS2 ROS 2 相关包，是 NMPC 编译必需依赖
- `unitree_sdk2`：保持与当前项目开发环境一致

如果别人只单独克隆 `quadruped_ros2_control`，但没有同工作区下的 `ocs2_ros2`，那么 NMPC 相关包无法正常编译。

## 6. rosdep

如果本机还没初始化 rosdep：

```bash
sudo rosdep init
rosdep update
```

然后在工作区根目录执行：

```bash
cd ~/uni_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 7. 编译顺序

在工作区根目录：

```bash
cd ~/uni_ws
source /opt/ros/humble/setup.bash
conda activate uros
colcon build --symlink-install
```

如果第一次只想先编译最核心包，可以先用：

```bash
colcon build --packages-up-to unitree_guide_controller keyboard_input go1_description --symlink-install
```

## 8. 运行前准备

运行前建议统一执行：

```bash
source /opt/ros/humble/setup.bash
source ~/uni_ws/install/setup.bash
conda activate uros
```

如果你复现的是 Gazebo Harmonic 仿真：

```bash
ros2 launch unitree_guide_controller gazebo.launch.py
ros2 run keyboard_input keyboard_input
```

## 9. 常见复现风险

别人复现失败，最常见的原因有：

- ROS 2 版本不一致（`jazzy` / `humble`）
- Gazebo 栈不一致（`gazebo_classic` / `gazebo_harmonic`）
- 工作区缺少 `ocs2_ros2`
- Unitree SDK 或外部仿真依赖版本不一致
- conda 环境过度覆盖系统 ROS Python 或编译器环境

如果希望严格复现，请优先完全匹配“第 1 节推荐复现基线”。

## 10. 推荐的 GitHub 上传顺序

建议按下面顺序整理仓库：

1. 先上传源码仓库 `quadruped_ros2_control`
2. 把本文件和 `environment.uros.yml` 一起加入仓库根目录
3. 在 `README.md` 中增加环境说明入口
4. 明确写出外部源码依赖：
   - `ocs2_ros2`
   - `unitree_sdk2`
5. 明确写出你实际验证过的仿真路径：
   - Gazebo Harmonic
   - Gazebo Classic
   - MuJoCo / Unitree
6. 补充你真实测试过的 launch / run 命令
7. 最后再发布视频、效果展示和性能说明

