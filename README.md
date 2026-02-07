# NavigationSim

## 介绍
东南大学3SE战队 RoboMaster 哨兵机器人导航仿真项目

本项目基于 ROS 2 和 Gazebo Ignition，提供 RMUC 2026 赛场环境下的哨兵机器人仿真系统，用于导航算法开发和测试。

## 主要功能

- **Gazebo 仿真环境**: 完整的 RMUC 2026 赛场模型
- **速度控制器**: 基于 ros2_control 的差速底盘控制器
- **Livox 点云转换**: PP2 格式到 Livox 自定义消息的转换节点

## 软件架构

```
navigationsim/
├── src/
│   └── sentry_velocity/
│       ├── gazeboworld/          # Gazebo 仿真世界和模型
│       │   ├── models/          # 场地模型 (RMUC 2026)
│       │   └── worlds/          # 世界文件
│       └── velocity_control/    # ROS 2 速度控制包
│           ├── config/          # 控制器配置
│           ├── launch/          # 启动文件
│           ├── src/             # 源代码
│           └── urdf/            # 机器人描述文件
```

### 依赖项

- ROS 2 (Humble 或更高版本)
- Gazebo Ignition
- ros2_control
- livox_ros_driver2
- tf2_ros
- robot_state_publisher

## 安装教程

1. **安装 ROS 2 和 Gazebo Ignition**
   ```bash
   # 请参考 ROS 2 官方文档安装对应版本
   ```

2. **克隆仓库**
   ```bash
   git clone <repository_url>
   cd navigationsim
   ```

3. **安装依赖**
   ```bash
   cd navigationsim
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **编译工作空间**
   ```bash
   colcon build
   source install/setup.bash
   ```

## 使用说明

### 修改配置路径  

- 将urdf文件末尾的`/home/lehan/navigationsim/install/velocity_control/share/velocity_control/config/velocity_controller.yaml`改为正确的yaml文件路径  
- 同理，将launch文件中的`gazeboworld_path`一项改为正确的`gazeboworld`路径


### 启动完整仿真

```bash
ros2 launch velocity_control run.launch.py
```

此命令将启动:
- Gazebo 仿真环境 (RMUC 2026 赛场)
- 机器人模型加载
- 速度控制器
- TF 树发布
- PP2 点云数据转换

### 单独启动控制器

```bash
ros2 launch velocity_control control.launch.py
```

## 配置说明

控制器参数在 `config/velocity_controller.yaml` 中配置:

- **轮间距**: 0.58m
- **轮半径**: 0.08m
- **最大力矩**: 30.0 Nm

## 参与贡献

1. Fork 本仓库
2. 新建 Feat_xxx 分支
3. 提交代码
4. 新建 Pull Request

## Lisence

TODO

