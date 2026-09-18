# 仿真舵轮控制

仅修改仿真执行接口，不修改 sentry2026 或导航算法。

- 现有键盘/导航坐标转换及 mux 保留。输入 `/cmd_vel_chassis` 已是底盘速度，不再次转换。
- `swerve_sim_controller_node` 保留舵轮运动学及超过 90° 时反转轮速的逻辑，读取轮速、舵角和舵转速。
- 零指令或命令超时：目标轮速为零；估计平移速度不超过 0.5 m/s 时，恢复首次有效反馈记录的初始舵角。停车转舵期间并非立即禁止舵转动。
- `sentry_motor_controller` 在 Gazebo 每个物理步读取真实关节反馈，执行 sentry2026 风格的 wheel PIDUpdate 和 steer PIDUpdate_extw：轮速误差闭环、周期角误差加实际舵转速抑制。
- 积分按仿真时间计算并限幅，输出为 N·m。增益是仿真 SI 参数，不是实车电流 PID 数值的直接复制。
- ROS 只发送 `/swerve/*_wheel_joint/target_speed` (rad/s)、`/swerve/*_steer_joint/target_angle` (rad)。桥接到同名 Gazebo 模型关节话题。
- 电机环命令超时 0.2 仿真秒后，轮子保持零速、舵保持初始化角；仿真暂停不积分。
- 旧 wheel JointController 和 steer JointPositionController 保留在 SDF XML 注释内，不加载。云台控制不变。

电机增益、力矩限制位于 `seu_sentry_description/resource/xmacro/new_seu_sentry_sim.sdf.xmacro` 的 `libsentry_motor_controller.so` 插件参数。

控制参考：`sentry2026/MovingShoot_Sentry2025_RMUC_Stage2/Board_up/3SE_Sentry2025_DM/3SE/Tasks/Src/robot_control_task.c` 的 `ChassisControlUpdate`，以及 `3SE/Applications/PID/pid.c`。
停车角按用户要求采用仿真初始角，不照搬实车固定角。未移植 CAN、STM32、裁判系统功率限制及依赖实车参数的前馈。

Gazebo 官方接口：
- https://gazebosim.org/api/sim/8/jointforcecmdcomponent.html
- https://gazebosim.org/api/sim/8/JointForceCmd_8hh.html

编译后需要重启 Gazebo，已有模型不会热替换插件：

```bash
cd /home/dengjiaxi/simulation_seu/navigationsim
source /opt/ros/jazzy/setup.bash
colcon build --packages-select seu_sentry_sim_control seu_sentry_description --symlink-install
source install/setup.bash
./simulation.sh
```
