# BME 双向具身足迹轨迹规划系统

本工程是 ROS1 Noetic / C++17 实现的双向具身足迹（Bidirectional Motion
Envelopes，BME）轨迹规划系统，包含 Hybrid A*、初值生成、CasADi/IPOPT
非线性优化、仿真回放以及 Nokov 动捕实车接入。

## 目录说明

- `planning/hybrid_astar/`：Hybrid A* 与 Reeds–Shepp 搜索实现。
- `include/planning/hybrid_astar/`：对应公共头文件。
- `optimizer/`：BME 初值与非线性优化。
- `scene/`：场景、动捕场景恢复及键盘编辑脚本。
- `visualize/`：规划阶段、轨迹回放、BME 和实际扫掠区域可视化。
- `execution/`：实车任务状态机、轨迹适配与每任务日志。
- `experiment/`：实车依赖包（消息、VRPN、Pure Pursuit、底盘）。

## 编译

将本包与 `sandbox_msgs` 放入 Noetic catkin 工作空间。实车模式还需要把
`vrpn_client_ros`、`pure_pursuit` 和 `chassis` 作为同级包放入 `src/`；完整
工作空间布局和依赖见 [REAL_VEHICLE.md](REAL_VEHICLE.md)。

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## 运行

仿真与普通规划沿用工程现有 launch。实车接入默认只规划、不驱动车辆：

```bash
roslaunch buffered_footprint_planner real_vehicle.launch server:=NOKOV_IP
```

确认坐标、单位、后轴参考点、障碍标记和物理急停后，才可显式打开执行：

```bash
roslaunch buffered_footprint_planner real_vehicle.launch \
  server:=NOKOV_IP enable_execution:=true serial_port:=/dev/ttyUSB0
```

默认 Nokov 端口为 `3883`、更新频率为 `100 Hz`。车辆刚体命名为
`VehicleN`，后轴候选标记命名为 `VehicleN_MarkerM`，障碍物标记命名为
`ObstacleN_MarkerM`。任一已发现障碍标记遮挡后，系统保留最后可信 OBB
用于显示，但立即禁止执行；无障碍现场必须显式设置
`mocap/allow_empty_obstacles: true`。

## 实车安全与数据

底盘原始控制协议、方向和限幅保持原工程不变。ROS 层的
`chassis_command_guard_node` 独立负责急停与命令超时，不要求修改或刷写
Arduino 固件。每次成功执行会在该规划结果目录生成
`actual_tracking.csv` 和 `control_commands.csv`。软件保护不能替代物理急停，
首次上车必须进行架空轮测试。

## 测试

```bash
catkin_make run_tests_buffered_footprint_planner
catkin_test_results
```

回归测试覆盖 Hybrid A*、超时、轨迹适配、OBB 等价朝向、动捕数据失效、
执行授权撤销和多轮任务状态。测试目标在 Release 构建中也显式启用断言。
