# 实车工作空间与现场检查

## 工作空间布局

项目总目录本身不是 ROS 包；BME 主包位于
`buffered_footprint_planner/`，各实验包位于 `experiment/`。整个项目只需作为
一个目录放入 catkin 工作空间，catkin 会递归发现所有有效 `package.xml`：

```text
real_vehicle_ws/
└── src/
    └── buffered_footprint_planner-cpp/
        ├── buffered_footprint_planner/
        └── experiment/
            ├── sandbox_msgs/
            ├── pure_pursuit/
            ├── vrpn_client_ros/
            └── chassis/
```

不再手工复制包，也不再创建软链接。

## 两种编译环境

本机开发环境保留以下文件，catkin 会跳过缺少现场库的两个功能包：

```text
experiment/vrpn_client_ros/CATKIN_IGNORE
experiment/chassis/CATKIN_IGNORE
```

本机直接在工作空间执行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

实验室电脑已安装 VRPN 和 TinyProtocol/tinyhal。现场只删除上述两个
`CATKIN_IGNORE`，再执行相同的 `catkin_make`。不要把该删除操作提交回仓库，
也不需要安装其他依赖、改动底盘算法或刷写 Arduino 固件。

## Nokov 与启动

Nokov 默认端口为 `3883`，更新频率为 `100 Hz`。名称约定：

- 车辆刚体：`VehicleN`
- 车辆后轴候选标记：`VehicleN_MarkerM`
- 障碍物刚体：`ObstacleN`（兼容 `ObsN`）
- 旧版障碍物标记：`ObstacleN_MarkerM`（仅 `marker_obb` 模式）

### 障碍物反光点与刚体标定

每个 `0.15 m × 0.15 m` 方盒安装四个反光点。其中一个反光点位于盒子几何中心，
另外三个采用非对称布局，使 Nokov 能唯一识别刚体及其方向；四点不需要位于盒子
四角。对每个障碍物在 Nokov 中完成以下标定：

1. 使用四个反光点建立刚体，名称设为 `ObstacleN` 或 `ObsN`。
2. 手动将刚体原点设为盒子几何中心的反光点。
3. 将刚体 x/y 轴与盒子的两条边对齐，并核对航向正方向。

程序动态发现 `/vrpn_client_node/ObstacleN/pose` 和
`/vrpn_client_node/ObsN/pose`，无需在 YAML 中填写障碍物数量或 ID。默认
`rigid_body` 模式不订阅障碍物 Marker 话题，盒体尺寸不会随反光点分布、数量或
遮挡改变。刚体位姿或四元数无效、数据超时时，系统保留最后可信矩形用于显示，
但禁止启动规划或继续执行轨迹；刚体暂时丢失不会从障碍物集合中删除。

### 障碍物几何配置

`config/real_vehicle.yaml` 的 `mocap` 节点包含：

```yaml
obstacle_geometry_mode: rigid_body
obstacle_length: 0.15
obstacle_width: 0.15
obstacle_center_offset_x: 0.0
obstacle_center_offset_y: 0.0
obstacle_yaw_offset: 0.0
```

长宽和中心偏移单位均为米，航向偏移单位为弧度。中心偏移定义在 Nokov 障碍物
刚体局部坐标系中，会随刚体航向旋转后再加到刚体原点；`obstacle_yaw_offset`
叠加在刚体航向与地图旋转角之后。所有障碍物共用这组参数。

如需恢复旧方式，将 `obstacle_geometry_mode` 设为 `marker_obb`。此时只发现
`ObstacleN_MarkerM`，每个障碍物至少需要三个新鲜且不共线的 Marker，并继续
使用 `obstacle_padding` 和 `minimum_obstacle_markers`。无障碍现场必须显式确认
`mocap/allow_empty_obstacles: true`。

安全默认的纯规划命令保持不变：

```bash
roslaunch buffered_footprint_planner real_vehicle.launch server:=NOKOV_IP
```

确认物理急停并完成架空轮测试后，才可打开执行：

```bash
roslaunch buffered_footprint_planner real_vehicle.launch \
  server:=NOKOV_IP enable_execution:=true serial_port:=/dev/ttyUSB0
```

## 现场检查

```bash
rostopic list | sort | grep /vrpn_client_node
rostopic type /vrpn_client_node/Vehicle0/pose
rostopic hz /vrpn_client_node/Vehicle0/pose
rostopic echo -n 1 /vrpn_client_node/Vehicle0/pose
rostopic type /vrpn_client_node/Obstacle0/pose
rostopic hz /vrpn_client_node/Obstacle0/pose
rostopic echo -n 1 /vrpn_client_node/Obstacle0/pose
rostopic echo /estop
rostopic echo /object
rostopic echo /traj
rostopic echo /chassis_raw
rostopic echo /chassis
rostopic echo /reached
```

现场必须确认 `position_scale`、地图旋转和平移、车辆与障碍物航向偏置、刚体到
后轴的变换、障碍物固定长宽和中心偏移、串口设备、转向符号和限幅。底盘原协议
及 Arduino 固件保持不变；
ROS 层 `chassis_command_guard_node` 独立负责急停和命令超时保护。
