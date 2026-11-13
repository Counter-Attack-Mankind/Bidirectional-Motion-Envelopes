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
- 障碍物标记：`ObstacleN_MarkerM`

每个障碍物至少需要三个新鲜且不共线的标记。任一已发现标记遮挡时，系统
保留最后可信 OBB 用于显示，但禁止执行。无障碍现场必须显式确认
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
rostopic echo /estop
rostopic echo /object
rostopic echo /traj
rostopic echo /chassis_raw
rostopic echo /chassis
rostopic echo /reached
```

现场必须确认 `position_scale`、地图旋转和平移、航向偏置、刚体到后轴的变换、
障碍物膨胀、串口设备、转向符号和限幅。底盘原协议及 Arduino 固件保持不变；
ROS 层 `chassis_command_guard_node` 独立负责急停和命令超时保护。
