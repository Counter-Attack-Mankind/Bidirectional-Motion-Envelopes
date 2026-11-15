# BME 双向具身足迹轨迹规划系统

本目录是 C++ 项目总目录，同时也是可直接放入 catkin 工作空间 `src/` 的
多功能包集合。目录组织参考 `fork_planner`：BME 主功能包和实车实验功能包
互为同级目录，catkin 会递归发现它们，不需要复制功能包或创建软链接。

```text
buffered_footprint_planner-cpp/
├── buffered_footprint_planner/   # BME 主功能包
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── common/ config/ execution/ include/
│   ├── launch/ optimizer/ planning/ scene/
│   └── src/ srv/ tests/ visualize/ results/
├── experiment/                   # 独立实验功能包集合
│   ├── sandbox_msgs/
│   ├── pure_pursuit/
│   ├── lqr_controller/
│   ├── nokov_localization/
│   ├── vrpn_client_ros/
│   └── chassis/
├── README.md
└── REAL_VEHICLE.md
```

## 本机开发环境

本机缺少底盘所需 TinyProtocol 等现场依赖，因此仓库有意保留：

- `experiment/vrpn_client_ros/CATKIN_IGNORE`
- `experiment/chassis/CATKIN_IGNORE`

不要在本机删除这两个标记，也不需要为验证本工程安装额外依赖。将整个项目
目录直接放在 Noetic 工作空间的 `src/` 下即可：

```bash
mkdir -p ~/catkin_ws/src
cp -a buffered_footprint_planner-cpp ~/catkin_ws/src/
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

catkin 会自动发现 BME、`sandbox_msgs`、Pure Pursuit 等未被忽略的包，并跳过
上述两个现场包。

回归测试：

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests_buffered_footprint_planner
catkin_test_results
```

## 实验室实车环境

实验室电脑已经安装 VRPN、TinyProtocol/tinyhal 及底盘所需依赖。仍然将整个
项目目录直接放入工作空间 `src/`，现场仅删除两个忽略标记后编译：

```bash
cd ~/catkin_ws/src/buffered_footprint_planner-cpp
rm experiment/vrpn_client_ros/CATKIN_IGNORE
rm experiment/chassis/CATKIN_IGNORE
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

不需要移动、复制或链接其中任何功能包，也不需要修改或刷写 Arduino 固件。
实车检查清单见 [REAL_VEHICLE.md](REAL_VEHICLE.md)。

## 启动方式

工程重组不改变 ROS 包名和 launch 文件名，原有四种命令保持不变：

```bash
roslaunch buffered_footprint_planner simulation.launch
roslaunch buffered_footprint_planner scene_create.launch
roslaunch buffered_footprint_planner experiment.launch task:=task1
roslaunch buffered_footprint_planner real_vehicle.launch server:=NOKOV_IP
```

实车执行仍需显式授权：

```bash
roslaunch buffered_footprint_planner real_vehicle.launch \
  server:=NOKOV_IP enable_execution:=true serial_port:=/dev/ttyUSB0
```

软件急停和命令超时保护不能替代物理急停。首次上车必须核对单位、坐标变换、
后轴参考点、转向符号和限幅，并进行架空轮测试。

## 实车障碍物标记

实车障碍物默认按 Nokov 刚体位姿和固定长宽建模。每个方盒安装四个反光点：
一个位于盒子几何中心，另外三个采用可稳定区分方向的非对称布局；反光点不要求
位于四角。在 Nokov 中将刚体原点手动设为中心反光点，并将刚体坐标轴与盒子边缘
对齐。刚体名称使用 `ObstacleN`，也兼容 `ObsN`，编号由运行时动态发现。

默认配置位于 `buffered_footprint_planner/config/real_vehicle.yaml`：

- `obstacle_geometry_mode: rigid_body` 使用刚体位姿；改为 `marker_obb` 可启用旧的
  `ObstacleN_MarkerM` 拟合方式。
- `obstacle_length`、`obstacle_width` 为所有障碍物共用的固定尺寸，单位为米。
- `obstacle_center_offset_x`、`obstacle_center_offset_y` 是 Nokov 障碍物刚体局部
  坐标系中的中心修正，单位为米。
- `obstacle_yaw_offset` 是刚体航向到盒体航向的修正，单位为弧度。

`rigid_body` 模式不会订阅障碍物 Marker 话题。刚体无效或超时时，最后可信矩形
仍用于显示，但规划和轨迹执行会被禁止；车辆刚体、车辆 Marker 和后轴中心算法
保持不变。详细标定与现场检查见 [REAL_VEHICLE.md](REAL_VEHICLE.md)。
