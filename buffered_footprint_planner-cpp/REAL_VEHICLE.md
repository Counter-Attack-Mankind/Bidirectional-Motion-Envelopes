# Real vehicle workspace and field checklist

`buffered_footprint_planner` is itself a catkin package. Catkin does not recurse
through it to find the packages under `experiment/`. Deploy the selected
packages as siblings in one workspace (symlinks are convenient):

```
real_vehicle_ws/src/
  buffered_footprint_planner/  -> buffered_footprint_planner-cpp
  sandbox_msgs/                -> buffered_footprint_planner-cpp/experiment/sandbox_msgs
  vrpn_client_ros/             (copy of experiment/vrpn_client_ros)
  pure_pursuit/                -> buffered_footprint_planner-cpp/experiment/pure_pursuit
  chassis/                     (copy of experiment/chassis)
```

Do not add `lqr_controller`. The archived `vrpn_client_ros` and `chassis`
directories contain `CATKIN_IGNORE`, so copy those two to the workspace and
remove the marker only from the copies. `sandbox_msgs` and `pure_pursuit` can be
copied or symlinked. Keeping the archived `experiment/` tree unchanged prevents
nested package discovery and duplicate `sandbox_msgs` definitions.

Ubuntu 20.04 / ROS Noetic dependencies include the dependencies already used by
BME (CasADi with pkg-config metadata, IPOPT, yaml-cpp) plus `libvrpn-dev`,
`ros-noetic-tf`, `ros-noetic-tf2-geometry-msgs`, and the TinyProtocol/tinyhal
libraries required by `chassis`. Build with `catkin_make -DCMAKE_BUILD_TYPE=Release`
or `catkin build`, then source `devel/setup.bash`.

The archived TinyProtocol source is provided as `experiment/third_party/tinyproto.zip`.
If `libtinyprotocol` is not already installed, extract it and run:

```
make -f Makefile.linux library
sudo make -f Makefile.linux DESTDIR=/usr/local install
sudo ldconfig
```

The chassis build requires both `TinyProtocol.h`/`hal/tiny_serial.h` and
`libtinyprotocol.a` (or `.so`). This repository does not silently install them
into the host system.

Planning only (safe default):

```
roslaunch buffered_footprint_planner real_vehicle.launch server:=NOKOV_IP
```

Execution mode (requires a physical emergency stop and a lifted-wheel test):

```
roslaunch buffered_footprint_planner real_vehicle.launch \
  server:=NOKOV_IP enable_execution:=true serial_port:=/dev/ttyUSB0
```

Nokov names must be `VehicleN` for the vehicle rigid body and
`ObstacleN_MarkerM` for each obstacle marker (`N` and `M` are decimal IDs).
At least three fresh, non-collinear markers are required per obstacle. Tracker
topics may appear after startup; the provider rescans continuously.

Field checks:

```
rostopic list | sort | grep /vrpn_client_node
rostopic type /vrpn_client_node/Vehicle0/pose
rostopic hz /vrpn_client_node/Vehicle0/pose
rostopic echo -n 1 /vrpn_client_node/Vehicle0/pose
rostopic echo /estop
rostopic echo /object
rostopic echo /traj
rostopic echo /chassis
rostopic echo /reached
```

Confirm `position_scale`, map rotation/translation, yaw offset, rigid-body to
rear-axle translation, obstacle padding, steering sign/limit and serial device
on site. Software stop logic is not a substitute for a physical emergency stop.
Flash the updated `experiment/chassis/main.ino` before powered driving: it adds
the MCU-side 350 ms command watchdog. Until that firmware is installed, only the
ROS-side chassis watchdog is active.
