#!/bin/bash
# ============================================================
#  实车轨迹回放（一键启动全部节点）
#  包含：roscore + 定位 + 底盘 + RViz场景 + 轨迹回放 + PP控制
#
#  用法：
#    ./exp_scripts/run_replay.sh            # 默认回放 trajectory_v2.csv
#    ./exp_scripts/run_replay.sh v2
#    ./exp_scripts/run_replay.sh v1
#    ./exp_scripts/run_replay.sh /abs/path/to/trajectory.csv
#
#  流程：
#    1. 自动启动所有节点
#    2. RViz显示场景+起终点足迹+朝向 → 对齐实车
#    3. 在【轨迹回放】标签页按 Enter → 发送轨迹 → 车开始跑
# ============================================================

SRC="source /home/libai/Code/condition_ws/devel/setup.bash"
CONFIG_DIR="/home/libai/Code/condition_ws/src/condition_planner/config"
REPLAY_INPUT="${1:-v2}"
TARGET=5
VRPN_SERVER="10.1.1.198"

# ── 控制器增益（每次实验在这里改）──────────────────────────────
KE=15.0       # 横向误差增益（K_e）：3 / 7 / 15 / 30 / 60
KYAW=4    # 航向误差增益（K_yaw）1.8
KP=4      # 纵向比例增益（Kp）1.0
# ──────────────────────────────────────────────────────────────

case "$REPLAY_INPUT" in
  v1)
    CSV="$CONFIG_DIR/trajectory_v1.csv"
    ;;
  v2)
    CSV="$CONFIG_DIR/trajectory_v2.csv"
    ;;
  *)
    CSV="$REPLAY_INPUT"
    ;;
esac

if [ ! -f "$CSV" ]; then
  echo "CSV not found: $CSV"
  echo "Usage:"
  echo "  ./exp_scripts/run_replay.sh            # default: trajectory_v2.csv"
  echo "  ./exp_scripts/run_replay.sh v2"
  echo "  ./exp_scripts/run_replay.sh v1"
  echo "  ./exp_scripts/run_replay.sh /abs/path/to/trajectory.csv"
  exit 1
fi

echo "=========================================="
echo "  Trajectory Replay - All-in-One"
echo "=========================================="
echo "  CSV:    $CSV"
echo "  Target: $TARGET"
echo "  VRPN:   $VRPN_SERVER"
echo "  Gains:  K_e=$KE  K_yaw=$KYAW  Kp=$KP"
echo "=========================================="

gnome-terminal --window \
    --title="roscore" -e "bash -c '$SRC; roscore; exec bash'" \
    --tab --title="VRPN+定位" -e "bash -c '$SRC; sleep 2; roslaunch condition_planner localization.launch server:=$VRPN_SERVER; exec bash'" \
    --tab --title="底盘" -e "bash -c '$SRC; sleep 3; rosrun chassis chassis_node; exec bash'" \
    --tab --title="RViz+场景" -e "bash -c '$SRC; sleep 3; roslaunch condition_planner replay_viz.launch csv_file:=$CSV; exec bash'" \
    --tab --title="轨迹回放(按Enter发送)" -e "bash -c '$SRC; sleep 5; rosrun condition_planner trajectory_replay_node _csv_file:=$CSV _target:=$TARGET; exec bash'" \
    --tab --title="Pure Pursuit" -e "bash -c '$SRC; sleep 5; rosrun pure_pursuit pure_pursuit_node _target:=$TARGET _K_e:=$KE _K_yaw:=$KYAW _Kp:=$KP; exec bash'"
