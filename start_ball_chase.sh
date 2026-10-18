#!/usr/bin/env bash
# Gazebo起動後に、直進補正とボール追跡をまとめて起動。
set -e
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  # 終了処理が完了するまで待つ。
  for pid in "${pids[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
ros2 run dtw_control heading_hold --ros-args -p use_sim_time:=true &
pids+=("$!")
ros2 run dtw_control ball_chase --ros-args -p use_sim_time:=true "$@" &
pids+=("$!")
echo '直進補正とボール追跡を起動しました。終了: Ctrl+C'
# 片方が終了したら、もう片方も終了。
wait -n "${pids[@]}"
