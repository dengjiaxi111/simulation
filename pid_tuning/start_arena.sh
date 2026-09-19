#!/usr/bin/env bash
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TASK_WS="$(dirname "$TASK_DIR")"
source /opt/ros/jazzy/setup.bash
source "$TASK_WS/install/setup.bash"
source "$TASK_WS/../navigationros2/install/setup.bash"
export ROS_DOMAIN_ID=77 GZ_PARTITION=seu_pid_tuning
exec ros2 launch velocity_control run.launch.py model_mode:=wheel enable_motion_adapter:=true wheel_model_path:="$TASK_DIR/observed_robot.sdf" world_path:="$TASK_WS/src/seu_sentry_sim_control/test/pid_arena.sdf"
