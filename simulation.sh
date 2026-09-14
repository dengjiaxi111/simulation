#!/usr/bin/env bash

# Start only the Gazebo/ROS simulation. Navigation algorithms are independent
# clients and are intentionally not started by this script.
set -Eeuo pipefail

SIM_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
NAVROS2_WS="${NAVROS2_WS:-${SIM_WS}/../navigationros2}"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
SIM_SETUP="${SIM_WS}/install/setup.bash"
NAVROS2_SETUP="${NAVROS2_WS}/install/setup.bash"
MODEL_MODE="${MODEL_MODE:-wheel}"
SPAWN_Z="${SPAWN_Z:-0.67}"

log() { printf '[simulation] %s\n' "$*"; }
die() { printf '[simulation] ERROR: %s\n' "$*" >&2; exit 1; }

stop_matching() {
  local signal="$1"; shift
  local pattern
  for pattern in "$@"; do
    pkill "-${signal}" -f "$pattern" 2>/dev/null || true
  done
}

cleanup_previous_run() {
  log "清理可能残留的 Gazebo、仿真节点和导航节点..."

  # Simulation and Gazebo processes.
  stop_matching TERM \
    '[r]os2 launch velocity_control run.launch.py' '[g]z sim' '[i]gn gazebo' \
    '[g]azebo --verbose' '[g]zserver' '[g]zclient' '[r]os_gz_bridge' '[p]arameter_bridge' \
    '[r]obot_state_publisher' '[c]ontroller_manager' '[s]werve_sim_controller_node' \
    '[w]heel_sim_adapter_node' '[k]eyboard_sim_adapter_node' '[k]eyboard_teleop_node' \
    '[c]md_vel_mux_node' '[c]ontrol_mode_once_node' '[p]p2_to_livox_node' \
    '[n]avigation_initialpose_gate.py' '[i]mu_alias_node' '[c]lock_bridge' \
    '[s]tartup_pose_release.py'

  # Both supported algorithm launch styles; only processes are stopped.
  stop_matching TERM \
    '[r]os2 launch velocity_control algorithm_sim.launch.py' \
    '[r]os2 launch nav_bringup navigation_s.launch.py' '[s]mall_point_lio_node' \
    '[s]mall_point_lio' '[l]io_after_tf.py' '[l]ocalization_initializer_node' '[n]av_server' \
    '[n]av2_' '[c]ontroller_server' '[p]lanner_server' '[b]t_navigator' \
    '[b]ehavior_server' '[m]ap_server' '[l]ifecycle_manager' '[c]ostmap' \
    '[p]ointcloud_obstacle_layer' '[l]ocal_obstacle_grid_node' '[f]ake_vel_transform' \
    '[s]entry_decision_node' '[r]viz2'

  sleep 1
  stop_matching KILL \
    '[r]os2 launch velocity_control run.launch.py' '[g]z sim' '[i]gn gazebo' \
    '[g]azebo --verbose' '[g]zserver' '[g]zclient' '[r]os_gz_bridge' '[p]arameter_bridge' \
    '[r]obot_state_publisher' '[c]ontroller_manager' '[s]werve_sim_controller_node' \
    '[w]heel_sim_adapter_node' '[k]eyboard_sim_adapter_node' '[k]eyboard_teleop_node' \
    '[c]md_vel_mux_node' '[c]ontrol_mode_once_node' '[p]p2_to_livox_node' \
    '[n]avigation_initialpose_gate.py' '[i]mu_alias_node' '[c]lock_bridge' \
    '[s]tartup_pose_release.py' \
    '[r]os2 launch velocity_control algorithm_sim.launch.py' \
    '[r]os2 launch nav_bringup navigation_s.launch.py' '[s]mall_point_lio_node' \
    '[s]mall_point_lio' '[l]io_after_tf.py' '[l]ocalization_initializer_node' '[n]av_server' '[n]av2_' \
    '[c]ontroller_server' '[p]lanner_server' '[b]t_navigator' '[b]ehavior_server' \
    '[m]ap_server' '[l]ifecycle_manager' '[c]ostmap' '[p]ointcloud_obstacle_layer' \
    '[l]ocal_obstacle_grid_node' '[f]ake_vel_transform' '[s]entry_decision_node' '[r]viz2'
}

check_environment() {
  [[ -f "${ROS_SETUP}" ]] || die "找不到 ${ROS_SETUP}"
  [[ -f "${SIM_SETUP}" ]] || die "找不到 ${SIM_SETUP}，请先构建 navigationsim"
  [[ -f "${NAVROS2_SETUP}" ]] || die "找不到 ${NAVROS2_SETUP}，请先构建 navigationros2"
  [[ "${MODEL_MODE}" == wheel || "${MODEL_MODE}" == leg ]] || die "MODEL_MODE 必须是 wheel 或 leg"
  command -v gnome-terminal >/dev/null 2>&1 || die "找不到 gnome-terminal，无法打开独立键盘控制终端"
}

main() {
  check_environment
  cleanup_previous_run

  # navigationros2 is sourced only for shared runtime dependencies (for
  # example robots_msgs); no navigation launch file is executed here.
  set +u
  source "${ROS_SETUP}"
  source "${SIM_SETUP}"
  source "${NAVROS2_SETUP}"
  set -u

  # Select the discrete NVIDIA renderer for Gazebo / OGRE on hybrid-GPU
  # machines.  Do not set DRI_PRIME: that may make Mesa choose the AMD card.
  unset DRI_PRIME || true
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  export __VK_LAYER_NV_optimus=NVIDIA_only
  export CUDA_VISIBLE_DEVICES=0
  export QT_XCB_GL_INTEGRATION=xcb_glx

  if ! nvidia-smi -L >/dev/null 2>&1; then
    log "警告：当前终端无法连接 NVIDIA 驱动；已强制 NVIDIA 环境，但 Gazebo/RViz 可能无法创建图形上下文"
  fi
  if command -v glxinfo >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
    local gl_renderer
    gl_renderer="$({ glxinfo -B 2>/dev/null || true; } | sed -n 's/^[[:space:]]*OpenGL renderer string:[[:space:]]*//p')"
    [[ "${gl_renderer}" == *NVIDIA* ]] || die \
      "NVIDIA GLX 未生效（当前 renderer: ${gl_renderer:-不可用}）；为避免回退 AMD，已停止启动"
    log "OpenGL renderer: ${gl_renderer}"
  else
    log "警告：无法在当前终端预检 OpenGL renderer；Gazebo 进程仍已强制使用 NVIDIA GLX"
  fi

  trap 'status=$?; log "停止仿真并清理本次启动的进程..."; cleanup_previous_run; exit "$status"' INT TERM EXIT

  log "启动单个 Gazebo 仿真模型（${MODEL_MODE}，spawn_z=${SPAWN_Z}，NVIDIA/GLX）"
  ros2 launch velocity_control run.launch.py \
    model_mode:="${MODEL_MODE}" \
    enable_motion_adapter:=true \
    wheel_spawn_z:="${SPAWN_Z}" &
  local simulation_pid=$!

  log "等待速度仲裁器启动..."
  local mux_ready=false
  for _ in $(seq 1 60); do
    if ! kill -0 "${simulation_pid}" 2>/dev/null; then
      wait "${simulation_pid}"
      die "仿真在速度仲裁器启动前退出"
    fi
    if ros2 node list 2>/dev/null | grep -Fxq '/cmd_vel_mux'; then
      mux_ready=true
      break
    fi
    sleep 0.25
  done
  [[ "${mux_ready}" == true ]] || die "等待 /cmd_vel_mux 超时"

  # Make every startup deterministic: zero chassis command, stationary gimbal,
  # and keyboard ownership until an algorithm receives /initialpose.
  ros2 run velocity_control control_mode_once_node --ros-args \
    -p mode:=keyboard -p use_sim_time:=true

  log "打开原键盘控制终端（K=键盘模式，N=导航模式）"
  gnome-terminal --title="Sentry 仿真键盘控制" -- bash -lc "
    source '${ROS_SETUP}'
    source '${SIM_SETUP}'
    source '${NAVROS2_SETUP}'
    exec ros2 run velocity_control keyboard_teleop_node --ros-args -p use_sim_time:=true
  " &

  log "仿真已启动；等待键盘操作或独立启动导航算法"
  wait "${simulation_pid}"
}

main "$@"
