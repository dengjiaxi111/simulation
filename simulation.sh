#!/usr/bin/env bash

set -Eeuo pipefail

readonly SIM_WS="/home/dengjiaxi/simulation_seu/navigationsim"
readonly NAV_WS="${NAV_WS:-/home/dengjiaxi/simulation_seu/navigation2026}"
readonly ROS_SETUP="/opt/ros/jazzy/setup.bash"
readonly SIM_SETUP="${SIM_WS}/install/setup.bash"
readonly NAV_SETUP="${NAV_WS}/install/setup.bash"

readonly MODEL_MODE="${MODEL_MODE:-wheel}"
readonly WHEEL_MODEL_PATH="${WHEEL_MODEL_PATH:-${SIM_WS}/src/seu_sentry_description/resource/xmacro/new_seu_sentry_sim.sdf.xmacro}"
readonly WHEEL_RESOURCE_PATH="${WHEEL_RESOURCE_PATH:-${SIM_WS}/src/seu_sentry_description/resource/models}"
readonly NAMESPACE="${NAMESPACE:-}"
readonly USE_RVIZ="${USE_RVIZ:-true}"
readonly USE_JOY="${USE_JOY:-true}"
readonly USE_KEYBOARD="${USE_KEYBOARD:-true}"
readonly USE_DECISION="${USE_DECISION:-true}"
readonly USE_SIM_POINTCLOUD_CONVERTER="${USE_SIM_POINTCLOUD_CONVERTER:-true}"
readonly AUTO_BUILD_SIM="${AUTO_BUILD_SIM:-true}"
readonly BUILD_JOBS="${BUILD_JOBS:-2}"
readonly NAV2_SIM_PARAMS="${NAV2_SIM_PARAMS:-/tmp/navigationsim_nav2_params.yaml}"
readonly SIMULATION_LOCK_FILE="${SIMULATION_LOCK_FILE:-/tmp/simulation_seu_gazebo.lock}"

export RCUTILS_COLORIZED_OUTPUT=1
export PYTHONUNBUFFERED=1
export SIM_WS
export NAV_WS

declare -a COMPONENT_PIDS=()
declare -a COMPONENT_NAMES=()
CLEANING_UP=0

info() {
  printf '\n\033[1;34m[仿真启动]\033[0m %s\n' "$*"
}

error() {
  printf '\n\033[1;31m[错误]\033[0m %s\n' "$*" >&2
}

source_setup() {
  set +u
  source "$1"
  set -u
}

process_is_running() {
  kill -0 "$1" 2>/dev/null
}

check_files() {
  local file
  for file in "${ROS_SETUP}" "${NAV_SETUP}"; do
    if [[ ! -f "${file}" ]]; then
      error "找不到必需文件：${file}"
      error "请先分别在 ${SIM_WS} 和 ${NAV_WS} 执行 colcon build --symlink-install。"
      exit 1
    fi
  done

  if [[ "${MODEL_MODE}" != "leg" && "${MODEL_MODE}" != "wheel" ]]; then
    error "MODEL_MODE 只能是 leg 或 wheel，当前是：${MODEL_MODE}"
    exit 1
  fi

  if [[ "${MODEL_MODE}" == "wheel" ]]; then
    for file in "${WHEEL_MODEL_PATH}" "${WHEEL_RESOURCE_PATH}"; do
      if [[ ! -e "${file}" ]]; then
        error "找不到 wheel 模式资源：${file}"
        exit 1
      fi
    done
  fi

  if ! command -v setsid >/dev/null 2>&1; then
    error "系统中找不到 setsid，无法可靠管理 Gazebo/ROS 子进程。"
    exit 1
  fi
  if ! command -v gnome-terminal >/dev/null 2>&1; then
    error "系统中找不到 gnome-terminal，无法弹出 Navigation2026 独立终端。"
    exit 1
  fi
}

acquire_simulation_lock() {
  exec 9>"${SIMULATION_LOCK_FILE}"
  if ! flock -n 9; then
    error "另一套仿真正在运行。请先在对应终端按 Ctrl+C 停止，再启动当前脚本。"
    exit 1
  fi
}

build_sim_workspace() {
  [[ "${AUTO_BUILD_SIM}" == "true" ]] || return 0

  info "构建当前仿真工作区，确保使用最新 ROS2 代码……"
  (
    cd "${SIM_WS}"
    source_setup "${ROS_SETUP}"
    source_setup "${NAV_SETUP}"
    export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS}"
    export MAKEFLAGS="-j${BUILD_JOBS}"
    CMAKE_PREFIX_PATH="${NAV_WS}/install/robots_msgs:${CMAKE_PREFIX_PATH:-}" colcon build --cmake-args -Drobots_msgs_DIR="${NAV_WS}/install/robots_msgs/share/robots_msgs/cmake" --symlink-install --executor sequential --parallel-workers "${BUILD_JOBS}" --packages-select seu_sentry_description seu_sentry_sim_control velocity_control
  )
}

check_wheel_dependencies() {
  [[ "${MODEL_MODE}" == "wheel" ]] || return 0

  if ! python3 -c 'import xmacro.xmacro4sdf' >/dev/null 2>&1; then
    error "wheel 模式缺少 Python 依赖 xmacro。"
    error "安装方式：python3 -m pip install --user --break-system-packages xmacro"
    exit 1
  fi

  if ! ros2 pkg prefix rmoss_gz_resources >/dev/null 2>&1; then
    error "wheel 模式缺少 rmoss_gz_resources。"
    error "请在 ${SIM_WS} 执行：vcs import --recursive src < src/pb2025_robot_description/dependencies.repos"
    error "然后重新 colcon build --symlink-install 并 source install/setup.bash。"
    exit 1
  fi

  local rmoss_prefix
  rmoss_prefix="$(ros2 pkg prefix rmoss_gz_resources)"
  if ! find "${rmoss_prefix}" -path '*rm25_example_robot.def.xmacro' -print -quit | grep -q .; then
    error "rmoss_gz_resources 已找到，但缺少 rm25_example_robot.def.xmacro。"
    error "请确认 dependencies.repos 已完整拉取，且 rmoss_gz_resources 安装空间完整。"
    exit 1
  fi
}

stop_process_groups() {
  local signal="$1"
  local pid
  for pid in "${COMPONENT_PIDS[@]}"; do
    if process_is_running "${pid}"; then
      kill "-${signal}" -- "-${pid}" 2>/dev/null || true
    fi
  done
}

shutdown() {
  local exit_code="${1:-0}"
  if (( CLEANING_UP )); then
    return
  fi
  CLEANING_UP=1
  trap - INT TERM EXIT

  if (( ${#COMPONENT_PIDS[@]} > 0 )); then
    info "正在停止本次启动的仿真和导航组件……"
    stop_process_groups INT
    # gnome-terminal runs its command through the desktop terminal server, so
    # that ROS process is not guaranteed to share the client process group.
    pkill -INT -f '[r]os2 launch nav_bringup navigation_s.launch.py' 2>/dev/null || true
    pkill -INT -f '[k]eyboard_teleop_node' 2>/dev/null || true
    sleep 2
    stop_process_groups TERM
    pkill -TERM -f '[r]os2 launch nav_bringup navigation_s.launch.py' 2>/dev/null || true
    pkill -TERM -f '[k]eyboard_teleop_node' 2>/dev/null || true
    sleep 1
    stop_process_groups KILL
    pkill -KILL -f '[r]os2 launch nav_bringup navigation_s.launch.py' 2>/dev/null || true
    pkill -KILL -f '[k]eyboard_teleop_node' 2>/dev/null || true
    wait "${COMPONENT_PIDS[@]}" 2>/dev/null || true
  fi

  info "所有仿真组件已停止。"
  exit "${exit_code}"
}

cleanup_residual_processes() {
  info "清理上一次运行残留的进程……"
  pkill -INT -f '[r]os2 launch velocity_control' 2>/dev/null || true
  pkill -INT -f '[r]os2 launch nav_bringup' 2>/dev/null || true
  pkill -INT -f '[r]os2 launch pointcloud_obstacle_layer' 2>/dev/null || true
  pkill -INT -f '[r]os2 launch fake_vel_transform' 2>/dev/null || true
  pkill -INT -f '[r]os2 run nav2_map_server' 2>/dev/null || true
  pkill -INT -f '[r]os2 run localization_initializer' 2>/dev/null || true
  pkill -INT -f '[r]os2 run small_point_lio' 2>/dev/null || true
  pkill -INT -f '[r]os2 run tf2_ros static_transform_publisher' 2>/dev/null || true
  pkill -INT -f '[r]os2 topic echo /initialpose' 2>/dev/null || true
  pkill -INT -f '[l]ivox_ros_driver' 2>/dev/null || true
  pkill -INT -f '[g]z sim' 2>/dev/null || true
  pkill -INT -f '[p]arameter_bridge' 2>/dev/null || true
  pkill -INT -f '[r]obot_state_publisher' 2>/dev/null || true
  pkill -INT -f '[p]p2_to_livox_node' 2>/dev/null || true
  pkill -INT -f '[i]mu_alias_node' 2>/dev/null || true
  pkill -INT -f '[w]heel_sim_adapter_node' 2>/dev/null || true
  pkill -INT -f '[k]eyboard_teleop_node' 2>/dev/null || true
  pkill -INT -f '[s]mall_point_lio' 2>/dev/null || true
  pkill -INT -f '[p]ointcloud_segmentation.*segmentation' 2>/dev/null || true
  pkill -INT -f '[f]ake_vel_transform' 2>/dev/null || true
  pkill -INT -f '[c]lock_bridge' 2>/dev/null || true
  pkill -INT -f '[b]ridge_node.*clock_bridge' 2>/dev/null || true
  pkill -INT -f '[i]gn_sim_pointcloud_tool' 2>/dev/null || true
  pkill -INT -f '[r]viz2' 2>/dev/null || true
  pkill -INT -f '[s]tatic_transform_publisher' 2>/dev/null || true
  pkill -INT -f '[m]ap_server.*initial_pose_map_server' 2>/dev/null || true
  pkill -INT -f '[p]ointcloud_segmentation_node' 2>/dev/null || true
  pkill -INT -f '[l]ocal_obstacle_grid_node' 2>/dev/null || true
  pkill -INT -f '[l]ocalization_initializer_node' 2>/dev/null || true
  pkill -INT -f '[s]mall_point_lio_node' 2>/dev/null || true
  pkill -INT -f '[s]werve_sim_controller_node' 2>/dev/null || true
  pkill -INT -f '[p]p2_to_livox_node' 2>/dev/null || true
  sleep 2
  pkill -TERM -f '[p]ointcloud_segmentation.*segmentation' 2>/dev/null || true
  pkill -TERM -f '[f]ake_vel_transform' 2>/dev/null || true
  pkill -TERM -f '[c]lock_bridge' 2>/dev/null || true
  pkill -TERM -f '[b]ridge_node.*clock_bridge' 2>/dev/null || true
  pkill -TERM -f '[p]p2_to_livox_node' 2>/dev/null || true
  pkill -TERM -f '[i]mu_alias_node' 2>/dev/null || true
  pkill -TERM -f '[w]heel_sim_adapter_node' 2>/dev/null || true
  pkill -TERM -f '[k]eyboard_teleop_node' 2>/dev/null || true
  pkill -TERM -f '[s]tatic_transform_publisher' 2>/dev/null || true
  pkill -TERM -f '[m]ap_server.*initial_pose_map_server' 2>/dev/null || true
  pkill -TERM -f '[l]ocalization_initializer_node' 2>/dev/null || true
  pkill -TERM -f '[s]mall_point_lio_node' 2>/dev/null || true
  pkill -TERM -f '[p]ointcloud_segmentation_node' 2>/dev/null || true
  pkill -TERM -f '[l]ocal_obstacle_grid_node' 2>/dev/null || true
  sleep 2
  pkill -KILL -f '[p]ointcloud_segmentation.*segmentation' 2>/dev/null || true
  pkill -KILL -f '[f]ake_vel_transform' 2>/dev/null || true
  pkill -KILL -f '[c]lock_bridge' 2>/dev/null || true
  pkill -KILL -f '[b]ridge_node.*clock_bridge' 2>/dev/null || true
  pkill -KILL -f '[p]p2_to_livox_node' 2>/dev/null || true
  pkill -KILL -f '[i]mu_alias_node' 2>/dev/null || true
  pkill -KILL -f '[w]heel_sim_adapter_node' 2>/dev/null || true
  pkill -KILL -f '[k]eyboard_teleop_node' 2>/dev/null || true
  pkill -KILL -f '[s]tatic_transform_publisher' 2>/dev/null || true
  pkill -KILL -f '[m]ap_server.*initial_pose_map_server' 2>/dev/null || true
  pkill -KILL -f '[l]ocalization_initializer_node' 2>/dev/null || true
  pkill -KILL -f '[s]mall_point_lio_node' 2>/dev/null || true
  pkill -KILL -f '[p]ointcloud_segmentation_node' 2>/dev/null || true
  pkill -KILL -f '[l]ocal_obstacle_grid_node' 2>/dev/null || true
  ros2 daemon stop 9>&- >/dev/null 2>&1 || true
  ros2 daemon start 9>&- >/dev/null 2>&1 || true
}

start_component() {
  local name="$1"
  local working_directory="$2"
  local setup_order="$3"
  local command="$4"

  info "启动 ${name}"
  setsid bash -lc "
    set -e
    exec 9>&-
    cd '${working_directory}'
    source '${ROS_SETUP}'
    ${setup_order}
    exec ${command}
  " &

  COMPONENT_PIDS+=("$!")
  COMPONENT_NAMES+=("${name}")
}

start_terminal_component() {
  local name="$1"
  local working_directory="$2"
  local setup_order="$3"
  local command="$4"

  info "启动 ${name}（独立终端）"
  setsid gnome-terminal --wait --title="${name}" -- bash -lc "
    set -e
    exec 9>&-
    cd '${working_directory}'
    source '${ROS_SETUP}'
    ${setup_order}
    exec ${command}
  " &

  COMPONENT_PIDS+=("$!")
  COMPONENT_NAMES+=("${name}")
}

start_untracked_component() {
  local name="$1"
  local working_directory="$2"
  local setup_order="$3"
  local command="$4"

  info "启动 ${name}（独立终端）"
  # Keyboard teleop must own a real TTY; a detached background shell makes
  # stdin non-interactive and the node exits immediately. Keep this terminal
  # outside COMPONENT_PIDS so pressing X does not stop the simulation supervisor.
  setsid gnome-terminal --title="${name}" -- bash -lc "
    set -e
    exec 9>&-
    cd '${working_directory}'
    source '${ROS_SETUP}'
    ${setup_order}
    exec ${command}
  " &
}

wait_for_message() {
  local topic="$1"
  local timeout_seconds="$2"
  local owner_pid="$3"
  local description="$4"
  local deadline=$((SECONDS + timeout_seconds))

  info "等待 ${description} 的实际数据（${topic}）……"
  while (( SECONDS < deadline )); do
    if ! process_is_running "${owner_pid}"; then
      error "等待 ${description} 时，所属组件已经退出。请查看上方日志。"
      return 1
    fi
    if timeout 2 ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done

  error "等待 ${description} 数据超时（${timeout_seconds} 秒）：${topic}"
  # A delayed topic must not tear down Gazebo/RViz/navigation. Keep waiting
  # components alive so the operator can diagnose or recover in the terminals.
  return 0
}

wait_for_node() {
  local node="$1"
  local timeout_seconds="$2"
  local owner_pid="$3"
  local description="$4"
  local deadline=$((SECONDS + timeout_seconds))

  info "等待 ${description}（${node}）……"
  while (( SECONDS < deadline )); do
    if ! process_is_running "${owner_pid}"; then
      error "等待 ${description} 时，所属组件已经退出。请查看上方日志。"
      return 1
    fi
    if ros2 node list 2>/dev/null | grep -Fxq "${node}"; then
      return 0
    fi
    sleep 1
  done

  error "等待 ${description} 超时（${timeout_seconds} 秒）：${node}"
  return 0
}

wait_for_initial_pose() {
  local gazebo_pid="$1"
  local rviz_pid="${2:-}"

  info "等待在 RViz 中使用 2D Pose Estimate 给定机器人初始位置（/initialpose）……"
  printf '%s\n' "收到初始位置前只运行仿真、传感器、定位和地图预览，不启动导航规划与控制。"
  while true; do
    if ! process_is_running "${gazebo_pid}"; then
      error "等待初始位置时 Gazebo 已退出，请查看上方日志。"
      return 1
    fi
    if [[ -n "${rviz_pid}" ]] && ! process_is_running "${rviz_pid}"; then
      error "等待初始位置时 RViz 已退出。"
      return 1
    fi
    if timeout 2 ros2 topic echo /initialpose --once >/dev/null 2>&1; then
      info "已收到 RViz 初始位置，现在启动 Navigation2026 规划与控制。"
      return 0
    fi
  done
}

activate_lifecycle_node() {
  local node="$1"
  local owner_pid="$2"
  local deadline=$((SECONDS + 30))

  info "激活 RViz 初始位置地图预览（${node}）……"
  while (( SECONDS < deadline )); do
    if ! process_is_running "${owner_pid}"; then
      error "地图预览节点在激活前已经退出。"
      return 1
    fi
    if ros2 lifecycle set "${node}" configure >/dev/null 2>&1 && \
       ros2 lifecycle set "${node}" activate >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done

  error "无法激活地图预览节点：${node}"
  return 1
}

namespaced_node() {
  local node="$1"
  if [[ -n "${NAMESPACE}" ]]; then
    printf "/%s/%s" "${NAMESPACE}" "${node#/}"
  else
    printf "/%s" "${node#/}"
  fi
}

nav_setup_prefix() {
  if [[ -n "${NAMESPACE}" ]]; then
    printf "export ROS_NAMESPACE=/%q;" "${NAMESPACE}"
  fi
}

generate_nav2_params() {
  cat > "${NAV2_SIM_PARAMS}" <<EOF
map_server:
  ros__parameters:
    use_sim_time: true
planner_server:
  ros__parameters:
    use_sim_time: true
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    odom_topic: /Odometry/EC
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: true
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.6
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      use_velocity_scaled_lookahead_dist: false
      transform_tolerance: 0.2
      use_rotate_to_heading: true
      allow_reversing: false
local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: odom
      robot_base_frame: base_link_fake
      update_frequency: 5.0
      publish_frequency: 2.0
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.05
      robot_radius: 0.3
      plugins: ["static_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: map
      robot_base_frame: base_link_fake
      update_frequency: 1.0
      publish_frequency: 1.0
      resolution: 0.05
      robot_radius: 0.3
      track_unknown_space: true
      plugins: ["static_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link_fake
    odom_topic: /Odometry/EC
behavior_server:
  ros__parameters:
    use_sim_time: true
    local_frame: odom
    global_frame: map
    robot_base_frame: base_link_fake
    local_costmap_topic: local_costmap/costmap_raw
    global_costmap_topic: global_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_footprint_topic: global_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
    wait:
      plugin: "nav2_behaviors::Wait"
EOF
}

monitor_components() {
  local exit_status=0
  local exited_pid=""
  local index

  set +e
  wait -n -p exited_pid "${COMPONENT_PIDS[@]}"
  exit_status=$?
  set -e

  for index in "${!COMPONENT_PIDS[@]}"; do
    if [[ "${COMPONENT_PIDS[$index]}" == "${exited_pid}" ]]; then
      error "${COMPONENT_NAMES[$index]} 已退出（状态码：${exit_status}），即将停止其余组件。"
      break
    fi
  done

  (( exit_status == 0 )) && exit_status=1
  shutdown "${exit_status}"
}

main() {
  # Reap any previous/orphaned simulation before validation or lock checks.
  # The cleanup patterns are limited to this project's Gazebo/ROS components.
  cleanup_residual_processes
  check_files
  acquire_simulation_lock
  build_sim_workspace

  if [[ ! -f "${SIM_SETUP}" ]]; then
    error "找不到必需文件：${SIM_SETUP}"
    error "请检查 ${SIM_WS} 的 colcon build 输出。"
    exit 1
  fi

  source_setup "${ROS_SETUP}"
  source_setup "${SIM_SETUP}"
  source_setup "${NAV_SETUP}"

  check_wheel_dependencies

  local required_pkg
  for required_pkg in nav_bringup nav_components nav2_map_server localization_initializer small_point_lio pointcloud_obstacle_layer fake_vel_transform robots_msgs; do
    if ! ros2 pkg prefix "${required_pkg}" >/dev/null 2>&1; then
      error "navigation2026 安装空间不可用：找不到 ${required_pkg}。"
      error "请在 ${NAV_WS} 重新 colcon build --symlink-install，并重新运行本脚本。"
      exit 1
    fi
  done
  start_component \
    "Gazebo 与机器人仿真（${MODEL_MODE}）" \
    "${SIM_WS}" \
    "source '${SIM_SETUP}'; source '${NAV_SETUP}'" \
    "ros2 launch velocity_control run.launch.py model_mode:=${MODEL_MODE} wheel_model_path:=${WHEEL_MODEL_PATH} wheel_resource_path:=${WHEEL_RESOURCE_PATH} enable_motion_adapter:=true"
  local gazebo_pid="${COMPONENT_PIDS[-1]}"

  # Open the interactive tools immediately. They can wait for ROS topics while
  # the remainder of the simulation stack is still coming up.
  local nav_prefix
  nav_prefix="$(nav_setup_prefix)"
  local bringup_prefix
  bringup_prefix="$(ros2 pkg prefix nav_bringup)"
  local bringup_share="${bringup_prefix}/share/nav_bringup"
  local map_file="${MAP_FILE:-${bringup_share}/maps/RMUC_final.yaml}"
  local nav_params="${NAV_PARAMS:-${bringup_share}/config/nav_params_s.yaml}"
  local rviz_config="${SIM_WS}/config/navigation_sim.rviz"

  local rviz_pid=""
  if [[ "${USE_RVIZ}" == "true" ]]; then
    start_component "RViz" "${NAV_WS}" "source ${SIM_SETUP}; source ${NAV_SETUP}; ${nav_prefix}" "rviz2 -d ${rviz_config}"
    rviz_pid="${COMPONENT_PIDS[-1]}"
  fi

  # navigation_s.launch.py normally publishes /static_map from nav_server.
  # Before the operator chooses an initial pose, publish only the occupancy map
  # with a lightweight lifecycle map server; planning and control remain stopped.
  start_component \
    "RViz 初始位置地图预览" \
    "${NAV_WS}" \
    "source ${SIM_SETUP}; source ${NAV_SETUP}; ${nav_prefix}" \
    "ros2 run nav2_map_server map_server --ros-args -r __node:=initial_pose_map_server -r map:=/static_map -p yaml_filename:=${map_file} -p use_sim_time:=true"
  local preview_map_pid="${COMPONENT_PIDS[-1]}"
  wait_for_node "$(namespaced_node initial_pose_map_server)" 30 "${preview_map_pid}" "初始位置地图预览节点"
  activate_lifecycle_node "$(namespaced_node initial_pose_map_server)" "${preview_map_pid}"

  wait_for_message "/clock" 60 "${gazebo_pid}" "Gazebo 仿真时钟"
  wait_for_message "/livox/lidar" 90 "${gazebo_pid}" "仿真 Livox 点云"
  wait_for_message "/livox/imu" 90 "${gazebo_pid}" "仿真 IMU"

  # run.launch.py 的 wheel_sim_bridge 已通过 swerve_bridge.yaml 桥接 /joint_states。
  wait_for_message "/joint_states" 30 "${gazebo_pid}" "云台及舵轮关节状态"

  # Publish the single ROS-side LiDAR mounting transform before LIO starts.
  # The Gazebo sensor pose and this transform must use the same calibration.
  start_component \
    "Livox 静态外参（base_link→livox_frame）" \
    "${NAV_WS}" \
    "source ${SIM_SETUP}; source ${NAV_SETUP}; ${nav_prefix}" \
    "ros2 run tf2_ros static_transform_publisher --x 0.04 --y 0.110 --z 0.1 --roll -0.785 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id livox_frame --ros-args -p use_sim_time:=true"

  if [[ "${USE_KEYBOARD}" == "true" ]]; then
    start_untracked_component \
      "键盘控制（K/N 模式，WASD/JL/QE）" \
      "${SIM_WS}" \
      "source ${SIM_SETUP}; source ${NAV_SETUP}; ${nav_prefix}" \
      "ros2 run velocity_control keyboard_teleop_node --ros-args -p use_sim_time:=true"
  fi

  start_component \
    "Small Point-LIO（仿真话题，不启动 Livox 实车驱动）" \
    "${NAV_WS}" \
    "source ${SIM_SETUP}; source ${NAV_SETUP}; ${nav_prefix}" \
    "ros2 run small_point_lio small_point_lio_node --ros-args --params-file ${NAV_WS}/src/localization/small_point_lio/config/mid360_sim.yaml -p use_sim_time:=true -p lidar_frame:=livox_frame -r __node:=small_point_lio"
  local lio_pid="${COMPONENT_PIDS[-1]}"

  local loc_config="${NAV_WS}/src/localization/localization_initializer/config/initializer_params_s.yaml"
  start_component \
    "Navigation2026 localization_initializer NDT 定位" \
    "${NAV_WS}" \
    "source ${SIM_SETUP}; source ${NAV_SETUP}; ${nav_prefix}" \
    "ros2 run localization_initializer localization_initializer_node --ros-args --params-file ${loc_config} -p auto_initialize:=false -p use_sim_time:=true"
  local localization_pid="${COMPONENT_PIDS[-1]}"

  start_component \
    "gimbal_yaw_link 零变换兼容帧" \
    "${NAV_WS}" \
    "source ${SIM_SETUP}; source ${NAV_SETUP}; ${nav_prefix}" \
    "ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id gimbal_yaw_link"

  start_component \
    "点云障碍提取" \
    "${NAV_WS}" \
    "source ${SIM_SETUP}; source ${NAV_SETUP}; ${nav_prefix}" \
    "ros2 launch pointcloud_obstacle_layer local_obstacle_layer.launch.py use_sim_time:=true"

  wait_for_node "$(namespaced_node small_point_lio)" 90 "${lio_pid}" "Small Point-LIO"
  wait_for_initial_pose "${gazebo_pid}" "${rviz_pid}"

  info "等待 localization_initializer NDT 配准成功（/localization/status）……"
  if ! timeout 600 bash -lc \
      "ros2 topic echo /localization/status 2>/dev/null | grep -m1 '定位成功'"; then
    if ! process_is_running "${localization_pid}"; then
      error "localization_initializer 已退出，无法完成 NDT 定位。"
    else
      error "等待 NDT 定位成功超时（600 秒），请检查点云地图和配准日志。"
    fi
    return 1
  fi

  start_component \
    "仿真速度转换" \
    "${NAV_WS}" \
    "source ${SIM_SETUP}; source ${NAV_SETUP}; ${nav_prefix}" \
    "ros2 launch fake_vel_transform fake_vel_transform_s.launch.py use_sim_time:=true use_fake_vel:=true"
  local fake_vel_pid="${COMPONENT_PIDS[-1]}"

  start_terminal_component \
    "Navigation2026 导航服务器（规划与控制）" \
    "${NAV_WS}" \
    "source ${SIM_SETUP}; source ${NAV_SETUP}; ${nav_prefix}" \
    "ros2 launch nav_bringup navigation_s.launch.py params_file:=${nav_params} map_file:=${map_file} use_sim_time:=true"
  local controller_pid="${COMPONENT_PIDS[-1]}"

  wait_for_message "/Odometry/EC" 60 "${fake_vel_pid}" "仿真底盘速度反馈"

  wait_for_node "$(namespaced_node nav_server)" 90 "${controller_pid}" "Navigation2026 nav_server"

  info "定位与导航服务已就绪，切换到导航模式并启动自动云台旋转。"
  ros2 topic pub --once /control_mode std_msgs/msg/String "{data: navigation}" >/dev/null

  if [[ "${USE_DECISION}" == "true" ]]; then
    if ros2 pkg prefix sentry_decision >/dev/null 2>&1; then
      start_component "决策节点" "${NAV_WS}" "source ${SIM_SETUP}; source ${NAV_SETUP}; ${nav_prefix}" "ros2 run sentry_decision sentry_decision_node --ros-args -p use_sim_time:=true"
    else
      error "USE_DECISION=true，但找不到 sentry_decision，已跳过决策节点。"
    fi
  fi

  info "Gazebo、仿真底盘、LIO、Navigation2026、RViz/决策已启动；未启动 Livox 实车驱动或串口驱动。"
  printf '%s\n' "所有日志会继续显示在本终端；按 Ctrl+C 可一次性停止。"
  monitor_components
}

trap 'shutdown 130' INT TERM
trap 'shutdown $?' EXIT

main "$@"
