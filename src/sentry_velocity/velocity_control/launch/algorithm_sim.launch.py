#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node, SetRemap


PROFILE_DIRECTORY = os.path.join(
    get_package_share_directory("velocity_control"), "config", "algorithm_profiles"
)


def _package_path(uri):
    if not isinstance(uri, str) or not uri.startswith("package://"):
        return uri
    package_and_path = uri[len("package://"):].split("/", 1)
    package = package_and_path[0]
    relative = package_and_path[1] if len(package_and_path) == 2 else ""
    return os.path.join(get_package_share_directory(package), relative)


def _resolve(value):
    if isinstance(value, dict):
        if set(value) == {"launch_argument"}:
            return LaunchConfiguration(value["launch_argument"])
        return {key: _resolve(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_resolve(item) for item in value]
    return _package_path(value)


def _make_action(spec):
    action_type = spec["type"]
    remappings = [tuple(pair) for pair in spec.get("remappings", [])]

    if action_type == "include":
        launch_file = _package_path(spec["launch"])
        action = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={
                key: str(value).lower() if isinstance(value, bool) else str(value)
                for key, value in spec.get("launch_arguments", {}).items()
            }.items(),
        )
        if remappings:
            action = GroupAction([
                *[SetRemap(src=source, dst=target) for source, target in remappings],
                action,
            ])
    elif action_type in ("node", "lifecycle_node"):
        node_class = LifecycleNode if action_type == "lifecycle_node" else Node
        parameters = [_resolve(item) for item in spec.get("parameters", [])]
        if (
            spec.get("package") == "nav2_map_server"
            and spec.get("executable") == "map_server"
        ):
            map_files = [
                parameter.get("yaml_filename")
                for parameter in parameters
                if isinstance(parameter, dict) and "yaml_filename" in parameter
            ]
            if not map_files or not isinstance(map_files[0], str):
                raise RuntimeError(
                    "navigationros2 map_server requires a yaml_filename parameter"
                )
            if not os.path.isfile(map_files[0]):
                raise RuntimeError(
                    f"navigationros2 map file does not exist: {map_files[0]}"
                )
        # Simulation time is enforced for every algorithm-side node.
        parameters.append({"use_sim_time": True})
        kwargs = {
            "package": spec["package"],
            "executable": spec["executable"],
            "name": spec.get("name"),
            "namespace": spec.get("namespace", ""),
            "output": "screen",
            "parameters": parameters,
            "remappings": remappings,
            "arguments": [_resolve(arg) for arg in spec.get("arguments", [])],
        }
        # Map/lifecycle nodes are external processes.  A transient DDS or
        # startup failure must not leave the rest of the algorithm running
        # forever without /map.  Profiles can opt into launch_ros' respawn
        # without changing the algorithm node itself.
        if spec.get("respawn", False):
            kwargs["respawn"] = True
            kwargs["respawn_delay"] = float(spec.get("respawn_delay", 2.0))
        action = node_class(**kwargs)
    else:
        raise RuntimeError(f"Unsupported profile action type: {action_type}")

    delay = float(spec.get("delay", 0.0))
    return TimerAction(period=delay, actions=[action]) if delay > 0.0 else action


def _launch_setup(context):
    profile_name = LaunchConfiguration("profile").perform(context)
    profile_path = os.path.join(PROFILE_DIRECTORY, f"{profile_name}.yaml")
    if not os.path.isfile(profile_path):
        available = ", ".join(
            sorted(os.path.splitext(name)[0] for name in os.listdir(PROFILE_DIRECTORY)
                   if name.endswith(".yaml"))
        )
        raise RuntimeError(
            f"Unknown algorithm profile '{profile_name}'. Available profiles: {available}"
        )

    with open(profile_path, "r", encoding="utf-8") as profile_file:
        profile = yaml.safe_load(profile_file) or {}

    required_packages = profile.get("required_packages", [])
    for package in required_packages:
        get_package_share_directory(package)

    actions = [SetEnvironmentVariable("ROS_STACK_SIZE", "16777216")]

    # LIO is intentionally allowed to start as soon as its static sensor
    # extrinsic is available.  Only Nav2 activation waits for the complete
    # dynamic chain produced afterwards:
    # odom -> base_link -> base_link_static -> base_link_fake.
    navigation_manager_action = None
    navigation_tf_gate = None
    for spec in profile.get("actions", []):
        if (
            spec.get("type") == "node"
            and spec.get("name") == "navigation_lifecycle_manager"
        ):
            navigation_manager_action = _make_action(spec)
            navigation_tf_gate = Node(
                package="velocity_control",
                executable="wait_for_tf.py",
                name="navigation_tf_gate",
                output="screen",
                parameters=[
                    {
                        "target_frame": "odom",
                        "source_frame": "base_link_fake",
                        "use_sim_time": True,
                    }
                ],
            )
            actions.append(navigation_tf_gate)
            continue
        actions.append(_make_action(spec))

    if navigation_manager_action is not None:
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=navigation_tf_gate,
                    on_exit=[navigation_manager_action],
                )
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "profile",
            default_value="navigationros2",
            description="YAML profile name from config/algorithm_profiles",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
