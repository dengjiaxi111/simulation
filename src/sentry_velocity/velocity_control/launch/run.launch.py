from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import os
import tempfile
import math
import xml.etree.ElementTree as ET


WHEEL_MODEL_DEFAULT = (
    "/home/dengjiaxi/simulation_seu/navigationsim/src/"
    "seu_sentry_description/resource/xmacro/seu_sentry_sim.sdf.xmacro"
)
WHEEL_RESOURCE_DEFAULT = (
    "/home/dengjiaxi/simulation_seu/navigationsim/src/"
    "seu_sentry_description/resource/models"
)
WHEEL_SPAWN_Z_DEFAULT = "1.000"


def _existing_paths(paths):
    return [path for path in paths if path and os.path.exists(path)]


def _path_value(paths):
    return os.pathsep.join(_existing_paths(paths))


def _prepend_env(name, paths):
    value = _path_value(paths + [os.environ.get(name, "")])
    if value:
        os.environ[name] = value
    return SetEnvironmentVariable(name=name, value=value)


def _optional_share_path(package_name, *parts):
    try:
        return os.path.join(get_package_share_directory(package_name), *parts)
    except PackageNotFoundError:
        return None


def _write_temp_robot(robot_desc, suffix):
    robot_file = tempfile.NamedTemporaryFile(
        mode="w",
        prefix="sentry_robot_",
        suffix=suffix,
        delete=False,
    )
    robot_file.write(robot_desc)
    robot_file.close()
    return robot_file.name


def _make_robot_state_publisher(robot_desc):
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
    )


def _patch_wheel_sdf(robot_xml):
    root = ET.fromstring(robot_xml)
    model = root.find("model")
    if model is None:
        return robot_xml

    for sensor in model.findall(".//sensor"):
        sensor_name = sensor.get("name", "")
        topic = sensor.find("topic")
        if topic is None:
            topic = ET.SubElement(sensor, "topic")
        if sensor.get("type") == "imu" and "mid360_imu" in sensor_name:
            topic.text = "/livox/imu"
        elif sensor.get("type") == "gpu_lidar" and "mid360_lidar" in sensor_name:
            topic.text = "/lidar"
            visualize = sensor.find("visualize")
            if visualize is None:
                visualize = ET.SubElement(sensor, "visualize")
            visualize.text = "false"

    # The sentry meshes are STL files and therefore do not carry the Gazebo
    # material information used by the PB2025 DAE assets.  Supply explicit SDF
    # materials so Gazebo does not render every link with its default white
    # shader.  Keep any material already authored in a source asset untouched.
    for link in model.findall(".//link"):
        link_name = link.get("name", "")
        if "wheel" in link_name:
            rgba = (0.035, 0.040, 0.050, 1.0)
        elif "steer" in link_name:
            rgba = (0.48, 0.52, 0.58, 1.0)
        elif link_name == "base_link":
            rgba = (0.08, 0.32, 0.78, 1.0)
        elif link_name == "chassis":
            rgba = (0.16, 0.22, 0.30, 1.0)
        else:
            continue
        for visual in link.findall("visual"):
            if visual.find("material") is not None:
                continue
            material = ET.SubElement(visual, "material")
            ambient = ET.SubElement(material, "ambient")
            ambient.text = "%.3f %.3f %.3f 1" % tuple(value * 0.55 for value in rgba[:3])
            diffuse = ET.SubElement(material, "diffuse")
            diffuse.text = "%.3f %.3f %.3f 1" % rgba[:3]
            specular = ET.SubElement(material, "specular")
            specular.text = "0.55 0.58 0.62 1"

    for plugin in model.findall("plugin"):
        if plugin.get("filename") == "gz-sim-joint-controller-system":
            joint_name = plugin.find("joint_name")
            if joint_name is not None and joint_name.text == "gimbal_yaw_joint":
                topic = plugin.find("topic")
                if topic is None:
                    topic = ET.SubElement(plugin, "topic")
                topic.text = "/model/sentry/joint/gimbal_yaw_joint/cmd_vel"
        if plugin.get("filename") != "MecanumDrive2":
            continue
        plugin.set("filename", "gz-sim-mecanum-drive-system")
        plugin.set("name", "gz::sim::systems::MecanumDrive")
        for child in list(plugin):
            plugin.remove(child)
        params = {
            "front_left_joint": "front_left_wheel_joint",
            "front_right_joint": "front_right_wheel_joint",
            "back_left_joint": "rear_left_wheel_joint",
            "back_right_joint": "rear_right_wheel_joint",
            "wheel_separation": "0.50",
            "wheelbase": "0.45",
            "wheel_radius": "0.075",
            "min_acceleration": "-5",
            "max_acceleration": "5",
            "topic": "/cmd_vel_chassis",
            "odom_topic": "/wheel/odometry",
            "frame_id": "odom",
            "child_frame_id": "chassis",
            "odom_publish_frequency": "50",
        }
        for key, value in params.items():
            element = ET.SubElement(plugin, key)
            element.text = value

    return ET.tostring(root, encoding="unicode")


def _reroot_wheel_urdf(robot_xml):
    root = ET.fromstring(robot_xml)
    yaw_joint = root.find("./joint[@name='gimbal_yaw_joint']")

    def xyz(element, tag, default=(0.0, 0.0, 0.0)):
        node = element.find(tag) if element is not None else None
        values = [float(v) for v in (node.get("xyz", "") if node is not None else "").split()]
        return tuple(values) if len(values) == 3 else default

    def rpy(element, tag, default=(0.0, 0.0, 0.0)):
        node = element.find(tag) if element is not None else None
        values = [float(v) for v in (node.get("rpy", "") if node is not None else "").split()]
        return tuple(values) if len(values) == 3 else default

    def mat_from_rpy(roll, pitch, yaw):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
                (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
                (-sp, cp * sr, cp * cr))

    def inverse_origin(joint):
        t = xyz(joint, "origin")
        roll, pitch, yaw = rpy(joint, "origin")
        R = mat_from_rpy(roll, pitch, yaw)
        inv_t = tuple(-sum(R[k][i] * t[k] for k in range(3)) for i in range(3))
        # URDF's fixed-axis RPY extraction for the inverse rotation.
        inv_roll = math.atan2(R[1][2], R[2][2])
        inv_pitch = math.asin(max(-1.0, min(1.0, -R[0][2])))
        inv_yaw = math.atan2(R[0][1], R[0][0])
        return inv_t, (inv_roll, inv_pitch, inv_yaw)

    inverse_t, inverse_rpy = inverse_origin(yaw_joint)

    remove_joints = {
        "base_to_chassis",
        "gimbal_yaw_odom_joint",
        "gimbal_pitch_odom_joint",
        "gimbal_yaw_joint",
    }
    # Sensors are published by Navigation2026's single base_link->livox_frame
    # extrinsic TF. Do not let the RViz URDF publish a second Livox branch.
    remove_links = {"gimbal_yaw_odom", "gimbal_pitch_odom", "gimbal_yaw", "livox_frame"}
    remove_links.update(
        element.get("name") for element in root.findall("./link")
        if any(token in element.get("name", "").lower() for token in ("livox", "lidar", "imu"))
    )
    for element in list(root):
        if (element.tag == "joint" and element.get("name") in remove_joints) or (
            element.tag == "link" and element.get("name") in remove_links
        ):
            root.remove(element)

    # Remove joints attached to links removed above (the sensor branch).
    for joint in list(root.findall("./joint")):
        parent = joint.find("parent")
        child = joint.find("child")
        if ((parent is not None and parent.get("link") in remove_links) or
                (child is not None and child.get("link") in remove_links)):
            root.remove(joint)

    yaw_joint = ET.SubElement(root, "joint", name="gimbal_yaw_joint", type="continuous")
    ET.SubElement(yaw_joint, "origin",
                  xyz="%.9g %.9g %.9g" % inverse_t,
                  rpy="%.9g %.9g %.9g" % inverse_rpy)
    ET.SubElement(yaw_joint, "parent", link="base_link")
    ET.SubElement(yaw_joint, "child", link="chassis")
    ET.SubElement(yaw_joint, "axis", xyz="0 0 -1")
    return ET.tostring(root, encoding="unicode")


def _load_leg_robot(pkg_path, controller_config_path):
    robot_path = os.path.join(pkg_path, "urdf", "robot_modified.urdf")
    with open(robot_path, "r") as infp:
        return infp.read().replace(
            "__VELOCITY_CONTROLLER_CONFIG__",
            controller_config_path,
        )


def _load_wheel_robot(robot_xmacro_path):
    from xmacro.xmacro4sdf import XMLMacro4sdf

    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(robot_xmacro_path)
    try:
        xmacro.generate()
    except Exception as exc:
        raise RuntimeError(
            "Failed to generate wheel robot SDF. Make sure pb2025_robot_description "
            "dependencies are installed, especially rmoss_gz_resources from "
            "pb2025_robot_description/dependencies.repos, and that its resource/models "
            "directory is available through the ROS/Gazebo resource path."
        ) from exc
    robot_sdf_xml = _patch_wheel_sdf(xmacro.to_string())

    try:
        from sdformat_tools.urdf_generator import UrdfGenerator
    except ImportError:
        return robot_sdf_xml, None

    urdf_generator = UrdfGenerator()
    urdf_generator.parse_from_sdf_string(robot_sdf_xml)
    return robot_sdf_xml, _reroot_wheel_urdf(urdf_generator.to_string())


def launch_setup(context):
    pkg_path = get_package_share_directory("velocity_control")
    gazeboworld_path = os.path.join(pkg_path, "gazeboworld")
    controller_config_path = os.path.join(pkg_path, "config", "velocity_controller.yaml")

    models_path = os.path.join(gazeboworld_path, "models")
    worlds_path = os.path.join(gazeboworld_path, "worlds")
    world_file = os.path.join(worlds_path, "rmuc_2026_world.sdf")

    model_mode = LaunchConfiguration("model_mode").perform(context)
    wheel_model_path = LaunchConfiguration("wheel_model_path").perform(context)
    wheel_resource_path = LaunchConfiguration("wheel_resource_path").perform(context)
    wheel_spawn_z = LaunchConfiguration("wheel_spawn_z").perform(context)
    enable_motion_adapter = LaunchConfiguration("enable_motion_adapter").perform(context)

    ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
    gz_vendor_plugin_path = os.path.join(
        "/opt/ros", ros_distro, "opt", "gz_sim_vendor", "lib", "gz-sim-8", "plugins"
    )

    resource_paths = [
        models_path,
        wheel_resource_path if model_mode == "wheel" else None,
        _optional_share_path("pb2025_robot_description", "resource", "models"),
        _optional_share_path("rmoss_gz_resources", "resource", "models"),
    ]
    system_plugin_paths = [
        os.path.join("/opt/ros", ros_distro, "lib"),
        gz_vendor_plugin_path,
    ]

    environment = _prepend_env("GZ_SIM_RESOURCE_PATH", resource_paths)
    ignition_resource_environment = _prepend_env("IGN_GAZEBO_RESOURCE_PATH", resource_paths)
    sdf_path_environment = _prepend_env("SDF_PATH", resource_paths)
    file_path_environment = _prepend_env("IGN_FILE_PATH", resource_paths)
    gz_system_plugin_path = _prepend_env("GZ_SIM_SYSTEM_PLUGIN_PATH", system_plugin_paths)

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file, "--verbose"],
        output="screen",
    )

    actions = [
        environment,
        ignition_resource_environment,
        sdf_path_environment,
        file_path_environment,
        gz_system_plugin_path,
        gazebo,
    ]

    if model_mode == "leg":
        robot_desc = _load_leg_robot(pkg_path, controller_config_path)
        robot_file = _write_temp_robot(robot_desc, ".urdf")
        robot_state_publisher = _make_robot_state_publisher(robot_desc)
        spawn_robot = ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-name", "sentry",
                "-file", robot_file,
                "-x", "4.5", "-y", "11", "-z", "0.7",
            ],
            output="screen",
        )

        load_joint_state_broadcaster = ExecuteProcess(
            cmd=[
                "ros2", "run", "controller_manager", "spawner",
                "joint_state_broadcaster",
                "-c", "/controller_manager",
                "--controller-manager-timeout", "60",
            ],
            output="screen",
        )

        load_velocity_controller = ExecuteProcess(
            cmd=[
                "ros2", "run", "controller_manager", "spawner",
                "velocity_controller",
                "-c", "/controller_manager",
                "--controller-manager-timeout", "60",
            ],
            output="screen",
        )

        actions += [
            robot_state_publisher,
            TimerAction(period=3.0, actions=[spawn_robot]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[TimerAction(period=2.0, actions=[load_joint_state_broadcaster])],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_velocity_controller],
                )
            ),
        ]
    elif model_mode == "wheel":
        swerve_config_path = os.path.join(
            get_package_share_directory("seu_sentry_sim_control"),
            "config",
            "swerve_sim_controller.yaml",
        )
        swerve_bridge_path = os.path.join(
            get_package_share_directory("seu_sentry_sim_control"),
            "config",
            "swerve_bridge.yaml",
        )
        robot_sdf, robot_urdf = _load_wheel_robot(wheel_model_path)
        robot_file = _write_temp_robot(robot_sdf, ".sdf")
        robot_state_publisher = (
            _make_robot_state_publisher(robot_urdf) if robot_urdf is not None else None
        )
        spawn_robot = ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-name", "sentry",
                "-file", robot_file,
                "-x", "4.5", "-y", "11", "-z", wheel_spawn_z,
            ],
            output="screen",
        )
        bridge_cmd_vel = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="wheel_sim_bridge",
            output="screen",
            parameters=[{
                "config_file": swerve_bridge_path,
                "use_sim_time": True,
            }],
        )
        wheel_sim_adapter = Node(
            package="velocity_control",
            executable="wheel_sim_adapter_node",
            name="wheel_sim_adapter",
            output="screen",
            parameters=[{"use_sim_time": True, "gimbal_spin_speed": 3.14}],
        )
        swerve_sim_controller = Node(
            package="seu_sentry_sim_control",
            executable="swerve_sim_controller_node",
            name="swerve_sim_controller",
            output="screen",
            parameters=[swerve_config_path, {"use_sim_time": True}],
        )
        if robot_state_publisher is not None:
            actions.append(robot_state_publisher)
        actions += [
            TimerAction(period=3.0, actions=[spawn_robot]),
            bridge_cmd_vel,
            swerve_sim_controller,
        ]
        if enable_motion_adapter.lower() == "true":
            actions.append(wheel_sim_adapter)
    else:
        raise RuntimeError(f"Unsupported model_mode: {model_mode}")

    bridge_sensors = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="sensor_bridge",
        arguments=[
            "/livox/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    clock_bridge = ExecuteProcess(
        cmd=[
            "ros2", "launch", "ros_gz_bridge", "clock_bridge.launch",
            "bridge_name:=clock_bridge",
        ],
        output="screen",
    )

    pp2_to_livox = Node(
        package="velocity_control",
        executable="pp2_to_livox_node",
        name="pp2_to_livox",
        output="screen",
        parameters=[{
            "input_topic": "/lidar/points",
            "output_topics": ["/livox/lidar"],
            "frame_id": "livox_frame",
            "use_sim_time": True,
        }],
    )

    actions += [bridge_sensors, clock_bridge, pp2_to_livox]
    return actions


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        "model_mode",
        default_value="leg",
        choices=["leg", "wheel"],
        description="Robot model mode: leg uses the original URDF; wheel uses the SEU four-module swerve SDF.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "wheel_model_path",
        default_value=WHEEL_MODEL_DEFAULT,
        description="Path to the wheel robot SDF xmacro file.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "wheel_resource_path",
        default_value=WHEEL_RESOURCE_DEFAULT,
        description="Path to the wheel robot Gazebo model resources.",
    ))
    ld.add_action(DeclareLaunchArgument(
        "wheel_spawn_z", default_value=WHEEL_SPAWN_Z_DEFAULT,
        description="Wheel chassis-origin spawn height (m).",
    ))
    ld.add_action(DeclareLaunchArgument(
        "enable_motion_adapter", default_value="true", choices=["true", "false"],
        description="Start wheel/gimbal motion adapter immediately.",
    ))
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
