from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # 路径配置
    gazeboworld_path = "/home/lehan/sim_ws/src/sentry_velocity/gazeboworld"
    pkg_path = get_package_share_directory('velocity_control')
    robot_path = os.path.join(pkg_path, "urdf", "robot_modified.urdf")
    
    models_path = os.path.join(gazeboworld_path, "models")
    worlds_path = os.path.join(gazeboworld_path, "worlds")
    world_file = os.path.join(worlds_path, "rmuc_2026_world.sdf")
    
    # 设置 Gazebo 资源路径
    environment = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_path)
    
    # 启动 Gazebo (with -r to publish /clock for simulation time)
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file, '--verbose'],
        output='screen'
    )
    
    # 读取 URDF 文件
    with open(robot_path, 'r') as infp:
        robot_desc = infp.read()
    
    # Robot State Publisher - 发布 TF 变换
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )
    
    # 生成机器人
    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'sentry',
            '-file', robot_path,
            '-x', '4.5', '-y', '11', '-z', '0.7'
        ],
        output='screen'
    )
    
    # 桥接传感器话题
    bridge_sensors = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # bridge both the driver-facing topic and the raw simulator IMU topic
            '/livox/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/world/default/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
        ],
        output='screen',
        parameters=[{'use_sim_time': True, 'qos_overrides./clock.publisher.reliability': 'best_effort'}]
    )

    # pp2 -> livox converter (subscribe to bridged /lidar/points and publish /livox/lidar)
    pp2_to_livox = Node(
        package='velocity_control',
        executable='pp2_to_livox_node',
        name='pp2_to_livox',
        output='screen',
        parameters=[{'input_topic': '/lidar/points', 'use_sim_time': True}]
    )
    
    # 按顺序添加动作
    ld.add_action(environment)
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    
    # 等待 3 秒让 Gazebo 完全启动
    ld.add_action(TimerAction(
        period=3.0,
        actions=[spawn_robot]
    ))
    
    ld.add_action(bridge_sensors)
    # start the converter after the bridge is up
    ld.add_action(pp2_to_livox)
    
    return ld
