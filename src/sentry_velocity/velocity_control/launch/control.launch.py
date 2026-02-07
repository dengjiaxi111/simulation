from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    ld = LaunchDescription()
    
    # 加载 joint_state_broadcaster
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    # 加载 velocity_controller
    load_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'velocity_controller'],
        output='screen'
    )
    
    # 先加载 joint_state_broadcaster
    ld.add_action(load_joint_state_broadcaster)
    
    # 等待 2 秒后加载 velocity_controller
    ld.add_action(TimerAction(
        period=2.0,
        actions=[ load_velocity_controller ]
    ))
    
    return ld
