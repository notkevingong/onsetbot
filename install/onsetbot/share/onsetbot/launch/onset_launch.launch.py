from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    echo_stm32_states_arg = DeclareLaunchArgument(
        'echo_stm32_states',
        default_value='true',
        description='Run ros2 topic echo /stm32_states in parallel.',
    )

    onset_gui = Node(
        package='onsetbot',
        executable='onset_gui',
        output='screen',
        emulate_tty=True,
    )

    actuator_commands = Node(
        package='onsetbot',
        executable='actuator_commands',
        output='screen',
        emulate_tty=True,
    )

    stm32_bridge = Node(
        package='onsetbot',
        executable='stm32_bridge',
        output='screen',
        emulate_tty=True,
    )

    odrive_can_bridge = Node(
        package='onsetbot',
        executable='odrive_can_bridge',
        output='screen',
        emulate_tty=True,
    )

    stm32_states_echo = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '/stm32_states'],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('echo_stm32_states')),
    )

    return LaunchDescription([
        echo_stm32_states_arg,
        onset_gui,
        actuator_commands,
        stm32_bridge,
        odrive_can_bridge,
        stm32_states_echo,
    ])
