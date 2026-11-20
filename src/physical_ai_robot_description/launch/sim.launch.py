from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('physical_ai_robot_description')
    world = LaunchConfiguration('world')
    default_world = os.path.join(package_share, 'worlds', 'corridor_speed_trial.world')
    xacro_file = LaunchConfiguration('model_xacro')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    physics_args = {
        'payload_mass': '0.0', 'payload_x': '0.0', 'payload_y': '0.0',
        'payload_height': '0.20', 'computer_mass': '0.0',
        'wheel_mu': '0.8', 'wheel_mu_lateral': '0.8', 'wheel_torque': '3.2',
        'laser_z': '0.3095', 'odometry_source': '1',
        'caster_mu': '0.6',
    }
    # Command tokenizes the completed string: preserve spaces in workspace paths.
    command = ['xacro "', xacro_file, '"']
    for name in physics_args:
        command.extend([' ', name, ':=', LaunchConfiguration(name)])
    robot_description = ParameterValue(
        Command(command),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world, 'gui': gui}.items(),
    )

    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in physics_args.items()],
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('model_xacro', default_value=os.path.join(package_share, 'urdf', 'cad_v11_robot.xacro')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(package_share, 'config', 'nav2_mdh100.yaml')),
        gazebo,
        Node(
            package='physical_ai_robot_description',
            executable='sim_motion_guard.py',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'physical_ai_robot', '-x', '0.0', '-y', '0.0', '-z', '0.01', '-timeout', '90'],
            output='screen',
        ),
    ])
