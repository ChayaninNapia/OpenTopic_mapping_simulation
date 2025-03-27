import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Package directories
    warehouse_pkg_dir = get_package_share_directory('robot_gazebo')
    warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch')
    rviz_config_path = os.path.join(warehouse_pkg_dir, 'rviz', 'robot.rviz')
    mir_description_dir = get_package_share_directory('mir_description')
    mir_gazebo_dir = get_package_share_directory('mir_gazebo')

    # World path
    world_path = os.path.join(warehouse_pkg_dir, 'worlds', 'warehouse.world')

    # Plugin and model paths
    custom_plugin_path = '/home/chayanin09/open_topic/install/gazebo_map_creator/lib'
    gazebo_models_path = os.path.join(warehouse_pkg_dir, 'models')

    # Env setup
    set_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_models_path)
    set_plugin_path = SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', custom_plugin_path + ':${GAZEBO_PLUGIN_PATH}')

    # Gzserver with system plugin
    start_gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            world_path,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_map_creator.so',
            '--verbose'
        ],
        output='screen'
    )

    # Gzclient (optional)
    start_gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Other robot-related launch processes
    launch_mir_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mir_description_dir, 'launch', 'mir_launch.py'))
    )

    launch_mir_gazebo_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mir_gazebo_dir, 'launch', 'include', 'mir_gazebo_common.py'))
    )

    launch_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mir_robot', '-topic', 'robot_description', '-b'],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(set_model_path)
    ld.add_action(set_plugin_path)
    ld.add_action(start_gzserver)
    ld.add_action(start_gzclient)
    ld.add_action(launch_teleop)
    ld.add_action(launch_mir_description)
    ld.add_action(launch_mir_gazebo_common)
    ld.add_action(spawn_robot)
    ld.add_action(rviz)

    return ld
