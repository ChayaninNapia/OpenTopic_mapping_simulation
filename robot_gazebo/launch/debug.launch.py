import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Locate paths
    pkg_share = FindPackageShare(package='robot_gazebo').find('robot_gazebo')
    world_path = os.path.join(pkg_share, 'worlds', 'warehouse.world')

    # Custom plugin path (absolute path to your .so)
    custom_plugin_path = '/home/chayanin09/open_topic/install/gazebo_map_creator/lib'
    plugin_name = 'libgazebo_map_creator.so'

    # Model path
    gazebo_models_path = os.path.join(pkg_share, 'models')

    # Launch config
    declare_headless = DeclareLaunchArgument('headless', default_value='False')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    # Set environment variables
    set_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_models_path)
    set_plugin_path = SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', custom_plugin_path + ':${GAZEBO_PLUGIN_PATH}')

    # Start gzserver with your plugin manually
    start_gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            world_path,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '-s', plugin_name,  # Your custom plugin here
            '--verbose'
        ],
        output='screen'
    )

    # Optional: gzclient if GUI needed
    start_gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_headless)
    ld.add_action(declare_use_sim_time)
    ld.add_action(set_model_path)
    ld.add_action(set_plugin_path)
    ld.add_action(start_gzserver)
    ld.add_action(start_gzclient)

    return ld
