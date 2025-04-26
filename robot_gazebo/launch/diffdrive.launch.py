import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    warehouse_pkg_dir = get_package_share_directory('robot_gazebo')
    warehouse_launch_path = os.path.join(warehouse_pkg_dir, 'launch')
    rviz_config_path = os.path.join(warehouse_pkg_dir, 'rviz', 'tf.rviz')

    # Add Here
    mir_description_dir = get_package_share_directory('mir_description')
    mir_gazebo_dir = get_package_share_directory('mir_gazebo')


    warehouse_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([warehouse_launch_path, '/world_bringup.launch.py'])
    )

    # Add Here
    launch_mir_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_description_dir, 'launch', 'mir_launch_diff.py')
        )
    )

    launch_mir_gazebo_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mir_gazebo_dir, 'launch',
                         'include', 'mir_gazebo_common.py')
        )
    )

    launch_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        namespace='',
        output='screen',
        prefix='xterm -e')
    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mir_robot',
                '-topic', 'robot_description',
                '-b'],  # bond node to gazebo model,
        namespace='',
        output='screen')
    
      # Create the RViz2 node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', rviz_config_path,
            '--ros-args', '--log-level', 'error'
        ],
        parameters=[{'use_sim_time': True}],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
    )
    
    odom_node = Node(
        package='robot_controller',              # ชื่อแพ็กเกจที่บรรจุโค้ด odom_diffdrive_node.py
        executable='odom_diffdrive_node.py',             # ชื่อตัว console_script จาก setup.py
        name='diff_drive_odometry_node',
        output='screen',
    )
    
    groundtruth_odom_node = Node(
        package='robot_controller',              # ชื่อแพ็กเกจที่บรรจุโค้ด odom_diffdrive_node.py
        executable='frame_visualize.py',             # ชื่อตัว console_script จาก setup.py
        name='diff_drive_odometry_node',
        output='screen',
    )
    
    ld = LaunchDescription()

    # ld.add_action(launch_teleop)
    ld.add_action(warehouse_world_cmd)
    ld.add_action(launch_mir_description)
    ld.add_action(launch_mir_gazebo_common)
    ld.add_action(spawn_robot)
    # ld.add_action(joint_state_broadcaster_spawner)
    # ld.add_action(robot_controller_spawner)
    ld.add_action(rviz)
    # ld.add_action(odom_node)
    # ld.add_action(groundtruth_odom_node)

    return ld

