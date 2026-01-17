import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'uav_landing_sim'
    pkg_share = get_package_share_directory(pkg_name)
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # World
    world_file = os.path.join(pkg_share, 'worlds', 'landing_zone.sdf')
    
    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r -s -v 4 ' + world_file}.items(),
    )

    # Spawn Landing Pad
    spawn_pad = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'landing_pad',
            '-file', os.path.join(pkg_share, 'models', 'landing_pad', 'model.sdf'),
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )

    # Spawn UAV
    spawn_uav = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'vtol_uav',
            '-file', os.path.join(pkg_share, 'models', 'vtol_uav', 'model.sdf'),
            '-x', '5', '-y', '5', '-z', '10' # Start offset to demonstrate landing
        ],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Camera
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            # Cmd Vel (Twist)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Odometry
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        output='screen'
    )

    # Nodes
    vision_node = Node(
        package=pkg_name,
        executable='vision_processor',
        name='vision_processor',
        output='screen'
    )

    control_node = Node(
        package=pkg_name,
        executable='flight_controller',
        name='flight_controller',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_pad,
        spawn_uav,
        bridge,
        vision_node,
        control_node
    ])
