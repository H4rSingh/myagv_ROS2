import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = True

    gazebo_launch_path = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("myagv"), "config", "ekf.yaml"]
    )

    robot_base = os.getenv('myagv_BASE')
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("myagv_description"), "urdf/robots", f"{robot_base}.urdf.xacro"]
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("myagv_gazebo"), "worlds", "turtlebot3_world.sdf"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('myagv_description'), 'launch', 'description.launch.py']
    )

    gz_bridge_config_path = PathJoinSubstitution(
        [FindPackageShare("myagv"), "config", "gz_bridge.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='gui',
            default_value='true',
            description='Enable Gazebo Client'
        ),

        DeclareLaunchArgument(
            name='urdf',
            default_value=urdf_path,
            description='URDF path'
        ),

        DeclareLaunchArgument(
            name='odom_topic',
            default_value='/odom',
            description='EKF out odometry topic'
        ),

        DeclareLaunchArgument(
            name='world',
            default_value=world_path,
            description='Gazebo world'
        ),

        DeclareLaunchArgument(
            name='spawn_x',
            default_value='0.5',
            description='Robot spawn position in X axis'
        ),

        DeclareLaunchArgument(
            name='spawn_y',
            default_value='0.0',
            description='Robot spawn position in Y axis'
        ),

        DeclareLaunchArgument(
            name='spawn_z',
            default_value='0.0',
            description='Robot spawn position in Z axis'
        ),

        DeclareLaunchArgument(
            name='spawn_yaw',
            default_value='0.0',
            description='Robot spawn heading'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={'gz_args': [' -r -s ', LaunchConfiguration('world')]}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            condition=IfCondition(LaunchConfiguration('gui')),
            launch_arguments={'gz_args': [' -g']}.items()
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'myagv',
                '-x', LaunchConfiguration('spawn_x'),
                '-y', LaunchConfiguration('spawn_y'),
                '-z', LaunchConfiguration('spawn_z'),
                '-Y', LaunchConfiguration('spawn_yaw'),
            ]
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output='screen',
            parameters=[{'config_file': gz_bridge_config_path}],
        ),

        Node(
            package='myagv_gazebo',
            executable='command_timeout',
            name='command_timeout'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, ekf_config_path],
            remappings=[("odometry/filtered", LaunchConfiguration("odom_topic"))]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'false',
                'urdf': LaunchConfiguration('urdf')
            }.items()
        )
    ])