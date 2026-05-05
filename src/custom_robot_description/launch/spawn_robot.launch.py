from ast import arguments
from sys import executable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, AppendEnvironmentVariable 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    custom_robot_description_path = FindPackageShare('custom_robot_description')
    urdf_file = PathJoinSubstitution([custom_robot_description_path, 'urdf', 'custom_robot.urdf'])
    sdf_file = PathJoinSubstitution([custom_robot_description_path, 'gazebo', 'example_world.sdf'])
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])

    # with open(urdf_file, 'r') as file_handler:
    #     robot_desc = file_handler.read()
    
    return LaunchDescription([
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([custom_robot_description_path, '..'])
        ),
        
        # Include and execute another launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [
                    PathJoinSubstitution([custom_robot_description_path, 'worlds', 'example_world.sdf']),  # Replace with your own world file
                    ' -r',
                    ' --gui-config /home/taku_ros/ros2_ws/src/custom_robot_description/config/default.config'
                ],
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        # Spawn a robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                # '-world', 'example_world',
                '-file', '/home/taku_ros/ros2_ws/src/custom_robot_description/urdf/custom_robot.urdf',
                '-name', 'custom_robot',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.0',
                '-P', '0.0',
                '-R', '0.0',
                '-Y', '0.0',
            ],
        ),

        # Bridging and remapping Gazebo topics to ROS 2 (replace with your own topics)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera/rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/rgbd/points@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
            ],
            output='screen'
        ),

    ])
