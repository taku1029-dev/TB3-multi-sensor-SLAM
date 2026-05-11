from ast import arguments
from sys import executable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    custom_robot_description_path = FindPackageShare('custom_robot_description')
    urdf_file = PathJoinSubstitution([custom_robot_description_path, 'urdf', 'custom_robot.urdf'])
    gz_gui_conf_path = PathJoinSubstitution([custom_robot_description_path, 'gazebo', 'gui.config'])
    sdf_file = PathJoinSubstitution([custom_robot_description_path, 'worlds', 'example_world.sdf'])
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])
    # Phase 1 EKF + gate share one yaml; ROS 2 dispatches by node name (ADR-006:38, ADR-014).
    ekf_phase1_config = PathJoinSubstitution([
        FindPackageShare('rl_navigation_pkg'), 'config', 'ekf_phase1.yaml',
    ])
    # slam_toolbox is phase-invariant per ADR-007:34.
    slam_toolbox_config = PathJoinSubstitution([
        FindPackageShare('rl_navigation_pkg'), 'config', 'slam_toolbox_params.yaml',
    ])
    # Nav2 plugin selection pinned by ADR-013 (NavfnPlanner + RegulatedPurePursuitController).
    nav2_config = PathJoinSubstitution([
        FindPackageShare('rl_navigation_pkg'), 'config', 'nav2_params.yaml',
    ])

    # SAC-6 (2026-05-12): slam_toolbox is intentionally NOT launched. Phase 1
    # training runs in a fixed arena (worlds/example_world.sdf) with static
    # obstacles, so dynamic SLAM contributes no signal that Nav2 cannot get
    # straight from /lidar via its obstacle_layer. Dropping slam_toolbox also
    # sidesteps the empirical "second lifecycle cycle fails" problem documented
    # in SAC-5. The required `map → odom` TF is supplied by a static identity
    # transform below — since map ≡ odom in this configuration, the EKF-based
    # reward (ADR-004:39-40, on /odom) is geometrically the same as the
    # slam-corrected pose would have been. The slam_toolbox_config variable is
    # left defined above for when SAC training graduates back to dynamic SLAM
    # (e.g. real-robot deployment); it is currently unused.
    _ = slam_toolbox_config  # silence unused-variable linters

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
                    ' --gui-config ',
                    gz_gui_conf_path
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
                '-file', urdf_file,
                '-name', 'custom_robot',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.0',
                '-P', '0.0',
                '-R', '0.0',
                '-Y', '0.0',
            ],
        ),

        # Bridging and remapping Gazebo topics to ROS 2.
        # See ADR-005 for the trained-system topic contract and ADR-003 for why /lidar uses LaserScan.
        # /model/custom_robot/pose is the sim-only ground-truth model pose published by the
        # PosePublisher plugin in the xacro; remapped to /ground_truth_pose for ADR-005 reward.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # /clock must come first — every node with use_sim_time:True (EKF, gate, agent)
                # blocks until /clock is published, which causes silent dropouts in
                # ekf_filter_node sensor_timeout otherwise.
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/camera/rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/rgbd/points@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/odom_wheel@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/model/custom_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/model/custom_robot/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            ],
            remappings=[
                ('/model/custom_robot/pose', '/ground_truth_pose'),
            ],
            output='screen'
        ),

        # robot_state_publisher: reads the URDF and publishes the static TF chain
        # (base_footprint -> base_link -> imu_link / base_scan / camera_link / wheel_*).
        # EKF needs imu_link -> base_footprint to transform IMU messages into base_link_frame;
        # without this, ekf_filter_node silently drops every input.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['cat ', urdf_file]),
                'use_sim_time': True,
            }],
            output='screen',
        ),

        # EKF input gate (ADR-011, ADR-014): buffers /odom_wheel and /imu latest-wins,
        # applies σ_wheel / σ_imu (set_parameters target), republishes onto
        # /ekf_in/odom_wheel and /ekf_in/imu only on /ekf_input_gate/release service call.
        Node(
            package='rl_navigation_pkg',
            executable='ekf_input_gate',
            name='ekf_input_gate',
            parameters=[ekf_phase1_config, {'use_sim_time': True}],
            output='screen',
        ),

        # Steady 10 Hz release clocker (interim — replaced by RLNavigation-v1's
        # step() once it lands). Without a deterministic clocker, /odom is bursty
        # and downstream Message Filters drop scans for "timestamp earlier than
        # all TF cache data".
        Node(
            package='rl_navigation_pkg',
            executable='release_driver',
            name='release_driver',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # robot_localization EKF (ADR-006 Phase 1: wheel + IMU). Subscribes to the gate's
        # private topics, publishes filtered pose on /odom and odom->base_footprint TF
        # (publish_tf:true is mandatory per ADR-007:37 for slam_toolbox's motion prior).
        # ekf_node's default output topic is /odometry/filtered; remap to /odom per
        # ADR-002:52 so /odom is EKF-owned post-cutover.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_phase1_config, {'use_sim_time': True}],
            remappings=[
                ('odometry/filtered', '/odom'),
            ],
            output='screen',
        ),

        # SAC-6 (2026-05-12): static map → odom identity TF replaces slam_toolbox.
        # `map ≡ odom` in this training configuration. Nav2 plans in the `map`
        # frame, but with no slam correction the map/odom drift is zero by
        # construction; Nav2 ends up planning against the EKF's view of the
        # world. The local + global costmaps fill themselves from /lidar via
        # ObstacleLayer (nav2_params.yaml's global_costmap is rolling-window +
        # obstacle-only, no StaticLayer), so a /map publisher is not needed.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            output='screen',
        ),

        # Nav2 stack (ADR-013): NavfnPlanner + RegulatedPurePursuitController + stock BT.
        # Nodes individually launched (not via nav2_bringup) so we can omit waypoint_follower /
        # velocity_smoother etc. that are not in scope here. lifecycle_manager_navigation
        # auto-configures + auto-activates the listed nodes via its `autostart:true` param.
        # Nav2 owns /cmd_vel (ADR-002:42); reads corrected pose from TF (map->odom->base_footprint),
        # not /odom directly. Global costmap consumes /map via nav2_costmap_2d's StaticLayer.
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[nav2_config, {'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_config, {'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[nav2_config, {'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav2_config, {'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }],
            output='screen',
        ),

    ])
