import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    ld = LaunchDescription()

    #Change use_sim_time param to False for REAL FLIGHT TESTS!!!
    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    static_tf2_odom_to_base = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            parameters=[
                {'use_sim_time': True}
                ],
        )
    
    static_tf2_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.47', '0.03', '0.275', '0', '0', '0', 'base_link', 'nav2_standard_vtol_0/lidar_link/lidar_depth'],
        parameters=[
            {'use_sim_time': True}
        ],
    )

    static_tf2_lidar_to_lidar_sensor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'nav2_standard_vtol_0/lidar_link/lidar_depth'],
        parameters=[
            {'use_sim_time': True}
        ],
    )


    tf2_map_to_odom = Node(
        package='dontscrewthepooch',
        executable='tf2_map_to_odom',
        parameters=[
                {'use_sim_time': True}
                ],
    )

    goal_pose_to_traj_setpoint = Node(
        package='dontscrewthepooch',
        executable='goal_pose_to_traj_setpoint',
        parameters=[
                {'use_sim_time': True}
                ],
    )

    #CORE NAV2 ELEMENTS

    bringup_dir = get_package_share_directory('nav2_bringup')
    dontscrewthepooch_dir = get_package_share_directory("dontscrewthepooch")
    params_dir = os.path.join(dontscrewthepooch_dir, "config")
    python_dir = os.path.join(dontscrewthepooch_dir, "dontscrewthepooch")
    nav2_params = os.path.join(params_dir, "nav2_params_no_map.yaml")
    goal_python = os.path.join(python_dir, "command_give.py")
    execute_python = ExecuteProcess(
        cmd=['python3', goal_python],
        output='screen'
    )
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    #NAV2 END

    ld.add_action(static_tf2_odom_to_base)
    ld.add_action(static_tf2_base_to_lidar)
    # ld.add_action(static_tf2_lidar_to_lidar_sensor)

    ld.add_action(tf2_map_to_odom)
    ld.add_action(navigation2_cmd)
    ld.add_action(execute_python)
    #ld.add_action(goal_pose_to_traj_setpoint)

    return ld