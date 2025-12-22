import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    robot_name = 'new_robot'
    package_name = 'mec'

    # ================== Load URDF/Xacro ==================
    model_file = os.path.join(
        get_package_share_directory(package_name), 'model', 'all_robot.urdf.xacro'
    )
    robot_description_config = xacro.process_file(model_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # ================== Load Controllers YAML ==================
    controller_yaml_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'driver.yaml'
    )
    with open(controller_yaml_file, 'r') as f:
        controller_params = yaml.safe_load(f)

    # ================== Gazebo World ==================
    world_file = os.path.join(
        get_package_share_directory(package_name), 'worlds', 'new.world'
    )
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'gui': 'true'}.items()
    )

    # ================== Robot State Publisher ==================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # ================== Spawn Entity ==================
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=['-topic', 'robot_description', '-entity', robot_name],
        output='screen'
    )

    # ================== Controller Manager ==================
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        parameters=[robot_description, {'use_sim_time': True}, controller_params],
        output='screen'
    )

    # ================== Spawners with Timer ==================
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            name='joint_state_broadcaster_spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        )]
    )

    wheel_controller_spawners = []
    for wheel in ['front_left_wheel_controller', 'front_right_wheel_controller',
                  'rear_left_wheel_controller', 'rear_right_wheel_controller']:
        wheel_controller_spawners.append(
            TimerAction(
                period=4.0,
                actions=[Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[wheel],
                    output='screen'
                )]
            )
        )

    # ================== Driver Node ==================
    driver_node = Node(
        package=package_name,
        executable='driver_node.py',
        name='mecanum_driver',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ================== Odometry Node ==================
    odom_node = Node(
        package=package_name,
        executable='odom_node.py',   # اسم الملف اللي في scripts/
        name='mecanum_odometry',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    # ================== Mecanum Motion (cmd_vel smoothing) ==================
    mecanum_motion_node = Node(
        package=package_name,          # mec
        executable='mecanum_motion.py',   # الاسم من setup.py
        name='mecanum_motion',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ================== RViz ==================
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), 'rviz', 'new.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # ================== Launch Description ==================
    ld = LaunchDescription()
    ld.add_action(gazebo_launch)
    ld.add_action(controller_manager_node)
    ld.add_action(joint_state_broadcaster_spawner)
    for spawner in wheel_controller_spawners:
        ld.add_action(spawner)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(driver_node)
    ld.add_action(odom_node)
    ld.add_action(mecanum_motion_node) 
    ld.add_action(rviz_node)

    return ld

