#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

"""

This launch file spawns a world and specified robots.
This node also launches all the necessary nodes needed for 
employing ROS2 nodes and implementing our solution

We have assumed the robot 1 to be a leader bot.
For a leader bot, the value of its leader_bot variable is zero(0).

"""

# TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

NUMBER_OF_BOTS = 4  # Initialise Total Number of bots you want to spawn.
RED_BOT_NO = 1      # The robot_number of the bot you want to be red colored. 

def generate_robot_list(number_of_robots):
    # This function creates a list of the robots you want to spawn (By default - Spawning in a line).
    robots = []

    for i in range(number_of_robots):
        robot_name = "v_"+str(i+1)  # Name of the robot (Ex. v_1, v_2, ...)
        x_pose = str(float(-i))     # Robot's spawning x-position
        robot = {'name': robot_name, 'x_pose': x_pose, 'y_pose': 0.0, 'z_pose': 0.01, 'yaw': 0.00}
        robots.append(robot)

    return robots 

def generate_launch_description(): 

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='turtlebot3_world')
    ros_gz_sim = get_package_share_directory('ros_ign_gazebo')
    
    ###### To be general for all robots ########
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',value=[
        os.path.join("/opt/ros/humble", "share"),
        ":" +
        os.path.join(get_package_share_directory('turtlebot3'), "models")])

    # Specify World to be Spawned.
    world_path = PathJoinSubstitution([get_package_share_directory('turtlebot3'), "models", "worlds", "empty_world.world"])
    
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'ign_gazebo.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world_path], 'on_exit_shutdown': 'true'}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'ign_gazebo.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    sim_time_declaration = DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock')

    world_name_declaration = DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name')

    # Spawn world
    ignition_spawn_world = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-file', world_path,
                   '-allow_renaming', 'false'],
        )
    
    ###### general things ended ########
    
    # Specify turtlebot3 model.sdf file to spawn the Robot
    robot_desc_path = os.path.join(
    get_package_share_directory('turtlebot3'),
    'models', 'turtlebot3', 'model_waffle.sdf')

    robot_desc = xacro.parse(open(robot_desc_path))
    xacro.process_doc(robot_desc)
    
    robot_red_desc_path = os.path.join(
    get_package_share_directory('turtlebot3'),
    'models', 'turtlebot3', 'model_waffle_red.sdf')

    robot_red_desc = xacro.parse(open(robot_red_desc_path))
    xacro.process_doc(robot_desc)
    
    # Names and poses of the robots
    robots = generate_robot_list(NUMBER_OF_BOTS)

    # We create the list of spawn robots commands
    ignition_spawn_entity_cmds = []
    robot_state_publisher_cmds = []

    # Creating List of arguments and remappings for ROS2 Bridge
    bridge_args = [
         # Clock (IGN -> ROS2)
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        # Joint states (IGN -> ROS2)
        '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',        
    ]
    bridge_remappings = [
        ("/odom/tf", "tf"),
    ]

    # Spawining Robots
    count = 1
    for robot in robots:
        name = robot['name']
        # Spawning red robot
        if count == RED_BOT_NO:
            # Spawn each robot
            ignition_spawn_entity_cmd = Node(
                package='ros_ign_gazebo',
                executable='create',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time }],
                arguments=['-entity', robot['name'],
                        '-name', robot['name'],
                        '-file', robot_red_desc_path,
                        '-sdf',
                        '-allow_renaming', 'true',
                        '-x', TextSubstitution(text=str(robot['x_pose'])),  # Initial x-position of robot
                        '-y', TextSubstitution(text=str(robot['y_pose'])),  # Initial y-position of robot
                        '-z', TextSubstitution(text=str(robot['z_pose'])),  # Initial z-position of robot
                        '-Y', TextSubstitution(text=str(robot['yaw'])),     # Initial yaw of robot
                        '-robot_namespace', robot['name']])
                        
            # Robot state publisher for each robot
            robot_state_publisher_cmd = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=robot['name'],
                output='screen',
                parameters=[{'frame_prefix': robot['name'] + '/',
                            'use_sim_time': use_sim_time,
                            'robot_description': robot_red_desc.toxml()}])
        else:
        # Spawning Default (White) robots
            # Spawn each robot
            ignition_spawn_entity_cmd = Node(
                package='ros_ign_gazebo',
                executable='create',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time }],
                arguments=['-entity', robot['name'],
                        '-name', robot['name'],
                        '-file', robot_desc_path,
                        '-sdf',
                        '-allow_renaming', 'true',
                        '-x', TextSubstitution(text=str(robot['x_pose'])),  # Initial x-position of robot
                        '-y', TextSubstitution(text=str(robot['y_pose'])),  # Initial y-position of robot
                        '-z', TextSubstitution(text=str(robot['z_pose'])),  # Initial z-position of robot
                        '-Y', TextSubstitution(text=str(robot['yaw'])),     # Initial yaw of robot
                        '-robot_namespace', robot['name']])
                        
            # Robot state publisher for each robot
            robot_state_publisher_cmd = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=robot['name'],
                output='screen',
                parameters=[{'frame_prefix': robot['name'] + '/',
                            'use_sim_time': use_sim_time,
                            'robot_description': robot_desc.toxml()}])

        # Bridge arguments for each robot
        bridge_args_robot = [
            # Velocity command (ROS2 -> IGN)
            f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # TF (IGN -> ROS2)
            f'/model/{name}/pose@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V',
            # Lidar (IGN -> ROS2)
            f'/world/empty/model/{name}/link/base_scan/sensor/gpu_lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            f'/world/empty/model/{name}/link/base_scan/sensor/gpu_lidar/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
        ]

        # Bridge remappings for each robot
        bridge_remappings_robot = [ 
            (f'/model/{name}/cmd_vel', f'/{name}/cmd_vel'),
            (f'/model/{name}/pose', f'/{name}/pose'),
            (f'/world/empty/model/{name}/link/base_scan/sensor/gpu_lidar/scan', f'/{name}/scan'),
            (f'/world/empty/model/{name}/link/base_scan/sensor/gpu_lidar/scan/points', f'/{name}/scan/points'),
        ]

        ignition_spawn_entity_cmds.append(ignition_spawn_entity_cmd)
        robot_state_publisher_cmds.append(robot_state_publisher_cmd)
        bridge_args.extend(bridge_args_robot)
        bridge_remappings.extend(bridge_remappings_robot)
        count += 1


    # Command Node for ROS2 bridge
    bridge_cmd = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=bridge_args,
        remappings=bridge_remappings,
        output='screen'
    )

    # Command Node for Static Transform Publisher 
    map_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='static_transform_publisher',
                        output='log',
                        parameters=[{'use_sim_time': use_sim_time}],
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'])


    # Command nodes for Platoon_info (platoon no., leader bot no.) handler for each bot
    platoon_handler_scripts = [] 
    count = 1
    for robot in robots:
        platoon_handler_script = Node(
            package='turtlebot3',
            executable='platoon_info_handler.py',
            arguments=[
                '--robot_no', TextSubstitution(text=str(count)),    # Specify Robot_number of the concerned bot
                '--platoon_no', TextSubstitution(text=str(1)),      # Specify initial platoon_number of the concerned bot      
                '--ros-args', '--'  # Ensures ROS 2 doesn't pass additional args
                       ],
            )
        count += 1
        platoon_handler_scripts.append(platoon_handler_script)

    # Command nodes for reference state publisher for each bot
    ref_state_scripts = [] 
    count = 1
    for robot in robots:
        ref_script = Node(
            package='turtlebot3',
            executable='ref_state.py',
            arguments=[
                '--robot_no', TextSubstitution(text=str(count)),    # Specify Robot_number of the concerned bot
                '--ros-args', '--'  # Ensures ROS 2 doesn't pass additional args
                       ],
            )
        count += 1
        ref_state_scripts.append(ref_script)

    # Command nodes for following the preceeding leader for each follower bot
    follower_scripts = [] 
    count = 1
    for robot in robots:
        if count != 1:      # Exempted for the first bot - Leader
            follow_script = Node(
                package='turtlebot3',
                executable='follower_move.py',
                arguments=[
                    '--robot_no', TextSubstitution(text=str(count)),        # Specify Robot_number of the concerned bot
                    '--leader_bot', TextSubstitution(text=str(count-1)),    # Specify initial leader bot for the concerned bot              
                    '--ros-args', '--'  # Ensures ROS 2 doesn't pass additional args
                        ],
                )
            follower_scripts.append(follow_script)
        count += 1

    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add actions to launch description
    ld.add_action(ign_resource_path)
    ld.add_action(sim_time_declaration)
    ld.add_action(world_name_declaration)
    ld.add_action(gzclient_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(ignition_spawn_world)
        
    for spawn_robot_cmd in ignition_spawn_entity_cmds:
        ld.add_action(spawn_robot_cmd)

    ld.add_action(bridge_cmd)
    ld.add_action(map_static_tf)
    
    for platoon_handler_script in platoon_handler_scripts:
        ld.add_action(platoon_handler_script)
    
    for ref_script in ref_state_scripts:
        ld.add_action(ref_script)
    
    for follow_script in follower_scripts:
        ld.add_action(follow_script)

    return ld