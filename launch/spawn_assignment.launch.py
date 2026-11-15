import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_erl1= get_package_share_directory('erl1_assignment1')

    world_path = os.path.join(pkg_erl1, 'worlds', 'assignment.sdf')
    pkg_models = os.path.join(pkg_erl1, 'models')
    gazebo_models_path = os.path.expanduser('~/gazebo_models')    
    
    gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    paths_to_add = [pkg_models, gazebo_models_path]
    for p in paths_to_add:
        if p not in gz_path:
            gz_path = p + (os.pathsep + gz_path if gz_path else "")
    os.environ["GZ_SIM_RESOURCE_PATH"] = gz_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='assignment.sdf',
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='my_robot.urdf',
        description='Name of the URDF description to load'
    )

    # Path to the robot description
    urdf_file_path = PathJoinSubstitution([
        pkg_erl1,
        "urdf",
        LaunchConfiguration('model')
    ])

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_erl1, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
        'world': LaunchConfiguration('world'),
        }.items()
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_erl1, 'rviz', 'urdf.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # Spawn the URDF model using the `/world/<world_name>/create` service
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "my_robot",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.5", "-Y", "0.0"  # Initial spawn position
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]),
             'use_sim_time': True},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_sim_time': True}],
    )

    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True,
             'camera.image.compressed.jpeg_quality': 75},
        ],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_erl1, 'config', 'ekf.yaml'),
            {'use_sim_time': True},
             ]
    )



    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            #"/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            #"/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
           # "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
           # "/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
           # "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
           # "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",

        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
    )



    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    #launchDescriptionObject.add_action(joint_state_publisher_gui_node)
    launchDescriptionObject.add_action(gz_bridge_node)
    launchDescriptionObject.add_action(gz_image_bridge_node)
    launchDescriptionObject.add_action(ekf_node)

    return launchDescriptionObject
