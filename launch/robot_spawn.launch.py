import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # ----------------------------------------------------------------------
    # 🚀 [WSL2 맞춤형 GPU 가속 & GUI 충돌 방지 설정]
    # ----------------------------------------------------------------------
    os.environ['MESA_GL_VERSION_OVERRIDE'] = '4.6'
    os.environ['MESA_GLSL_VERSION_OVERRIDE'] = '460'
    os.environ['QT_QPA_PLATFORM'] = 'xcb'
    bad_envs = [
        '__NV_PRIME_RENDER_OFFLOAD',
        '__GLX_VENDOR_LIBRARY_NAME',
        'LIBGL_ALWAYS_SOFTWARE',
        'GALLIUM_DRIVER',
        'QT_X11_NO_MITSHM'
    ]
    for key in bad_envs:
        if key in os.environ:
            del os.environ[key]
    # ----------------------------------------------------------------------

    package_name = 'project2'
    pkg_share = get_package_share_directory(package_name)
    xacro_file = os.path.join(pkg_share, 'urdf', 'ros_dd.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'ros_dd.world')
    robot_description_raw = Command(['xacro ', xacro_file])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-x', '0.52',
            '-y', '-4.30',
            '-z', '0.05',
            '-Y', '1.5708'
        ],
        output='screen'
    )

    # /scan 토픽 브릿지 (Gazebo → ROS2)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        bridge,
    ])
