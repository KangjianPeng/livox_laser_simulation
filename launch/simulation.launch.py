import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('livox_laser_simulation')
    package_prefix = get_package_prefix('livox_laser_simulation')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    resource_path = os.path.dirname(package_share)

    world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(package_share, 'worlds', 'demo_room.sdf'),
        description='SDF world file',
    )
    model = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(
            package_share, 'models', 'mid360', 'model.sdf'),
        description='Livox sensor model file',
    )
    entity_name = DeclareLaunchArgument(
        'entity_name', default_value='livox_lidar',
        description='Name assigned to the spawned Gazebo model')
    use_rviz = DeclareLaunchArgument(
        'rviz', default_value='true', description='Start RViz2')
    headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run only the Gazebo server with headless rendering')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v 3 ', LaunchConfiguration('world')]}.items(),
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [
                '-r -s --headless-rendering -v 3 ', LaunchConfiguration('world')]
        }.items(),
        condition=IfCondition(LaunchConfiguration('headless')),
    )
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', LaunchConfiguration('model'),
            '-name', LaunchConfiguration('entity_name')],
        output='screen',
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(package_share, 'rviz', 'simulation.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH',
            os.path.join(package_prefix, 'lib')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        world,
        model,
        entity_name,
        use_rviz,
        headless,
        gazebo,
        gazebo_headless,
        spawn,
        bridge,
        rviz,
    ])
