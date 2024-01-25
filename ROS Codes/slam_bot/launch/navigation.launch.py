import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from nav2_common.launch import RewrittenYaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('slam_bot')
    params_file = LaunchConfiguration('params_file')

    # slam_toolbox_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('slam_toolbox'), 'launch'), '/online_async_launch.py']),
    #     launch_arguments=[('params_file', params_file), ('map_subscribe_transient_local','false')])

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('nav2_bringup'), 'launch'), '/navigation_launch.py']),
        launch_arguments=[('params_file', params_file)])

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch'), '/display.launch.py']))    
    
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')
    
    param_substitutions = {
        'use_sim_time': use_sim_time,
        # 'default_bt_xml_filename': default_bt_xml_filename,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local}
    
    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)
    
    slam_toolbox_node = launch_ros.actions.Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='async_slam_toolbox_node',
        parameters=[configured_params]
    )
    
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Full path to the Nav2 parameters file'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'map_subscribe_transient_local', default_value='false',
            description='Whether to set the map subscriber QoS to transient local'),
                
        slam_toolbox_node,
        # slam_toolbox_launch,
        nav2_launch,
        display_launch,
    ])