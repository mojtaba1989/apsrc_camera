import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('apsrc_camera')
    yaml_path = os.path.join(pkg_share, 'config', 'camera_config.yaml')

    yaml_enb = LaunchConfiguration('yaml_enb').perform(context).lower() == 'true'

    # Always pass camera_name and dir
    override_params = {
        'camera_name': LaunchConfiguration('camera_name').perform(context),
        'dir': LaunchConfiguration('dir').perform(context),
    }

    # Only override YAML values if explicitly provided on the CLI
    int_args = ['device_id', 'width', 'height', 'fps']
    for arg in int_args:
        val = LaunchConfiguration(arg).perform(context)
        if val != '':
            override_params[arg] = int(val)

    rotate_val = LaunchConfiguration('rotate').perform(context)
    if rotate_val != '':
        override_params['rotate'] = rotate_val.lower() == 'true'

    node_params = []
    if yaml_enb:
        node_params.append(yaml_path)
    node_params.append(override_params)

    webcam_node = Node(
        package='apsrc_camera',
        executable='webcam_node',
        name=LaunchConfiguration('node_name'),
        output='screen',
        parameters=node_params,
    )

    return [webcam_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='C920'),
        DeclareLaunchArgument('node_name', default_value=LaunchConfiguration('camera_name')),
        DeclareLaunchArgument('device_id', default_value='0'),
        DeclareLaunchArgument('width', default_value=''),
        DeclareLaunchArgument('height', default_value=''),
        DeclareLaunchArgument('fps', default_value=''),
        DeclareLaunchArgument('rotate', default_value=''),
        DeclareLaunchArgument('yaml_enb', default_value='true'),
        DeclareLaunchArgument(
            'dir',
            default_value=EnvironmentVariable('pwd', default_value='/tmp')
        ),
        OpaqueFunction(function=launch_setup),
    ])