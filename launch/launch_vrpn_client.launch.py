# Copyright 2026 TDC
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the conditions of the BSD
# 3-Clause license are met.

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_USE_YAML = '__use_yaml__'


def _as_bool(value):
    normalized = value.strip().lower()
    if normalized in ('1', 'true', 'yes', 'on'):
        return True
    if normalized in ('0', 'false', 'no', 'off'):
        return False
    raise ValueError(f'Invalid boolean launch value: {value}')


def _as_trackers(value):
    return [tracker.strip() for tracker in value.split(',') if tracker.strip()]


def _launch_setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def use_yaml(name):
        return value(name) == _USE_YAML

    parameters_file = Path(value('parameters_file'))
    parameter_overrides = {}

    if not use_yaml('server'):
        parameter_overrides['server'] = value('server')
    if not use_yaml('port'):
        parameter_overrides['port'] = int(value('port'))
    if not use_yaml('update_frequency'):
        parameter_overrides['update_frequency'] = float(
            value('update_frequency'))
    if not use_yaml('refresh_tracker_frequency'):
        parameter_overrides['refresh_tracker_frequency'] = float(
            value('refresh_tracker_frequency'))
    if not use_yaml('frame_id'):
        parameter_overrides['frame_id'] = value('frame_id')
    if not use_yaml('use_server_time'):
        parameter_overrides['use_server_time'] = _as_bool(
            value('use_server_time'))
    if not use_yaml('broadcast_tf'):
        parameter_overrides['broadcast_tf'] = _as_bool(value('broadcast_tf'))
    if not use_yaml('process_sensor_id'):
        parameter_overrides['process_sensor_id'] = _as_bool(
            value('process_sensor_id'))
    if not use_yaml('trackers'):
        trackers = _as_trackers(value('trackers'))
        parameter_overrides['trackers'] = trackers if trackers else ['']
    if not use_yaml('topic_prefix'):
        parameter_overrides['topic_prefix'] = value('topic_prefix')
    if not use_yaml('pose_topic'):
        parameter_overrides['pose_topic'] = value('pose_topic')
    if not use_yaml('twist_topic'):
        parameter_overrides['twist_topic'] = value('twist_topic')
    if not use_yaml('accel_topic'):
        parameter_overrides['accel_topic'] = value('accel_topic')

    return [
        Node(
            package='vrpn_client_ros2',
            executable='vrpn_client_node',
            name=value('node_name'),
            namespace=value('node_namespace'),
            output='screen',
            parameters=[str(parameters_file), parameter_overrides],
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory('vrpn_client_ros2')
    default_parameters_file = str(Path(package_share, 'config', 'param.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'parameters_file',
            default_value=default_parameters_file,
            description='Path to a ROS 2 YAML parameter file.',
        ),
        DeclareLaunchArgument(
            'node_name',
            default_value='vrpn_client_node',
            description='Name of the VRPN client node.',
        ),
        DeclareLaunchArgument(
            'node_namespace',
            default_value='',
            description='Namespace for the VRPN client node and output topics.',
        ),
        DeclareLaunchArgument(
            'server',
            default_value=_USE_YAML,
            description='VRPN server host or IP address.',
        ),
        DeclareLaunchArgument(
            'port',
            default_value=_USE_YAML,
            description='VRPN server port.',
        ),
        DeclareLaunchArgument(
            'update_frequency',
            default_value=_USE_YAML,
            description='VRPN mainloop polling frequency in Hz.',
        ),
        DeclareLaunchArgument(
            'refresh_tracker_frequency',
            default_value=_USE_YAML,
            description='Tracker discovery frequency in Hz. Use 0 to disable.',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value=_USE_YAML,
            description='Frame id used for published messages and TF parents.',
        ),
        DeclareLaunchArgument(
            'use_server_time',
            default_value=_USE_YAML,
            description='Use VRPN server timestamps instead of ROS time.',
        ),
        DeclareLaunchArgument(
            'broadcast_tf',
            default_value=_USE_YAML,
            description='Publish TF transforms for tracker poses.',
        ),
        DeclareLaunchArgument(
            'process_sensor_id',
            default_value=_USE_YAML,
            description='Append VRPN sensor id to topics and TF child frames.',
        ),
        DeclareLaunchArgument(
            'trackers',
            default_value=_USE_YAML,
            description='Comma-separated tracker names. Empty uses discovery.',
        ),
        DeclareLaunchArgument(
            'topic_prefix',
            default_value=_USE_YAML,
            description='Optional relative prefix before tracker topic names.',
        ),
        DeclareLaunchArgument(
            'pose_topic',
            default_value=_USE_YAML,
            description='Pose topic basename under each tracker.',
        ),
        DeclareLaunchArgument(
            'twist_topic',
            default_value=_USE_YAML,
            description='Twist topic basename under each tracker.',
        ),
        DeclareLaunchArgument(
            'accel_topic',
            default_value=_USE_YAML,
            description='Accel topic basename under each tracker.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
