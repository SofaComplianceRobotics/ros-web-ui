from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

certfile = './back/server_cert.pem'
keyfile = './back/server_key.pem'

if certfile and keyfile:
    # Check if the certificate and key file exist
    ssl = True
    import os
    if not os.path.exists(certfile) or not os.path.exists(keyfile):
        print(f"Certificate file or keyfile do not exist.")
        ssl = False




def generate_launch_description():
    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument('url_path', default_value='/'),
        DeclareLaunchArgument('port', default_value='9090'),
        DeclareLaunchArgument('address', default_value=''),
        DeclareLaunchArgument('ssl', default_value='true' if ssl else 'false'),
        DeclareLaunchArgument('certfile', default_value=certfile),
        DeclareLaunchArgument('keyfile', default_value=keyfile),
        DeclareLaunchArgument('authenticate', default_value='false'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('retry_startup_delay', default_value='5.0'),
        DeclareLaunchArgument('fragment_timeout', default_value='600'),
        DeclareLaunchArgument('delay_between_messages', default_value='0'),
        DeclareLaunchArgument('max_message_size', default_value='10000000'),
        DeclareLaunchArgument('unregister_timeout', default_value='10.0'),
        DeclareLaunchArgument('use_compression', default_value='false'),
        DeclareLaunchArgument('call_services_in_new_thread', default_value='true'),
        DeclareLaunchArgument('default_call_service_timeout', default_value='5.0'),
        DeclareLaunchArgument('send_action_goals_in_new_thread', default_value='true'),
        DeclareLaunchArgument('topics_glob', default_value=''),
        DeclareLaunchArgument('services_glob', default_value=''),
        DeclareLaunchArgument('params_glob', default_value=''),
        DeclareLaunchArgument('params_timeout', default_value='5.0'),
        DeclareLaunchArgument('bson_only_mode', default_value='false'),
        DeclareLaunchArgument('respawn', default_value='false'),
        DeclareLaunchArgument('binary_encoder', default_value='default', condition=UnlessCondition(LaunchConfiguration('bson_only_mode'))),
    ]

    # SSL group
    ssl_group = GroupAction(
        actions=[
            Node(
                package='rosbridge_server',
                executable='rosbridge_websocket',
                name='rosbridge_websocket',
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                respawn=LaunchConfiguration('respawn'),
                parameters=[{
                    'certfile': LaunchConfiguration('certfile'),
                    'keyfile': LaunchConfiguration('keyfile'),
                    'port': LaunchConfiguration('port'),
                    'address': LaunchConfiguration('address'),
                    'url_path': LaunchConfiguration('url_path'),
                    'retry_startup_delay': LaunchConfiguration('retry_startup_delay'),
                    'fragment_timeout': LaunchConfiguration('fragment_timeout'),
                    'delay_between_messages': LaunchConfiguration('delay_between_messages'),
                    'max_message_size': LaunchConfiguration('max_message_size'),
                    'unregister_timeout': LaunchConfiguration('unregister_timeout'),
                    'use_compression': LaunchConfiguration('use_compression'),
                    'call_services_in_new_thread': LaunchConfiguration('call_services_in_new_thread'),
                    'default_call_service_timeout': LaunchConfiguration('default_call_service_timeout'),
                    'send_action_goals_in_new_thread': LaunchConfiguration('send_action_goals_in_new_thread'),
                    'topics_glob': LaunchConfiguration('topics_glob'),
                    'services_glob': LaunchConfiguration('services_glob'),
                    'params_glob': LaunchConfiguration('params_glob'),
                }],
                condition=IfCondition(LaunchConfiguration('ssl'))
            )
        ]
    )

    # Non-SSL group
    non_ssl_group = GroupAction(
        actions=[
            Node(
                package='rosbridge_server',
                executable='rosbridge_websocket',
                name='rosbridge_websocket',
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                respawn=LaunchConfiguration('respawn'),
                parameters=[{
                    'port': LaunchConfiguration('port'),
                    'address': LaunchConfiguration('address'),
                    'url_path': LaunchConfiguration('url_path'),
                    'retry_startup_delay': LaunchConfiguration('retry_startup_delay'),
                    'fragment_timeout': LaunchConfiguration('fragment_timeout'),
                    'delay_between_messages': LaunchConfiguration('delay_between_messages'),
                    'max_message_size': LaunchConfiguration('max_message_size'),
                    'unregister_timeout': LaunchConfiguration('unregister_timeout'),
                    'use_compression': LaunchConfiguration('use_compression'),
                    'call_services_in_new_thread': LaunchConfiguration('call_services_in_new_thread'),
                    'send_action_goals_in_new_thread': LaunchConfiguration('send_action_goals_in_new_thread'),
                    'topics_glob': LaunchConfiguration('topics_glob'),
                    'services_glob': LaunchConfiguration('services_glob'),
                    'params_glob': LaunchConfiguration('params_glob'),
                    'bson_only_mode': LaunchConfiguration('bson_only_mode'),
                }],
                condition=UnlessCondition(LaunchConfiguration('ssl'))
            )
        ]
    )

    # ROSAPI node
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        namespace=LaunchConfiguration('namespace'),
        respawn=LaunchConfiguration('respawn'),
        parameters=[{
            'topics_glob': LaunchConfiguration('topics_glob'),
            'services_glob': LaunchConfiguration('services_glob'),
            'params_glob': LaunchConfiguration('params_glob'),
            'params_timeout': LaunchConfiguration('params_timeout'),
        }]
    )

    return LaunchDescription(launch_args + [ssl_group, non_ssl_group, rosapi_node])
