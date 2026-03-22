"""
启动 WebSocket 服务器的 launch 文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='WebSocket 服务器监听地址'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8765',
        description='WebSocket 服务器端口'
    )
    
    video_topic_arg = DeclareLaunchArgument(
        'video_topic',
        default_value='/image_raw',
        description='视频话题名称'
    )
    
    servo_device_arg = DeclareLaunchArgument(
        'servo_device',
        default_value='/dev/servo_pulse',
        description='舵机设备路径'
    )
    
    video_fps_arg = DeclareLaunchArgument(
        'video_fps',
        default_value='30.0',
        description='视频帧率（FPS）'
    )
    
    # WebSocket 服务器节点
    # 使用 ExecuteProcess 直接调用可执行文件，避免 ROS2 自动添加 --ros-args 参数
    # 因为 websocket_server 使用标准 argparse，不支持 ROS2 参数
    # Python 包的可执行文件在 install/lib/package_name/executable_name（不是 package_name/package_name/executable_name）
    pkg_share = get_package_share_directory('my_arm_vision')
    # 路径应该是: install/lib/my_arm_vision/websocket_server
    executable_path = os.path.join(
        pkg_share.replace('share', 'lib'),
        'websocket_server'  # 直接是 executable_name，不需要中间的 package_name
    )
    
    websocket_server_node = ExecuteProcess(
        cmd=[
            executable_path,
            '--host', LaunchConfiguration('host'),
            '--port', LaunchConfiguration('port'),
            '--video-topic', LaunchConfiguration('video_topic'),
            '--servo-device', LaunchConfiguration('servo_device'),
            '--video-fps', LaunchConfiguration('video_fps'),
        ],
        output='screen',
        name='websocket_server',
    )
    
    return LaunchDescription([
        host_arg,
        port_arg,
        video_topic_arg,
        servo_device_arg,
        video_fps_arg,
        websocket_server_node,
    ])

