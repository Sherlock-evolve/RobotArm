"""
启动摄像头和 YOLOv8 目标检测的 launch 文件
包含：v4l2_camera 驱动 + YOLOv8 检测节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """生成包含摄像头和 YOLOv8 检测的启动描述"""
    
    # 获取包路径
    pkg_share = FindPackageShare('my_arm_vision').find('my_arm_vision')
    
    # 声明启动参数
    camera_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='摄像头设备路径'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='left_camera',
        description='相机名称（left_camera 或 right_camera）'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(pkg_share, 'model', 'yolov8n.pt'),
        description='YOLOv8 模型文件路径'
    )
    
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.5',
        description='检测置信度阈值 (0.0-1.0)'
    )
    
    # 1. v4l2_camera 摄像头驱动节点
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': [640, 480],  # 与 left_camera.yaml / right_camera.yaml 一致
            'output_encoding': 'rgb8',
            'camera_name': LaunchConfiguration('camera_name'),
            'camera_info_url': ['file://', PathJoinSubstitution([
                FindPackageShare('my_arm_vision'),
                'config',
                [LaunchConfiguration('camera_name'), '.yaml']
            ])],
            'frame_id': 'camera_link',
        }],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ]
    )
    
    # 2. YOLOv8 目标检测节点
    yolo_detector_node = Node(
        package='my_arm_vision',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'camera_frame': 'camera_link',
        }],
        output='screen'
    )
    
    return LaunchDescription([
        camera_device_arg,
        camera_name_arg,
        model_path_arg,
        conf_threshold_arg,
        camera_node,
        yolo_detector_node,
    ])


