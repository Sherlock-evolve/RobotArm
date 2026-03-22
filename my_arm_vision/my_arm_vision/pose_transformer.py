#!/usr/bin/env python3
"""
坐标转换节点
将相机坐标系下的目标位姿转换为机械臂基坐标系下的位姿
使用 TF2 进行坐标变换
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs


class PoseTransformer(Node):
    """坐标转换节点：相机坐标系 -> 基坐标系"""
    
    def __init__(self):
        super().__init__('pose_transformer')
        
        # 参数
        self.target_frame = self.declare_parameter(
            'target_frame', 
            'world_frame'  # 目标坐标系（机械臂基坐标系）
        ).value
        
        self.source_frame = self.declare_parameter(
            'source_frame',
            'camera_link'  # 源坐标系（相机坐标系，与URDF中的camera_link一致）
        ).value
        
        # TF2 缓冲区和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 订阅相机坐标系下的位姿
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/detected_object_pose_camera',
            self.pose_callback,
            10
        )
        
        # 发布基坐标系下的位姿
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/detected_object_pose_base',
            10
        )
        
        self.get_logger().info('坐标转换节点已启动')
        self.get_logger().info(f'  源坐标系: {self.source_frame}')
        self.get_logger().info(f'  目标坐标系: {self.target_frame}')
    
    def pose_callback(self, msg):
        """处理接收到的位姿，进行坐标转换"""
        try:
            # 确保源坐标系匹配
            if msg.header.frame_id != self.source_frame:
                # 尝试使用消息中的 frame_id
                source_frame = msg.header.frame_id
            else:
                source_frame = self.source_frame
            
            # 查找坐标变换
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
            except TransformException as ex:
                self.get_logger().warn(
                    f'无法获取坐标变换 {source_frame} -> {self.target_frame}: {ex}'
                )
                return
            
            # 转换位姿
            transformed_pose = tf2_geometry_msgs.do_transform_pose(msg, transform)
            
            # 更新 header
            transformed_pose.header.stamp = self.get_clock().now().to_msg()
            transformed_pose.header.frame_id = self.target_frame
            
            # 发布转换后的位姿
            self.pose_pub.publish(transformed_pose)
            
            self.get_logger().info(
                f'坐标转换成功: '
                f'({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}) '
                f'[{source_frame}] -> '
                f'({transformed_pose.pose.position.x:.3f}, '
                f'{transformed_pose.pose.position.y:.3f}, '
                f'{transformed_pose.pose.position.z:.3f}) '
                f'[{self.target_frame}]'
            )
            
        except Exception as e:
            self.get_logger().error(f'坐标转换错误: {str(e)}', exc_info=True)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PoseTransformer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点启动失败: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

