"""
WebSocket 服务器：控制舵机 + 传输视频流

功能：
1. 接收手机 app 的控制命令，控制舵机（写入 /dev/servo_pulse）
2. 从 ROS2 话题获取视频流并发送给手机 app
3. 支持双向实时通信

使用：
    ros2 run my_arm_vision websocket_server
"""

import asyncio
import json
import threading
import time
from typing import Optional

import websockets
from websockets.server import WebSocketServerProtocol
from websockets.exceptions import ConnectionClosed

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class VideoStreamSubscriber(Node):
    """订阅 ROS2 视频话题，提供最新帧"""
    
    def __init__(self, topic_name: str = '/image_raw'):
        super().__init__('websocket_video_subscriber')
        self.topic_name = topic_name
        self.bridge = CvBridge()
        self.latest_frame: Optional[bytes] = None
        self.latest_frame_lock = threading.Lock()
        
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10
        )
        
        self.get_logger().info(f'订阅视频话题: {topic_name}')
    
    def image_callback(self, msg: Image):
        """接收图像并编码为 JPEG"""
        try:
            # 转换为 OpenCV 图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 编码为 JPEG（质量 60，降低质量以提高传输速度）
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]
            _, buffer = cv2.imencode('.jpg', cv_image, encode_param)
            
            if buffer is not None:
                jpeg_data = buffer.tobytes()
                with self.latest_frame_lock:
                    self.latest_frame = jpeg_data
        except Exception as e:
            self.get_logger().warn(f'图像处理失败: {e}')
    
    def get_latest_frame(self) -> Optional[bytes]:
        """获取最新帧（线程安全）"""
        with self.latest_frame_lock:
            return self.latest_frame


class ServoController:
    """舵机控制器：写入 /dev/servo_pulse"""
    
    def __init__(self, device_path: str = '/dev/servo_pulse'):
        self.device_path = device_path
        self.write_lock = threading.Lock()
    
    def write_servo(self, servo_id: int, pulse_ns: int) -> tuple[bool, Optional[str]]:
        """
        写入舵机命令
        
        Args:
            servo_id: 舵机 ID (0-5)
            pulse_ns: 脉宽（纳秒，400000-2600000）
        
        Returns:
            (success, error_message)
        """
        # 验证参数
        if servo_id < 0 or servo_id >= 6:
            return False, f'Invalid servo_id: {servo_id} (must be 0-5)'
        
        if pulse_ns < 400000 or pulse_ns > 2600000:
            return False, f'Invalid pulse_ns: {pulse_ns} (must be 400000-2600000)'
        
        try:
            with self.write_lock:
                with open(self.device_path, 'w') as f:
                    f.write(f'{servo_id} {pulse_ns}\n')
                    f.flush()
            return True, None
        except PermissionError:
            return False, f'Permission denied: {self.device_path}'
        except FileNotFoundError:
            return False, f'Device not found: {self.device_path}'
        except Exception as e:
            return False, f'Write error: {str(e)}'
    
    def write_servos_batch(self, servos: list[dict]) -> list[dict]:
        """
        批量写入多个舵机命令
        
        Args:
            servos: [{"servo_id": 0, "pulse_ns": 1500000}, ...]
        
        Returns:
            [{"servo_id": 0, "success": True, "error": None}, ...]
        """
        import time
        results = []
        try:
            with self.write_lock:
                with open(self.device_path, 'w') as f:
                    for servo in servos:
                        servo_id = int(servo.get('servo_id', -1))
                        pulse_ns = int(servo.get('pulse_ns', 0))
                        
                        # 验证参数
                        if servo_id < 0 or servo_id >= 6:
                            results.append({
                                'servo_id': servo_id,
                                'success': False,
                                'error': f'Invalid servo_id: {servo_id}'
                            })
                            continue
                        
                        if pulse_ns < 400000 or pulse_ns > 2600000:
                            results.append({
                                'servo_id': servo_id,
                                'success': False,
                                'error': f'Invalid pulse_ns: {pulse_ns}'
                            })
                            continue
                        
                        # 写入命令（每个命令单独一行）
                        f.write(f'{servo_id} {pulse_ns}\n')
                        # 立即刷新，确保每个命令都被发送
                        f.flush()
                        # 添加小延迟，确保设备有时间处理每个命令（约 1ms）
                        time.sleep(0.001)
                        
                        results.append({
                            'servo_id': servo_id,
                            'pulse_ns': pulse_ns,
                            'success': True,
                            'error': None
                        })
            
            return results
        except Exception as e:
            return [{
                'servo_id': -1,
                'success': False,
                'error': f'Batch write error: {str(e)}'
            }]


class WebSocketServer:
    """WebSocket 服务器主类"""
    
    def __init__(
        self,
        host: str = '0.0.0.0',
        port: int = 8765,
        video_topic: str = '/image_raw',
        servo_device: str = '/dev/servo_pulse',
        video_fps: float = 30.0
    ):
        self.host = host
        self.port = port
        self.video_fps = video_fps
        self.frame_interval = 1.0 / video_fps
        
        # 初始化 ROS2（如果还没有初始化）
        if not rclpy.ok():
            rclpy.init()
        
        # 创建舵机控制器
        self.servo_controller = ServoController(servo_device)
        
        # 创建视频订阅者（在独立线程中运行 ROS2）
        self.video_subscriber = VideoStreamSubscriber(video_topic)
        self.ros_thread = threading.Thread(
            target=self._run_ros_spin,
            daemon=True
        )
        self.ros_thread.start()
        
        print(f'WebSocket 服务器初始化完成')
        print(f'  地址: ws://{host}:{port}')
        print(f'  视频话题: {video_topic}')
        print(f'  舵机设备: {servo_device}')
        print(f'  视频帧率: {video_fps} FPS')
    
    def _run_ros_spin(self):
        """在独立线程中运行 ROS2 spin"""
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self.video_subscriber)
        try:
            executor.spin()
        except Exception as e:
            print(f'ROS2 spin 错误: {e}')
    
    async def handle_client(self, websocket: WebSocketServerProtocol, path: str):
        """处理客户端连接"""
        client_addr = f'{websocket.remote_address[0]}:{websocket.remote_address[1]}'
        print(f'客户端连接: {client_addr}')
        
        # 启动视频流任务
        video_task = asyncio.create_task(
            self.send_video_stream(websocket)
        )
        
        try:
            async for message in websocket:
                try:
                    # 解析 JSON 消息
                    data = json.loads(message)
                    msg_type = data.get('type')
                    
                    if msg_type == 'servo_control':
                        # 控制单个舵机
                        servo_id = int(data.get('servo_id'))
                        pulse_ns = int(data.get('pulse_ns'))
                        
                        success, error = self.servo_controller.write_servo(servo_id, pulse_ns)
                        
                        response = {
                            'type': 'servo_response',
                            'success': success,
                            'servo_id': servo_id,
                            'pulse_ns': pulse_ns
                        }
                        if error:
                            response['error'] = error
                        
                        await websocket.send(json.dumps(response))
                    
                    elif msg_type == 'servos_batch':
                        # 批量控制多个舵机
                        servos = data.get('servos', [])
                        results = self.servo_controller.write_servos_batch(servos)
                        
                        response = {
                            'type': 'servos_batch_response',
                            'results': results
                        }
                        await websocket.send(json.dumps(response))
                    
                    elif msg_type == 'ping':
                        # 心跳检测
                        await websocket.send(json.dumps({
                            'type': 'pong',
                            'timestamp': time.time()
                        }))
                    
                    else:
                        # 未知消息类型
                        await websocket.send(json.dumps({
                            'type': 'error',
                            'error': f'Unknown message type: {msg_type}'
                        }))
                
                except json.JSONDecodeError:
                    await websocket.send(json.dumps({
                        'type': 'error',
                        'error': 'Invalid JSON format'
                    }))
                except Exception as e:
                    await websocket.send(json.dumps({
                        'type': 'error',
                        'error': f'Processing error: {str(e)}'
                    }))
        
        except ConnectionClosed:
            print(f'客户端断开连接: {client_addr}')
        except Exception as e:
            print(f'连接错误: {e}')
        finally:
            video_task.cancel()
            try:
                await video_task
            except asyncio.CancelledError:
                pass
    
    async def send_video_stream(self, websocket: WebSocketServerProtocol):
        """持续发送视频流"""
        print('开始发送视频流')
        last_send_time = time.time()
        
        try:
            while True:
                # 控制发送频率：确保按照设定的 FPS 发送
                current_time = time.time()
                elapsed = current_time - last_send_time
                
                if elapsed >= self.frame_interval:
                    # 获取最新帧
                    frame_data = self.video_subscriber.get_latest_frame()
                    
                    if frame_data is not None:
                        try:
                            # 发送二进制数据（JPEG 编码的图像）
                            await websocket.send(frame_data)
                            last_send_time = current_time
                        except Exception as e:
                            print(f'发送视频帧失败: {e}')
                            break
                    else:
                        # 如果没有新帧，等待一小段时间
                        await asyncio.sleep(0.05)
                else:
                    # 还没到发送时间，等待剩余时间
                    sleep_time = self.frame_interval - elapsed
                    await asyncio.sleep(min(sleep_time, 0.05))
        
        except asyncio.CancelledError:
            print('视频流任务取消')
        except Exception as e:
            print(f'视频流错误: {e}')
    
    async def run(self):
        """启动服务器"""
        # websockets 16.0+ 的 handler 只接收 websocket 参数
        # path 信息可以从 websocket.path 获取
        async def handler(websocket):
            path = getattr(websocket, 'path', '')
            await self.handle_client(websocket, path)
        
        async with websockets.serve(
            handler,
            self.host,
            self.port,
            ping_interval=20,
            ping_timeout=10
        ):
            print(f'WebSocket 服务器启动成功')
            print(f'等待客户端连接...')
            await asyncio.Future()  # 永久运行


def main(args=None):
    """主函数"""
    import argparse
    import sys
    
    # 创建解析器，允许未知参数（用于处理 ROS2 自动添加的参数）
    parser = argparse.ArgumentParser(
        description='WebSocket 服务器：控制舵机 + 传输视频流',
        allow_abbrev=False  # 禁用缩写，避免与 ROS2 参数冲突
    )
    parser.add_argument(
        '--host',
        type=str,
        default='0.0.0.0',
        help='服务器监听地址（默认: 0.0.0.0）'
    )
    parser.add_argument(
        '--port',
        type=int,
        default=8765,
        help='服务器端口（默认: 8765）'
    )
    parser.add_argument(
        '--video-topic',
        type=str,
        default='/image_raw',
        help='视频话题名称（默认: /image_raw）'
    )
    parser.add_argument(
        '--servo-device',
        type=str,
        default='/dev/servo_pulse',
        help='舵机设备路径（默认: /dev/servo_pulse）'
    )
    parser.add_argument(
        '--video-fps',
        type=float,
        default=30.0,
        help='视频帧率（默认: 30.0 FPS）'
    )
    
    # 处理 ROS2 参数（如果通过 launch 文件启动）
    # ROS2 会自动添加 --ros-args 等参数，使用 parse_known_args 忽略未知参数
    try:
        if args is None:
            # 过滤掉 ROS2 参数
            filtered_argv = []
            i = 0
            while i < len(sys.argv[1:]):
                arg = sys.argv[i + 1]
                if arg == '--ros-args':
                    # 跳过 --ros-args 及其后续参数，直到遇到下一个 -- 或结束
                    i += 1
                    while i < len(sys.argv[1:]) and sys.argv[i + 1] != '--':
                        if sys.argv[i + 1].startswith('__'):
                            i += 1
                            continue
                        if sys.argv[i + 1] in ['-r', '-p']:
                            i += 2
                            continue
                        i += 1
                    continue
                elif arg.startswith('__'):
                    i += 1
                    continue
                filtered_argv.append(arg)
                i += 1
            parsed_args, unknown = parser.parse_known_args(filtered_argv)
        else:
            parsed_args, unknown = parser.parse_known_args(args)
        
        # 如果有未知参数（ROS2 添加的），可以忽略或记录
        if unknown:
            print(f'忽略未知参数: {unknown}')
    except SystemExit:
        # argparse 在遇到未知参数时会调用 sys.exit，我们需要捕获它
        # 但实际上 parse_known_args 不应该退出
        raise
    
    # 创建并运行服务器
    server = WebSocketServer(
        host=parsed_args.host,
        port=parsed_args.port,
        video_topic=parsed_args.video_topic,
        servo_device=parsed_args.servo_device,
        video_fps=parsed_args.video_fps
    )
    
    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        print('\n服务器关闭')
    finally:
        if rclpy.ok():
            server.video_subscriber.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

