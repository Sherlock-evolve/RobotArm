import json
from typing import List

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


class YoloDetectorNode(Node):
  """
  简单的 YOLOv8 目标检测节点。

  订阅:
    - /image_raw (sensor_msgs/Image)
  发布:
    - /yolo_detections (std_msgs/String，JSON 格式的检测结果)
    - /yolo_detections_image (sensor_msgs/Image，带检测框的可视化图像)

  参数:
    - model_path (string): YOLOv8 .pt 模型路径
    - conf_threshold (float): 置信度阈值
    - camera_frame (string): 相机坐标系名称，仅用于日志
  """

  def __init__(self):
    super().__init__("yolo_detector")

    # 声明参数
    pkg_share = get_package_share_directory("my_arm_vision")
    default_model = f"{pkg_share}/model/yolov8n.pt"

    self.declare_parameter("model_path", default_model)
    self.declare_parameter("conf_threshold", 0.5)
    self.declare_parameter("camera_frame", "camera_link")

    self.model_path: str = (
      self.get_parameter("model_path").get_parameter_value().string_value
    )
    self.conf_threshold: float = (
      self.get_parameter("conf_threshold").get_parameter_value().double_value
    )
    self.camera_frame: str = (
      self.get_parameter("camera_frame").get_parameter_value().string_value
    )

    self.get_logger().info(f"Loading YOLOv8 model from: {self.model_path}")
    try:
      from ultralytics import YOLO  # type: ignore
    except Exception as exc:  # pragma: no cover - 运行时依赖
      self.get_logger().error(
        "导入 ultralytics 失败，请先在 ROS2 环境中安装:\n"
        "  pip install ultralytics\n"
        f"详细错误: {exc}"
      )
      raise

    try:
      self.model = YOLO(self.model_path)
    except Exception as exc:
      self.get_logger().error(
        f"加载 YOLO 模型失败: {self.model_path}\n错误: {exc}"
      )
      raise

    self.bridge = CvBridge()
    self.detections_pub = self.create_publisher(String, "yolo_detections", 10)
    self.image_pub = self.create_publisher(Image, "yolo_detections_image", 10)

    self.image_sub = self.create_subscription(
      Image, "image_raw", self.image_callback, 10
    )

    # 用于调试的计数器
    self.frame_count = 0

    self.get_logger().info(
      f"YOLOv8 detector ready. model={self.model_path}, "
      f"conf_threshold={self.conf_threshold:.2f}, "
      f"camera_frame={self.camera_frame}"
    )
    self.get_logger().info(f"订阅话题: image_raw ")
    self.get_logger().info("等待图像数据...")
    
    # 添加定时器，定期检查是否收到数据
    self.last_frame_time = None
    self.check_timer = self.create_timer(5.0, self._check_data_reception)

  # ------------------------------------------------------------------------------
  # Callbacks
  # ------------------------------------------------------------------------------
  def image_callback(self, msg: Image) -> None:
    """接收图像并运行 YOLOv8 推理。"""
    self.frame_count += 1
    self.last_frame_time = self.get_clock().now()
    
    # 每10帧打印一次日志，避免日志过多
    if self.frame_count % 10 == 1:
      self.get_logger().info(f"收到图像 #{self.frame_count}, 尺寸: {msg.width}x{msg.height}")

    try:
      cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as exc:
      self.get_logger().warn(f"图像转换失败: {exc}")
      return

    try:
      # 运行推理
      results = self.model(
        cv_image, verbose=False, conf=self.conf_threshold
      )
    except Exception as exc:
      self.get_logger().error(f"YOLO 推理失败: {exc}")
      import traceback
      self.get_logger().error(f"详细错误: {traceback.format_exc()}")
      return

    detections = self._results_to_detections(results)
    
    if self.frame_count % 10 == 1:
      self.get_logger().info(f"检测到 {len(detections)} 个目标")

    # 在图像上绘制检测框
    annotated_image = self._draw_detections(cv_image.copy(), detections)

    # 发布带检测框的图像
    try:
      image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
      image_msg.header = msg.header
      self.image_pub.publish(image_msg)
      if self.frame_count % 10 == 1:
        self.get_logger().debug("已发布检测图像")
    except Exception as exc:
      self.get_logger().error(f"发布检测图像失败: {exc}")
      import traceback
      self.get_logger().error(f"详细错误: {traceback.format_exc()}")

    # 发布 JSON 结果
    try:
      msg_out = String()
      msg_out.data = json.dumps(
        {
          "header": {
            "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
            "frame_id": msg.header.frame_id or self.camera_frame,
          },
          "detections": detections,
        },
        ensure_ascii=False,
      )
      self.detections_pub.publish(msg_out)
      if self.frame_count % 10 == 1:
        self.get_logger().debug("已发布 JSON 检测结果")
    except Exception as exc:
      self.get_logger().error(f"发布 JSON 结果失败: {exc}")
      import traceback
      self.get_logger().error(f"详细错误: {traceback.format_exc()}")

  # ------------------------------------------------------------------------------
  # Helpers
  # ------------------------------------------------------------------------------
  def _draw_detections(self, image: np.ndarray, detections: List[dict]) -> np.ndarray:
    """在图像上绘制检测框和标签。"""
    for det in detections:
      bbox = det["bbox"]
      x_min = int(bbox["x_min"])
      y_min = int(bbox["y_min"])
      x_max = int(bbox["x_max"])
      y_max = int(bbox["y_max"])
      conf = det["confidence"]
      cls_name = det["class_name"]
      cls_id = det["class_id"]

      # 根据类别 ID 生成颜色（使用 HSV 色彩空间确保颜色区分度）
      hue = int((cls_id * 137.508) % 180)  # 使用黄金角度确保颜色分布均匀
      color = cv2.cvtColor(
        np.uint8([[[hue, 255, 255]]]), cv2.COLOR_HSV2BGR
      )[0][0].tolist()
      color = tuple(map(int, color))

      # 绘制检测框
      cv2.rectangle(image, (x_min, y_min), (x_max, y_max), color, 2)

      # 准备标签文本
      label = f"{cls_name} {conf:.2f}"
      
      # 计算文本大小
      font = cv2.FONT_HERSHEY_SIMPLEX
      font_scale = 0.6
      thickness = 2
      (text_width, text_height), baseline = cv2.getTextSize(
        label, font, font_scale, thickness
      )

      # 绘制标签背景
      label_y = max(y_min, text_height + 5)
      cv2.rectangle(
        image,
        (x_min, label_y - text_height - 5),
        (x_min + text_width, label_y + baseline),
        color,
        -1,
      )

      # 绘制标签文本
      cv2.putText(
        image,
        label,
        (x_min, label_y),
        font,
        font_scale,
        (255, 255, 255),
        thickness,
        cv2.LINE_AA,
      )

    return image

  def _results_to_detections(self, results) -> List[dict]:
    """将 ultralytics 结果转换为简单的 JSON 结构。"""
    detections: List[dict] = []
    if not results:
      return detections

    res = results[0]
    if not hasattr(res, "boxes") or res.boxes is None:
      return detections

    names = self.model.names if hasattr(self.model, "names") else {}

    for box in res.boxes:
      try:
        xyxy = box.xyxy[0].tolist()  # [x1, y1, x2, y2]
        conf = float(box.conf[0])
        cls_id = int(box.cls[0])
        cls_name = names.get(cls_id, str(cls_id))
      except Exception:
        continue

      detections.append(
        {
          "bbox": {
            "x_min": xyxy[0],
            "y_min": xyxy[1],
            "x_max": xyxy[2],
            "y_max": xyxy[3],
          },
          "confidence": conf,
          "class_id": cls_id,
          "class_name": cls_name,
        }
      )

    return detections

  def _check_data_reception(self) -> None:
    """定期检查是否收到图像数据。"""
    if self.frame_count == 0:
      self.get_logger().warn(
        "⚠️  尚未收到任何图像数据！\n"
      )
    elif self.last_frame_time is not None:
      time_since_last = (self.get_clock().now() - self.last_frame_time).nanoseconds / 1e9
      if time_since_last > 3.0:
        self.get_logger().warn(
          f"⚠️  已 {time_since_last:.1f} 秒未收到新图像（共处理 {self.frame_count} 帧）"
        )


def main(args=None) -> None:
  rclpy.init(args=args)
  node = YoloDetectorNode()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()


