#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from yolo_msg.msg import BoundingBox, BoundingBoxArray

from ultralytics import YOLO
import cv2
import numpy as np
from cv_bridge import CvBridge
import time

CLASS_COLOR = {
    "yellow": (0, 255, 255),
    "blue": (255, 0, 0)
}


class YoloSegNode(Node):
    def __init__(self):
        super().__init__('yolo_seg_node')
        self.bridge = CvBridge()

        # YOLOv8 모델 로드
        self.model = YOLO('/home/acca/acca_ws/src/ACCA_2025/src/yolo/models/best.pt')
        self.labels = self.model.names

        # ROS2 통신
        self.image_sub = self.create_subscription(Image, 'concated_cam', self.image_callback, 10)
        self.yellow_pub = self.create_publisher(PointStamped, 'yellow_cone', 10)
        self.blue_pub = self.create_publisher(PointStamped, 'blue_cone', 10)
        self.pixel_circle_pub = self.create_publisher(PointStamped, 'pixel_circle', 10)
        self.bbox_pub = self.create_publisher(BoundingBoxArray, 'bounding_box', 10)

        cv2.namedWindow("Detections", cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        start = time.perf_counter()
        results = self.model(frame)[0]
        end = time.perf_counter()
        latency = (end - start) * 1000
        self.get_logger().info(f"YOLOv8-seg Inference Time: {latency:.2f} ms")

        if results.masks is None or results.boxes is None:
            self.get_logger().warn("No detections or masks.")
            return

        masks = results.masks.data.cpu().numpy()
        boxes = results.boxes
        bbox_array = BoundingBoxArray()
        bbox_array.header.stamp = self.get_clock().now().to_msg()
        bbox_array.header.frame_id = "camera_map"

        for i, box in enumerate(boxes):
            if i >= len(masks):
                self.get_logger().warn(f"No mask for box {i}, skipping.")
                continue

            cls_id = int(box.cls[0].item())
            class_name = self.labels.get(cls_id, "unknown")
            conf = float(box.conf[0].item())
            if conf < 0.5:
                continue

            # 박스 좌표 및 크기
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
            width, height = x2 - x1, y2 - y1

            # 마스크 리사이즈 및 중심 계산
            mask = masks[i]
            mask_resized = cv2.resize(mask, (width, height))
            cx_mask, cy_mask = self.get_mask_center(mask_resized, i)
            cx_box = width // 2
            cy_box = height // 2

            # 중심 보정 (가중 평균)
            alpha = 0.7
            cx_local = int(alpha * cx_mask + (1 - alpha) * cx_box)
            cy_local = int(alpha * cy_mask + (1 - alpha) * cy_box)

            cx = x1 + cx_local
            cy = y1 + cy_local

            # 메시지 생성
            bbox = BoundingBox()
            bbox.x = x1
            bbox.y = y1
            bbox.width = width
            bbox.height = height
            bbox.class_name = class_name
            bbox.confidence = conf
            bbox.center_x = float(cx)
            bbox.center_y = float(cy)

            bbox_array.boxes.append(bbox)

            # 시각화
            color = CLASS_COLOR.get(class_name, (0, 255, 0))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{class_name}", (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.circle(frame, (cx, cy), 6, color, -1)

            # 중심점 퍼블리시
            self.publish_center_point(class_name, cx, cy)

        self.bbox_pub.publish(bbox_array)

        cv2.imshow("Detections", frame)
        if cv2.waitKey(1) == ord('q'):
            rclpy.shutdown()

    def get_mask_center(self, mask, i):
        mask_bin = (mask > 0.5).astype(np.uint8)
        M = cv2.moments(mask_bin)
        if M["m00"] == 0:
            self.get_logger().warn(f"Mask {i} has zero area.")
            return mask.shape[1] // 2, mask.shape[0] // 2  # fallback
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return cx, cy

    def publish_center_point(self, class_name, x, y):
        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = "camera_link"
        pt.point.x = float(x)
        pt.point.y = float(y)
        pt.point.z = 0.0

        if class_name == "yellow":
            self.yellow_pub.publish(pt)
        elif class_name == "blue":
            self.blue_pub.publish(pt)

        self.pixel_circle_pub.publish(pt)


def main(args=None):
    rclpy.init(args=args)
    node = YoloSegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
