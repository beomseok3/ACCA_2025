#!/usr/bin/env python3
"""
manta_yolo_cam_node.py
ROS 2 (rclpy) + Vimba Python + ONNXRuntime
publishes /image_raw (sensor_msgs/Image)
"""

import sys, threading, time, argparse
from collections import deque
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import onnxruntime as ort

# ── Vimba Python SDK ────────────────────────────
from vmbpy import Vimba, FrameStatus     # ← 모듈 이름만 vmbpy

# ── YOLO helper (간단) ──────────────────────────
def load_yolo(model_path, gpu=True):
    providers = ['CUDAExecutionProvider', 'CPUExecutionProvider'] if gpu else ['CPUExecutionProvider']
    sess = ort.InferenceSession(model_path, providers=providers)
    input_name  = sess.get_inputs()[0].name
    output_name = sess.get_outputs()[0].name
    return sess, input_name, output_name

def yolo_infer(sess, in_name, out_name, img_bgr):
    img = cv2.resize(img_bgr, (640,640))
    blob = cv2.dnn.blobFromImage(img, 1/255.0, swapRB=True)
    out  = sess.run([out_name], {in_name: blob})[0][0]   # (N, 84)
    # 간단히 conf>0.4 가정, xywh 변환 등 생략 (디버그용 박스)
    dets = []
    for row in out:
        conf = row[4]
        if conf < 0.4: continue
        cls   = int(np.argmax(row[5:]))
        xc,yc,w,h = row[:4] * [img_bgr.shape[1],img_bgr.shape[0],img_bgr.shape[1],img_bgr.shape[0]]
        dets.append((cls, conf, int(xc-w/2), int(yc-h/2), int(w), int(h)))
    return dets

def draw_boxes(img, dets):
    for cls, conf, x,y,w,h in dets:
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
        cv2.putText(img,f'{cls}:{conf:.2f}',(x,y-5),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1)

# ── ROS 2 Node ──────────────────────────────────
class MantaYoloNode(Node):
    def __init__(self, model, names, show_gui):
        super().__init__('manta_yolo_cam_py')
        self.bridge   = CvBridge()
        self.pub      = self.create_publisher(Image, 'image_raw', 10)
        self.show_gui = show_gui
        # YOLO init
        self.sess, self.in_name, self.out_name = load_yolo(model, gpu=True)
        # simple queue
        self.q = deque(maxlen=64)

        # Vimba init
        self.vimba = Vimba.get_instance()
        self.vimba.__enter__()
        cams = self.vimba.get_all_cameras()
        if not cams:
            self.get_logger().error('No camera')
            rclpy.shutdown(); return
        self.cam = cams[0]
        self.cam.open()

        # start acquisition
        self.cam.start_streaming(handler=self.frame_handler, buffer_count=8)

        # consumer thread
        threading.Thread(target=self.consume_loop, daemon=True).start()

    def frame_handler(self, cam, frame):
        if frame.get_status() != FrameStatus.Complete:
            cam.queue_frame(frame); return
        img = frame.as_numpy_ndarray()
        img_bgr = cv2.cvtColor(img, cv2.COLOR_BayerBG2BGR)
        self.q.append(img_bgr)
        cam.queue_frame(frame)

    def consume_loop(self):
        while rclpy.ok():
            if not self.q:
                time.sleep(0.001); continue
            frame = self.q.popleft()
            dets  = yolo_infer(self.sess, self.in_name, self.out_name, frame)
            draw_boxes(frame, dets)

            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)

            if self.show_gui:
                cv2.imshow('manta_yolo', frame)
                if cv2.waitKey(1)==27: rclpy.shutdown(); break

    def destroy_node(self):
        self.cam.stop_streaming()
        self.cam.close()
        self.vimba.__exit__(None,None,None)
        super().destroy_node()

# ── main ─────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('model'); parser.add_argument('names')
    parser.add_argument('--no-gui', action='store_true')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)

    node = MantaYoloNode(args.model, args.names, not args.no_gui)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
