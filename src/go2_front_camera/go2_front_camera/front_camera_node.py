"""ROS2 node: poll Go2 main-board videohub GetImageSample (api_id=1001)
and republish each JPEG as sensor_msgs/CompressedImage + sensor_msgs/Image.

Works over the wired 192.168.123.0/24 link (CycloneDDS), no WebRTC needed.
"""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from unitree_api.msg import Request, Response


VIDEOHUB_API_ID_GET_IMAGE_SAMPLE = 1001


class FrontCameraNode(Node):
    def __init__(self):
        super().__init__('go2_front_camera')

        # ---- parameters ----
        self.declare_parameter('poll_rate_hz', 10.0)
        self.declare_parameter('frame_id', 'front_camera')
        self.declare_parameter('publish_decoded', True)
        self.declare_parameter('response_timeout_s', 1.0)

        self.poll_rate = float(
            self.get_parameter('poll_rate_hz').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_decoded = bool(
            self.get_parameter('publish_decoded').value)
        self.response_timeout = float(
            self.get_parameter('response_timeout_s').value)

        # ---- pubs/subs ----
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=5)

        self.req_pub = self.create_publisher(
            Request, '/api/videohub/request', qos)
        self.resp_sub = self.create_subscription(
            Response, '/api/videohub/response', self._on_response, qos)

        self.image_pub = self.create_publisher(
            Image, '/go2_camera/front/image_raw', sensor_qos)
        self.compressed_pub = self.create_publisher(
            CompressedImage, '/go2_camera/front/image_raw/compressed',
            sensor_qos)

        self.bridge = CvBridge()
        self._pending_id = None
        self._sent_at = 0.0
        self._frames_ok = 0
        self._frames_err = 0
        self._last_report = time.monotonic()

        # ---- timers ----
        period = 1.0 / max(self.poll_rate, 0.5)
        self.create_timer(period, self._poll)
        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f"go2_front_camera up. polling videohub api_id=1001 "
            f"at {self.poll_rate:.1f} Hz, frame_id='{self.frame_id}', "
            f"publish_decoded={self.publish_decoded}")

    # -------- request / response cycle --------

    def _poll(self):
        # Skip new poll if previous one is still in-flight (and not timed out).
        if self._pending_id is not None:
            if (time.monotonic() - self._sent_at) < self.response_timeout:
                return
            # Timed out; count as error and continue.
            self._frames_err += 1
            self._pending_id = None

        req = Request()
        # Use microsecond timestamp as request id so we can match responses.
        req.header.identity.id = int(time.time() * 1e6)
        req.header.identity.api_id = VIDEOHUB_API_ID_GET_IMAGE_SAMPLE
        req.header.lease.id = 0
        req.header.policy.priority = 0
        req.header.policy.noreply = False
        req.parameter = ""
        req.binary = []

        self._pending_id = req.header.identity.id
        self._sent_at = time.monotonic()
        self.req_pub.publish(req)

    def _on_response(self, msg: Response):
        # Match by request id when possible; otherwise accept latest.
        if self._pending_id is not None and \
                msg.header.identity.id != self._pending_id:
            # Not the response we asked for. Ignore.
            return
        self._pending_id = None

        if msg.header.status.code != 0:
            self.get_logger().warn(
                f"GetImageSample returned non-zero status: "
                f"{msg.header.status.code}")
            self._frames_err += 1
            return

        data = bytes(msg.binary)
        if len(data) < 4 or data[:3] != b'\xff\xd8\xff':
            self.get_logger().warn(
                f"Response binary is not a JPEG (len={len(data)}, "
                f"head={data[:8].hex()})")
            self._frames_err += 1
            return

        stamp = self.get_clock().now().to_msg()

        # Always publish CompressedImage (cheap; no decode).
        compressed = CompressedImage()
        compressed.header.stamp = stamp
        compressed.header.frame_id = self.frame_id
        compressed.format = 'jpeg'
        compressed.data = list(data)
        self.compressed_pub.publish(compressed)

        if self.publish_decoded:
            arr = np.frombuffer(data, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                self.get_logger().warn("cv2.imdecode failed on JPEG frame")
                self._frames_err += 1
                return
            image_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            self.image_pub.publish(image_msg)

        self._frames_ok += 1

    def _report(self):
        now = time.monotonic()
        dt = now - self._last_report
        rate = self._frames_ok / dt if dt > 0 else 0.0
        self.get_logger().info(
            f"published {self._frames_ok} frames "
            f"({rate:.1f} Hz), errors={self._frames_err}")
        self._frames_ok = 0
        self._frames_err = 0
        self._last_report = now


def main():
    rclpy.init()
    node = FrontCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
