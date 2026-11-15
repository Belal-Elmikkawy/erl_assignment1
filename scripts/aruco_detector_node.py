#!/usr/bin/env python3
"""
ArUco Detector Node
Author: Belal Elmikkawy

Detects ArUco markers in the camera image and publishes their pixel features.

Topics
  subscribes  image_topic          (sensor_msgs/Image)       camera image
              camera_info_topic    (sensor_msgs/CameraInfo)  fx and cx
              marker_reached_topic (std_msgs/Int32)          id of the marker just reached
  publishes   detection_topic      (std_msgs/Float32MultiArray)  one message per frame
              marker_image_topic   (sensor_msgs/Image)       circle + id, only when a marker is reached
              debug_image_topic    (sensor_msgs/Image)       every frame annotated

detection_topic layout, one message per camera frame:
    [cx, fx,  id0, u0, v0, s0,  id1, u1, v1, s1, ...]
    cx, fx   principal point x and focal length x in pixels (from camera_info)
    id       marker id
    u, v     marker centre in pixels
    s        apparent side of the marker in pixels
  len(data) == 2 means that no marker is visible in that frame.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray, Int32
from cv_bridge import CvBridge, CvBridgeError


def make_detector(dict_name):
    """Return detect(gray) -> list of (marker_id, corners as a 4x2 array).

    Supports both the old (OpenCV <= 4.6) and the new (OpenCV >= 4.7) aruco API.
    """
    name = dict_name.upper()
    if not name.startswith('DICT_'):
        name = 'DICT_' + name
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"Unknown ArUco dictionary '{dict_name}'")
    dictionary_id = getattr(cv2.aruco, name)

    if hasattr(cv2.aruco, 'ArucoDetector'):                        # OpenCV >= 4.7
        detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(dictionary_id),
            cv2.aruco.DetectorParameters())

        def raw_detect(gray):
            corners, ids, _ = detector.detectMarkers(gray)
            return corners, ids
    else:                                                          # OpenCV <= 4.6
        dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        params = cv2.aruco.DetectorParameters_create()

        def raw_detect(gray):
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
            return corners, ids

    def detect(gray):
        corners, ids = raw_detect(gray)
        if ids is None:
            return []
        return [(int(i), np.asarray(c, dtype=np.float64).reshape(4, 2))
                for c, i in zip(corners, ids.flatten())]

    return detect


def marker_features(pts):
    """Return centre (u, v), apparent side s and circle radius r in pixels from 4x2 corners."""
    u = float(np.mean(pts[:, 0]))
    v = float(np.mean(pts[:, 1]))
    side_lengths = [
        np.linalg.norm(pts[0] - pts[1]),
        np.linalg.norm(pts[1] - pts[2]),
        np.linalg.norm(pts[2] - pts[3]),
        np.linalg.norm(pts[3] - pts[0])
    ]
    s = float(np.mean(side_lengths))
    # Circle radius: half the diagonal plus a small margin
    diag = np.linalg.norm(pts[0] - pts[2])
    r = int(diag / 2.0) + 4
    return u, v, s, r


def draw_marker(image, marker_id, pts):
    """Draw a green circle around the marker and its id."""
    u, v, _, r = marker_features(pts)
    cv2.circle(image, (int(u), int(v)), r, (0, 255, 0), 3)

    # Keep the text inside the image
    text_x = max(5, int(u) - r)
    text_y = max(30, int(v) - r - 10)
    cv2.putText(
        image,
        f"id {marker_id}",
        (text_x, text_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0),
        2
    )


class ArucoDetectionNode(Node):
    def __init__(self):
        super().__init__("aruco_detector_node")

        # Parameters
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('marker_image_topic', '/marker_image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('detection_topic', '/detected_marker')
        self.declare_parameter('aruco_dictionary_name', 'DICT_ARUCO_ORIGINAL')
        self.declare_parameter('marker_size_meters', 0.2)   # for reference only
        self.declare_parameter('marker_reached_topic', '/marker_reached')
        self.declare_parameter('debug_image_topic', '/aruco_debug')
        self.declare_parameter('publish_debug', True)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.marker_image_topic = self.get_parameter('marker_image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        dict_name = self.get_parameter('aruco_dictionary_name').get_parameter_value().string_value
        self.marker_reached_topic = self.get_parameter('marker_reached_topic').get_parameter_value().string_value
        self.debug_image_topic = self.get_parameter('debug_image_topic').get_parameter_value().string_value
        self.publish_debug = self.get_parameter('publish_debug').get_parameter_value().bool_value

        self.get_logger().info(f'Subscribing to image topic: {self.image_topic}')
        self.get_logger().info(f'Publishing annotated image (when a marker is reached) to: {self.marker_image_topic}')

        # Detector
        self.bridge = CvBridge()
        self.detect = make_detector(dict_name)

        # Camera intrinsics, filled by camera_info
        self.fx = None
        self.cx = None

        # Last sighting of every id as (frame, corners)
        self.last_seen = {}

        # Subscribers (sensor QoS matches a best-effort camera publisher)
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data
        )
        self.reached_sub = self.create_subscription(
            Int32,
            self.marker_reached_topic,
            self.reached_callback,
            10
        )

        # Publishers (the marker image is latched, so late viewers still receive it)
        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_img_pub = self.create_publisher(
            Image,
            self.marker_image_topic,
            latched
        )
        self.debug_pub = self.create_publisher(
            Image,
            self.debug_image_topic,
            1
        )
        self.detection_pub = self.create_publisher(
            Float32MultiArray,
            self.detection_topic,
            10
        )

    def camera_info_callback(self, msg: CameraInfo):
        """Store fx and cx from the first camera_info message."""
        if self.fx is None:
            self.fx = msg.k[0]  # k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.cx = msg.k[2]
            self.get_logger().info(f'CameraInfo received: fx={self.fx:.2f}, cx={self.cx:.2f}')

    def image_callback(self, msg: Image):
        """Detect the markers in the frame and publish their features."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        img_h, img_w = cv_image.shape[:2]

        # Until camera_info arrives: cx = image centre, fx = 0.0 (unknown)
        cx = self.cx if self.cx is not None else img_w / 2.0
        fx = self.fx if self.fx is not None else 0.0

        detections = self.detect(gray)

        data = [float(cx), float(fx)]

        debug_image = cv_image.copy() if self.publish_debug else None

        for marker_id, pts in detections:
            u, v, s, _ = marker_features(pts)
            data += [float(marker_id), u, v, s]

            self.last_seen[marker_id] = (cv_image, pts)

            if debug_image is not None:
                draw_marker(debug_image, marker_id, pts)

        detection_msg = Float32MultiArray()
        detection_msg.data = data
        self.detection_pub.publish(detection_msg)

        if debug_image is not None:
            try:
                ros_debug = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                ros_debug.header = msg.header
                self.debug_pub.publish(ros_debug)
            except CvBridgeError as e:
                self.get_logger().error(f'failed to publish debug image: {e}')

    def reached_callback(self, msg: Int32):
        """Publish the annotated image of the marker that was just reached."""
        marker_id = int(msg.data)
        if marker_id not in self.last_seen:
            self.get_logger().warn(f'Marker {marker_id} reached but it was never seen, no image published')
            return
        frame, pts = self.last_seen[marker_id]
        annotated_image = frame.copy()
        draw_marker(annotated_image, marker_id, pts)
        try:
            self.marker_img_pub.publish(self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8'))
            self.get_logger().info(f'Marker {marker_id} reached: image published on {self.marker_image_topic}')
        except CvBridgeError as e:
            self.get_logger().error(f'failed to publish marker image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
