#!/usr/bin/env python3
"""
ArUco Detector Node
Author: Belal Elmikkawy

Topics
  subscribes  image_topic          (sensor_msgs/Image)       camera image
              camera_info_topic    (sensor_msgs/CameraInfo)  fx and cx
              marker_reached_topic (std_msgs/Int32)          id of the marker just reached
  publishes   detection_topic      (std_msgs/Float32MultiArray)  one message per frame
              marker_image_topic   (sensor_msgs/Image)       circle + id, ONLY when a marker is reached
              debug_image_topic    (sensor_msgs/Image)       every frame annotated (for debugging)

detection_topic layout (CHANGE 2), one message per camera frame:
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
# CHANGE 3b: QoS profiles, to be compatible with a best-effort camera publisher
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image, CameraInfo
# CHANGE 1: Int32 is needed for the /marker_reached subscriber
from std_msgs.msg import Float32MultiArray, Int32
from cv_bridge import CvBridge, CvBridgeError


# CHANGE 4 (and 3a): detector that works with the old (OpenCV <= 4.6) and the new
# (OpenCV >= 4.7) aruco API, and that fails loudly when the dictionary name is wrong.
def make_detector(dict_name):
    """Return detect(gray) -> list of (marker_id, corners as a 4x2 array)."""
    name = dict_name.upper()
    if not name.startswith('DICT_'):          # accept 'ARUCO_ORIGINAL' and 'DICT_ARUCO_ORIGINAL'
        name = 'DICT_' + name
    if not hasattr(cv2.aruco, name):          # CHANGE 3a: no silent fallback any more
        raise ValueError(f"Unknown ArUco dictionary '{dict_name}'")
    dictionary_id = getattr(cv2.aruco, name)

    if hasattr(cv2.aruco, 'ArucoDetector'):                        # OpenCV >= 4.7
        detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(dictionary_id),
            cv2.aruco.DetectorParameters())

        def raw_detect(gray):
            corners, ids, _ = detector.detectMarkers(gray)
            return corners, ids
    else:                                                          # OpenCV <= 4.6 (your machine)
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
    """pts: 4x2 corners. Returns centre u, v, apparent side s and circle radius r (pixels)."""
    u = float(np.mean(pts[:, 0]))
    v = float(np.mean(pts[:, 1]))
    side_lengths = [
        np.linalg.norm(pts[0] - pts[1]),
        np.linalg.norm(pts[1] - pts[2]),
        np.linalg.norm(pts[2] - pts[3]),
        np.linalg.norm(pts[3] - pts[0])
    ]
    s = float(np.mean(side_lengths))
    # Radius for the bounding circle, covers the marker diagonal
    diag = np.linalg.norm(pts[0] - pts[2])
    r = int(diag / 2.0) + 4
    return u, v, s, r


def draw_marker(image, marker_id, pts):
    """Green circle around the marker + text with its id (same drawing as before, now a function
    because it is used by both the debug image and the /marker_image)."""
    u, v, _, r = marker_features(pts)
    cv2.circle(image, (int(u), int(v)), r, (0, 255, 0), 3)

    # CHANGE 5: keep the text inside the image (the marker can be near the top or the left edge)
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

        # -------------------------------------------------------------
        # Configurable Parameters
        # -------------------------------------------------------------
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('marker_image_topic', '/marker_image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('detection_topic', '/detected_marker')
        self.declare_parameter('aruco_dictionary_name', 'DICT_ARUCO_ORIGINAL')
        # CHANGE 3d: not used by the pixel based servoing, kept only as documentation
        self.declare_parameter('marker_size_meters', 0.2)
        # CHANGE 1: new parameters
        self.declare_parameter('marker_reached_topic', '/marker_reached')
        self.declare_parameter('debug_image_topic', '/aruco_debug')
        self.declare_parameter('publish_debug', True)   # set False on the ROSbot to save bandwidth

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

        # -------------------------------------------------------------
        # ArUco Dictionary & Detector Setup
        # -------------------------------------------------------------
        self.bridge = CvBridge()
        # CHANGE 3a + 4: replaces getattr(..., default), getPredefinedDictionary and the params code
        self.detect = make_detector(dict_name)

        # Camera Calibration focal length
        self.fx = None
        self.cx = None

        # CHANGE 1: last sighting of every id, as (frame, corners), used to draw /marker_image
        self.last_seen = {}

        # -------------------------------------------------------------
        # Subscribers and Publishers
        # -------------------------------------------------------------
        # CHANGE 3b: qos_profile_sensor_data (best effort) instead of depth 10 (reliable)
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
        # CHANGE 1: the controller tells us which marker it just reached
        self.reached_sub = self.create_subscription(
            Int32,
            self.marker_reached_topic,
            self.reached_callback,
            10
        )

        # CHANGE 1: /marker_image is published only when a marker is reached. Transient local
        # keeps the last image, so a viewer opened later still receives it.
        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_img_pub = self.create_publisher(
            Image,
            self.marker_image_topic,
            latched
        )
        # CHANGE 1: the continuous stream (your old behaviour) moved to its own debug topic
        self.debug_pub = self.create_publisher(
            Image,
            self.debug_image_topic,
            1
        )

        # Publisher for visual servoing controller
        self.detection_pub = self.create_publisher(
            Float32MultiArray,
            self.detection_topic,
            10
        )

    def camera_info_callback(self, msg: CameraInfo):
        """Extract camera intrinsic matrix (fx, cx)"""
        if self.fx is None:
            self.fx = msg.k[0]  # k = [fx, 0, cx, 0, fy, cy, 0 , 0, 1]
            self.cx = msg.k[2]
            self.get_logger().info(f'CameraInfo received: fx={self.fx:.2f}, cx={self.cx:.2f}')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        img_h, img_w = cv_image.shape[:2]

        # CHANGE 2: use the principal point from camera_info; image width / 2 only as a
        # fallback until camera_info arrives. (center_y was never used, so it is removed.)
        cx = self.cx if self.cx is not None else img_w / 2.0
        fx = self.fx if self.fx is not None else 0.0     # 0.0 means "not known yet"

        # Detect Markers (CHANGE 4: works with both OpenCV versions)
        detections = self.detect(gray)

        # CHANGE 2: ONE message per frame. It always starts with [cx, fx] and then has 4 numbers
        # per marker, so an empty frame gives [cx, fx] and the controller can see "nothing visible".
        data = [float(cx), float(fx)]

        debug_image = cv_image.copy() if self.publish_debug else None

        for marker_id, pts in detections:
            u, v, s, _ = marker_features(pts)
            data += [float(marker_id), u, v, s]

            # CHANGE 1: remember the frame where this id was last seen
            self.last_seen[marker_id] = (cv_image, pts)

            if debug_image is not None:
                draw_marker(debug_image, marker_id, pts)

        # Publish detection info for the visual servoing node (once per frame)
        detection_msg = Float32MultiArray()
        detection_msg.data = data
        self.detection_pub.publish(detection_msg)

        # CHANGE 1: the continuous annotated stream now goes to the debug topic
        if debug_image is not None:
            try:
                ros_debug = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                ros_debug.header = msg.header
                self.debug_pub.publish(ros_debug)
            except CvBridgeError as e:
                self.get_logger().error(f'failed to publish debug image: {e}')

    # CHANGE 1: new callback, this is where /marker_image is published
    def reached_callback(self, msg: Int32):
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
        rclpy.try_shutdown()    # CHANGE 3c: was rclpy.shutdown(), which can raise after Ctrl+C


if __name__ == "__main__":
    main()