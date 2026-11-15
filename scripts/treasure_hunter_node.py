#!/usr/bin/env python3
"""
Treasure Hunter Node
Author: Belal Elmikkawy

Approaches each ArUco marker with image based visual servoing, executes the rotation
associated with it and looks for the next one. The final marker ends the hunt.

States
  WAIT_ODOM  wait for the first odometry message
  FORWARD    drive straight and look for an unused marker
  SERVO      approach the marker: omega = -k_w (u - cx),  v = k_v (sd - s)
  REACHED    stop, publish the reached id and mark it as done
  ROTATE     turn by the angle of the marker, closed loop on the odometry yaw
  FOUND      final marker reached: stop and announce the treasure

Callbacks only store the latest data; the control loop timer is the only publisher of cmd_vel.

Topics
  subscribes  detection_topic      (std_msgs/Float32MultiArray)  [cx, fx, id, u, v, s, ...]
              odom_topic           (nav_msgs/Odometry)           yaw
  publishes   cmd_vel_topic        (geometry_msgs/Twist)
              marker_reached_topic (std_msgs/Int32)
              treasure_topic       (std_msgs/String)
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32, String

WAIT_ODOM = 'WAIT_ODOM'
FORWARD = 'FORWARD'
SERVO = 'SERVO'
REACHED = 'REACHED'
ROTATE = 'ROTATE'
FOUND = 'FOUND'


def clamp(x, lo, hi):
    """Limit x to [lo, hi]."""
    return max(lo, min(hi, x))


def wrap_angle(a):
    """Normalise an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def quaternion_to_yaw(q):
    """Return the yaw angle of a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


def parse_detections(data):
    """Convert [cx, fx, id, u, v, s, ...] to (cx, fx, [(id, u, v, s), ...]), None if malformed."""
    if len(data) < 2 or (len(data) - 2) % 4 != 0:
        return None
    markers = []
    for k in range(2, len(data), 4):
        markers.append((int(round(data[k])), data[k + 1], data[k + 2], data[k + 3]))
    return data[0], data[1], markers


class TreasureHunterNode(Node):

    def __init__(self):
        super().__init__('treasure_hunter_node')

        # Topics
        self.declare_parameter('detection_topic', '/detected_marker')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('marker_reached_topic', '/marker_reached')
        self.declare_parameter('treasure_topic', '/treasure_found')
        self.declare_parameter('control_rate', 20.0)              # Hz
        # Visual servoing
        self.declare_parameter('target_side_px', 166.0)           # desired marker side sd
        self.declare_parameter('reference_fx', 381.36)            # fx of the simulated camera
        self.declare_parameter('scale_target_with_fx', True)      # sd = target_side_px * fx / reference_fx
        self.declare_parameter('k_w', 0.003)                      # rad/s per pixel
        self.declare_parameter('k_v', 0.004)                      # m/s per pixel
        self.declare_parameter('u_tolerance_px', 15.0)
        self.declare_parameter('s_tolerance_px', 6.0)
        self.declare_parameter('align_px', 100.0)                 # no forward motion if |u - cx| >= this
        self.declare_parameter('reached_count', 2)                # consecutive good detections needed
        self.declare_parameter('min_marker_side_px', 40.0)        # ignore far (small) markers
        self.declare_parameter('meas_hold_time', 0.6)             # s, reuse the last measurement this long
        self.declare_parameter('lost_timeout', 2.0)               # s, give up and search again
        self.declare_parameter('reached_pause', 1.0)              # s, stand still after reaching
        # Motion limits
        self.declare_parameter('forward_speed', 0.25)             # m/s while searching
        self.declare_parameter('max_linear', 0.3)                 # m/s
        self.declare_parameter('max_angular', 0.6)                # rad/s
        # Instructions: marker id -> rotation in degrees (positive = counter clockwise)
        self.declare_parameter('instruction_ids', [1, 2, 3])
        self.declare_parameter('instruction_deltas_deg', [-90.0, 180.0, 90.0])
        self.declare_parameter('final_marker_id', 4)
        # Closed loop rotation and heading hold
        self.declare_parameter('yaw_tolerance_deg', 2.0)
        self.declare_parameter('k_rot', 1.0)
        self.declare_parameter('max_rot', 0.6)                    # rad/s
        self.declare_parameter('min_rot', 0.15)                   # rad/s, avoids stalling near the target
        self.declare_parameter('heading_hold', True)
        self.declare_parameter('k_heading', 1.0)
        # Round the new heading to a multiple of this step from the start heading (0 = off)
        self.declare_parameter('snap_step_deg', 0.0)

        p = lambda name: self.get_parameter(name).value          # noqa: E731
        self.target_side_px = float(p('target_side_px'))
        self.reference_fx = float(p('reference_fx'))
        self.scale_with_fx = bool(p('scale_target_with_fx'))
        self.k_w = float(p('k_w'))
        self.k_v = float(p('k_v'))
        self.u_tol = float(p('u_tolerance_px'))
        self.s_tol = float(p('s_tolerance_px'))
        self.align_px = float(p('align_px'))
        self.reached_count = int(p('reached_count'))
        self.min_side = float(p('min_marker_side_px'))
        self.hold_time = float(p('meas_hold_time'))
        self.lost_timeout = float(p('lost_timeout'))
        self.reached_pause = float(p('reached_pause'))
        self.forward_speed = float(p('forward_speed'))
        self.max_linear = float(p('max_linear'))
        self.max_angular = float(p('max_angular'))
        ids = [int(x) for x in p('instruction_ids')]
        deltas = [float(x) for x in p('instruction_deltas_deg')]
        if len(ids) != len(deltas):
            raise ValueError('instruction_ids and instruction_deltas_deg must have the same length')
        self.instructions = {i: math.radians(d) for i, d in zip(ids, deltas)}
        self.final_id = int(p('final_marker_id'))
        self.yaw_tol = math.radians(float(p('yaw_tolerance_deg')))
        self.k_rot = float(p('k_rot'))
        self.max_rot = float(p('max_rot'))
        self.min_rot = float(p('min_rot'))
        self.heading_hold = bool(p('heading_hold'))
        self.k_heading = float(p('k_heading'))
        self.snap_step = math.radians(float(p('snap_step_deg')))

        # State
        self.state = WAIT_ODOM
        self.state_since = 0.0
        self.done = set()                # ids already executed
        self.yaw = None                  # latest yaw from the odometry
        self.cx = 320.0                  # principal point from the detector
        self.fx = None
        self.detections = []             # markers of the latest frame: (id, u, v, s)
        self.det_seq = 0                 # number of detection messages received
        self.seq_processed = 0
        self.target_id = None            # marker being approached
        self.meas = None                 # (time, u, s) last measurement of the target
        self.good_hits = 0               # consecutive detections inside the tolerance
        self.reached_id = None
        self.heading_target = None       # heading to keep while going ahead
        self.rotation_target = None
        self.start_yaw = 0.0
        self.rot_dir = 1.0

        # Subscribers, publishers and control timer
        self.create_subscription(Float32MultiArray, p('detection_topic'), self.detection_callback, 10)
        self.create_subscription(Odometry, p('odom_topic'), self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, p('cmd_vel_topic'), 10)
        self.reached_pub = self.create_publisher(Int32, p('marker_reached_topic'), 10)
        self.treasure_pub = self.create_publisher(String, p('treasure_topic'), 10)

        self.create_timer(1.0 / float(p('control_rate')), self.control_loop)
        self.get_logger().info(
            f'Treasure hunter ready. Instructions (rad): {self.instructions}, final marker {self.final_id}')

    def detection_callback(self, msg):
        """Store the markers of the latest frame."""
        parsed = parse_detections(list(msg.data))
        if parsed is None:
            self.get_logger().warn('Malformed /detected_marker message ignored')
            return
        cx, fx, markers = parsed
        self.cx = cx
        if fx > 0.0:
            self.fx = fx
        self.detections = markers
        self.det_seq += 1

    def odom_callback(self, msg):
        """Store the latest yaw."""
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def now(self):
        """Current time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, new_state):
        """Switch state and record when it started."""
        if new_state != self.state:
            self.get_logger().info(f'State {self.state} -> {new_state}')
        self.state = new_state
        self.state_since = self.now()

    def target_side(self):
        """Desired marker side sd in pixels, scaled with the focal length if enabled."""
        sd = self.target_side_px
        if self.scale_with_fx and self.fx:
            sd *= self.fx / self.reference_fx
        return sd

    def publish_cmd(self, v, w):
        """Publish a velocity command within the limits."""
        cmd = Twist()
        cmd.linear.x = float(clamp(v, -self.max_linear, self.max_linear))
        cmd.angular.z = float(clamp(w, -self.max_angular, self.max_angular))
        self.cmd_pub.publish(cmd)

    def unused_candidates(self):
        """Markers of the latest frame that are not done yet and big enough."""
        return [d for d in self.detections if d[0] not in self.done and d[3] >= self.min_side]

    def hold_heading(self):
        """Angular velocity that keeps the robot on the target heading."""
        if not self.heading_hold or self.heading_target is None:
            return 0.0
        return clamp(self.k_heading * wrap_angle(self.heading_target - self.yaw),
                     -self.max_angular, self.max_angular)

    def control_loop(self):
        """Run the current state and publish its velocity command."""
        now = self.now()
        if self.yaw is None:                       # no odometry yet: stand still
            self.publish_cmd(0.0, 0.0)
            return
        if self.state == WAIT_ODOM:
            self.heading_target = self.yaw
            self.start_yaw = self.yaw
            self.set_state(FORWARD)

        if self.state == FORWARD:
            v, w = self.do_forward(now)
        elif self.state == SERVO:
            v, w = self.do_servo(now)
        elif self.state == REACHED:
            v, w = self.do_reached(now)
        elif self.state == ROTATE:
            v, w = self.do_rotate(now)
        else:                                      # FOUND
            v, w = 0.0, 0.0
        self.publish_cmd(v, w)

    def do_forward(self, now):
        """Drive straight until an unused marker is visible, then start servoing on it."""
        candidates = self.unused_candidates()
        if candidates:
            best = max(candidates, key=lambda d: d[3])        # biggest = closest
            self.target_id = best[0]
            self.meas = (now, best[1], best[3])
            self.good_hits = 0
            self.seq_processed = self.det_seq
            self.get_logger().info(f'Marker {self.target_id} found, start visual servoing')
            self.set_state(SERVO)
            return 0.0, 0.0
        return self.forward_speed, self.hold_heading()

    def do_servo(self, now):
        """Approach the target marker until it is centred and has the desired size."""
        sd = self.target_side()

        # Use each new frame once: update the measurement and count the good detections
        if self.det_seq != self.seq_processed:
            self.seq_processed = self.det_seq
            det = next((d for d in self.detections if d[0] == self.target_id), None)
            if det is not None:
                self.meas = (now, det[1], det[3])
                good = abs(det[1] - self.cx) < self.u_tol and abs(sd - det[3]) < self.s_tol
                self.good_hits = self.good_hits + 1 if good else 0
                if self.good_hits >= self.reached_count:
                    return self.enter_reached()

        # Handle a lost or briefly missing target
        age = now - self.meas[0]
        if age > self.lost_timeout:                # lost: search again
            self.get_logger().warn(f'Marker {self.target_id} lost, searching again')
            self.heading_target = self.yaw
            self.target_id = None
            self.set_state(FORWARD)
            return 0.0, 0.0
        if age > self.hold_time:                   # short gap: wait
            return 0.0, 0.0

        # Control law
        _, u, s = self.meas
        err_u = u - self.cx
        err_s = sd - s
        w = -self.k_w * err_u                      # marker on the right -> turn right (negative)
        v = self.k_v * err_s                       # too far (s < sd) -> forward
        v *= clamp(1.0 - abs(err_u) / self.align_px, 0.0, 1.0)   # align first, then approach
        if abs(err_u) < 0.3 * self.u_tol:
            w = 0.0
        if abs(err_s) < 0.5 * self.s_tol:
            v = 0.0
        self.get_logger().info(
            f'SERVO id {self.target_id}: u-cx={err_u:+.0f}px  sd-s={err_s:+.0f}px  -> v={v:+.2f} w={w:+.2f}',
            throttle_duration_sec=1.0)
        return v, w

    def enter_reached(self):
        """Mark the target as done and notify the detector."""
        self.reached_id = self.target_id
        self.done.add(self.reached_id)
        msg = Int32()
        msg.data = int(self.reached_id)
        self.reached_pub.publish(msg)
        self.get_logger().info(f'Marker {self.reached_id} REACHED')
        self.set_state(REACHED)
        return 0.0, 0.0

    def do_reached(self, now):
        """Pause, then end the hunt or start the rotation of the reached marker."""
        if now - self.state_since < self.reached_pause:
            return 0.0, 0.0
        if self.reached_id == self.final_id:
            text = f'Treasure found! (marker {self.reached_id})'
            self.get_logger().info(text)
            out = String()
            out.data = text
            self.treasure_pub.publish(out)
            self.set_state(FOUND)
            return 0.0, 0.0
        if self.reached_id not in self.instructions:
            self.get_logger().warn(f'No instruction for marker {self.reached_id}, going ahead')
            self.heading_target = self.yaw
            self.set_state(FORWARD)
            return 0.0, 0.0
        delta = self.instructions[self.reached_id]
        target = wrap_angle(self.yaw + delta)
        if self.snap_step > 0.0:
            k = round(wrap_angle(target - self.start_yaw) / self.snap_step)
            target = wrap_angle(self.start_yaw + k * self.snap_step)
        self.rotation_target = target
        self.rot_dir = 1.0 if delta >= 0.0 else -1.0
        self.get_logger().info(
            f'Instruction of marker {self.reached_id}: rotate {math.degrees(delta):+.0f} deg')
        self.set_state(ROTATE)
        return 0.0, 0.0

    def do_rotate(self, now):
        """Rotate to the target yaw, then go forward along the new heading."""
        err = wrap_angle(self.rotation_target - self.yaw)
        if abs(err) > math.pi - 0.15:              # near 180 deg: keep the planned direction
            err = math.copysign(abs(err), self.rot_dir)
        if abs(err) < self.yaw_tol:
            self.heading_target = self.rotation_target
            self.target_id = None
            self.set_state(FORWARD)
            return 0.0, 0.0
        w = math.copysign(clamp(abs(self.k_rot * err), self.min_rot, self.max_rot), err)
        return 0.0, w


def main(args=None):
    rclpy.init(args=args)
    node = TreasureHunterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_cmd(0.0, 0.0)             # stop the robot on exit
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()