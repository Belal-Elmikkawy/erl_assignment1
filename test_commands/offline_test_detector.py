"""Offline test of aruco_detector_node.py: the real code, with rclpy / cv_bridge / msgs faked.
usage: python3 -I test_detector.py <node.py> <textures_dir> <out_dir>
"""
import importlib.util, os, sys, types, math, glob
import numpy as np
import cv2

node_path, tex_dir, out_dir = sys.argv[1:4]
os.makedirs(out_dir, exist_ok=True)

# ---------------------------------------------------------------- fakes for ROS
class Msg:
    def __init__(self, **kw): self.__dict__.update(kw)
class Image(Msg): pass
class CameraInfo(Msg): pass
class Float32MultiArray(Msg):
    def __init__(self): self.data = []
class Int32(Msg):
    def __init__(self, data=0): self.data = data

class Param:
    def __init__(self, v): self.value = v
    def get_parameter_value(self):
        v = self.value
        return types.SimpleNamespace(string_value=v, bool_value=v, double_value=v)

class Logger:
    def __init__(self): self.lines = []
    def _log(self, lvl, m): self.lines.append((lvl, m)); print(f'    [{lvl}] {m}')
    def info(self, m, **k): self._log('INFO', m)
    def warn(self, m, **k): self._log('WARN', m)
    def error(self, m, **k): self._log('ERROR', m)

class Pub:
    def __init__(self, topic, qos): self.topic, self.qos, self.msgs = topic, qos, []
    def publish(self, m): self.msgs.append(m)

OVERRIDES = {}
class Node:
    def __init__(self, name):
        self.name, self._p, self.pubs, self.subs, self._log = name, {}, {}, {}, Logger()
    def declare_parameter(self, n, d): self._p[n] = OVERRIDES.get(n, d)
    def get_parameter(self, n): return Param(self._p[n])
    def get_logger(self): return self._log
    def create_subscription(self, t, topic, cb, qos): self.subs[topic] = (t, cb, qos)
    def create_publisher(self, t, topic, qos):
        p = Pub(topic, qos); self.pubs[topic] = p; return p
    def destroy_node(self): pass

class Bridge:
    def imgmsg_to_cv2(self, m, desired_encoding=None): return m.img.copy()
    def cv2_to_imgmsg(self, img, encoding=None): return Image(img=img, encoding=encoding, header=None)

class QoSProfile:
    def __init__(self, **kw): self.__dict__.update(kw)
enum = lambda *n: types.SimpleNamespace(**{x: x for x in n})

def mod(name, **attrs):
    m = types.ModuleType(name); m.__dict__.update(attrs); sys.modules[name] = m; return m
mod('rclpy', init=lambda **k: None, spin=lambda n: None, try_shutdown=lambda: None)
mod('rclpy.node', Node=Node)
mod('rclpy.qos', QoSProfile=QoSProfile, qos_profile_sensor_data='SENSOR_DATA',
    ReliabilityPolicy=enum('RELIABLE', 'BEST_EFFORT'),
    DurabilityPolicy=enum('TRANSIENT_LOCAL', 'VOLATILE'))
mod('sensor_msgs'); mod('sensor_msgs.msg', Image=Image, CameraInfo=CameraInfo)
mod('std_msgs'); mod('std_msgs.msg', Float32MultiArray=Float32MultiArray, Int32=Int32)
mod('cv_bridge', CvBridge=Bridge, CvBridgeError=Exception)

spec = importlib.util.spec_from_file_location('det', node_path)
det = importlib.util.module_from_spec(spec); spec.loader.exec_module(det)

# ---------------------------------------------------------------- helpers
FX, CX, W, H, L = 381.36, 320.0, 640, 480, 0.2
results = []
def check(name, ok, info=''):
    results.append((name, ok)); print(f"{'PASS' if ok else 'FAIL'}  {name}  {info}")

textures = sorted(glob.glob(os.path.join(tex_dir, '*.png')))

def marker_crop(path):
    """Returns the texture and the 4 corners of the marker inside it (found with the node's detector)."""
    img = cv2.imread(path)
    big = cv2.copyMakeBorder(img, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=(255, 255, 255))
    d = det.make_detector('DICT_ARUCO_ORIGINAL')(cv2.cvtColor(big, cv2.COLOR_BGR2GRAY))
    return big, d

def render(tex_big, tex_corners, side_px, u, v, yaw_deg=0.0, noise=0.007, bg=150, base=None):
    """Put the marker in a 640x480 frame: side_px wide, centred at (u, v), rotated around the
    vertical axis by yaw (perspective foreshortening), gaussian noise like the gazebo camera."""
    half = side_px / 2.0
    k = math.cos(math.radians(yaw_deg))          # horizontal shrink
    t = math.sin(math.radians(yaw_deg)) * 0.15   # near side taller than far side
    dst = np.float32([[u - half * k, v - half * (1 + t)], [u + half * k, v - half * (1 - t)],
                      [u + half * k, v + half * (1 - t)], [u - half * k, v + half * (1 + t)]])
    # map the whole texture (marker + border) using the marker corners as reference
    src = np.float32(tex_corners)
    M = cv2.getPerspectiveTransform(src, dst)
    frame = np.full((H, W, 3), bg, np.uint8) if base is None else base.copy()
    warped = cv2.warpPerspective(tex_big, M, (W, H), borderValue=(bg, bg, bg))
    mask = cv2.warpPerspective(np.full(tex_big.shape[:2], 255, np.uint8), M, (W, H))
    frame[mask > 0] = warped[mask > 0]
    f = frame.astype(np.float32) + np.random.normal(0, noise * 255, frame.shape)
    return np.clip(f, 0, 255).astype(np.uint8)

np.random.seed(0)

# ---------------------------------------------------------------- 1. textures decode to the right id
print('\n== 1. Marker textures with DICT_ARUCO_ORIGINAL')
tex = {}
for p in textures:
    big, d = marker_crop(p)
    ids = [i for i, _ in d]
    expected = int(os.path.basename(p).split('_')[-1].split('.')[0])
    check(f'{os.path.basename(p)} -> id {expected}', ids == [expected], f'detected {ids}')
    if d: tex[expected] = (big, d[0][1])

# ---------------------------------------------------------------- 2. dictionary handling
print('\n== 2. Dictionary parameter')
try:
    det.make_detector('DICT_FOO'); check('unknown dictionary raises', False)
except ValueError as e:
    check('unknown dictionary raises ValueError', True, str(e))
check("'ARUCO_ORIGINAL' (no DICT_) accepted", callable(det.make_detector('ARUCO_ORIGINAL')))
big, c = tex[1]
frame = render(big, c, 166, 320, 240)
wrong = det.make_detector('DICT_4X4_50')(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
check('wrong dictionary (4X4_50) detects nothing', wrong == [], f'got {[i for i,_ in wrong]}')

# ---------------------------------------------------------------- 3. size / centre vs pinhole model
print('\n== 3. Apparent side s = fx*L/d and centre u (all 4 markers, with camera noise)')
detect = det.make_detector('DICT_ARUCO_ORIGINAL')
for d_m in [3.75, 2.0, 1.0, 0.46]:
    s_true = FX * L / d_m
    for mid, (big, c) in tex.items():
        for u_true, yaw in [(320, 0), (200, 0), (450, 30)]:
            fr = render(big, c, s_true, u_true, 240, yaw)
            got = detect(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY))
            g = [x for x in got if x[0] == mid]
            if not g:
                check(f'id {mid} d={d_m} m u={u_true} yaw={yaw}', False, 'NOT detected'); continue
            u, v, s, r = det.marker_features(g[0][1])
            s_exp = s_true * (1 + math.cos(math.radians(yaw))) / 2   # mean of the 4 sides
            ok = abs(u - u_true) < 2.0 and abs(s - s_exp) / s_exp < 0.05
            check(f'id {mid} d={d_m:.2f} m u={u_true} yaw={yaw:2d}', ok,
                  f's={s:6.1f} (exp {s_exp:6.1f})  u={u:6.1f}')

# ---------------------------------------------------------------- 4. the node itself
print('\n== 4. Node: parameters, QoS, message layout, /marker_image')
node = det.ArucoDetectionNode()
check('subscribes /camera/image with sensor QoS', node.subs.get('/camera/image', (0, 0, 0))[2] == 'SENSOR_DATA')
check('subscribes /camera/camera_info', '/camera/camera_info' in node.subs)
check('subscribes /marker_reached', '/marker_reached' in node.subs)
mi = node.pubs['/marker_image'].qos
check('/marker_image is latched (transient local, reliable)',
      getattr(mi, 'durability', None) == 'TRANSIENT_LOCAL' and getattr(mi, 'reliability', None) == 'RELIABLE')

img_cb = node.subs['/camera/image'][1]
info_cb = node.subs['/camera/camera_info'][1]
reach_cb = node.subs['/marker_reached'][1]
det_pub, dbg_pub, mk_pub = node.pubs['/detected_marker'], node.pubs['/aruco_debug'], node.pubs['/marker_image']

# image before camera_info -> fx = 0 marker
img_cb(Image(img=np.full((H, W, 3), 150, np.uint8), header='h0'))
check('before camera_info: data == [320, 0]', list(det_pub.msgs[-1].data) == [320.0, 0.0], det_pub.msgs[-1].data)

info_cb(CameraInfo(k=[FX, 0, 321.5, 0, FX, 240, 0, 0, 1]))
img_cb(Image(img=np.full((H, W, 3), 150, np.uint8), header='h1'))
check('empty frame after camera_info: data == [cx, fx] (cx from K)',
      list(det_pub.msgs[-1].data) == [321.5, FX], det_pub.msgs[-1].data)

# two markers in one frame
f2 = render(*tex[1], 40, 90, 240)
both = render(*tex[2], 60, 530, 260, base=f2, noise=0.0)
img_cb(Image(img=both, header='h2'))
data = list(det_pub.msgs[-1].data)
ids = data[2::4]
check('two markers -> len = 2 + 4*2 and ids {1,2}', len(data) == 10 and set(ids) == {1.0, 2.0}, [round(x, 1) for x in data])
check('one /detected_marker per frame', len(det_pub.msgs) == 3)
check('debug image published every frame', len(dbg_pub.msgs) == 3)
check('/marker_image NOT published while just detecting', len(mk_pub.msgs) == 0)
cv2.imwrite(os.path.join(out_dir, 'aruco_debug_two_markers.png'), dbg_pub.msgs[-1].img)

# approach marker 1 at the target size, then "reach" it
f = render(*tex[1], 166, 330, 235)
img_cb(Image(img=f, header='h3'))
reach_cb(Int32(1))
check('/marker_reached 1 -> one /marker_image', len(mk_pub.msgs) == 1)
out = mk_pub.msgs[-1].img
check('/marker_image encoding bgr8, 640x480', mk_pub.msgs[-1].encoding == 'bgr8' and out.shape == (480, 640, 3))
green = np.all(out == (0, 255, 0), axis=2).sum()
check('/marker_image has the green circle + text', green > 500, f'{green} green px')
check('original frame not modified (no circle in last_seen)',
      np.all(node.last_seen[1][0] == (0, 255, 0), axis=2).sum() == 0)
cv2.imwrite(os.path.join(out_dir, 'marker_image_id1.png'), out)

reach_cb(Int32(3))
check('/marker_reached 3 (never seen) -> no image, WARN', len(mk_pub.msgs) == 1 and
      any('never seen' in m for l, m in node._log.lines if l == 'WARN'))

# marker near the top-left corner: text must stay inside the image
f = render(*tex[4], 120, 70, 70)
img_cb(Image(img=f, header='h4')); reach_cb(Int32(4))
cv2.imwrite(os.path.join(out_dir, 'marker_image_id4_corner.png'), mk_pub.msgs[-1].img)
check('marker at top-left corner: image still published', len(mk_pub.msgs) == 2)

# publish_debug = False
OVERRIDES['publish_debug'] = False
n2 = det.ArucoDetectionNode()
n2.subs['/camera/image'][1](Image(img=f, header='h5'))
check('publish_debug:=false -> no debug image, detections still published',
      len(n2.pubs['/aruco_debug'].msgs) == 0 and len(n2.pubs['/detected_marker'].msgs) == 1)

# ---------------------------------------------------------------- 5. detection range
print('\n== 5. Detection range (frontal, centred, with noise)')
for mid, (big, c) in tex.items():
    far = None
    for d_m in np.arange(2.0, 12.01, 0.25):
        fr = render(big, c, FX * L / d_m, 320, 240)
        if any(x[0] == mid for x in detect(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY))):
            far = d_m
    print(f'    id {mid}: detected up to ~{far:.2f} m  (s = {FX*L/far:.1f} px)')

n_ok = sum(ok for _, ok in results)
print(f'\n{n_ok}/{len(results)} checks passed')
