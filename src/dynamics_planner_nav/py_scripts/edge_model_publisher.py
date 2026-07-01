import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from shapely.geometry import Polygon, MultiPolygon, MultiPoint
from shapely.ops import unary_union

MODELS = [1, 4, 6, 8, 10]
PUB_TOPIC = '/robot_collision_models'
VIZ_TOPIC = '/robot_collision_models_viz'
SUB_TOPIC = '/robot_edge_profile'
VIZ_FRAME = 'base_link'
RATE_HZ = 5
COLORS = [
    (1, 1, 1),
    (1, 0.2, 0.2),
    (1, 0.6, 0),
    (0.2, 0.8, 1),
    (0.4, 1, 0.4),
]


def parse_parts(d):
    parts = {}
    npart = int(d[0])
    i = 1
    for _ in range(npart):
        if i + 2 > len(d):
            return parts
        pid = int(d[i])
        cnt = int(d[i + 1])
        i += 2
        parts[pid] = np.array(d[i:i + 2 * cnt]).reshape(-1, 2)
        i += 2 * cnt
    return parts


def outer_ring(parts):
    """所有部位点 -> 直接求【凸包】(shapely Polygon)。
    5 档模型(圆/矩形/N边形)都只用凸包, 无需昂贵的并集 union, 这样 PC 计算快得多。"""
    pts = []
    for seg in parts.values():
        if len(seg) < 1:
            continue
        r, a = seg[:, 0], seg[:, 1]
        pts.append(np.c_[r * np.cos(a), r * np.sin(a)])
    if not pts:
        return None
    hull = MultiPoint(np.vstack(pts)).convex_hull
    if hull.geom_type != 'Polygon':
        return None
    h2 = hull.simplify(0.02)
    if h2.geom_type == 'Polygon' and len(h2.exterior.coords) >= 4:
        hull = h2
    return hull


def model_circle(poly, margin):
    """N=1: 外接圆 -> 返回 ('circle', cx, cy, r+margin)。膨胀=半径加 margin。"""
    pts = np.asarray(poly.exterior.coords)[:-1]
    cx, cy = poly.centroid.x, poly.centroid.y
    rmax = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy).max()
    return ('circle', cx, cy, rmax + margin)


def min_area_rect(poly, margin):
    """N=4: 最小面积有向矩形(旋转卡壳), 贴狗朝向的长方形。四边各外扩 margin。"""
    H = np.asarray(poly.convex_hull.exterior.coords)[:-1]
    best = None
    for i in range(len(H)):
        e = H[(i + 1) % len(H)] - H[i]
        th = np.arctan2(e[1], e[0])
        c, s = np.cos(-th), np.sin(-th)
        R = np.array([[c, -s], [s, c]])
        rot = H @ R.T
        mn = rot.min(0)
        mx = rot.max(0)
        area = (mx[0] - mn[0]) * (mx[1] - mn[1])
        if best is None or area < best[0]:
            mn2 = mn - margin
            mx2 = mx + margin
            corners = np.array([
                [mn2[0], mn2[1]],
                [mx2[0], mn2[1]],
                [mx2[0], mx2[1]],
                [mn2[0], mx2[1]],
            ])
            best = (area, corners @ R)
    return best[1]


def subdivide_to(V, N):
    """把闭合多边形顶点细分到正好 N 个: 反复在【最长的边】中点插一个点。
    插点落在边上, 形状不变、仍外包。用于请求点数 > 凸包边数时凑够 N。"""
    pts = [np.asarray(p, float) for p in V]
    while len(pts) < N:
        m = len(pts)
        lens = [np.hypot(*(pts[(i + 1) % m] - pts[i])) for i in range(m)]
        i = int(np.argmax(lens))
        mid = (pts[i] + pts[(i + 1) % m]) / 2
        pts.insert(i + 1, mid)
    return np.array(pts)


def min_area_ngon(poly, N, margin):
    """N>=5: 最小面积外包 N 边形, 每条边贴凸包真实边。带防尖刺保护。
    膨胀: 每条支撑边外移 margin(支撑值 c -> c+margin), 顶点自动外扩, 点数不变、保持尖角。"""
    P = np.asarray(poly.convex_hull.exterior.coords)[:-1]
    m = len(P)
    area2 = np.dot(P[:, 0], np.roll(P[:, 1], -1)) - np.dot(P[:, 1], np.roll(P[:, 0], -1))
    if area2 < 0:
        P = P[::-1]
    cen = P.mean(axis=0)
    rmax = np.hypot(P[:, 0] - cen[0], P[:, 1] - cen[1]).max()
    far_limit = 1.8 * rmax + margin

    def line(i):
        p = P[i]
        q = P[(i + 1) % m]
        d = q - p
        n = np.array([d[1], -d[0]], float)
        n /= np.hypot(*n) + 1e-12
        return (n, n @ p + margin)

    def verts(act):
        L = [line(i) for i in act]
        V = []
        for k in range(len(L)):
            (n1, c1), (n2, c2) = L[k], L[(k + 1) % len(L)]
            A = np.array([n1, n2])
            b = np.array([c1, c2])
            det = A[0, 0] * A[1, 1] - A[0, 1] * A[1, 0]
            V.append(P[act[(k + 1) % len(act)]] if abs(det) < 1e-12 else np.linalg.solve(A, b))
        return np.array(V)

    def area(V):
        x, y = V[:, 0], V[:, 1]
        return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))

    def too_far(V):
        return np.hypot(V[:, 0] - cen[0], V[:, 1] - cen[1]).max() > far_limit

    if N >= m:
        V = verts(list(range(m)))
        return subdivide_to(V, N)
    act = list(range(m))
    while len(act) > N:
        best = None
        for k in range(len(act)):
            V = verts(act[:k] + act[k + 1:])
            if too_far(V):
                continue
            a = area(V)
            if best is None or a < best[0]:
                best = (a, k)
        if best is None:
            for k in range(len(act)):
                a = area(verts(act[:k] + act[k + 1:]))
                if best is None or a < best[0]:
                    best = (a, k)
        act.pop(best[1])
    return verts(act)


def compute_model(poly, N, margin):
    if N == 1:
        return model_circle(poly, margin)
    if N == 4:
        return min_area_rect(poly, margin)
    return min_area_ngon(poly, N, margin)


class EdgeModelPublisher(Node):

    def __init__(self):
        super().__init__('edge_model_publisher')
        self.declare_parameter('margin', 0.05)
        self.create_subscription(Float64MultiArray, SUB_TOPIC, self.on_msg, 10)
        self.pub = self.create_publisher(Float64MultiArray, PUB_TOPIC, 10)
        self.viz = self.create_publisher(MarkerArray, VIZ_TOPIC, 10)
        self.get_logger().info(
            f'edge_model_publisher: {SUB_TOPIC} -> {PUB_TOPIC} (+{VIZ_TOPIC}) '
            f'[event-driven, margin={self.get_parameter("margin").value}m]')

    def _line_marker(self, idx, xy_closed):
        m = Marker()
        m.header.frame_id = VIZ_FRAME
        m.header.stamp = rclpy.time.Time().to_msg()
        m.ns = 'edge_models'
        m.id = idx
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.012
        r, g, b = COLORS[idx % len(COLORS)]
        m.color.r, m.color.g, m.color.b, m.color.a = float(r), float(g), float(b), 1.0
        m.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in xy_closed]
        return m

    def on_msg(self, msg):
        d = list(msg.data)
        if len(d) < 2:
            return
        poly = outer_ring(parse_parts(d))
        if poly is None:
            return
        margin = float(self.get_parameter('margin').value)
        data = [float(len(MODELS))]
        markers = MarkerArray()
        for idx, N in enumerate(MODELS):
            res = compute_model(poly, N, margin)
            if isinstance(res, tuple) and res[0] == 'circle':
                _, cx, cy, r = res
                data += [1, 1, float(cx), float(cy), float(r)]
                t = np.linspace(0, 2 * np.pi, 48)
                ring = list(zip(cx + r * np.cos(t), cy + r * np.sin(t)))
                markers.markers.append(self._line_marker(idx, ring))
            else:
                V = np.asarray(res)
                data += [0, float(len(V))]
                for p in V:
                    data += [float(p[0]), float(p[1])]
                closed = list(map(tuple, V)) + [tuple(V[0])]
                markers.markers.append(self._line_marker(idx, closed))
        self.pub.publish(Float64MultiArray(data=data))
        self.viz.publish(markers)


def main():
    rclpy.init()
    n = EdgeModelPublisher()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
