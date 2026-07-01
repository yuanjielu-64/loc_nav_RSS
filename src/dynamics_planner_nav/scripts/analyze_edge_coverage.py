#!/usr/bin/env python3
# 数值评估: 狗端发布的边缘轮廓 是否"包住"真实网格投影。
# 方法: 把网格点和轮廓都按角度分桶(1度), 比较每个方向上 轮廓半径 vs 网格最大半径。
import os, math, time, xml.etree.ElementTree as ET
import numpy as np, trimesh, rclpy
from rclpy.node import Node
import tf2_ros
from std_msgs.msg import Float64MultiArray

PKG = os.path.expanduser("~/Desktop/navigation/nav_loc_ws/src/nav_loc_localization")
URDF = os.path.join(PKG, "urdf", "go2.urdf"); BASE = "base_link"

def rpy_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r); cp, sp = math.cos(p), math.sin(p); cy, sy = math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
def quat_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
def T(R, t): M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M

def parse_urdf(path):
    root = ET.parse(path).getroot(); out = {}
    for link in root.findall("link"):
        vis = []
        for v in link.findall("visual"):
            g = v.find("geometry"); m = g.find("mesh") if g is not None else None
            if m is None: continue
            fn = m.get("filename").replace("package://nav_loc_localization/", PKG + "/")
            sc = np.array([float(x) for x in m.get("scale", "1 1 1").split()])
            o = v.find("origin"); xyz = [0, 0, 0]; rpy = [0, 0, 0]
            if o is not None:
                xyz = [float(x) for x in o.get("xyz", "0 0 0").split()]; rpy = [float(x) for x in o.get("rpy", "0 0 0").split()]
            vis.append((fn, T(rpy_R(*rpy), xyz), sc))
        if vis: out[link.get("name")] = vis
    return out

def parse_parts(d):
    parts = {}; npart = int(d[0]); i = 1
    for _ in range(npart):
        if i + 2 > len(d): break
        pid = int(d[i]); cnt = int(d[i+1]); i += 2
        parts[pid] = np.array(d[i:i+2*cnt]).reshape(-1, 2); i += 2*cnt
    return parts

class N(Node):
    def __init__(self):
        super().__init__("analyze_edge"); self.buf = tf2_ros.Buffer(); self.l = tf2_ros.TransformListener(self.buf, self); self.edge = None
        self.create_subscription(Float64MultiArray, "/robot_edge_profile", lambda m: setattr(self, "edge", list(m.data)), 10)
    def M(self, f):
        try:
            t = self.buf.lookup_transform(BASE, f, rclpy.time.Time()); tr = t.transform.translation; q = t.transform.rotation
            return T(quat_R(q.x, q.y, q.z, q.w), [tr.x, tr.y, tr.z])
        except Exception: return None

def main():
    vis = parse_urdf(URDF); cache = {}
    rclpy.init(); node = N(); end = time.time() + 6.0
    while time.time() < end and (node.edge is None): rclpy.spin_once(node, timeout_sec=0.1)
    # 网格投影点
    pts = []
    for link, vl in vis.items():
        Ml = node.M(link)
        if Ml is None: continue
        for (fn, Mv, sc) in vl:
            if not os.path.exists(fn): continue
            if fn not in cache: cache[fn] = trimesh.load(fn, force="mesh")
            V = np.asarray(cache[fn].vertices) * sc
            pts.append((Ml @ Mv @ np.c_[V, np.ones(len(V))].T).T[:, :2])
    P = np.concatenate(pts, 0)
    mr = np.hypot(P[:, 0], P[:, 1]); ma = np.arctan2(P[:, 1], P[:, 0])

    # 轮廓点(所有部位合并: 每个方向取所有部位最大半径 = 整机外包络)
    parts = parse_parts(node.edge)
    ex, ey = [], []
    for pid, seg in parts.items():
        if len(seg) < 2: continue
        ex.append(seg[:, 0] * np.cos(seg[:, 1])); ey.append(seg[:, 0] * np.sin(seg[:, 1]))
    ex = np.concatenate(ex); ey = np.concatenate(ey)
    er = np.hypot(ex, ey); ea = np.arctan2(ey, ex)

    # 按 1 度分桶, 比较每个方向: 网格最大半径 vs 轮廓最大半径
    B = 360
    bins = np.linspace(-np.pi, np.pi, B + 1)
    mesh_max = np.full(B, np.nan); edge_max = np.full(B, np.nan)
    mi = np.clip(np.digitize(ma, bins) - 1, 0, B - 1)
    ei = np.clip(np.digitize(ea, bins) - 1, 0, B - 1)
    for b in range(B):
        mm = mr[mi == b]; ee = er[ei == b]
        if len(mm): mesh_max[b] = mm.max()
        if len(ee): edge_max[b] = ee.max()

    valid = ~np.isnan(mesh_max) & ~np.isnan(edge_max)
    diff = edge_max[valid] - mesh_max[valid]   # >0 轮廓在外(安全), <0 轮廓在内(网格露出=危险)
    deg = np.degrees(((bins[:-1] + bins[1:]) / 2)[valid])

    print("=" * 64)
    print(f"网格点: {len(P)}  轮廓点: {len(ex)}  有效对比方向: {valid.sum()}/{B}")
    print(f"网格最大半径: {np.nanmax(mesh_max):.3f} m   轮廓最大半径: {np.nanmax(edge_max):.3f} m")
    print("-" * 64)
    print("差值 = 轮廓半径 - 网格半径  (>0=轮廓包住网格 安全 / <0=网格露出 危险)")
    print(f"  平均差值: {diff.mean()*100:+.1f} cm")
    print(f"  最小差值: {diff.min()*100:+.1f} cm  (最危险方向)")
    print(f"  最大差值: {diff.max()*100:+.1f} cm")
    under = diff < -0.01   # 网格露出超过 1cm 的方向
    print(f"  网格露出>1cm 的方向数: {under.sum()}/{valid.sum()}  ({100*under.sum()/valid.sum():.0f}%)")
    if under.sum() > 0:
        worst = deg[under][np.argsort(diff[under])[:8]]
        worstv = np.sort(diff[under])[:8] * 100
        print("  露出最严重的方向(度: cm):")
        for d, v in zip(worst, worstv):
            print(f"    {d:+6.0f}° : {v:+.1f} cm")
    print("=" * 64)
    if under.sum() == 0:
        print("结论: 轮廓在所有方向都包住真实网格 → 安全, 可直接用于避障。")
    elif under.sum() / valid.sum() < 0.05 and diff.min() > -0.03:
        print("结论: 基本包住, 仅极少方向轻微露出(<3cm) → 可用, 建议加 2-3cm 余量更稳。")
    else:
        print("结论: 多处网格露出/露出较多 → 轮廓偏小, 建议狗端加 pad_m 余量。")

    node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()

