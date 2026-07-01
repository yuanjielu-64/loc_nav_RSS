#!/usr/bin/env python3
# 把狗端 /robot_edge_profile 的 6 部位当成一个整体: 求并集外轮廓(去掉重叠在内部的点),
# 再简化成不同数量的【特征角点】, 对比看形状。
import os, time
import numpy as np, rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = os.path.expanduser("~/Desktop/navigation/nav_loc_ws/src/dynamics_planner_nav/tmp/downsample.png")
COUNTS = [1, 4, 10, 50, 100, 150]

def parse_parts(d):
    parts = {}; npart = int(d[0]); i = 1
    for _ in range(npart):
        if i + 2 > len(d): break
        pid = int(d[i]); cnt = int(d[i+1]); i += 2
        parts[pid] = np.array(d[i:i+2*cnt]).reshape(-1, 2); i += 2*cnt
    return parts

def outer_ring(parts):
    """6 部位 -> 各自多边形 -> 并集 -> 最大外环坐标(有序闭合, 已去除内部重叠点)。"""
    polys = []
    for seg in parts.values():
        if len(seg) < 3: continue
        r, a = seg[:, 0], seg[:, 1]
        xy = np.c_[r*np.cos(a), r*np.sin(a)]
        p = Polygon(xy)
        if not p.is_valid:
            p = p.buffer(0)   # 修复自交多边形
        if not p.is_empty:
            polys.append(p)
    u = unary_union(polys)
    if isinstance(u, MultiPolygon):
        u = max(u.geoms, key=lambda g: g.area)   # 取面积最大的连通块
    return np.asarray(u.exterior.coords)[:-1]      # 去掉重复的首尾闭合点
    """N 个等角方向的支撑线(切线)围成的凸多边形: 必然外包、方向均匀。
    N=4 时即为轴对齐包围盒(矩形)。适合点少时给规整外框。"""
    pts = np.asarray(true_poly.exterior.coords)[:-1]
def visvalingam(coords, N):
    """折线简化: 反复删掉'重要度(三角面积)最小'的点, 直到剩 N 个 -> 保留角点。"""
    pts = [tuple(p) for p in coords]
    if N >= len(pts):
        return np.array(pts)
    def area(a, b, c):
        return abs((b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1])) * 0.5
    idx = list(range(len(pts)))
    while len(idx) > N:
        m = len(idx)
        areas = [area(pts[idx[(j-1) % m]], pts[idx[j]], pts[idx[(j+1) % m]]) for j in range(m)]
        idx.pop(int(np.argmin(areas)))
    return np.array([pts[i] for i in idx])

def main():
    rclpy.init(); node = Sub(); end = time.time() + 6
    while time.time() < end and node.d is None: rclpy.spin_once(node, timeout_sec=0.1)
    if not node.d:
        print("没收到 /robot_edge_profile"); rclpy.shutdown(); return
    parts = parse_parts(node.d)
    total = sum(len(s) for s in parts.values())
    ring = outer_ring(parts)               # 整机外轮廓(并集, 无内部重叠点)
    ring, true_poly = outer_ring(parts)    # 整机外轮廓 + shapely 多边形
    cx, cy = ring[:, 0].mean(), ring[:, 1].mean()
    rmax = np.hypot(ring[:, 0]-cx, ring[:, 1]-cy).max()

    fig, axes = plt.subplots(2, 3, figsize=(13, 9))
    lim = (np.abs(ring).max()) * 1.2
    lim = (np.abs(ring).max()) * 1.25
    for ax, N in zip(axes.ravel(), COUNTS):
        # 背景: 真实整机外轮廓(浅灰), 方便对照
        ax.plot(np.r_[ring[:,0], ring[0,0]], np.r_[ring[:,1], ring[0,1]], "-", color="0.8", lw=1)
        # 背景: 真实整机外轮廓(浅灰), 对照看简化轮廓是否包在外面
        ax.plot(np.r_[ring[:,0], ring[0,0]], np.r_[ring[:,1], ring[0,1]], "-", color="0.75", lw=1.2)
        if N == 1:
            ang = np.linspace(-np.pi, np.pi, 100)
            ax.plot(cx + rmax*np.cos(ang), cy + rmax*np.sin(ang), "-", color="tab:gray", lw=1.5)
            ax.plot([cx], [cy], "o", color="tab:red", ms=6)
            ax.set_title(f"N=1 (外接圆 r={rmax:.2f}m)")
        else:
            v = visvalingam(ring, N)
            ax.fill(np.r_[v[:,0], v[0,0]], np.r_[v[:,1], v[0,1]], color="tab:blue", alpha=0.15)
            ax.plot(np.r_[v[:,0], v[0,0]], np.r_[v[:,1], v[0,1]], "-", color="tab:blue", lw=1.5)
            v = reduce_enclosing(ring, N, true_poly)
            ax.fill(np.r_[v[:,0], v[0,0]], np.r_[v[:,1], v[0,1]], color="tab:blue", alpha=0.12)
            ax.plot(np.r_[v[:,0], v[0,0]], np.r_[v[:,1], v[0,1]], "-", color="tab:blue", lw=1.6)
            ax.plot(v[:,0], v[:,1], ".", color="tab:red", ms=5)
            ax.set_title(f"N={N} 特征点")
            ax.set_title(f"N={len(v)} 特征点(外包)")
        ax.scatter([0], [0], c="k", marker="+", s=80)
        ax.arrow(0, 0, 0.12, 0, head_width=0.02, color="g", length_includes_head=True)
        ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim)
    fig.suptitle(f"整机外轮廓(并集去内部点) 特征降采样 — 原始合计 {total} 点", fontsize=13)
    fig.suptitle(f"外包络简化(始终包住真实形状) — 原始 {total} 点 / 外轮廓 {len(ring)} 点", fontsize=13)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.tight_layout(); fig.savefig(OUT, dpi=120, facecolor="white")
    print("saved:", OUT, "| 原始点:", total, "| 外轮廓点:", len(ring))
    node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()





