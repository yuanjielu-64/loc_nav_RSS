#!/usr/bin/env python3
# 验证 FINE(精细) 体积模型的边缘极坐标采样是否正确。
# 狗形近似(基于 Unitree Go2 公开规格, 站立约 0.70x0.31m, 四足落足点比躯干略宽):
#   躯干(小): box 长 TORSO_L x 宽 TORSO_W
#   四只脚:  半径 FOOT_R 的圆, 落足点在 (±FOOT_DX, ±FOOT_DY), 伸到躯干外形成 4 个凸起
# 这些数值是公开近似, 之后按官网精确尺寸再调。
import math
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---- 狗形 FINE 模型参数 (m, 体坐标系: x 朝前, y 朝左) ----
# 全部来自 go2.urdf 实测：
#   躯干碰撞盒 base_link: 0.3762 x 0.0935 (长x宽)
#   四髋关节(腿根): (±0.1934, ±0.0465)
# 脚用圆近似；FOOT_R 代表腿/脚的横向伸展(站姿越宽越大)，是唯一可调旋钮。
TORSO_L, TORSO_W = 0.3762, 0.0935   # URDF base_link 碰撞盒
HIP_DX, HIP_DY = 0.1934, 0.0465     # URDF 髋关节位置
FOOT_R = 0.09                       # 腿/脚横向伸展半径(可调)
FOOT_DX, FOOT_DY = HIP_DX, HIP_DY

htl, htw = TORSO_L / 2.0, TORSO_W / 2.0

def box_sdf(px, py, cx, cy, half_l, half_w):
    dx = abs(px - cx) - half_l
    dy = abs(py - cy) - half_w
    outside = math.hypot(max(dx, 0.0), max(dy, 0.0))
    inside = min(max(dx, dy), 0.0)
    return outside + inside

def circle_sdf(px, py, cx, cy, r):
    return math.hypot(px - cx, py - cy) - r

# 并集 SDF：躯干 box + 四只脚圆
SHAPES_BOX = [(0.0, 0.0, htl, htw)]
SHAPES_CIRCLE = [( FOOT_DX,  FOOT_DY, FOOT_R),
                 ( FOOT_DX, -FOOT_DY, FOOT_R),
                 (-FOOT_DX,  FOOT_DY, FOOT_R),
                 (-FOOT_DX, -FOOT_DY, FOOT_R)]

def sd(px, py):
    d = min(box_sdf(px, py, *b) for b in SHAPES_BOX)
    d = min(d, min(circle_sdf(px, py, *c) for c in SHAPES_CIRCLE))
    return d

def circum_radius():
    r = math.hypot(htl, htw)
    for (cx, cy, rr) in SHAPES_CIRCLE:
        r = max(r, math.hypot(cx, cy) + rr)
    return r

def edge_radius(angle):
    dx, dy = math.cos(angle), math.sin(angle)
    if sd(0.0, 0.0) > 0.0:
        return 0.0
    lo, hi = 0.0, circum_radius() + 1e-3
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if sd(mid * dx, mid * dy) < 0.0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)

# 每 5° 采样, 360° 共 72 点 (x 朝前=0°, CCW 为正)
angles_deg = list(range(0, 360, 5))
pts = []
for a in angles_deg:
    th = math.radians(a)
    r = edge_radius(th)
    pts.append((r * math.cos(th), r * math.sin(th)))

xs = [p[0] for p in pts]
ys = [p[1] for p in pts]

fig, ax = plt.subplots(figsize=(7, 7))
# 真实形状参考(躯干矩形 + 四只脚圆)
ax.add_patch(plt.Rectangle((-htl, -htw), TORSO_L, TORSO_W, fill=False, ec="0.6", ls="--", label="torso"))
for (cx, cy, rr) in SHAPES_CIRCLE:
    ax.add_patch(plt.Circle((cx, cy), rr, fill=False, ec="0.6", ls="--"))
# 采样的边缘点 + 连线(闭合)
ax.plot(xs + [xs[0]], ys + [ys[0]], "-", color="tab:blue", lw=1, alpha=0.7)
ax.scatter(xs, ys, c="tab:red", s=18, zorder=3, label="edge pts @5deg")
ax.scatter([0], [0], c="k", marker="+", s=80, label="center")
ax.annotate("x (front)", xy=(circum_radius()*0.9, 0), color="g")
ax.arrow(0, 0, circum_radius()*0.8, 0, head_width=0.02, color="g", length_includes_head=True)

ax.set_aspect("equal")
ax.grid(True, alpha=0.3)
ax.set_title("FINE volume model — edge radial sampling (every 5 deg)")
ax.legend(loc="upper right", fontsize=8)

out_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tmp")
os.makedirs(out_dir, exist_ok=True)
out_path = os.path.join(out_dir, "fine_model_profile.png")
fig.savefig(out_path, dpi=130, bbox_inches="tight")
print("saved:", out_path)
print("circum_radius =", round(circum_radius(), 4))
print("sample r range = [%.4f, %.4f]" % (min(math.hypot(x, y) for x, y in pts),
                                          max(math.hypot(x, y) for x, y in pts)))

