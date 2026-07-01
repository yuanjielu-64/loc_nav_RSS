#!/usr/bin/env python3
# 从 TF(robot_state_publisher 用 URDF+/joint_states 做的 FK)读取每条腿各关节
# 在 base_link 下的真实位置，投影到地面(x朝前,y朝左)，画出【当前姿态】的真实 footprint。
# 趴着/站着会不同。存图到 tmp/real_footprint.png。
import math, os
import rclpy
from rclpy.node import Node
import tf2_ros
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LEGS = ["FL", "FR", "RL", "RR"]
PARTS = ["hip", "thigh", "calf", "foot"]   # 腿的关节链(投影成折线)
TRUNK_L, TRUNK_W = 0.3762, 0.0935          # URDF base_link 碰撞盒
BASE = "base_link"

class Grab(Node):
    def __init__(self):
        super().__init__("real_footprint_grab")
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

    def lookup(self, frame):
        try:
            t = self.buf.lookup_transform(BASE, frame, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

def main():
    rclpy.init()
    node = Grab()
    # 先 spin 几秒把 TF 缓冲填满
    import time
    end = time.time() + 5.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    legs_xy = {}
    ok = True
    for leg in LEGS:
        pts = []
        for part in PARTS:
            xy = node.lookup(f"{leg}_{part}")
            pts.append(xy)
        legs_xy[leg] = pts
        if any(p is None for p in pts):
            ok = False
            node.get_logger().warn(f"{leg}: 部分 TF 缺失 {pts}")

    fig, ax = plt.subplots(figsize=(7, 7))
    # 躯干盒
    ax.add_patch(plt.Rectangle((-TRUNK_L/2, -TRUNK_W/2), TRUNK_L, TRUNK_W,
                               fill=False, ec="0.5", ls="--", label="trunk box"))
    colors = {"FL": "tab:red", "FR": "tab:orange", "RL": "tab:blue", "RR": "tab:green"}
    for leg, pts in legs_xy.items():
        valid = [p for p in pts if p is not None]
        if len(valid) >= 2:
            xs = [p[0] for p in valid]; ys = [p[1] for p in valid]
            ax.plot(xs, ys, "-o", color=colors[leg], ms=4, lw=2, label=leg)
        if pts[-1] is not None:  # foot
            ax.scatter([pts[-1][0]], [pts[-1][1]], color=colors[leg], s=80, zorder=5,
                       edgecolors="k")
    ax.scatter([0], [0], c="k", marker="+", s=90)
    ax.arrow(0, 0, 0.18, 0, head_width=0.02, color="g", length_includes_head=True)
    ax.annotate("x(front)", xy=(0.19, 0.01), color="g")
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
    ax.set_title("Real footprint from TF (current pose) — feet=big dots")
    ax.legend(loc="upper right", fontsize=8)

    out_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tmp")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "real_footprint.png")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print("saved:", out, "| all_tf_ok=", ok)
    # 打印脚的真实落点
    for leg, pts in legs_xy.items():
        print(leg, "foot=", pts[-1])
    node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()

