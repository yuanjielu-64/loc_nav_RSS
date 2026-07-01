#!/usr/bin/env python3
# 订阅 /robot_edge_profile (非均匀 半径+角度 成对), 格式 [M, r0,a0, r1,a1, ...], 渲染验证。
import os, time
import numpy as np, rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

class Sub(Node):
    def __init__(self):
        super().__init__("edge_pairs_view"); self.d=None
        self.create_subscription(Float64MultiArray,"/robot_edge_profile",
                                 lambda m: setattr(self,"d",list(m.data)),1)

def main():
    rclpy.init(); node=Sub(); end=time.time()+6.0
    while time.time()<end and node.d is None:
        rclpy.spin_once(node,timeout_sec=0.1)
    if not node.d or len(node.d)<5:
        print("未收到 /robot_edge_profile 或太短:", None if not node.d else len(node.d)); rclpy.shutdown(); return
    d=node.d; M=int(d[0]); pr=np.array(d[1:1+2*M]).reshape(M,2)
    radii, angles = pr[:,0], pr[:,1]
    x=radii*np.cos(angles); y=radii*np.sin(angles)

    fig,ax=plt.subplots(figsize=(8,8))
    # 按角度排序后连线, 形成闭合轮廓
    order=np.argsort(angles); xs,ys=x[order],y[order]
    ax.fill(xs,ys, facecolor=(0.2,0.55,0.95,0.20), edgecolor="r", lw=1.8, label=f"edge M={M}")
    ax.scatter(x,y,c="r",s=14,zorder=5)
    ax.scatter([0],[0],c="k",marker="+",s=90); ax.arrow(0,0,0.15,0,head_width=0.02,color="g",length_includes_head=True)
    ax.set_aspect("equal"); ax.grid(True,alpha=0.3); ax.legend(loc="upper right")
    ax.set_title(f"/robot_edge_profile (radius,angle pairs) M={M}")
    out=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),"tmp","edge_pairs.png")
    os.makedirs(os.path.dirname(out),exist_ok=True)
    fig.savefig(out,dpi=130,bbox_inches="tight",facecolor="white")
    print("saved:",out,f"| M={M}  r∈[{radii.min():.3f},{radii.max():.3f}]  ang∈[{angles.min():.2f},{angles.max():.2f}]rad")
    node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()

