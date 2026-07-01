#!/usr/bin/env python3
# PC 端订阅狗发布的 5 档边缘轮廓 /robot_edge_profile/{fine,...}, 解析并渲染验证。
# 消息: Float64MultiArray = [N, r_0..r_{N-1}], angle_k = -pi + 2pi*(k+0.5)/N (x前,CCW正)
import os, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

TOPICS=[("fine","/robot_edge_profile/fine"),
        ("semi_fine","/robot_edge_profile/semi_fine"),
        ("normal","/robot_edge_profile/normal"),
        ("coarse","/robot_edge_profile/coarse"),
        ("point_mass","/robot_edge_profile/point_mass")]

class Sub(Node):
    def __init__(self):
        super().__init__("edge_profile_view")
        self.data={}
        for name,topic in TOPICS:
            self.create_subscription(Float64MultiArray, topic,
                                     lambda m,n=name: self.data.__setitem__(n, list(m.data)), 10)

def main():
    rclpy.init(); node=Sub()
    end=time.time()+6.0
    while time.time()<end and len(node.data)<len(TOPICS):
        rclpy.spin_once(node, timeout_sec=0.1)

    fig,axes=plt.subplots(1,5,figsize=(22,5))
    for ax,(name,topic) in zip(axes,TOPICS):
        d=node.data.get(name)
        if not d or len(d)<2:
            ax.set_title(f"{name}\n(未收到)"); ax.set_aspect("equal"); continue
        N=int(d[0]); r=np.array(d[1:1+N])
        ang=-math.pi+2*math.pi*(np.arange(N)+0.5)/N
        x=r*np.cos(ang); y=r*np.sin(ang)
        ax.plot(np.r_[x,x[0]],np.r_[y,y[0]],"-r",lw=1.5)
        ax.scatter(x,y,c="r",s=10,zorder=5)
        ax.scatter([0],[0],c="k",marker="+",s=60)
        ax.arrow(0,0,0.15,0,head_width=0.02,color="g",length_includes_head=True)
        ax.set_aspect("equal"); ax.grid(True,alpha=0.3)
        ax.set_title(f"{name}: N={N}\nrmax={r.max():.3f}m")
        print(f"{name:11s} N={N:3d}  r=[{r.min():.3f},{r.max():.3f}]")

    out=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),"tmp","edge_profile_received.png")
    os.makedirs(os.path.dirname(out),exist_ok=True)
    fig.savefig(out,dpi=120,bbox_inches="tight",facecolor="white")
    print("saved:",out,"| 收到档数:",len(node.data))
    node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()

