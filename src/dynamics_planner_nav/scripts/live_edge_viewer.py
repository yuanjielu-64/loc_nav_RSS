#!/usr/bin/env python3
# PC 端【实时动态】查看狗发布的 5 档边缘轮廓, 随姿势刷新。
# 需要图形界面(WSLg/X)。订阅 /robot_edge_profile/{fine,...}, FuncAnimation 持续更新。
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

TOPICS=[("fine","/robot_edge_profile/fine"),
        ("semi_fine","/robot_edge_profile/semi_fine"),
        ("normal","/robot_edge_profile/normal"),
        ("coarse","/robot_edge_profile/coarse"),
        ("point_mass","/robot_edge_profile/point_mass")]

class Sub(Node):
    def __init__(self):
        super().__init__("edge_live_view")
        self.data={}
        for name,topic in TOPICS:
            self.create_subscription(Float64MultiArray, topic,
                                     lambda m,n=name: self.data.__setitem__(n,list(m.data)), 10)

def main():
    rclpy.init(); node=Sub()
    fig,axes=plt.subplots(1,5,figsize=(22,5)); fig.suptitle("Go2 edge profile (live) — 动一动狗腿看变化")
    arts=[]
    for ax,(name,_) in zip(axes,TOPICS):
        line,=ax.plot([],[], "-r", lw=1.5)
        pts=ax.scatter([],[],c="r",s=10,zorder=5)
        ax.scatter([0],[0],c="k",marker="+",s=60)
        ax.set_aspect("equal"); ax.grid(True,alpha=0.3); ax.set_xlim(-0.7,0.7); ax.set_ylim(-0.7,0.7)
        ax.set_title(name)
        arts.append((name,ax,line,pts))

    def animate(_):
        rclpy.spin_once(node, timeout_sec=0.0)
        for name,ax,line,pts in arts:
            d=node.data.get(name)
            if not d or len(d)<2: continue
            N=int(d[0]); r=np.array(d[1:1+N])
            ang=-math.pi+2*math.pi*(np.arange(N)+0.5)/N
            x=r*np.cos(ang); y=r*np.sin(ang)
            line.set_data(np.r_[x,x[0]], np.r_[y,y[0]])
            pts.set_offsets(np.c_[x,y])
            ax.set_title(f"{name}: N={N} rmax={r.max():.2f}m")
        return []

    ani=FuncAnimation(fig, animate, interval=150)
    plt.show()
    node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()

