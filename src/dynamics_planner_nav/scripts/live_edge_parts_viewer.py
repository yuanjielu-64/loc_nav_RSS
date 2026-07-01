#!/usr/bin/env python3
# PC 端【实时】查看 /robot_edge_profile 分段格式: 5 个部位各自闭合轮廓。
# 布局: data[0]=部位数; 每部位 [part_id, count, r0,a0, r1,a1, ...]
# 颜色: body=深灰, FL=蓝, FR=橙, RL=绿, RR=红。坐标 base_link(x前,y左,原点+)。
import math
import numpy as np, rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

NAME={0:"body",1:"head",2:"FL",3:"FR",4:"RL",5:"RR"}
COLOR={0:"0.3",1:"tab:purple",2:"tab:blue",3:"tab:orange",4:"tab:green",5:"tab:red"}

def parse(d):
    parts={}; npart=int(d[0]); i=1
    for _ in range(npart):
        if i+2>len(d): break
        pid=int(d[i]); cnt=int(d[i+1]); i+=2
        seg=np.array(d[i:i+2*cnt]).reshape(-1,2); i+=2*cnt
        parts[pid]=seg   # (r,a)
    return parts

class Sub(Node):
    def __init__(self):
        super().__init__("edge_parts_live"); self.d=None
        self.create_subscription(Float64MultiArray,"/robot_edge_profile",
                                 lambda m: setattr(self,"d",list(m.data)),10)

def main():
    rclpy.init(); node=Sub()
    fig,ax=plt.subplots(figsize=(8,8))
    fig.suptitle("/robot_edge_profile  live  (5 parts)")
    lines={pid:ax.plot([],[],"-",color=COLOR[pid],lw=1.8,label=NAME[pid])[0] for pid in NAME}
    ax.scatter([0],[0],c="k",marker="+",s=90)
    ax.arrow(0,0,0.15,0,head_width=0.02,color="g",length_includes_head=True)
    ax.set_aspect("equal"); ax.grid(True,alpha=0.3); ax.set_xlim(-0.7,0.7); ax.set_ylim(-0.7,0.7)
    ax.legend(loc="upper right",fontsize=9)

    def animate(_):
        rclpy.spin_once(node, timeout_sec=0.0)
        if not node.d or len(node.d)<2: return []
        parts=parse(node.d); rmax=0
        for pid,line in lines.items():
            seg=parts.get(pid)
            if seg is None or len(seg)<2: line.set_data([],[]); continue
            r,a=seg[:,0],seg[:,1]; x=r*np.cos(a); y=r*np.sin(a)
            line.set_data(np.r_[x,x[0]], np.r_[y,y[0]]); rmax=max(rmax,r.max())
        ax.set_title(f"parts={len(parts)}  rmax={rmax:.2f}m")
        return []

    ani=FuncAnimation(fig, animate, interval=150)
    plt.show()
    node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()


