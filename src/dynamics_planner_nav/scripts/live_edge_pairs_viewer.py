#!/usr/bin/env python3
# PC 端【实时】查看 /robot_edge_profile (非均匀 半径,角度 成对), 随姿势刷新。需要 GUI。
import math
import numpy as np, rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Sub(Node):
    def __init__(self):
        super().__init__("edge_pairs_live"); self.d=None
        self.create_subscription(Float64MultiArray,"/robot_edge_profile",
                                 lambda m: setattr(self,"d",list(m.data)),10)

def main():
    rclpy.init(); node=Sub()
    fig,ax=plt.subplots(figsize=(8,8))
    fig.suptitle("/robot_edge_profile  live  (move the dog to check)")
    fillpoly=[None]
    line,=ax.plot([],[], "-r", lw=1.8)
    pts=ax.scatter([],[],c="r",s=12,zorder=5)
    ax.scatter([0],[0],c="k",marker="+",s=90)
    ax.arrow(0,0,0.15,0,head_width=0.02,color="g",length_includes_head=True)
    ax.set_aspect("equal"); ax.grid(True,alpha=0.3); ax.set_xlim(-0.7,0.7); ax.set_ylim(-0.7,0.7)

    def animate(_):
        rclpy.spin_once(node, timeout_sec=0.0)
        d=node.d
        if not d or len(d)<5: return []
        M=int(d[0]); pr=np.array(d[1:1+2*M]).reshape(M,2)
        r,a=pr[:,0],pr[:,1]; x=r*np.cos(a); y=r*np.sin(a)
        order=np.argsort(a); xs,ys=x[order],y[order]
        line.set_data(np.r_[xs,xs[0]], np.r_[ys,ys[0]])
        pts.set_offsets(np.c_[x,y])
        if fillpoly[0] is not None:
            try: fillpoly[0].remove()
            except Exception: pass
        fillpoly[0]=ax.fill(np.r_[xs,xs[0]], np.r_[ys,ys[0]], color=(0.2,0.55,0.95,0.18))[0]
        ax.set_title(f"M={M}  rmax={r.max():.2f}m")
        return []

    ani=FuncAnimation(fig, animate, interval=150)
    plt.show()
    node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()

