#!/usr/bin/env python3
# 对比图: 真实狗网格地面投影(灰) vs 狗端发布的 5 部位边缘轮廓(彩色), 同一姿态叠加。
import os, math, time, xml.etree.ElementTree as ET
import numpy as np, trimesh, rclpy
from rclpy.node import Node
import tf2_ros
from std_msgs.msg import Float64MultiArray
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

PKG=os.path.expanduser("~/Desktop/navigation/nav_loc_ws/src/nav_loc_localization")
URDF=os.path.join(PKG,"urdf","go2.urdf"); BASE="base_link"
NAME={0:"body",1:"head",2:"FL",3:"FR",4:"RL",5:"RR"}
COLOR={0:"0.3",1:"tab:purple",2:"tab:blue",3:"tab:orange",4:"tab:green",5:"tab:red"}

def rpy_R(r,p,y):
    cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);cy,sy=math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def T(R,t): M=np.eye(4);M[:3,:3]=R;M[:3,3]=t;return M

def parse_urdf(path):
    root=ET.parse(path).getroot(); out={}
    for link in root.findall("link"):
        vis=[]
        for v in link.findall("visual"):
            g=v.find("geometry"); m=g.find("mesh") if g is not None else None
            if m is None: continue
            fn=m.get("filename").replace("package://nav_loc_localization/",PKG+"/")
            sc=np.array([float(x) for x in m.get("scale","1 1 1").split()])
            o=v.find("origin"); xyz=[0,0,0]; rpy=[0,0,0]
            if o is not None:
                xyz=[float(x) for x in o.get("xyz","0 0 0").split()]; rpy=[float(x) for x in o.get("rpy","0 0 0").split()]
            vis.append((fn,T(rpy_R(*rpy),xyz),sc))
        if vis: out[link.get("name")]=vis
    return out

def parse_parts(d):
    parts={}; npart=int(d[0]); i=1
    for _ in range(npart):
        if i+2>len(d): break
        pid=int(d[i]); cnt=int(d[i+1]); i+=2
        parts[pid]=np.array(d[i:i+2*cnt]).reshape(-1,2); i+=2*cnt
    return parts

class N(Node):
    def __init__(self):
        super().__init__("compare_real_edge"); self.buf=tf2_ros.Buffer(); self.l=tf2_ros.TransformListener(self.buf,self); self.edge=None
        self.create_subscription(Float64MultiArray,"/robot_edge_profile",lambda m:setattr(self,"edge",list(m.data)),10)
    def M(self,f):
        try:
            t=self.buf.lookup_transform(BASE,f,rclpy.time.Time()); tr=t.transform.translation; q=t.transform.rotation
            return T(quat_R(q.x,q.y,q.z,q.w),[tr.x,tr.y,tr.z])
        except Exception: return None

def main():
    vis=parse_urdf(URDF); cache={}
    rclpy.init(); node=N(); end=time.time()+6.0
    while time.time()<end and (node.edge is None): rclpy.spin_once(node,timeout_sec=0.1)
    # 网格投影
    pts=[]
    for link,vl in vis.items():
        Ml=node.M(link)
        if Ml is None: continue
        for (fn,Mv,sc) in vl:
            if not os.path.exists(fn): continue
            if fn not in cache: cache[fn]=trimesh.load(fn,force="mesh")
            V=np.asarray(cache[fn].vertices)*sc
            pts.append((Ml@Mv@np.c_[V,np.ones(len(V))].T).T[:,:2])
    P=np.concatenate(pts,0) if pts else np.zeros((0,2))

    fig,ax=plt.subplots(figsize=(9,9))
    if len(P): ax.scatter(P[:,0],P[:,1],s=1,c="0.82",label="real mesh (projected)")
    if node.edge:
        parts=parse_parts(node.edge)
        for pid,seg in parts.items():
            if len(seg)<2: continue
            r,a=seg[:,0],seg[:,1]; x=r*np.cos(a); y=r*np.sin(a)
            ax.plot(np.r_[x,x[0]],np.r_[y,y[0]],"-",color=COLOR.get(pid,"k"),lw=2,label=NAME.get(pid,str(pid)))
    ax.scatter([0],[0],c="k",marker="+",s=90); ax.arrow(0,0,0.15,0,head_width=0.02,color="g",length_includes_head=True)
    ax.set_aspect("equal"); ax.grid(True,alpha=0.3); ax.legend(loc="upper right",fontsize=9)
    ax.set_title("Real mesh (gray) vs published 5-part edge profiles (colored)")
    out=os.path.join(PKG.replace("nav_loc_localization","dynamics_planner_nav"),"tmp","compare_real_vs_edge.png")
    os.makedirs(os.path.dirname(out),exist_ok=True)
    fig.savefig(out,dpi=130,bbox_inches="tight",facecolor="white")
    print("saved:",out,"| edge_received:",node.edge is not None,"| mesh_pts:",len(P))
    node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()


