#!/usr/bin/env python3
# 从真实网格(URDF dae)+实时TF, 把狗投影到地面, 提取【极坐标边缘轮廓】:
# 以 base_link 为中心, 每 STEP_DEG 度取该方向最外的边缘半径 → 紧凑的一圈轮廓,
# 将来碰撞检测用: clearance ≈ 障碍物到中心距离 − edge_radius(障碍物方位角)。
import os, math, xml.etree.ElementTree as ET
import numpy as np, trimesh, rclpy
from rclpy.node import Node
import tf2_ros
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

PKG=os.path.expanduser("~/Desktop/navigation/nav_loc_ws/src/nav_loc_localization")
URDF=os.path.join(PKG,"urdf","go2.urdf"); BASE="base_link"
STEP_DEG=5

def rpy_R(r,p,y):
    cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);cy,sy=math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def T(R,t):
    M=np.eye(4);M[:3,:3]=R;M[:3,3]=t;return M

def parse(path):
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

class TF(Node):
    def __init__(self): super().__init__("edge_profile"); self.buf=tf2_ros.Buffer(); self.l=tf2_ros.TransformListener(self.buf,self)
    def M(self,f):
        try:
            t=self.buf.lookup_transform(BASE,f,rclpy.time.Time()); tr=t.transform.translation; q=t.transform.rotation
            return T(quat_R(q.x,q.y,q.z,q.w),[tr.x,tr.y,tr.z])
        except Exception: return None

def main():
    vis=parse(URDF); cache={}
    rclpy.init(); node=TF()
    import time; end=time.time()+5.0
    while time.time()<end: rclpy.spin_once(node,timeout_sec=0.1)

    pts=[]   # 所有顶点投影到地面的 xy
    for link,vl in vis.items():
        Ml=node.M(link)
        if Ml is None: continue
        for (fn,Mv,sc) in vl:
            if not os.path.exists(fn): continue
            if fn not in cache: cache[fn]=trimesh.load(fn,force="mesh")
            V=np.asarray(cache[fn].vertices)*sc
            Vw=(Ml@Mv@np.c_[V,np.ones(len(V))].T).T[:,:3]
            pts.append(Vw[:,:2])
    P=np.concatenate(pts,0)

    # 按角度分桶, 每桶取最大半径 = 该方向最外边缘
    ang=np.arctan2(P[:,1],P[:,0]); rad=np.hypot(P[:,0],P[:,1])
    nb=int(360/STEP_DEG)
    edge_r=np.zeros(nb)
    for b in range(nb):
        lo=-math.pi+2*math.pi*b/nb; hi=-math.pi+2*math.pi*(b+1)/nb
        m=(ang>=lo)&(ang<hi)
        edge_r[b]=rad[m].max() if m.any() else np.nan
    # 空桶用相邻插值
    idx=np.arange(nb); good=~np.isnan(edge_r)
    edge_r=np.interp(idx, idx[good], edge_r[good], period=nb)
    bin_ang=(-math.pi+2*math.pi*(np.arange(nb)+0.5)/nb)
    ex=edge_r*np.cos(bin_ang); ey=edge_r*np.sin(bin_ang)

    fig,ax=plt.subplots(figsize=(8,8))
    ax.scatter(P[:,0],P[:,1],s=1,c="0.8",label="projected mesh pts")
    ax.plot(np.r_[ex,ex[0]],np.r_[ey,ey[0]],"-r",lw=2,label=f"edge profile @{STEP_DEG}deg")
    ax.scatter(ex,ey,c="r",s=18,zorder=5)
    ax.scatter([0],[0],c="k",marker="+",s=90); ax.arrow(0,0,0.18,0,head_width=0.02,color="g",length_includes_head=True)
    ax.set_aspect("equal"); ax.grid(True,alpha=0.3); ax.legend(loc="upper right",fontsize=8)
    ax.set_title("Dog edge profile from real mesh (ground projection)")
    out=os.path.join(PKG.replace("nav_loc_localization","dynamics_planner_nav"),"tmp","edge_profile.png")
    os.makedirs(os.path.dirname(out),exist_ok=True)
    fig.savefig(out,dpi=130,bbox_inches="tight",facecolor="white")
    print("saved:",out)
    print("edge_r (m) every %d deg:"%STEP_DEG)
    print(np.array2string(edge_r,precision=3,max_line_width=120))
    node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()

