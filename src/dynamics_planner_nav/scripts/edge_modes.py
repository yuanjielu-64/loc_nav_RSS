#!/usr/bin/env python3
# 5 个精细度模式 = 同一条真实边缘轮廓按不同点数采样: 200/150/100/50/10。
# 用凸包点(启动一次)，每帧只变换+分桶。打印每种点数的每帧耗时, 并出 5 联图。
import os, math, time, xml.etree.ElementTree as ET
import numpy as np, trimesh, rclpy
from scipy.spatial import ConvexHull
from rclpy.node import Node
import tf2_ros
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

PKG=os.path.expanduser("~/Desktop/navigation/nav_loc_ws/src/nav_loc_localization")
URDF=os.path.join(PKG,"urdf","go2.urdf"); BASE="base_link"
MODES=[("FINE",200),("SEMI_FINE",150),("NORMAL",100),("COARSE",50),("POINT_MASS",10)]

def rpy_R(r,p,y):
    cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);cy,sy=math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def T(R,t): M=np.eye(4);M[:3,:3]=R;M[:3,3]=t;return M

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
    def __init__(self): super().__init__("edge_modes"); self.buf=tf2_ros.Buffer(); self.l=tf2_ros.TransformListener(self.buf,self)
    def M(self,f):
        try:
            t=self.buf.lookup_transform(BASE,f,rclpy.time.Time()); tr=t.transform.translation; q=t.transform.rotation
            return T(quat_R(q.x,q.y,q.z,q.w),[tr.x,tr.y,tr.z])
        except Exception: return None

def profile(P, nb):
    ang=np.arctan2(P[:,1],P[:,0]); rad=np.hypot(P[:,0],P[:,1])
    b=((ang+math.pi)/(2*math.pi)*nb).astype(int); b=np.clip(b,0,nb-1)
    er=np.full(nb,-np.inf); np.maximum.at(er,b,rad)
    idx=np.arange(nb); good=np.isfinite(er)
    return np.interp(idx, idx[good], er[good], period=nb)

def main():
    vis=parse(URDF); hull={}
    for link,vl in vis.items():
        hv=[]
        for (fn,Mv,sc) in vl:
            if not os.path.exists(fn): continue
            V=np.asarray(trimesh.load(fn,force="mesh").vertices)*sc
            H=V[np.unique(ConvexHull(V).vertices)]
            hv.append((Mv@np.c_[H,np.ones(len(H))].T).T[:,:3])
        if hv: hull[link]=np.concatenate(hv,0)

    rclpy.init(); node=TF(); end=time.time()+5.0
    while time.time()<end: rclpy.spin_once(node,timeout_sec=0.1)
    Ms={lk:node.M(lk) for lk in hull}
    XY=np.concatenate([(Ms[lk]@np.c_[V,np.ones(len(V))].T).T[:,:2] for lk,V in hull.items() if Ms[lk] is not None],0)

    fig,axes=plt.subplots(1,5,figsize=(22,5))
    for ax,(name,n) in zip(axes,MODES):
        t0=time.time()
        for _ in range(200): er=profile(XY,n)
        ms=(time.time()-t0)/200*1000
        ba=(-math.pi+2*math.pi*(np.arange(n)+0.5)/n)
        ex=er*np.cos(ba); ey=er*np.sin(ba)
        ax.scatter(XY[:,0],XY[:,1],s=2,c="0.85")
        ax.plot(np.r_[ex,ex[0]],np.r_[ey,ey[0]],"-r",lw=1.5)
        ax.scatter(ex,ey,c="r",s=10,zorder=5)
        ax.scatter([0],[0],c="k",marker="+",s=60)
        ax.set_aspect("equal"); ax.grid(True,alpha=0.3)
        ax.set_title(f"{name}: {n} pts\n{ms:.2f} ms/frame")
        print(f"{name:10s} {n:3d} pts -> {ms:.3f} ms/frame")
    out=os.path.join(PKG.replace("nav_loc_localization","dynamics_planner_nav"),"tmp","edge_modes.png")
    os.makedirs(os.path.dirname(out),exist_ok=True)
    fig.savefig(out,dpi=120,bbox_inches="tight",facecolor="white")
    print("saved:",out)
    node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()

