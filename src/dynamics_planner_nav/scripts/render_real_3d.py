#!/usr/bin/env python3
# 用真实 URDF 网格(dae) + 实时 TF 位姿, 离屏渲染 Go2 当前姿态的 3D 图。
# 不需要 GL：trimesh/pycollada 只负责加载网格, matplotlib(Agg) 负责画。
import os, math, xml.etree.ElementTree as ET
import numpy as np
import trimesh
import rclpy
from rclpy.node import Node
import tf2_ros
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

PKG = os.path.expanduser("~/Desktop/navigation/nav_loc_ws/src/nav_loc_localization")
URDF = os.path.join(PKG, "urdf", "go2.urdf")
BASE = "base_link"
MAX_FACES = 100000 # 每个网格画图时最多三角形(基本不抽样, 保证躯干/头也密实)

def rpy_to_R(r, p, y):
    cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp,   cp*sr,          cp*cr]])

def quat_to_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
                     [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

def T(R, t):
    M=np.eye(4); M[:3,:3]=R; M[:3,3]=t; return M

def parse_urdf_visuals(path):
    tree=ET.parse(path); root=tree.getroot()
    out={}  # link -> list of (mesh_abs_path, M_visual_origin, scale)
    for link in root.findall("link"):
        name=link.get("name"); vis=[]
        for v in link.findall("visual"):
            geo=v.find("geometry"); mesh=geo.find("mesh") if geo is not None else None
            if mesh is None: continue
            fn=mesh.get("filename")
            fn=fn.replace("package://nav_loc_localization/", PKG+"/")
            sc=mesh.get("scale","1 1 1"); scale=np.array([float(x) for x in sc.split()])
            o=v.find("origin"); xyz=[0,0,0]; rpy=[0,0,0]
            if o is not None:
                xyz=[float(x) for x in o.get("xyz","0 0 0").split()]
                rpy=[float(x) for x in o.get("rpy","0 0 0").split()]
            vis.append((fn, T(rpy_to_R(*rpy), xyz), scale))
        if vis: out[name]=vis
    return out

class TF(Node):
    def __init__(self):
        super().__init__("render3d")
        self.buf=tf2_ros.Buffer(); self.l=tf2_ros.TransformListener(self.buf,self)
    def M(self, frame):
        try:
            t=self.buf.lookup_transform(BASE, frame, rclpy.time.Time())
            tr=t.transform.translation; q=t.transform.rotation
            return T(quat_to_R(q.x,q.y,q.z,q.w), [tr.x,tr.y,tr.z])
        except Exception:
            return None

def main():
    visuals=parse_urdf_visuals(URDF)
    cache={}
    rclpy.init(); node=TF()
    import time; end=time.time()+5.0
    while time.time()<end: rclpy.spin_once(node, timeout_sec=0.1)

    tris=[]   # (N,3,3) 累积所有三角形
    for link, vis in visuals.items():
        Ml = node.M(link)
        if Ml is None: continue
        for (fn, Mvis, scale) in vis:
            if not os.path.exists(fn): continue
            if fn not in cache:
                m=trimesh.load(fn, force="mesh")
                cache[fn]=m
            m=cache[fn]
            V=np.asarray(m.vertices)*scale
            Vh=np.c_[V, np.ones(len(V))]
            M = Ml @ Mvis
            Vw=(M @ Vh.T).T[:, :3]
            F=np.asarray(m.faces)
            if len(F)>MAX_FACES:
                idx=np.random.choice(len(F), MAX_FACES, replace=False); F=F[idx]
            tris.append(Vw[F])
    if not tris:
        print("no meshes rendered"); return
    tris=np.concatenate(tris, axis=0)

    # 按三角面法向做简单光照, 产生明暗立体感
    v0=tris[:,0]; v1=tris[:,1]; v2=tris[:,2]
    n=np.cross(v1-v0, v2-v0)
    ln=np.linalg.norm(n,axis=1,keepdims=True); ln[ln==0]=1; n=n/ln
    light=np.array([0.3,0.4,0.85]); light=light/np.linalg.norm(light)
    inten=np.clip(n@light, 0, 1)            # 0~1
    shade=0.35+0.65*inten                   # 抬底, 避免全黑
    base=np.array([0.20,0.55,0.95])         # 主体蓝
    facecolors=np.clip(base[None,:]*shade[:,None], 0, 1)
    facecolors=np.c_[facecolors, np.ones(len(facecolors))]

    allv=tris.reshape(-1,3); zmin=allv[:,2].min()
    mn=allv.min(0); mx=allv.max(0); ctr=(mn+mx)/2; rng=(mx-mn).max()/2

    views=[("iso",25,-60),("top",89,-90),("side",5,-90)]
    fig=plt.figure(figsize=(16,6))
    for i,(name,elev,azim) in enumerate(views,1):
        ax=fig.add_subplot(1,3,i,projection="3d")
        pc=Poly3DCollection(tris, facecolors=facecolors, edgecolor="none")
        ax.add_collection3d(pc)
        # 地面投影(蓝灰阴影)更明显
        shadow=tris.copy(); shadow[:,:,2]=zmin
        ax.add_collection3d(Poly3DCollection(shadow, facecolor=(0.1,0.1,0.1), edgecolor="none", alpha=0.12))
        ax.set_xlim(ctr[0]-rng,ctr[0]+rng); ax.set_ylim(ctr[1]-rng,ctr[1]+rng); ax.set_zlim(ctr[2]-rng,ctr[2]+rng)
        ax.set_xlabel("x(front)"); ax.set_ylabel("y(left)"); ax.set_zlabel("z")
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(f"Go2 @current pose — {name}")
        try: ax.set_box_aspect((1,1,1))
        except Exception: pass

    out_dir=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),"tmp")
    os.makedirs(out_dir, exist_ok=True)
    out=os.path.join(out_dir,"real_3d.png")
    fig.savefig(out, dpi=130, bbox_inches="tight", facecolor="white")
    print("saved:", out, "|", len(tris), "triangles")
    node.destroy_node(); rclpy.shutdown()

if __name__=="__main__":
    main()



