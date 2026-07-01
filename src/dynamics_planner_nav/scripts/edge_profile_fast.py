#!/usr/bin/env python3
# 边缘轮廓【快速版】: 启动时把每个连杆网格求凸包(10万点->几十点), 每帧只变换凸包点。
# 对比"全网格 vs 凸包"的每帧耗时与结果差异, 证明凸包又快又等价。
import os, math, time, xml.etree.ElementTree as ET
import numpy as np, trimesh, rclpy
from scipy.spatial import ConvexHull
from rclpy.node import Node
import tf2_ros

PKG=os.path.expanduser("~/Desktop/navigation/nav_loc_ws/src/nav_loc_localization")
URDF=os.path.join(PKG,"urdf","go2.urdf"); BASE="base_link"; STEP_DEG=5; NB=int(360/STEP_DEG)

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
    def __init__(self): super().__init__("edge_fast"); self.buf=tf2_ros.Buffer(); self.l=tf2_ros.TransformListener(self.buf,self)
    def M(self,f):
        try:
            t=self.buf.lookup_transform(BASE,f,rclpy.time.Time()); tr=t.transform.translation; q=t.transform.rotation
            return T(quat_R(q.x,q.y,q.z,q.w),[tr.x,tr.y,tr.z])
        except Exception: return None

def profile_from_xy(P):
    ang=np.arctan2(P[:,1],P[:,0]); rad=np.hypot(P[:,0],P[:,1])
    b=((ang+math.pi)/(2*math.pi)*NB).astype(int); b=np.clip(b,0,NB-1)
    er=np.full(NB,-np.inf)
    np.maximum.at(er, b, rad)  # 每桶最大半径
    idx=np.arange(NB); good=np.isfinite(er)
    return np.interp(idx, idx[good], er[good], period=NB)

def main():
    vis=parse(URDF)
    full={}   # link -> (Mv applied verts in link frame, full)
    hull={}   # link -> 凸包顶点(link frame)
    for link,vl in vis.items():
        fv=[]; hv=[]
        for (fn,Mv,sc) in vl:
            if not os.path.exists(fn): continue
            m=trimesh.load(fn,force="mesh")
            V=(np.asarray(m.vertices)*sc)
            Vh=(Mv@np.c_[V,np.ones(len(V))].T).T[:,:3]
            fv.append(Vh)
            H=V[np.unique(ConvexHull(V).vertices)]       # 3D 凸包顶点(link frame)
            Hh=(Mv@np.c_[H,np.ones(len(H))].T).T[:,:3]
            hv.append(Hh)
        if fv: full[link]=np.concatenate(fv,0); hull[link]=np.concatenate(hv,0)

    nfull=sum(len(v) for v in full.values()); nhull=sum(len(v) for v in hull.values())
    print(f"顶点数: 全网格={nfull}  凸包={nhull}  (压缩 {nfull/max(nhull,1):.0f}x)")

    rclpy.init(); node=TF()
    end=time.time()+5.0
    while time.time()<end: rclpy.spin_once(node,timeout_sec=0.1)
    Ms={lk:node.M(lk) for lk in full}

    def compute(src):
        pts=[]
        for lk,V in src.items():
            M=Ms[lk]
            if M is None: continue
            pts.append((M@np.c_[V,np.ones(len(V))].T).T[:,:2])
        return profile_from_xy(np.concatenate(pts,0))

    # 计时(各跑 50 次取平均, 排除加载/TF)
    N=50
    t0=time.time();
    for _ in range(N): er_full=compute(full)
    tf_full=(time.time()-t0)/N*1000
    t0=time.time()
    for _ in range(N): er_hull=compute(hull)
    tf_hull=(time.time()-t0)/N*1000

    diff=np.nanmax(np.abs(er_full-er_hull))
    print(f"每帧耗时: 全网格={tf_full:.2f} ms  凸包={tf_hull:.2f} ms  (加速 {tf_full/max(tf_hull,1e-6):.1f}x)")
    print(f"两者边缘半径最大差异 = {diff*1000:.2f} mm")

    # ---- 渲染结果对比 ----
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    # 凸包点投影(用于背景散点)
    hull_xy=[]
    for lk,V in hull.items():
        if Ms[lk] is None: continue
        hull_xy.append((Ms[lk]@np.c_[V,np.ones(len(V))].T).T[:,:2])
    HX=np.concatenate(hull_xy,0)
    ba=(-math.pi+2*math.pi*(np.arange(NB)+0.5)/NB)
    fx=er_full*np.cos(ba); fy=er_full*np.sin(ba)
    hx=er_hull*np.cos(ba); hy=er_hull*np.sin(ba)
    fig,ax=plt.subplots(figsize=(8,8))
    ax.scatter(HX[:,0],HX[:,1],s=4,c="0.8",label="hull pts (projected)")
    ax.plot(np.r_[fx,fx[0]],np.r_[fy,fy[0]],"--",color="0.4",lw=1.5,label="edge: full mesh")
    ax.plot(np.r_[hx,hx[0]],np.r_[hy,hy[0]],"-r",lw=2,label="edge: convex-hull (fast)")
    ax.scatter(hx,hy,c="r",s=16,zorder=5)
    ax.scatter([0],[0],c="k",marker="+",s=90); ax.arrow(0,0,0.18,0,head_width=0.02,color="g",length_includes_head=True)
    ax.set_aspect("equal"); ax.grid(True,alpha=0.3); ax.legend(loc="upper right",fontsize=8)
    ax.set_title(f"Fast edge profile: hull {tf_hull:.1f}ms vs full {tf_full:.1f}ms (max diff {diff*1000:.0f}mm)")
    out=os.path.join(PKG.replace("nav_loc_localization","dynamics_planner_nav"),"tmp","edge_profile_fast.png")
    os.makedirs(os.path.dirname(out),exist_ok=True)
    fig.savefig(out,dpi=130,bbox_inches="tight",facecolor="white")
    print("saved:",out)
    node.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()





