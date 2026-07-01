#!/usr/bin/env python3
"""
障碍代价曲线对比 (obstacle cost curves)

把你的【原版设计】(DDP / MPPI_DDP / DWA_DDP 三个变体) 和【归一化势场版】
画在一起，直观对比 "代价 vs 障碍距离" 的形状，帮助选定最终方案。

用法:
    python3 plot_obstacle_cost.py            # 弹窗显示
    python3 plot_obstacle_cost.py --save     # 存成 obstacle_cost.png (无显示环境/WSL 用这个)

依赖: numpy, matplotlib   (没有就 pip install numpy matplotlib)
"""
import argparse
import numpy as np

OBS_RANGE = 4.0  # 你代码里的 obs_range_


# ---------------- 原版设计 ----------------
def cost_ddp_original(d):
    """DDP 原版: 300/150 台阶 + 中场(线性+反比), 1.0m 归零"""
    if d < 0.05:
        return 300.0
    if d < 0.1:
        return 150.0
    if d < 1.0:
        return OBS_RANGE - d + 1.0 / d
    return 0.0


def cost_mppi_original(d):
    """MPPI_DDP 原版: 近场 1/d^2 (封顶1e6) + 中场(线性+反比), 0.5m 归零"""
    if d < 0.1:
        return min(1.0 / (d + 1e-6) ** 2, 1e6)
    if d < 0.5:
        return OBS_RANGE - d + 1.0 / d
    return 0.0


def cost_dwa_original(d):
    """DWA_DDP 原版: 近场 1/d^2 + 其余(线性 + 4/d), 远场不归零"""
    if d < 0.1:
        return min(1.0 / (d + 1e-6) ** 2, 1e6)
    return OBS_RANGE - d + 4.0 / d


# ---------------- 归一化势场版 (我改的) ----------------
def cost_normalized(d, lethal=0.15, influence=0.8, peak=300.0, p=1.0):
    """归一化: [lethal,influence] 上从 peak 连续降到 0, 近处按 1/d 上翘"""
    if d <= lethal:
        return peak
    if d >= influence:
        return 0.0
    inv_infl = 1.0 / influence
    ratio = (1.0 / d - inv_infl) / (1.0 / lethal - inv_infl)
    return peak * ratio ** p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--save", action="store_true", help="存 PNG 而不是弹窗")
    args = ap.parse_args()

    import matplotlib
    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    d = np.linspace(0.01, 1.2, 1200)

    curves = {
        "DDP original (300/150 steps)":   ([cost_ddp_original(x) for x in d],  "tab:blue",   "-"),
        "DWA_DDP original (1/d^2 + 4/d)":  ([cost_dwa_original(x) for x in d],  "tab:red",    "-"),
        "Normalized p=1":   ([cost_normalized(x, p=1.0) for x in d], "tab:orange", "--"),
        "Normalized p=1.5": ([cost_normalized(x, p=1.5) for x in d], "tab:olive",  "--"),
        "Normalized p=2":   ([cost_normalized(x, p=2.0) for x in d], "tab:purple", "--"),
        "Normalized p=3":   ([cost_normalized(x, p=3.0) for x in d], "tab:brown",  "--"),
    }

    # Top: full range (log Y, see near-field blow-up). Bottom: clipped 0~300 (see shape)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 9))

    for name, (y, color, ls) in curves.items():
        ax1.plot(d, y, label=name, color=color, linestyle=ls, linewidth=1.8)
        ax2.plot(d, y, label=name, color=color, linestyle=ls, linewidth=1.8)

    ax1.set_title("Obstacle Cost - full range (log Y, near-field blow-up)")
    ax1.set_yscale("log")
    ax1.set_xlabel("obstacle distance d (m)")
    ax1.set_ylabel("cost (log)")
    ax1.axvline(0.1, color="gray", ls=":", lw=1)
    ax1.axvline(0.8, color="gray", ls=":", lw=1)
    ax1.grid(True, which="both", alpha=0.3)
    ax1.legend(fontsize=9)

    ax2.set_title("Obstacle Cost - clipped 0~300 (shape / steepness)")
    ax2.set_ylim(0, 300)
    ax2.set_xlabel("obstacle distance d (m)")
    ax2.set_ylabel("cost")
    ax2.axvline(0.1, color="gray", ls=":", lw=1)
    ax2.axvline(0.8, color="gray", ls=":", lw=1)
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=9)

    fig.tight_layout()

    if args.save:
        out = "obstacle_cost.png"
        fig.savefig(out, dpi=130)
        print(f"saved -> {out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()



