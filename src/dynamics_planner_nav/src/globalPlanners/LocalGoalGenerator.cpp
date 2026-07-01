// LocalGoalGenerator.cpp — 状态相关 local goal 生成/获取的实现(声明见 .hpp)
#include "LocalGoalGenerator.hpp"
#include "../robot/Go2.hpp"   // 仅为 Robot_config::RobotState 具名枚举

#include <algorithm>
#include <cmath>
#include <limits>

namespace lgoal {

namespace {
    // 各状态的前瞻距离调参(米 / 无量纲)。集中在此，便于联调与日后写 paper 做 ablation。
    //   L = clamp(K * reach, Lmin, Lmax)，reach = max_vel_x * horizon_time(动力学可达位移)。
    // NORMAL：拉远 → 平滑跟向。K 接近 1(几乎吃满可达)，地板 = base_lookahead。
    constexpr double K_NORMAL     = 1.0;
    constexpr double L_NORMAL_MAX = 3.0;
    // CAUTIOUS：拉近 → 贴线灵敏。K 小、天花板 = base_lookahead(绝不超过标称)；
    //   下限 1.5m，避免窄道里 goal 贴得过近(过近会让目标代价失稳/原地打转)。
    constexpr double K_CAUTIOUS     = 0.5;
    constexpr double L_CAUTIOUS_MIN = 1.5;
    // RECOVER：保守固定近点，温和重新贴回路径(脱困方向另由 8 方向 clearance 决定)。
    //   太近(如 0.25)会把 goal 代价强拉到脚边、干扰脱困；取 1.0m 给一个有意义的重捕方向。
    constexpr double L_RECOVER = 1.0;

    constexpr double HORIZON_FALLBACK = 1.0;   // horizon_time<=0 时的兜底(s)

    inline double reachOf(const LocalGoalQuery &q) {
        const double T = (q.horizon_time > 0.0) ? q.horizon_time : HORIZON_FALLBACK;
        return std::max(0.0, q.max_vel_x) * T;
    }
}  // namespace

// ===== 三套前瞻距离方程 =====

double lookaheadNormal(const LocalGoalQuery &q) {
    // 拉远到动力学可达边界，但不少于标称、不多于上限。
    const double L = K_NORMAL * reachOf(q);
    return std::clamp(L, q.base_lookahead, L_NORMAL_MAX);
}

double lookaheadCautious(const LocalGoalQuery &q) {
    // 拉近：取可达位移的一部分，封顶到标称前瞻，窄道里 goal 更贴身、轨迹更平滑。
    const double L = K_CAUTIOUS * reachOf(q);
    return std::clamp(L, L_CAUTIOUS_MIN, q.base_lookahead);
}

double lookaheadRecover(const LocalGoalQuery &q) {
    (void)q;
    // 脱困期不追远目标，仅给一个很近的重捕点；具体挪动方向交给 DDP 的 8 方向 clearance。
    return L_RECOVER;
}

// ===== 统一入口：弧长选点 =====

int generateLocalGoal(const std::vector<std::vector<double>> &path_odom,
                      const LocalGoalQuery &q) {
    const int n = static_cast<int>(path_odom.size());
    if (n == 0) return -1;
    if (n == 1) return 0;

    // (1) 按状态选对应方程算前瞻距离 L。
    double L;
    switch (q.robot_state) {
        case Robot_config::CAUTIOUS: L = lookaheadCautious(q); break;
        case Robot_config::RECOVER:  L = lookaheadRecover(q);  break;
        case Robot_config::NORMAL:
        default:                     L = lookaheadNormal(q);   break;
    }

    // (2) 投影：找路径上离机器人最近的点 i0(用平方距离省 sqrt)。从 i0 起往前看，
    //     避免 U 形回绕时选到几何近、拓扑远的回程点。
    int i0 = 0;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (int i = 0; i < n; ++i) {
        if (path_odom[i].size() < 2) continue;
        const double dx = path_odom[i][0] - q.rx;
        const double dy = path_odom[i][1] - q.ry;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) { best_d2 = d2; i0 = i; }
    }

    // (3) 从 i0 起沿路径累计弧长，命中 >= L 的第一个点即 local goal；不够远则末点兜底。
    double acc = 0.0;
    for (int i = i0 + 1; i < n; ++i) {
        if (path_odom[i].size() < 2 || path_odom[i - 1].size() < 2) continue;
        const double dx = path_odom[i][0] - path_odom[i - 1][0];
        const double dy = path_odom[i][1] - path_odom[i - 1][1];
        acc += std::sqrt(dx * dx + dy * dy);
        if (acc >= L) return i;
    }
    return n - 1;   // 路径总长不足 L：用末点
}

}  // namespace lgoal
