// Go2.hpp — Robot state/IO manager for Go2 navigation (ROS2 Humble)
// Manages ROS communication, state caching, and planner interface

#ifndef DYNAMICS_PLANNER_NAV_GO2_HPP
#define DYNAMICS_PLANNER_NAV_GO2_HPP

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/empty.hpp>

// Nav2 dependencies (replacing move_base)
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

// TF2：按激光时间戳查 odom→base_link，修正运动时"用最新位姿变换旧激光"的偏移。
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// STL
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <functional>
#include <array>

// Utils
#include "../utils/AsyncTaskExecutor.hpp"
#include "Go2_footprint.hpp"
#include "Go2_parameters.hpp"   // go2::PlannerParams(规划器可调参数 / parameter-learning 面)

// Forward declarations
class Go2Callbacks;
class GlobalPathHandler;

class Robot_config : public rclcpp::Node {
    // Friend classes for modular access
    friend class Go2Callbacks;
    friend class GlobalPathHandler;   // globalPlanners/GlobalPathHandler：/plan 入库 + local goal 生成

public:
    // ============================================================
    // Nested types
    // ============================================================
    // Robot pose/velocity snapshot
    // 全向机器人(Go2 四足狗)状态：vx_ 为机体前向速度，vy_ 为机体侧向速度，
    // angular_velocity_ 为偏航角速度。差速车(Jackal)时 vy_ 恒为 0；保留旧 6 参构造，
    // 老 planner 不改即可编译(vy_ 默认 0)，DDP 升级为全向后用 7 参构造显式带上 vy。
    class PoseState {
    public:
        PoseState()
            : x_(0.0), y_(0.0), theta_(0.0), vx_(0.0), vy_(0.0),
              angular_velocity_(0.0), valid_(false) {}

        // 旧接口(差速)：无侧向速度，vy_ 默认 0。
        PoseState(double x, double y, double theta, double v, double w, bool valid)
            : x_(x), y_(y), theta_(theta), vx_(v), vy_(0.0),
              angular_velocity_(w), valid_(valid) {
        }

        // 全向接口：显式带上侧向速度 vy。
        PoseState(double x, double y, double theta, double vx, double vy, double w, bool valid)
            : x_(x), y_(y), theta_(theta), vx_(vx), vy_(vy),
              angular_velocity_(w), valid_(valid) {
        }

        std::vector<double> pose() const { return {x_, y_}; }

        double x_;
        double y_;
        double theta_;
        double vx_;                // 机体前向速度 vx
        double vy_;                // 机体侧向速度 vy(全向；差速车恒 0)
        double angular_velocity_;
        bool valid_;
    };

    // Tuning parameters snapshot
    // 实际定义已抽到 Go2_parameters.hpp 的 go2::PlannerParams(parameter-learning 面)；
    // 这里保留 TuningParams 别名，老代码 Robot_config::TuningParams / getTuningParams() 不变。
    using TuningParams = go2::PlannerParams;


    struct VelocityLimits {
        double min_linear;
        double max_linear;
        double min_angular;
        double max_angular;
        double min_lateral = 0.0;
        double max_lateral = 0.0;
    };


    // ============================================================
    // Enums
    // ============================================================
    // Supported local planners
    enum Algorithm {
        DWA,
        DWA_DDP,
        MPPI,
        MPPI_DDP,
        DDP,
        TEB,
        TEB_DDP
    };

    // High-level robot operating modes
    enum RobotState {
        INIT = 0,
        NORMAL = 1,
        CAUTIOUS = 2,
        BLIND = 3,
        BRAKE = 4,
        RECOVER = 5,
        ROTATE = 6,
        BACK = 7,
        FORWARD = 8,
        TEST = 9,
        IDLE = 10
    };

    // Active map source
    enum MapSource {
        ONLY_COSTMAP_RECEIVED = 0,
        ONLY_LASER_RECEIVED = 1,
        NO_ANY_RECEIVED = 2
    };

    // 机器人静态碰撞体积档位(models_static_)，由粗到精，仅三档；索引与该枚举对齐。
    //   收不到 /robot_collision_models 的真实体积(models_dynamic_)时，按本枚举取静态档回退。
    enum VolumeModel {
        VOL_POINT_MASS = 0,  // 质点
        VOL_CIRCLE     = 1,  // 外接圆
        VOL_RECTANGLE  = 2,  // 矩形：ROBOT_LENGTH x ROBOT_WIDTH
    };

    // 机体系 8 个方向扇区(每 45°一个)，CCW 排列，0=正前、逆时针递增。
    // 用作 direction_clearance_ 数组的【具名下标】，避免裸数字 0..7 看不懂方向。
    //   direction_clearance_[i] 的含义：第 i 个方向上，最近障碍点到机器人【该方向物理边缘】
    //   的距离(米)。>0=还有余量，<0=已侵入机身，+INF=该方向无障碍。
    //   (= 激光中心距 range − Go2_footprint.edgeRadiusAtAngle(该方向方位角))
    enum DirSector {
        DIR_FRONT       = 0,  //   0° 正前
        DIR_FRONT_LEFT  = 1,  //  45° 左前
        DIR_LEFT        = 2,  //  90° 正左
        DIR_BACK_LEFT   = 3,  // 135° 左后
        DIR_BACK        = 4,  // 180° 正后
        DIR_BACK_RIGHT  = 5,  // 225°(-135°) 右后
        DIR_RIGHT       = 6,  // 270°(-90°)  正右
        DIR_FRONT_RIGHT = 7,  // 315°(-45°)  右前
        DIR_SECTOR_COUNT = 8
    };

    // ============================================================
    // Construction
    // ============================================================
    // bridge_max_velocity：桥能下发给狗的最大线速度(由 -m CLI 传入)。各状态的 max_vel_x
    // 都基于它推导，初始化时一次定好，运行中不再做额外夹紧。
    explicit Robot_config(double bridge_max_velocity = 2.0);
    ~Robot_config() = default;

    // ============================================================
    // Configuration: algorithm / state / limits / tuning
    // ============================================================
    void setAlgorithm(Algorithm a) { algorithm = a; }
    Algorithm getAlgorithm() const { return algorithm; }

    void setRobotState(RobotState state);
    RobotState getRobotState() const { return currentState.load(); }

    void setRobotVelocityLimits(RobotState state);
    VelocityLimits getVelocityLimits() const;

    void setDt(double t) { dt = t; }
    double getDt() const { return dt; }

    TuningParams getTuningParams() const;
    void setTuningParams(const TuningParams &tp);

    // ============================================================
    // Goals (local / global)
    // ============================================================
    void setLocalGoal(std::vector<double> &lg, double x, double y) {
        std::lock_guard<std::mutex> lock(path_goal_mutex_);
        local_goal = {lg[0], lg[1]};
        local_goal_odom = {x, y};
    }
    std::vector<double> getLocalGoalCfg() { return local_goal; }            // base_link 机体帧
    std::vector<double> getLocalGoalOdomCfg() { return local_goal_odom; }   // odom 世界帧
    std::vector<double> getGlobalGoalCfg() { return global_goal; }          // base_link 机体帧
    std::vector<double> getGlobalGoalOdomCfg() { return global_goal_odom; } // odom 世界帧
    // 判定是否到达 global goal：到达则置 global_goal_reached 并清 local_goal_received(停止规划)。
    bool checkGoalReached(double cur_x, double cur_y);
    bool isLocalGoalReady() const { return local_goal_received; }

    // ============================================================
    // Pose / velocity
    // ============================================================
    // 线程安全地取当前位姿：robot_state 由 odom 回调线程写、planner 线程读，加锁拷贝防竞争。
    PoseState getPoseState() const {
        std::lock_guard<std::mutex> lock(robot_state_mutex_);
        return robot_state;
    }
    // 按【时间戳】查 base_link 在 odom 帧的位姿(用 TF odom→base_link)：修正运动/转弯时
    // "用最新位姿变换更早采集的激光"导致的偏移。TF 查不到(超时/外推)时回退到最新 getPoseState()。
    // 只取 x/y/theta(平面导航够用)，速度字段沿用最新值。
    PoseState getPoseStateAt(const rclcpp::Time &stamp) const;
    double getVelocity() const { return robot_state.vx_; }
    double getAngularVelocity() const { return robot_state.angular_velocity_; }

    // ============================================================
    // Sensor data: laser / costmap / map
    // ============================================================
    // 激光点【odom 世界系】线程安全拷贝(laser 回调线程写、planner 读，加锁防竞争)。
    // 每点转成 {x, y}(planner 通用格式)。map = getLaserData() 即用它。
    std::vector<std::vector<double>> getLaserData() const;
    std::vector<std::vector<double>> getCostmapDataOdom() const { return costmapDataOdom; }
    // 返回 const& 而非拷贝：map 仅在规划线程 setup()->checkMapReady 写、同线程 planner 读
    // (单线程访问)；8 个评估子线程读的是 DDP 自己扁平化后的 solve_obs_x_/y_，不碰 map。
    // 故无需每帧深拷贝整份 vector<vector<double>>。各 planner 用 const auto& 接即零拷贝。
    const std::vector<std::vector<double>>& getDataMap() const { return map; }
    std::vector<double> getTimeInterval() const;   // timeInterval 线程安全拷贝(/dy_dt 回调写、planner 读)
    bool checkMapReady(bool goal_ok);

    // 8 方向(每 45°一扇区)到最近障碍的【边缘距】线程安全拷贝。下标用 DirSector 枚举：
    // DIR_FRONT / DIR_FRONT_LEFT / ... / DIR_FRONT_RIGHT。值 >0=余量, <0=侵入, +INF=无障碍。
    void getDirectionClearance(std::array<double, DIR_SECTOR_COUNT> &out) const;

    // 找 8 方向里余量最大(最开阔)的方向，输出其【机体系单位速度向量】(ux,uy)，
    // 返回该方向的余量(m)。RECOVER 受限脱困用：朝最开阔方向挪(可能侧移/斜移/后退)。
    double mostOpenDirection(double &ux, double &uy) const;

    // 硬斥力：只统计余量 < hard_dist 的扇区(贴得太近)，每个按 (hard_dist - 余量) 的强度
    // 朝该扇区【反方向】推，对 8 扇区求合力。输出机体系合力向量 (fx, fy)，返回合力模长。
    //   左右都近 → 横向分量抵消 → 合力≈0(居中不动)；只有前方近 → 合力指向后(把狗往后顶)。
    //   余量 +INF/≥hard_dist 的方向不产生硬斥力。供 RECOVER 把狗顶出 hard_dist 外。
    double computeHardRepulsion(double hard_dist, double &fx, double &fy) const;

    // 逃逸方向(开阔度加权 + goal 门控引力)：每个【已观测】扇区按"空旷度(余量,截顶 clear_cap)"
    // 投票(朝该方向走，越空权重越大)，再乘 goal 门控项 (1 + goal_gain·max(0,cos(扇区角−goal_angle)))，
    // 让朝 goal 的方向在【能走的前提下】优先。引力乘在空旷度上→朝障碍(余量≈0)方向权重≈0，绝不顶墙。
    //   goal_angle: 机体系下到 local goal 的方位角(rad，0=正前)。输出机体系单位方向(ux,uy)，
    //   返回合向量模长(<1e-6 表示四周全未观测/无开阔方向，调用方需兜底)。
    double computeEscapeDirection(double goal_angle, double goal_gain,
                                  double &ux, double &uy) const;

    std::vector<double> getFootprint() const {
        const bool full_body = getRobotState() != BACK && currentMap == ONLY_LASER_RECEIVED;
        return full_body ? std::vector<double>{ROBOT_LENGTH, ROBOT_WIDTH}
                         : std::vector<double>{POINT_MASS_LENGTH, POINT_MASS_WIDTH};
    }

    // 静态体积档集合(models_static_)：构造时由 setStaticVolumes() 按机身尺寸一次性建好，
    // 之后不再修改(immutable)，按 VolumeModel 枚举索引(质点/外接圆/矩形)。
    // getStaticVolumes() 返回 const 引用(零拷贝)：models_static_ 构造后不变，无需加锁。
    void setStaticVolumes();                              // 初始化静态档(构造时调用)
    const std::vector<go2::Footprint>& getStaticVolumes() const; // 返回 models_static_ 的 const 引用
    std::vector<go2::Footprint> getDynamicVolumes() const {
        std::lock_guard<std::mutex> lock(volumes_mutex_);
        return models_dynamic_;
    }
    bool checkVolumeReady() const { return volume_received; }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Control() { return cmd_vel_pub_; }

    bool setup();
    bool checkGazeboPaused() const;
    // 速度状态机：用实测线速度 + 时间迟滞自动切 NORMAL/CAUTIOUS/BRAKE(仅 DDP 系)。
    // 由 velocityCallback 每帧调用。实现见 Go2_prepare.cpp。
    void updateSpeedStateMachine(double linear_speed);

    // 画轨迹只需要"轨迹点 + 画前几个点"。原先的 theta_/t 参数在 makeTrajectory 里
    // 都被丢弃(死参数)，已移除，避免调用方为没用的参数构造数据。
    void viewTrajectories(std::vector<PoseState> &trajectories, int nr_steps_) const;
    void view_Goal(std::vector<double> &goal, std::vector<double> &goal1) const;
    void publishRobotState() const;

    // RECOVER 脱困斥力可视化：把机体系硬斥力向量旋到 odom 帧，从机器人当前位置画红箭头。
    //   发到 /recover_repulsion(MarkerArray)。传入机体系合力 (fx 前向, fy 左向)。
    void viewRepulsion(double hard_fx, double hard_fy) const;

    // perception 处理后的激光点(odom 系 laserData_odom) -> POINTS Marker，发到 /perception/laser_points，
    // 供 RViz 核对感知到的障碍点。
    void viewLaserPoints(const std::vector<std::vector<double>> &pts) const;

    // ============================================================
    // Public state / data members
    // ============================================================
    Algorithm algorithm;
    // 状态由多线程读写(velocityCallback/processValidGlobalPath 在执行器线程，
    // setup/DDP 在规划线程)，用 atomic 消除竞争。
    std::atomic<RobotState> currentState;
    MapSource currentMap;
    // costmap 处理开关：当前规划只用激光(getLaserData)，costmapCallback 遍历整张栅格
    // 是纯死工。默认关闭(回调里 early-return)；将来启用 BACK 倒车用 costmap 时置 true。
    bool use_costmap_{false};

    bool local_goal_received{};
    bool global_goal_received{};
    // 是否已收到 PC 端 /robot_collision_models(外包碰撞模型)。未收到则 setup() 不 ready,
    // 避免在没有真实碰撞模型时就开始规划。
    bool volume_received{false};
    // 到达 global goal 后锁存为 true：停止规划，避免狗在目标附近一直打转。
    // 仅在 path_goal_mutex_ 保护下读写（odometryCallback 置位 / processValidGlobalPath 读取 /
    // goalCallback 收到新目标时复位）。
    bool global_goal_reached{false};
    double goal_reached_threshold{1.0};  // 到达判定半径(米)
    bool param_received{};
    bool canBeSolved{};

    double rotating_angle;
    double dt{};

    std::array<double, DIR_SECTOR_COUNT> direction_clearance_{};

    // RECOVER 脱困用：recover_times = 已在 RECOVER 停留的帧数(超上限报失败)；
    // recover_to_low_count = 前方连续开阔的帧数(去抖，够了才退回 CAUTIOUS)。见 DDP.cpp。
    int recover_times = 0;
    int recover_to_low_count = 0;
    // 脱困【升级档位】：同一招卡着没效果就逐级 +1，档位越高、挪动/转动尺度越大(逐级加码)。
    // 进入 RECOVER 时归 0，脱困成功/退出时复位。用法见 DDP.cpp 的 RECOVER 分支。
    int recover_level = 0;
    // 斥力"推不动"计数：每帧比 hard_mag 与历史最小值——还在继续变小(创新低=推得动)就清零；
    // 几乎不再变小(没创新低=推不动)才 +1。超过上限 → 放弃推、交给旋转破姿态。见 DDP::repulsionPush。
    int push_stuck_count = 0;
    double push_min_force = 1e9;   // 本次斥力见过的最小 hard_mag(创新低判据)；进入 RECOVER 时复位大值
    // 斥力迟滞带：触发用 0.2m、推到 0.3m。push_active=是否正在斥力中——任一方向<0.2 触发置 true，
    // 一直推到所有方向≥0.3 才置 false。避免用同一阈值在边界反复抖。见 DDP::repulsionPush。
    bool push_active = false;
    // 边转边顶：Stuck 分支算出的纯斥力反方向顶出速度(body frame)，供 rotateToGoal 叠加到旋转上。
    double push_escape_vx = 0.0;
    double push_escape_vy = 0.0;
    // 本片窄区"推不到 kPushDist(0.3)"的标记：推到物理极限(stuck)且身体安全时置 true →
    // 不再对同一窄区反复空推，交给 Plan；走到开阔(min_clear≥kPushDist)时清 false 重新武装。
    bool push_gaveup = false;
    // RECOVER 子阶段锁存：0=Rotate(朝目标对准) 1=Plan(规划，进入后不再判朝向)。
    // 进入 RECOVER 时置 0(Rotate)；对准到 ±30° 内锁存为 Plan，从此不因朝向回旋(消除摇摆)。
    int recover_phase = 0;
    // 上次退出 RECOVER(回 CAUTIOUS)的时刻：用于判断是否"短时间内又掉回 RECOVER"。
    // 若再次进入 RECOVER 距上次退出 < kRecoverRelapseWindow，则 recover_level++(加码)，
    // 否则归 0。构造时初始化为 now()。
    rclcpp::Time recover_exit_time;

    // RECOVER 进度看门狗：用【实测位移】判断狗是否真的在动(不信 ddp 的规划速度——recoverParameters
    // 下 ddp 恒有解，但真路被堵时最优轨迹速度可能≈0，狗龟速/不动)。只在 Plan 阶段计：滑动窗口
    // kProgressTimeout 内实测位移 < kProgressDist → 判死锁，触发逃逸动作。见 DDP::recoverEscape。
    bool   recover_progress_init = false;
    double recover_progress_x = 0.0;
    double recover_progress_y = 0.0;
    rclcpp::Time recover_progress_time;
    // RECOVER 逃逸动作：判死锁后【逐帧】执行"倒退 + 朝更空一侧原地转"，打破"卡住不动"的局部陷阱。
    // 不用固定时长，靠实测反馈结束：累计转角 ≥ 阈值 或 累计位移 ≥ 阈值(先到为准)即收尾回 Rotate。
    // recover_escape_dir：+1 左转 / -1 右转(朝余量更大的一侧)；recover_escape_theta：逃逸起始朝向(算累计转角)。
    bool   recover_escape_active = false;
    double recover_escape_theta = 0.0;
    double recover_escape_dir = 1.0;


    // 全局路径(来自 /plan，/plan 本身是 odom 帧)。两帧版本：
    //   global_paths      —— 【base_link 机体帧】：processValidGlobalPath 用机器人当前 odom 位姿
    //                         把每点 world→body 变换得到。供需要机体帧的消费者(如 TEB)。
    //   global_paths_odom —— 【odom 世界帧】：/plan 原始坐标。DDP 系 planner(traj 是 odom)用它。
    std::vector<std::vector<double>> global_paths;
    std::vector<std::vector<double>> global_paths_odom;

    std::vector<double> local_goal_odom;   // 前瞻局部目标【odom 世界帧】(DDP 系用，配 getLocalGoalOdomCfg)
    std::vector<double> global_goal_odom;   // 全局目标【odom 世界帧】[x,y,yaw](配 getGlobalGoalOdomCfg；到达判定/DDP 用)


    std::vector<std::vector<double>> costmapDataOdom;
    std::vector<std::vector<double>> costmapData;
    // 与 laserData_odom 同序：每个障碍点的【原始 range】(到雷达的标量距离，米)，非坐标。
    std::vector<double> laserDataDistance;

    // DDP 时间扭曲(time-warp)：T_i = 2.0·(i/N)^1.4，总时域 2.0s，近端 dt 小(积分精/
    // 近场避障准)、远端 dt 大(省步数)。原为 N=20；现按同一条曲线重采样到 N=15。
    // 注意：步数必须与各 *Parameters 里的 nr_steps_ 一致(15)，否则 generateTrajectory /
    // ddp_planning 按 nr_steps_ 索引 timeInterval 会越界。
    std::vector<double> timeInterval = {
        0.0451, 0.0739, 0.0911, 0.1042, 0.1152, 0.1250, 0.1336,
        0.1415, 0.1488, 0.1553, 0.1617, 0.1679, 0.1735, 0.1789, 0.1841
    };

    // ============================================================
    // Planner tunables（规划器可调参数 / parameter-learning 面）
    // ------------------------------------------------------------
    // 这一组就是被学习/调参的旋钮，整组的读出/写入走 getTuningParams()/setTuningParams()
    // (实现见 Go2_parameters.cpp，参数容器见 Go2_parameters.hpp 的 go2::PlannerParams)。
    // 注意：max_vel_x/max_vel_y/max_vel_theta 还会被 setRobotVelocityLimits 按状态运行时改写，
    // 兼具"参数 + 运行时状态"双重身份，故仍作为成员留在此处。
    // ============================================================
    // DWA parameters
    double max_vel_x = 1.5;   // 前向速度上限(m/s)，Go2 室内导航偏积极值(硬件可更高)
    // 桥的最大线速度(构造时由 -m 传入)：NORMAL/BLIND 直接用它，CAUTIOUS 取其一半。
    double bridge_max_velocity_ = 2.0;
    double max_vel_y = 0.5;   // 侧移速度上限(m/s)，约 0.4×vx；Go2 螃蟹步稳定区间。差速车设 0
    double max_vel_theta = 2.0;   // 偏航角速度上限(rad/s)，比硬件极限低以利跟踪
    double vx_sample = 10;
    double vTheta_samples = 10;
    double path_distance_bias = 0.7;
    double goal_distance_bias = 0.7;

    // MPPI/DDP parameters
    double nr_pairs_ = 600;
    double nr_steps_ = 20;
    double linear_stddev = 0.1;
    double angular_stddev = 0.05;
    double lambda = 1.0;
    double local_goal_distance = 2.0;
    double distance = 0.3;
    double robot_radius_ = 0.01;

    int num_threads = 8;



    std::shared_ptr<Go2Callbacks> callbacks_;
    std::shared_ptr<AsyncTaskExecutor> async_executor_;

    // 体积档(原 RobotVolumes 已展平到此)：
    //   models_static_ ：构造时按机身尺寸一次性建好的【理论】档(质点/外接圆/矩形)，永远有值。
    //   models_dynamic_：/robot_collision_models 发来的【真实】外包模型，收到才有，优先用。
    // 读写都用 volumes_mutex_；volume_received 兼作"已收到真实体积"的就绪标志(一发不清)。
    mutable std::mutex volumes_mutex_;
    std::vector<go2::Footprint> models_static_;
    std::vector<go2::Footprint> models_dynamic_;

    std::atomic<bool> state_override_active_{false};
    RobotState state_override_value_{NORMAL};


protected:
    // Thread safety
    mutable std::mutex robot_state_mutex_;
    mutable std::mutex timeInterval_mutex_;
    mutable std::mutex laser_data_mutex_;
    mutable std::mutex path_goal_mutex_;
    mutable std::mutex obstacle_mutex_;

    // ROS2 Subscriptions
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr force_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_update_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr velocity_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr array_dt_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr params_sub_;
    rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr global_path_clt_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_costmaps_clt_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr robot_volume_sub_;

    // ROS2 Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr repulsion_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr laser_points_pub_;

    // Nav2 Costmap
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

    // TF：按时间戳查 odom→base_link 的缓冲与监听(在构造时初始化)。
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Internal state
    TuningParams tuning_params_{};
    std::vector<double> global_goal;
    std::vector<double> local_goal;

    std::vector<std::vector<double>> laserData_odom;
    std::vector<std::vector<double>> map;

    PoseState robot_state;

    rclcpp::Time normal_to_low_time;
    bool normal_to_low_active = false;
    rclcpp::Time low_to_normal_time;
    bool low_to_normal_active = false;
    rclcpp::Time low_to_brake_time;
    bool low_to_brake_active = false;
    // RECOVER→CAUTIOUS 退出迟滞：实测 |vx| ≥ 0.2×bridge 持续 2s = 真的跑起来了(脱困成功)。
    rclcpp::Time recover_fast_time;
    bool recover_fast_active = false;
    // 速度状态机：上一次看到的状态(检测"刚进入 NORMAL"，复位迟滞计时、避免用残留旧
    // 时间戳一帧误切 CAUTIOUS)；以及进入 NORMAL 的时刻(起步加速宽限期锚点)。
    RobotState last_speed_machine_state_ = INIT;
    rclcpp::Time normal_enter_time;



    static constexpr double ROBOT_LENGTH = 0.72;
    static constexpr double ROBOT_WIDTH = 0.36;
    static constexpr double POINT_MASS_LENGTH = 0.02;
    static constexpr double POINT_MASS_WIDTH = 0.02;
};

#endif // DYNAMICS_PLANNER_NAV_GO2_HPP
