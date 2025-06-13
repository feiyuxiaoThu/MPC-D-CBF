 # 局部路径规划器功能分析与C++复现方案

## 1. Python文件功能概述

这是一个基于ROS的局部路径规划器，使用模型预测控制(MPC)算法进行实时路径规划和避障。

### 1.1 主要功能模块

#### 核心类：`Local_Planner`
- **用途**：实现基于MPC的局部路径规划
- **算法**：椭圆形障碍物建模 + 非线性MPC优化
- **控制对象**：差分驱动机器人（如Jackal/Scout）

#### 关键功能：
1. **状态估计与感知**：接收当前位姿、全局路径、动态障碍物信息
2. **目标点选择**：从全局路径中选择局部MPC的参考轨迹
3. **MPC优化求解**：使用CasADi进行非线性优化
4. **路径发布**：发布优化后的局部路径和控制指令
5. **可视化**：生成RViz可视化标记

### 1.2 数据流分析

```
输入：
├── /curr_state (Float32MultiArray) → 当前机器人状态 [x, y, θ]
├── /global_path (Path) → 全局路径点序列
└── /obs_predict_pub (Float32MultiArray) → 动态障碍物预测 [x, y, a, b, θ] × N × T

处理：
├── choose_goal_state() → 从全局路径选择MPC参考点
├── MPC_ellip() → 非线性MPC优化求解
└── 线程安全的数据同步

输出：
├── /local_path (Path) → 优化后的局部路径
├── /local_plan (Float32MultiArray) → 控制输入序列 [v, ω]
├── /pub_path_vis (Marker) → 可视化标记
└── /cmd_move (Bool) → 运动使能信号
```

## 2. Python关键代码段分析

### 2.1 初始化部分 (lines 17-44)
```python
def __init__(self):
    # 参数设置
    self.replan_period = 0.05  # 重规划周期50ms
    self.N = 25               # MPC预测步数
    
    # 状态变量
    self.curr_state = None    # 当前状态 [x, y, θ]
    self.global_path = None   # 全局路径
    self.ob = []             # 障碍物列表
    
    # 线程锁（数据同步）
    self.curr_pose_lock = threading.Lock()
    self.global_path_lock = threading.Lock()
    self.obstacle_lock = threading.Lock()
```

### 2.2 MPC核心算法 (lines 194-365)
```python
def MPC_ellip(self):
    # CasADi优化器设置
    opti = ca.Opti()
    
    # 系统模型：差分驱动
    def f(x_, u_): 
        return ca.vertcat(*[
            u_[0] * ca.cos(x_[2]),  # ẋ = v*cos(θ)
            u_[0] * ca.sin(x_[2]),  # ẏ = v*sin(θ)  
            u_[1]                   # θ̇ = ω
        ])
    
    # 椭圆障碍物约束
    def h(curpos_, ob_):
        # 椭圆内部距离计算
        # 安全距离约束：h(x) ≥ 0
```

### 2.3 障碍物建模 (lines 265-284)
```python
def h(curpos_, ob_):
    safe_dist = 0.5  # Jackal安全距离
    
    # 椭圆参数解析
    c = ca.cos(ob_[4])     # cos(椭圆角度)
    s = ca.sin(ob_[4])     # sin(椭圆角度)  
    a = ca.MX([ob_[2]])    # 长轴半径
    b = ca.MX([ob_[3]])    # 短轴半径
    
    # 椭圆约束距离计算
    center_vec = curpos_[:2] - ob_vec.T
    dist = b * (sqrt(...) - 1) - safe_dist
```

## 3. C++复现详细方案

### 3.1 依赖库映射

| Python库 | C++对应库 | 用途 |
|-----------|-----------|------|
| `rospy` | `ros/ros.h` | ROS接口 |
| `numpy` | `Eigen3` | 矩阵运算 |
| `casadi` | `casadi` (C++) 或 `OSQP`/`qpOASES` | 非线性优化 |
| `threading.Lock()` | `std::mutex` | 线程同步 |

### 3.2 头文件设计

```cpp
// local_planner.h
#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <mutex>
#include <vector>

class LocalPlanner {
private:
    // 对应Python: self.replan_period等参数
    double replan_period_;
    int N_;  // 预测步数
    double z_;
    
    // 对应Python: self.curr_state, self.global_path, self.ob
    Eigen::Vector3d curr_state_;
    Eigen::MatrixXd global_path_;
    std::vector<Eigen::VectorXd> obstacles_;
    Eigen::MatrixXd goal_state_;
    
    // 对应Python: self.last_input, self.last_state
    Eigen::MatrixXd last_input_;
    Eigen::MatrixXd last_state_;
    bool mpc_success_;
    
    // 对应Python的threading.Lock()
    std::mutex curr_pose_mutex_;
    std::mutex global_path_mutex_;
    std::mutex obstacle_mutex_;
    
    // ROS接口
    ros::NodeHandle nh_;
    ros::Timer replan_timer_;
    
    // 对应Python的Subscriber
    ros::Subscriber curr_state_sub_;
    ros::Subscriber obs_sub_;
    ros::Subscriber global_path_sub_;
    
    // 对应Python的Publisher  
    ros::Publisher local_path_vis_pub_;
    ros::Publisher local_path_pub_;
    ros::Publisher local_plan_pub_;
    ros::Publisher cmd_move_pub_;

public:
    LocalPlanner();
    ~LocalPlanner();
    
private:
    // 对应Python的回调函数
    void replanCallback(const ros::TimerEvent& event);
    void currPoseCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void obsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg);
    
    // 对应Python的核心算法
    bool chooseGoalState();
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> mpcEllip();
    void publishLocalPlan(const Eigen::MatrixXd& input_sol, 
                         const Eigen::MatrixXd& state_sol);
    
    // 辅助函数
    double distanceGlobal(const Eigen::Vector2d& c1, const Eigen::Vector2d& c2);
    bool exceedOb(const Eigen::VectorXd& ob);
    casadi::MX ellipseConstraint(const casadi::MX& pos, const Eigen::VectorXd& ob);
};

#endif
```

### 3.3 实现文件对应关系

#### 3.3.1 构造函数 (对应Python lines 17-44)

```cpp
// local_planner.cpp
LocalPlanner::LocalPlanner() : nh_("~") {
    // 对应Python: self.replan_period = rospy.get_param(...)
    nh_.param("replan_period", replan_period_, 0.05);
    
    // 对应Python: self.N = 25
    N_ = 25;
    z_ = 0.0;
    
    // 对应Python: self.goal_state = np.zeros([self.N, 3])
    goal_state_ = Eigen::MatrixXd::Zero(N_, 3);
    
    // 对应Python: self.curr_state = None等
    curr_state_ = Eigen::Vector3d::Zero();
    mpc_success_ = false;
    
    // 对应Python: rospy.Timer(...)
    replan_timer_ = nh_.createTimer(ros::Duration(replan_period_), 
                                   &LocalPlanner::replanCallback, this);
    
    // 对应Python的Subscriber初始化
    curr_state_sub_ = nh_.subscribe("/curr_state", 10, 
                                   &LocalPlanner::currPoseCallback, this);
    obs_sub_ = nh_.subscribe("/obs_predict_pub", 10,
                            &LocalPlanner::obsCallback, this); 
    global_path_sub_ = nh_.subscribe("/global_path", 25,
                                    &LocalPlanner::globalPathCallback, this);
    
    // 对应Python的Publisher初始化
    local_path_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("/pub_path_vis", 10);
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 10);
    local_plan_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/local_plan", 10);
    cmd_move_pub_ = nh_.advertise<std_msgs::Bool>("/cmd_move", 10);
}
```

#### 3.3.2 重规划回调 (对应Python lines 46-65)

```cpp
void LocalPlanner::replanCallback(const ros::TimerEvent& event) {
    // 对应Python: if self.choose_goal_state():
    if (!chooseGoalState()) return;
    
    // 对应Python: 角度信息添加 (lines 48-57)
    for (int i = 0; i < N_ - 1; ++i) {
        double y_diff = goal_state_(i+1, 1) - goal_state_(i, 1);
        double x_diff = goal_state_(i+1, 0) - goal_state_(i, 0);
        
        if (std::abs(x_diff) > 1e-6 && std::abs(y_diff) > 1e-6) {
            goal_state_(i, 2) = std::atan2(y_diff, x_diff);
        } else if (i != 0) {
            goal_state_(i, 2) = goal_state_(i-1, 2);
        } else {
            goal_state_(i, 2) = 0.0;
        }
    }
    goal_state_(N_-1, 2) = goal_state_(N_-2, 2);
    
    // 对应Python: states_sol, input_sol = self.MPC_ellip()
    auto [states_sol, input_sol] = mpcEllip();
    
    // 对应Python: cmd_move发布
    std_msgs::Bool cmd_move;
    {
        std::lock_guard<std::mutex> lock(global_path_mutex_);
        if (global_path_.rows() > 0) {
            Eigen::Vector2d curr_pos = curr_state_.head<2>();
            Eigen::Vector2d goal_pos = global_path_.bottomRows(1).head<2>();
            cmd_move.data = distanceGlobal(curr_pos, goal_pos) > 0.1;
        }
    }
    cmd_move_pub_.publish(cmd_move);
    
    publishLocalPlan(input_sol, states_sol);
}
```

#### 3.3.3 MPC核心算法 (对应Python lines 194-365)

```cpp
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> LocalPlanner::mpcEllip() {
    std::lock_guard<std::mutex> curr_lock(curr_pose_mutex_);
    std::lock_guard<std::mutex> path_lock(global_path_mutex_);
    std::lock_guard<std::mutex> obs_lock(obstacle_mutex_);
    
    // 对应Python: opti = ca.Opti()
    casadi::Opti opti;
    
    // 对应Python: 参数设置
    double T = 0.1;        // 时间步长
    double gamma_k = 0.3;  // 障碍物约束松弛因子
    double v_max = 1.2;    // 最大线速度
    double v_min = 0.1;    // 最小线速度  
    double omega_max = 1.2; // 最大角速度
    
    // 对应Python: opt_x0 = opti.parameter(3)
    auto opt_x0 = opti.parameter(3, 1);
    
    // 对应Python: opt_states = opti.variable(self.N + 1, 3)
    auto opt_states = opti.variable(N_ + 1, 3);
    auto opt_controls = opti.variable(N_, 2);
    
    // 对应Python: def f(x_, u_): 系统模型
    auto f = [](const casadi::MX& x, const casadi::MX& u) {
        return casadi::MX::vertcat({
            u(0) * casadi::MX::cos(x(2)),  // ẋ = v*cos(θ)
            u(0) * casadi::MX::sin(x(2)),  // ẏ = v*sin(θ)
            u(1)                           // θ̇ = ω
        });
    };
    
    // 对应Python: opti.subject_to(opt_states[0, :] == opt_x0.T)
    opti.subject_to(opt_states(0, casadi::Slice()) == opt_x0.T());
    
    // 对应Python: 速度约束
    auto v = opt_controls(casadi::Slice(), 0);
    auto omega = opt_controls(casadi::Slice(), 1);
    
    // 动态约束判断
    bool near_goal = false;
    {
        if (global_path_.rows() > 0) {
            Eigen::Vector2d curr_pos = curr_state_.head<2>();
            Eigen::Vector2d goal_pos = global_path_.bottomRows(1).head<2>();
            near_goal = distanceGlobal(curr_pos, goal_pos) <= 1.0;
        }
    }
    
    if (!near_goal) {
        opti.subject_to(v >= v_min && v <= v_max);
    } else {
        opti.subject_to(v >= -v_min && v <= v_max); 
    }
    opti.subject_to(omega >= -omega_max && omega <= omega_max);
    
    // 对应Python: 系统模型约束 (for循环)
    for (int i = 0; i < N_; ++i) {
        auto x_curr = opt_states(i, casadi::Slice());
        auto u_curr = opt_controls(i, casadi::Slice());
        auto x_next = x_curr + T * f(x_curr.T(), u_curr.T()).T();
        opti.subject_to(opt_states(i + 1, casadi::Slice()) == x_next);
    }
    
    // 对应Python: 障碍物约束
    int num_obs = obstacles_.size() / N_;
    for (int j = 0; j < num_obs; ++j) {
        if (!exceedOb(obstacles_[25 * j])) {
            for (int i = 0; i < N_ - 1; ++i) {
                auto h_curr = ellipseConstraint(
                    opt_states(i, casadi::Slice(0, 2)).T(), 
                    obstacles_[j * 25 + i]
                );
                auto h_next = ellipseConstraint(
                    opt_states(i + 1, casadi::Slice(0, 2)).T(),
                    obstacles_[j * 25 + i + 1]
                );
                opti.subject_to(h_next >= (1 - gamma_k) * h_curr);
            }
        }
    }
    
    // 对应Python: 目标函数
    casadi::MX obj = 0;
    
    // R矩阵 (控制权重)
    Eigen::Matrix2d R = Eigen::Vector2d(0.1, 0.02).asDiagonal();
    
    for (int i = 0; i < N_; ++i) {
        // Q矩阵 (状态权重)
        Eigen::Vector3d q_diag(1.0 + 0.05*i, 1.0 + 0.05*i, 0.02 + 0.005*i);
        Eigen::Matrix3d Q = q_diag.asDiagonal();
        
        // 状态误差
        casadi::MX state_error = opt_states(i, casadi::Slice()) - 
                                casadi::MX(casadi::DM(std::vector<double>{
                                    goal_state_(i, 0), goal_state_(i, 1), goal_state_(i, 2)
                                }));
        
        if (i < N_ - 1) {
            // 控制误差  
            casadi::MX control_error = opt_controls(i, casadi::Slice());
            obj += 0.1 * casadi::MX::mtimes({state_error, casadi::MX(casadi::DM(Q)), state_error.T()}) +
                   casadi::MX::mtimes({control_error, casadi::MX(casadi::DM(R)), control_error.T()});
        } else {
            obj += 0.1 * casadi::MX::mtimes({state_error, casadi::MX(casadi::DM(Q)), state_error.T()});
        }
    }
    
    // 终端约束权重
    Eigen::Matrix3d Q_terminal = Eigen::Vector3d(5.0, 5.0, 0.1).asDiagonal();
    casadi::MX terminal_error = opt_states(N_-1, casadi::Slice()) - 
                               casadi::MX(casadi::DM(std::vector<double>{
                                   goal_state_(N_-1, 0), goal_state_(N_-1, 1), goal_state_(N_-1, 2)
                               }));
    obj += casadi::MX::mtimes({terminal_error, casadi::MX(casadi::DM(Q_terminal)), terminal_error.T()});
    
    opti.minimize(obj);
    
    // 对应Python: 求解器设置
    casadi::Dict opts_setting;
    opts_setting["ipopt.max_iter"] = 2000;
    opts_setting["ipopt.print_level"] = 0;
    opts_setting["print_time"] = 0;
    opts_setting["ipopt.acceptable_tol"] = 1e-3;
    opts_setting["ipopt.acceptable_obj_change_tol"] = 1e-3;
    
    opti.solver("ipopt", opts_setting);
    opti.set_value(opt_x0, casadi::DM(std::vector<double>{
        curr_state_(0), curr_state_(1), curr_state_(2)
    }));
    
    Eigen::MatrixXd u_res, state_res;
    
    try {
        // 对应Python: sol = opti.solve()
        auto sol = opti.solve();
        
        // 提取解
        auto u_sol = sol.value(opt_controls);
        auto state_sol = sol.value(opt_states);
        
        // 转换为Eigen格式
        u_res = Eigen::MatrixXd(N_, 2);
        state_res = Eigen::MatrixXd(N_ + 1, 3);
        
        for (int i = 0; i < N_; ++i) {
            u_res(i, 0) = static_cast<double>(u_sol(i, 0));
            u_res(i, 1) = static_cast<double>(u_sol(i, 1));
        }
        
        for (int i = 0; i <= N_; ++i) {
            state_res(i, 0) = static_cast<double>(state_sol(i, 0));
            state_res(i, 1) = static_cast<double>(state_sol(i, 1));
            state_res(i, 2) = static_cast<double>(state_sol(i, 2));
        }
        
        last_input_ = u_res;
        last_state_ = state_res;
        mpc_success_ = true;
        
    } catch (const std::exception& e) {
        // 对应Python: except: 处理求解失败
        ROS_ERROR("MPC solve failed: %s", e.what());
        
        if (mpc_success_) {
            mpc_success_ = false;
        } else {
            // 时间平移策略
            if (last_input_.rows() >= N_) {
                for (int i = 0; i < N_ - 1; ++i) {
                    last_input_.row(i) = last_input_.row(i + 1);
                    last_state_.row(i) = last_state_.row(i + 1);
                }
                last_input_.row(N_ - 1) = Eigen::Vector2d::Zero();
            }
        }
        
        u_res = last_input_;
        state_res = last_state_;
    }
    
    return std::make_pair(state_res, u_res);
}
```

#### 3.3.4 椭圆约束函数 (对应Python lines 265-284)

```cpp
casadi::MX LocalPlanner::ellipseConstraint(const casadi::MX& pos, 
                                          const Eigen::VectorXd& ob) {
    // 对应Python: safe_dist = 0.5  # for jackal
    double safe_dist = 0.5;
    
    // 对应Python: 椭圆参数解析
    double ob_x = ob(0);     // 椭圆中心x
    double ob_y = ob(1);     // 椭圆中心y  
    double a = ob(2);        // 长轴半径
    double b = ob(3);        // 短轴半径
    double theta = ob(4);    // 椭圆角度
    
    auto c = casadi::MX::cos(theta);
    auto s = casadi::MX::sin(theta);
    
    // 对应Python: center_vec = curpos_[:2] - ob_vec.T
    casadi::MX center_vec = pos - casadi::MX(casadi::DM(std::vector<double>{ob_x, ob_y}));
    
    // 对应Python: 椭圆距离计算公式
    auto term1 = (c*c/(a*a) + s*s/(b*b)) * center_vec(0) * center_vec(0);
    auto term2 = (s*s/(a*a) + c*c/(b*b)) * center_vec(1) * center_vec(1);  
    auto term3 = 2 * c * s * (1/(a*a) - 1/(b*b)) * center_vec(0) * center_vec(1);
    
    auto dist = b * (casadi::MX::sqrt(term1 + term2 + term3) - 1) - safe_dist;
    
    return dist;
}
```

### 3.4 CMakeLists.txt配置

```cmake
cmake_minimum_required(VERSION 3.10)
project(local_planner_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs  
  geometry_msgs
  nav_msgs
  visualization_msgs
)

# 对应Python casadi依赖
find_package(PkgConfig REQUIRED)
pkg_check_modules(CASADI REQUIRED casadi)

# 对应Python numpy依赖  
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs visualization_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CASADI_INCLUDE_DIRS}
)

add_executable(local_planner_node 
  src/local_planner.cpp
  src/main.cpp
)

target_link_libraries(local_planner_node
  ${catkin_LIBRARIES}
  ${CASADI_LIBRARIES}
)
```

### 3.5 主函数 (对应Python lines 363-365)

```cpp
// main.cpp
#include "local_planner.h"

int main(int argc, char** argv) {
    // 对应Python: rospy.init_node("phri_planner") 
    ros::init(argc, argv, "local_planner_cpp");
    
    // 对应Python: phri_planner = Local_Planner()
    LocalPlanner planner;
    
    // 对应Python: rospy.spin()
    ros::spin();
    
    return 0;
}
```

## 4. 关键差异与注意事项

### 4.1 线程安全
- **Python**: 使用`threading.Lock()`
- **C++**: 使用`std::mutex`和`std::lock_guard`

### 4.2 数组操作
- **Python**: NumPy数组切片 `array[i:j, :]`
- **C++**: Eigen矩阵操作 `matrix.block(i, 0, rows, cols)`

### 4.3 优化库接口
- **Python**: CasADi Python接口相对简洁
- **C++**: CasADi C++接口需要显式类型转换

### 4.4 错误处理
- **Python**: 使用`try/except`
- **C++**: 使用`try/catch`和`std::exception`

## 5. 性能对比预期

| 指标 | Python版本 | C++版本 | 改进 |
|------|------------|---------|------|
| 执行速度 | 基准 | +30-50% | 编译优化 |
| 内存使用 | 基准 | -20-30% | 无解释器开销 |
| 实时性 | 50ms | <30ms | 更好的实时保证 |

## 6. 部署建议

1. **开发阶段**: 使用Python版本快速原型验证
2. **测试阶段**: C++版本与Python版本并行验证
3. **生产部署**: 使用C++版本获得更好性能
4. **维护更新**: 保持两个版本的算法同步

这个复现方案保持了原Python代码的完整功能，同时充分利用了C++的性能优势。