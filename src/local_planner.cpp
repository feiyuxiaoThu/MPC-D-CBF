#include "local_planner.h"
#include <algorithm>
#include <cmath>
#include <vector>
#include <fstream>
#include <iomanip>






void LocalPlanner::saveDataForVisualization2D() const {
    // 1. 保存 last_state_ (MPC 预测轨迹)
    std::ofstream mpc_file("../plot/2D/mpc_trajectory.txt");
    if (mpc_file.is_open()) {
        for (int i = 0; i < last_state_.rows(); ++i) {
            mpc_file << std::fixed << std::setprecision(5) << last_state_(i, 0) << " " << last_state_(i, 1) << std::endl;
        }
        mpc_file.close();
    }

    // 2. 保存 goal_state_ (参考轨迹)
    std::ofstream goal_file("../plot/2D/reference_trajectory.txt");
    if (goal_file.is_open()) {
        for (int i = 0; i < goal_state_.rows(); ++i) {
            goal_file << std::fixed << std::setprecision(5) << goal_state_(i, 0) << " " << goal_state_(i, 1) << std::endl;
        }
        goal_file.close();
    }

    // 3. 保存 global_path_ (全局路径)
    std::ofstream global_path_file("../plot/2D/global_path.txt");
    if (global_path_file.is_open()) {
        for (int i = 0; i < global_path_.rows(); ++i) {
            global_path_file << std::fixed << std::setprecision(5) << global_path_(i, 0) << " " << global_path_(i, 1) << std::endl;
        }
        global_path_file.close();
    }

    // 4. 保存 obstacles_ (所有时间步的障碍物)
    std::ofstream obs_file("../plot/2D/obstacles.txt");
    if (obs_file.is_open()) {
        if (!obstacles_.empty()) {
            for (const auto& ob : obstacles_) {
                for (int k = 0; k < ob.size(); ++k) {
                    obs_file << std::fixed << std::setprecision(5) << ob(k) << (k == ob.size() - 1 ? "" : " ");
                }
                obs_file << std::endl;
            }
        }
        obs_file.close();
    }

    // 5. 保存配置参数
    std::ofstream config_file("../plot/2D/config.txt");
    if (config_file.is_open()) {
        config_file << "N " << N_ << std::endl;
        config_file.close();
    }

    std::cout << "Visualization data2D saved to files." << std::endl;
}

// 对应Python: def distance_global(c1, c2)
double LocalPlanner::distanceGlobal(const Eigen::Vector2d& c1, const Eigen::Vector2d& c2) {
    return std::sqrt((c1(0) - c2(0)) * (c1(0) - c2(0)) + (c1(1) - c2(1)) * (c1(1) - c2(1)));
}

// 角度归一化到[-π, π]范围
double LocalPlanner::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// 计算两个角度之间的最小差值
double LocalPlanner::angleDifference(double angle1, double angle2) {
    double diff = angle1 - angle2;
    return normalizeAngle(diff);
}

// 对应Python: __init__函数 (lines 17-44)
LocalPlanner::LocalPlanner() {
    // 对应Python: self.replan_period = rospy.get_param('/local_planner/replan_period', 0.05)
    //nh_.param("/local_planner/replan_period", replan_period_, 0.05);
    
    // 对应Python: self.N = 25, self.z = 0
    N_ = 80; // MPC预测步数
    z_ = 0.0;
    replan_period_ = 0.1;
    L_ = 2.5; // 车辆轴距 (m)
    
    // 对应Python: self.goal_state = np.zeros([self.N, 3])
    goal_state_ = Eigen::MatrixXd::Zero(N_, 4);
    
    // 对应Python: self.curr_state = None等初始化
    curr_state_ = Eigen::Vector4d::Zero();
    mpc_success_ = false;
    curr_state_received_ = true;
    global_path_received_ = true;
    use_vo_cbf_ = true; // 激活 CBF/VO 约束
    use_vo_constraint_ = false; // 默认激活VO约束
    w_vo_slack_ = 1.0;      // VO松弛变量的默认权重
    w_track_ = 5000.0;           // 轨迹跟踪误差的默认权重
    w_a_rate_ = 2.0;          // 加速度变化率(jerk)的默认权重
    w_delta_rate_ = 3.0;      // 转角变化率的默认权重

    currPoseCallback(); // curr_state_
    obsCallback();
    globalPathCallback();

     // 初始化本车位置、障碍物和目标的参考轨迹
     /*
    curr_state_sub_ = nh_.subscribe("/curr_state", 10, 
                                   &LocalPlanner::currPoseCallback, this);
    obs_sub_ = nh_.subscribe("/obs_predict_pub", 10,
                            &LocalPlanner::obsCallback, this); 
    global_path_sub_ = nh_.subscribe("/global_path", 25,
                                    &LocalPlanner::globalPathCallback, this);
    */
    // 对应Python: self.last_input = [], self.last_state = []
    last_input_ = Eigen::MatrixXd::Zero(N_, 2);
    last_state_ = Eigen::MatrixXd::Zero(N_ + 1, 4);
    
    //ROS_INFO("Local Planner initialized with N=%d, replan_period=%.3f", N_, replan_period_);
}

LocalPlanner::~LocalPlanner() {}

// 对应Python: __replan_cb函数 (lines 46-65)
void LocalPlanner::replanCallback() {
    // 对应Python: if self.choose_goal_state():
    std::cout <<"Replan!!!"<<std::endl;
    static int call_count = 0;
    call_count++;
    //ROS_INFO("replanCallback called #%d at time %.3f", call_count, ros::Time::now().toSec());
    
    if (!chooseGoalState()) return;
    
    std::cout << "chooseGoalState success, about to call mpcEllip()" << std::endl;
    
    // 对应Python: 角度信息添加 (lines 48-57) - 修复角度跳变问题
    for (int i = 0; i < N_ - 1; ++i) {
        double y_diff = goal_state_(i+1, 1) - goal_state_(i, 1);
        double x_diff = goal_state_(i+1, 0) - goal_state_(i, 0);
        
        if (std::abs(x_diff) > 1e-6 && std::abs(y_diff) > 1e-6) {
            double raw_angle = std::atan2(y_diff, x_diff);
            
            // 第一个点：如果与当前状态角度差太大，调整到最近的等效角度
            if (i == 0) {
                double current_angle = curr_state_(2);
                double angle_diff = angleDifference(raw_angle, current_angle);
                // 如果角度差超过π/2，选择最接近当前角度的等效角度
                if (std::abs(angle_diff) > M_PI/2) {
                    // 尝试加减2π找到最接近的角度
                    double alt_angle1 = raw_angle + 2.0 * M_PI;
                    double alt_angle2 = raw_angle - 2.0 * M_PI;
                    
                    double diff1 = std::abs(angleDifference(alt_angle1, current_angle));
                    double diff2 = std::abs(angleDifference(alt_angle2, current_angle));
                    double diff_orig = std::abs(angle_diff);
                    
                    if (diff1 < diff_orig && diff1 < diff2) {
                        raw_angle = alt_angle1;
                    } else if (diff2 < diff_orig && diff2 < diff1) {
                        raw_angle = alt_angle2;
                    }
                }
            }
            
            goal_state_(i, 2) = raw_angle;
        } else if (i != 0) {
            goal_state_(i, 2) = goal_state_(i-1, 2);
        } else {
            // 如果第一个点没有明显方向，使用当前状态角度
            goal_state_(i, 2) = curr_state_(2);
        }
    }
    goal_state_(N_-1, 2) = goal_state_(N_-2, 2);
    
    // 归一化所有角度
    for (int i = 0; i < N_; ++i) {
        goal_state_(i, 2) = normalizeAngle(goal_state_(i, 2));
    }
    
    // 对应Python: states_sol, input_sol = self.MPC_ellip()
    auto [states_sol, input_sol] = mpcEllip();

    std::cout << "mpcEllip() completed successfully and the solution state is" << std::endl;
    std::cout << states_sol << std::endl;
    std::cout << "the input solution is" << std::endl;
    std::cout << input_sol ;

    //ROS_INFO("mpcEllip() completed successfully");
    
    
    
    // 对应Python: cmd_move发布 (lines 61-63)
    // bool cmd_move;
    // {
    //     std::lock_guard<std::mutex> lock(global_path_mutex_);
    //     if (global_path_received_ && global_path_.rows() > 0) {
    //         Eigen::Vector2d curr_pos = curr_state_.head<2>();
    //         Eigen::Vector2d goal_pos = global_path_.bottomRows(1).leftCols(2).transpose();
    //         cmd_move = distanceGlobal(curr_pos, goal_pos) > 0.1;
    //     } else {
    //         cmd_move = false;
    //     }
    // }
   
    //ROS_INFO("start local_plan_pub_ published");
    //publishLocalPlan(input_sol, states_sol);
}

//  接受本车初始轨迹，可以假设为 0 0
void LocalPlanner::currPoseCallback() {
    std::lock_guard<std::mutex> lock(curr_pose_mutex_);
    bool use_ego_cor = true;
    if (use_ego_cor) {
        // 起始状态: 位于(0,0), 速度为3m/s, 方向朝上(π/2)
        curr_state_ << 0.0, 0.0, M_PI / 2.0, 8.0;
        curr_state_received_ = true;
    }
}

// 对应Python: __obs_cb函数 (lines 75-81)
void LocalPlanner::obsCallback() {
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    obstacles_.clear();

    // --- 障碍物 1: 位于初始路径上的静态障碍物 ---
    Eigen::VectorXd static_ob(5);
    static_ob << -12.0, 20.0, 1.0, 1.0, 0.0; // 位置(0, 8), 半径 1.0m
    for(int i = 0; i < N_; i++){
        obstacles_.push_back(static_ob);
    }

    // --- 障碍物 2: 从左向右穿行的动态障碍物 ---
    Eigen::VectorXd dynamic_ob_initial(5);
    dynamic_ob_initial << -20.0, 12.0, 1.5, 0.8, 0.0; // 初始位置(-10, 12), 尺寸(1.5, 0.8), 方向朝右
    
    double obs_vx = 2.0; // X方向速度

    // 生成动态障碍物的预测轨迹
    for(int i = 0; i < N_; i++){
        Eigen::VectorXd pred_ob = dynamic_ob_initial;
        // 根据速度和时间步更新X坐标
        pred_ob(0) = dynamic_ob_initial(0) + obs_vx * (i * replan_period_);
        obstacles_.push_back(pred_ob);
    }
}

// 对应Python: __global_path_cb函数 (lines 83-90)
// 接受参考线轨迹
void LocalPlanner::globalPathCallback() {
    std::lock_guard<std::mutex> lock(global_path_mutex_);

    std::vector<Eigen::Vector4d> path_points;
    const double target_v = 8.0; // m/s
    const double R = 10.0;       // 转弯半径 (m)
    const double final_x = -20.0;
    const double final_y = 20.0;

    // 路径点分布
    const int num_points_seg1 = 20; // 第一段直线
    const int num_points_seg2 = 50; // 第二段圆弧
    const int num_points_seg3 = 20; // 第三段直线

    // --- 段 1: 沿Y轴直行 ---
    // 从 (0,0) 到 (0, 10)
    double start_y_seg1 = 0.0;
    double end_y_seg1 = final_y - R;
    for (int i = 0; i <= num_points_seg1; ++i) {
        double y = start_y_seg1 + ((double)i / num_points_seg1) * (end_y_seg1 - start_y_seg1);
        path_points.push_back(Eigen::Vector4d(0.0, y, M_PI / 2.0, target_v));
    }

    // --- 段 2: 90度圆弧左转 ---
    // 从 (0, 10) 到 (-10, 20)
    // 圆心为 (-10, 10)
    Eigen::Vector2d center(-R, final_y - R);
    for (int i = 1; i <= num_points_seg2; ++i) {
        double phi = (double)i / num_points_seg2 * (M_PI / 2.0); // 角度从0到π/2
        double x = center(0) + R * std::cos(phi);
        double y = center(1) + R * std::sin(phi);
        double theta = M_PI / 2.0 + phi;
        path_points.push_back(Eigen::Vector4d(x, y, normalizeAngle(theta), target_v));
    }

    // --- 段 3: 沿X轴负方向直行 ---
    // 从 (-10, 20) 到 (-20, 20)
    double start_x_seg3 = -R;
    double end_x_seg3 = final_x;
    for (int i = 1; i <= num_points_seg3; ++i) {
        double x = start_x_seg3 + ((double)i / num_points_seg3) * (end_x_seg3 - start_x_seg3);
        path_points.push_back(Eigen::Vector4d(x, final_y, M_PI, target_v));
    }

    // 将路径点复制到成员变量
    if (!path_points.empty()) {
        global_path_ = Eigen::MatrixXd(path_points.size(), 4);
        for(size_t i = 0; i < path_points.size(); ++i) {
            global_path_.row(i) = path_points[i];
        }
        global_path_received_ = true;
    }
}

// 对应Python: choose_goal_state函数 (lines 154-175)
bool LocalPlanner::chooseGoalState() {
    std::lock_guard<std::mutex> curr_lock(curr_pose_mutex_);
    std::lock_guard<std::mutex> path_lock(global_path_mutex_);
    std::cout << "we begin choosegoalstae" << global_path_.rows() << std::endl;
    
    if (!global_path_received_ || !curr_state_received_ || global_path_.rows() == 0) {
        return false;
    }

    int waypoint_num = global_path_.rows();
    
    // 对应Python: num = np.argmin(...)
    double min_dist = std::numeric_limits<double>::max();
    int num = 0;
    for (int i = 0; i < waypoint_num; ++i) {
        Eigen::Vector2d waypoint = global_path_.row(i).head<2>();
        double dist = distanceGlobal(curr_state_.head<2>(), waypoint);
        if (dist < min_dist) {
            min_dist = dist;
            num = i;
        }
    }

    // 对应Python: scale = 1, num_list生成
    int scale = 1;
    std::vector<int> num_list;
    for (int i = 0; i < N_; ++i) {
        int num_path = std::min(waypoint_num - 1, num + i * scale);
        num_list.push_back(num_path);
    }

    // 对应Python: for k in range(self.N): self.goal_state[k] = ...
    for (int k = 0; k < N_; ++k) {
        goal_state_.row(k) = global_path_.row(num_list[k]);
    }

    return true;
}

// 对应Python中的系统模型函数f (在MPC_ellip中定义)
casadi::MX LocalPlanner::systemModel(const casadi::MX& x, const casadi::MX& u) {
    casadi::MX v = x(3);
    casadi::MX a = u(0);
    casadi::MX delta = u(1);

    return casadi::MX::vertcat(std::vector<casadi::MX>{
        v * casadi::MX::cos(x(2)),      // ẋ = v*cos(θ)
        v * casadi::MX::sin(x(2)),      // ẏ = v*sin(θ)
        v / L_ * casadi::MX::tan(delta),// θ̇ = v/L*tan(δ)
        a                               // v̇ = a
    });
}

// 对应Python中的椭圆约束函数h (lines 265-284)
casadi::MX LocalPlanner::ellipseConstraint(const casadi::MX& pos, const Eigen::VectorXd& ob) {
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
    casadi::MX ob_vec = casadi::MX(casadi::DM({ob_x, ob_y}));
    casadi::MX center_vec = pos - ob_vec;
    
    // 对应Python: 椭圆距离计算公式
    auto term1 = (c*c/(a*a) + s*s/(b*b)) * center_vec(0) * center_vec(0);
    auto term2 = (s*s/(a*a) + c*c/(b*b)) * center_vec(1) * center_vec(1);  
    auto term3 = 2 * c * s * (1/(a*a) - 1/(b*b)) * center_vec(0) * center_vec(1);
    
    auto dist = b * (casadi::MX::sqrt(term1 + term2 + term3) - 1) - safe_dist;
    
    return dist;
}

// 对应Python中的quadratic函数 (lines 287)
casadi::MX LocalPlanner::quadratic(const casadi::MX& x, const Eigen::MatrixXd& A) {
    // Convert Eigen::MatrixXd to casadi::DM by converting to std::vector first
    std::vector<double> A_data;
    A_data.reserve(A.rows() * A.cols());
    for (int i = 0; i < A.rows(); ++i) {
        for (int j = 0; j < A.cols(); ++j) {
            A_data.push_back(A(i, j));
        }
    }
    casadi::DM A_dm = casadi::DM::reshape(casadi::DM(A_data), A.rows(), A.cols());
    casadi::MX A_mx = casadi::MX(A_dm);
    return casadi::MX::mtimes({x, A_mx, x.T()});
}

// 对应Python中的exceed_ob函数 (lines 235-263)
// Note: This function should be called when locks are already held
bool LocalPlanner::exceedOb(const Eigen::VectorXd& ob) {
    // Remove the lock since locks are already held in calling function
    if (!global_path_received_ || global_path_.rows() == 0) return true;
    
    //ROS_INFO("exceedOb: checking obstacle [%.3f, %.3f, %.3f, %.3f, %.3f]", 
    //         ob(0), ob(1), ob(2), ob(3), ob(4));
    
    double l_long_axis = ob(2);
    double l_short_axis = ob(3);
    
    // Add safety check for axis lengths
    if (l_long_axis <= 0 || l_short_axis <= 0) {
        //ROS_WARN("exceedOb: invalid axis lengths, l_long=%.3f, l_short=%.3f", l_long_axis, l_short_axis);
        return true;
    }
    
    Eigen::Vector2d long_axis(std::cos(ob(4)) * l_long_axis, std::sin(ob(4)) * l_long_axis);

    Eigen::Vector2d ob_vec(ob(0), ob(1));
    Eigen::Vector2d center_vec = goal_state_.row(N_-1).head<2>().transpose() - ob_vec;
    double dist_center = center_vec.norm();
    
    //ROS_INFO("exceedOb: dist_center=%.3f", dist_center);
    
    // Add safety check for zero distance
    if (dist_center < 1e-6) {
        //ROS_WARN("exceedOb: dist_center too small: %.6f", dist_center);
        return false; // Obstacle at goal position
    }
    
    double cos_ = center_vec.dot(long_axis) / (dist_center * l_long_axis);
    //ROS_INFO("exceedOb: cos_=%.3f", cos_);

    double d;
    if (std::abs(cos_) > 0.1) {
        double tan_square = 1.0 / (cos_ * cos_) - 1.0;
        // if (tan_square < 0) {
        //     ROS_WARN("exceedOb: negative tan_square=%.3f", tan_square);
        //     tan_square = 0;
        // }
        d = std::sqrt((l_long_axis * l_long_axis * l_short_axis * l_short_axis * (1 + tan_square)) / 
                     (l_short_axis * l_short_axis + l_long_axis * l_long_axis * tan_square));
    } else {
        d = l_short_axis;
    }
    
    //ROS_INFO("exceedOb: calculated d=%.3f", d);

    Eigen::Vector2d cross_pt = ob_vec + d * center_vec / dist_center;
    Eigen::Vector2d vec1 = goal_state_.row(N_-1).head<2>().transpose() - cross_pt;
    Eigen::Vector2d vec2 = curr_state_.head<2>() - cross_pt;
    double theta = vec1.dot(vec2);
    
    bool result = theta > 0;
    //ROS_INFO("exceedOb: theta=%.3f, result=%s", theta, result ? "true" : "false");

    return result;
}

// 对应Python: MPC_ellip函数 (lines 177-365) - 核心MPC算法
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> LocalPlanner::mpcEllip() {
    //ROS_INFO("=== mpcEllip() STARTED ===");
    
    std::lock_guard<std::mutex> curr_lock(curr_pose_mutex_);
    std::lock_guard<std::mutex> path_lock(global_path_mutex_);
    std::lock_guard<std::mutex> obs_lock(obstacle_mutex_);
    
    //ROS_INFO("Acquired all locks, checking data status");
    //ROS_INFO("curr_state_received_: %s", curr_state_received_ ? "true" : "false");
    //ROS_INFO("global_path_received_: %s", global_path_received_ ? "true" : "false");
    //ROS_INFO("curr_state_: [%.3f, %.3f, %.3f]", curr_state_(0), curr_state_(1), curr_state_(2));
    //ROS_INFO("goal_state_ dimensions: %ld x %ld", goal_state_.rows(), goal_state_.cols());
    //ROS_INFO("obstacles_ count: %zu", obstacles_.size());
    
    if (!curr_state_received_) {
        //ROS_ERROR("Current state not received, cannot execute MPC");
        return std::make_pair(last_state_, last_input_);
    }
    
    if (!global_path_received_ || goal_state_.rows() != N_) {
        //ROS_ERROR("Global path not received or goal_state dimension error, goal_state_.rows()=%ld, N_=%d", 
       //           goal_state_.rows(), N_);
        return std::make_pair(last_state_, last_input_);
    }
    
    // 对应Python: opti = ca.Opti()
    casadi::Opti opti;
    //ROS_INFO("Created casadi optimizer");
    
    // 对应Python: 参数设置 (lines 182-188)
    double T = 0.1;        // 时间步长
    double gamma_k = 0.3;  // 障碍物约束松弛因子
    double v_max = 10.0;    // 最大线速度
    double v_min = 0.0;    // 最小线速度  
    double a_max = 2.0;    // 最大加速度
    double a_min = -3.0;   // 最大减速度
    double delta_max = M_PI / 4; // 最大前轮转角
    
    //ROS_INFO("MPC parameters: T=%.3f, gamma_k=%.3f, v_max=%.3f, v_min=%.3f, omega_max=%.3f", 
    //         T, gamma_k, v_max, v_min, omega_max);
    
    try {
        // 对应Python: opt_x0 = opti.parameter(3)
        auto opt_x0 = opti.parameter(4, 1);
        //ROS_INFO("Created initial state parameter opt_x0");
        
        // 对应Python: opt_states = opti.variable(self.N + 1, 3)
        auto opt_states = opti.variable(N_ + 1, 4);
        auto opt_controls = opti.variable(N_, 2);
        //ROS_INFO("Created state variables opt_states(%d x 3) and control variables opt_controls(%d x 2)", N_+1, N_);
        
        // 对应Python: v = opt_controls[:, 0], omega = opt_controls[:, 1]
        auto a = opt_controls(casadi::Slice(), 0);
        auto delta = opt_controls(casadi::Slice(), 1);
        auto v = opt_states(casadi::Slice(0, N_), 3);
        //ROS_INFO("Extracted velocity and angular velocity control variables");
        
        // 对应Python: opti.subject_to(opt_states[0, :] == opt_x0.T) (line 291)
        opti.subject_to(opt_states(0, casadi::Slice()) == opt_x0.T());
        //ROS_INFO("Added initial state constraint");
        
        // 对应Python: 速度约束 (lines 293-296)
        bool near_goal = false;
        if (global_path_received_ && global_path_.rows() > 0) {
            Eigen::Vector2d curr_pos = curr_state_.head<2>();
            Eigen::Vector2d goal_pos = global_path_.bottomRows(1).leftCols(2).transpose();
            double dist_to_goal = distanceGlobal(curr_pos, goal_pos);
            near_goal = dist_to_goal <= 1.0;
            //ROS_INFO("Distance to goal: %.3f, near goal: %s", dist_to_goal, near_goal ? "yes" : "no");
        }
        
        // 状态和控制约束
        opti.subject_to(v >= v_min);
        opti.subject_to(v <= v_max);
        opti.subject_to(a >= a_min);
        opti.subject_to(a <= a_max);
        opti.subject_to(delta >= -delta_max);
        opti.subject_to(delta <= delta_max);
        //ROS_INFO("Added state and control constraints");
        
        // 对应Python: 系统模型约束 (lines 299-301)
        //ROS_INFO("Starting to add system model constraints");
        for (int i = 0; i < N_; ++i) {
            auto x_curr = opt_states(i, casadi::Slice());
            auto u_curr = opt_controls(i, casadi::Slice());
            auto x_next = x_curr + T * systemModel(x_curr.T(), u_curr.T()).T();
            opti.subject_to(opt_states(i + 1, casadi::Slice()) == x_next);
            
            if (i < 3) { // Only print first few to avoid log spam
                //ROS_INFO("Added system model constraint for step %d", i);
            }
        }
        //ROS_INFO("Completed all %d system model constraints", N_);
        casadi::MX obj = 0;

        // obstacles_.clear();

        if (!use_vo_cbf_) {
            // ------------------ 原始椭圆约束 (BEGIN) ------------------
            int num_obs = 0;
            if (!obstacles_.empty()) {
                double slack_weight = 1000.0;
                if (obstacles_.size() >= static_cast<size_t>(N_)) {
                    num_obs = obstacles_.size() / N_;
                    for (int j = 0; j < num_obs; ++j) {
                        int base_idx = j * N_;
                        if (base_idx < static_cast<int>(obstacles_.size())) {
                            if (!exceedOb(obstacles_[base_idx])) {
                                auto slack_vars = opti.variable(N_ - 1);
                                opti.subject_to(slack_vars >= 0);
                                casadi::MX slack_penalty = 0;
                                for (int i = 0; i < N_ - 1; ++i) {
                                    int idx1 = base_idx + i;
                                    int idx2 = base_idx + i + 1;
                                    if (idx1 < static_cast<int>(obstacles_.size()) && 
                                        idx2 < static_cast<int>(obstacles_.size())) {
                                        auto h_curr = ellipseConstraint(
                                            opt_states(i, casadi::Slice(0, 2)).T(), 
                                            obstacles_[idx1]
                                        );
                                        auto h_next = ellipseConstraint(
                                            opt_states(i + 1, casadi::Slice(0, 2)).T(),
                                            obstacles_[idx2]
                                        );
                                        opti.subject_to(h_next + slack_vars(i) >= (1 - gamma_k) * h_curr);
                                        slack_penalty = slack_penalty + slack_vars(i) * slack_vars(i);
                                    } else {
                                        break;
                                    }
                                }
                                obj = obj + slack_weight * slack_penalty;
                            }
                        }
                    }
                }
            }
            // ------------------ 原始椭圆约束 (END) ------------------
        } else {
            // ------------------ CBF/VO 约束 (BEGIN) ------------------
            
            double k_cbf = 0.5;      // CBF 增益
            double k_vo = 1.0;       // VO 增益
            // double w_slack = 100.0; // VO 松弛变量权重 - 改为可调参数 w_vo_slack_
            double ego_radius = 0.5; // 自车半径

            int num_obs_sequences = obstacles_.empty() ? 0 : obstacles_.size() / N_;

            for (int j = 0; j < num_obs_sequences; ++j) {
                casadi::MX slack_vo;
                if (use_vo_constraint_) {
                    slack_vo = opti.variable(N_);
                    opti.subject_to(slack_vo >= 0);
                    obj += w_vo_slack_ * casadi::MX::sumsqr(slack_vo);
                }

                // 通过比较连续两个时间步的位置来动态推断障碍物速度
                Eigen::VectorXd obs_t0 = obstacles_[j * N_];
                Eigen::VectorXd obs_t1 = obstacles_[j * N_ + 1];
                double obs_vx = (obs_t1(0) - obs_t0(0)) / replan_period_;
                double obs_vy = (obs_t1(1) - obs_t0(1)) / replan_period_;
                casadi::MX obs_vel = casadi::MX::vertcat({obs_vx, obs_vy});

                for (int i = 0; i < N_ -1; ++i) {
                    // -- 状态提取 --
                    auto ego_state_i = opt_states(i, casadi::Slice()).T();
                    auto ego_state_i1 = opt_states(i + 1, casadi::Slice()).T();

                    auto ego_pos_i = ego_state_i(casadi::Slice(0,2));
                    auto ego_pos_i1 = ego_state_i1(casadi::Slice(0,2));
                    auto ego_vel_i = casadi::MX::vertcat({ego_state_i(3) * casadi::MX::cos(ego_state_i(2)), ego_state_i(3) * casadi::MX::sin(ego_state_i(2))});
                    auto ego_vel_i1 = casadi::MX::vertcat({ego_state_i1(3) * casadi::MX::cos(ego_state_i1(2)), ego_state_i1(3) * casadi::MX::sin(ego_state_i1(2))});

                    auto obs_pos_i = casadi::MX::vertcat({obstacles_[j*N_ + i](0), obstacles_[j*N_ + i](1)});
                    auto obs_pos_i1 = casadi::MX::vertcat({obstacles_[j*N_ + i + 1](0), obstacles_[j*N_ + i + 1](1)});
                    double obs_radius = obstacles_[j*N_ + i](2); // 简化为圆形

                    auto p_rel_i = ego_pos_i - obs_pos_i;
                    auto v_rel_i = ego_vel_i - obs_vel;
                    auto p_rel_i1 = ego_pos_i1 - obs_pos_i1;
                    auto v_rel_i1 = ego_vel_i1 - obs_vel;
                    double R_sum_sq = std::pow(ego_radius + obs_radius, 2);

                    // -- CBF 硬约束 (基于刹车距离) --
                    double u_max = -a_min; // 最大减速度值
                    double safe_dist_cbf = 1.0; // 最小安全缓冲距离

                    // h at time i
                    auto dist_i = casadi::MX::sqrt(casadi::MX::sumsqr(p_rel_i));
                    auto n_rel_i = p_rel_i / dist_i;
                    auto radial_vel_i = casadi::MX::mtimes(n_rel_i.T(), v_rel_i);
                    auto h_cbf_i = dist_i - safe_dist_cbf - (radial_vel_i * radial_vel_i) / (2 * u_max);

                    // h at time i+1
                    auto dist_i1 = casadi::MX::sqrt(casadi::MX::sumsqr(p_rel_i1));
                    auto n_rel_i1 = p_rel_i1 / dist_i1;
                    auto radial_vel_i1 = casadi::MX::mtimes(n_rel_i1.T(), v_rel_i1);
                    auto h_cbf_i1 = dist_i1 - safe_dist_cbf - (radial_vel_i1 * radial_vel_i1) / (2 * u_max);

                    opti.subject_to(h_cbf_i1 >= (1 - k_cbf * T) * h_cbf_i);

                    // -- VO 软约束 (可选) --
                    if (use_vo_constraint_) {
                        // h_vo at time i
                        auto p_rel_dot_v_rel_i = casadi::MX::mtimes(p_rel_i.T(), v_rel_i);
                        auto norm_v_rel_i = casadi::MX::sqrt(casadi::MX::sumsqr(v_rel_i));
                        auto sqrt_term_i = casadi::MX::sqrt(casadi::MX::sumsqr(p_rel_i) - R_sum_sq);
                        auto h_vo_i = p_rel_dot_v_rel_i + norm_v_rel_i * sqrt_term_i;

                        // h_vo at time i+1
                        auto p_rel_dot_v_rel_i1 = casadi::MX::mtimes(p_rel_i1.T(), v_rel_i1);
                        auto norm_v_rel_i1 = casadi::MX::sqrt(casadi::MX::sumsqr(v_rel_i1));
                        auto sqrt_term_i1 = casadi::MX::sqrt(casadi::MX::sumsqr(p_rel_i1) - R_sum_sq);
                        auto h_vo_i1 = p_rel_dot_v_rel_i1 + norm_v_rel_i1 * sqrt_term_i1;
                        
                        opti.subject_to(h_vo_i1 >= (1 - k_vo * T) * h_vo_i - slack_vo(i));
                    }
                }
            }
            // ------------------ CBF/VO 约束 (END) ------------------
        }
        
        // 对应Python: 目标函数 (lines 311-327)

        // -- 权重矩阵定义 --
        Eigen::Matrix4d Q = Eigen::Vector4d(1.0, 1.0, 0.5, 0.2).asDiagonal(); // x, y, theta, v 的状态误差权重
        Eigen::Matrix2d R = Eigen::Vector2d(0.01, 0.01).asDiagonal(); // a, delta 的控制量大小权重
        Eigen::Matrix2d R_rate = Eigen::Vector2d(w_a_rate_, w_delta_rate_).asDiagonal(); // a, delta 的控制量变化率权重
        Eigen::Matrix4d Q_terminal = Eigen::Vector4d(100.0, 100.0, 5.0, 2.0).asDiagonal(); // 终端状态误差权重

        // -- 目标函数构建 --
        for (int i = 0; i < N_; ++i) {
            // 1. 状态跟踪误差 (State Tracking Cost)
            Eigen::VectorXd goal_i_eigen = goal_state_.row(i);
            std::vector<double> goal_i_std(goal_i_eigen.data(), goal_i_eigen.data() + goal_i_eigen.size());
            auto state_error = opt_states(i, casadi::Slice()).T() - casadi::DM(goal_i_std);
            // 对角度误差进行特殊处理，避免跳变问题
            casadi::MX angle_error_term = casadi::MX::sin(opt_states(i, 2) - goal_state_(i, 2));
            // 将原始角度误差替换为sin形式的误差
            state_error(2) = angle_error_term;
            obj += w_track_ * quadratic(state_error.T(), Q);

            // 2. 控制量大小惩罚 (Control Magnitude Cost)
            auto u_i = opt_controls(i, casadi::Slice()).T();
            obj += quadratic(u_i.T(), R);

            // 3. 控制量变化率惩罚 (Control Rate Cost)
            if (i > 0) {
                auto u_prev = opt_controls(i-1, casadi::Slice()).T();
                auto u_rate_error = u_i - u_prev;
                obj += quadratic(u_rate_error.T(), R_rate);
            }
        }

        // 4. 终端状态误差惩罚 (Terminal State Cost)
        Eigen::VectorXd terminal_goal_eigen = goal_state_.row(N_-1);
        std::vector<double> terminal_goal_std(terminal_goal_eigen.data(), terminal_goal_eigen.data() + terminal_goal_eigen.size());
        auto terminal_state_error = opt_states(N_, casadi::Slice()).T() - casadi::DM(terminal_goal_std);
        casadi::MX terminal_angle_error_term = casadi::MX::sin(opt_states(N_, 2) - goal_state_(N_-1, 2));
        terminal_state_error(2) = terminal_angle_error_term;
        obj += w_track_ * quadratic(terminal_state_error.T(), Q_terminal);
        
        opti.minimize(obj);
        //ROS_INFO("Set optimization objective function");
        
        // 对应Python: 求解器设置 (lines 329-332)
        casadi::Dict opts_setting;
        opts_setting["ipopt.max_iter"] = 2000;
        opts_setting["ipopt.print_level"] = 0;
        opts_setting["print_time"] = 0;
        opts_setting["ipopt.acceptable_tol"] = 1e-3;
        opts_setting["ipopt.acceptable_obj_change_tol"] = 1e-3;
        
        opti.solver("ipopt", opts_setting);
        //ROS_INFO("Configured IPOPT solver");
        
        opti.set_value(opt_x0, casadi::DM({curr_state_(0), curr_state_(1), curr_state_(2), curr_state_(3)}));
        //ROS_INFO("Set initial state parameter: [%.3f, %.3f, %.3f]", curr_state_(0), curr_state_(1), curr_state_(2));
        
        Eigen::MatrixXd u_res, state_res;
        
        // 对应Python: sol = opti.solve() (line 335)
        //ROS_INFO("Starting to solve optimization problem...");
        auto sol = opti.solve();
        //ROS_INFO("Optimization solved successfully!");
        
        // 提取解
        auto u_sol = sol.value(opt_controls);
        auto state_sol = sol.value(opt_states);
        //ROS_INFO("Extracted optimization solution");
        
        // 转换为Eigen格式
        u_res = Eigen::MatrixXd(N_, 2);
        state_res = Eigen::MatrixXd(N_ + 1, 4);
        
        for (int i = 0; i < N_; ++i) {
            u_res(i, 0) = static_cast<double>(u_sol(i, 0));
            u_res(i, 1) = static_cast<double>(u_sol(i, 1));
        }
        
        for (int i = 0; i <= N_; ++i) {
            state_res(i, 0) = static_cast<double>(state_sol(i, 0));
            state_res(i, 1) = static_cast<double>(state_sol(i, 1));
            state_res(i, 2) = static_cast<double>(state_sol(i, 2));
            state_res(i, 3) = static_cast<double>(state_sol(i, 3));
        }
        
        std::cout <<"Successfully converted solution to Eigen format" << std::endl;
        std::cout << "First control input: " << u_res(0, 0) << " " << u_res(0, 1) << std::endl;
        std::cout <<"First state:" << state_res(0, 0) << " " <<  state_res(0, 1) << " "<<  state_res(0, 2) << " " << state_res(0,3) << std::endl;
        
        last_input_ = u_res;
        last_state_ = state_res;
        mpc_success_ = true;
        
        //ROS_INFO("=== mpcEllip() COMPLETED SUCCESSFULLY ===");
        return std::make_pair(state_res, u_res);
        
    } catch (const std::exception& e) {
        // 对应Python: except: 处理求解失败 (lines 343-354)
        //ROS_ERROR("MPC solve failed, exception: %s", e.what());
        
        if (mpc_success_) {
            //ROS_WARN("Using last successful solution but marking as failed");
            mpc_success_ = false;
        } else {
            //ROS_WARN("Executing time shift strategy");
            // 时间平移策略
            if (last_input_.rows() >= N_) {
                for (int i = 0; i < N_ - 1; ++i) {
                    last_input_.row(i) = last_input_.row(i + 1);
                    last_state_.row(i) = last_state_.row(i + 1);
                }
                last_input_.row(N_ - 1) = Eigen::Vector2d::Zero();
                //ROS_INFO("Completed time shift");
            } else {
                //ROS_WARN("last_input_ dimensions insufficient, cannot perform time shift");
                // 初始化默认值
                last_input_ = Eigen::MatrixXd::Zero(N_, 2);
                last_state_ = Eigen::MatrixXd::Zero(N_ + 1, 3);
            }
        }
        
        Eigen::MatrixXd u_res = last_input_;
        Eigen::MatrixXd state_res = last_state_;
        
        //ROS_INFO("=== mpcEllip() ENDED WITH EXCEPTION, returning backup solution ===");
        return std::make_pair(state_res, u_res);
    }
}

/*
void LocalPlanner::publishLocalPlan(const Eigen::MatrixXd& input_sol, 
                                   const Eigen::MatrixXd& state_sol) {
    nav_msgs::Path local_path;
    std_msgs::Float32MultiArray local_plan;
    visualization_msgs::Marker local_path_vis;
    
    // 对应Python: 可视化设置 (lines 95-102)
    local_path_vis.type = visualization_msgs::Marker::LINE_LIST;
    local_path_vis.scale.x = 0.05;
    local_path_vis.color.g = local_path_vis.color.b = local_path_vis.color.a = 1.0;
    local_path_vis.color.r = 0.0;

    local_path_vis.header.stamp = ros::Time::now();
    local_path.header.stamp = ros::Time::now();

    local_path.header.frame_id = "world";
    local_path_vis.header.frame_id = "world";
    
    // 对应Python: for i in range(self.N): (lines 104-152)
    for (int i = 0; i < N_; ++i) {
        // 路径点发布
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = state_sol(i, 0);
        this_pose_stamped.pose.position.y = state_sol(i, 1);
        this_pose_stamped.pose.position.z = z_;
        this_pose_stamped.pose.orientation.x = 0;
        this_pose_stamped.pose.orientation.y = 0;
        this_pose_stamped.pose.orientation.z = 0;
        this_pose_stamped.pose.orientation.w = 1;
        this_pose_stamped.header.seq = i;
        this_pose_stamped.header.stamp = ros::Time::now();
        this_pose_stamped.header.frame_id = "world";
        local_path.poses.push_back(this_pose_stamped);
        
        // 控制输入发布
        if (i < input_sol.rows()) {
            for (int j = 0; j < 2; ++j) {
                local_plan.data.push_back(static_cast<float>(input_sol(i, j)));
            }
        }

        // 可视化矩形框 (对应Python lines 115-152)
        geometry_msgs::Point pt;
        pt.x = state_sol(i, 0);
        pt.y = state_sol(i, 1);
        pt.z = z_;

        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 0.82;
        color.b = 0.1;
        color.a = 1.0;

        if (i < N_ - 1) {
            double x_diff = state_sol(i+1, 0) - state_sol(i, 0);
            double y_diff = state_sol(i+1, 1) - state_sol(i, 1);
            double theta = 0.0;
            if (std::abs(x_diff) > 1e-6 && std::abs(y_diff) > 1e-6) {
                theta = std::atan2(y_diff, x_diff);
            }
            
            double w = 0.7;
            double l = 0.92;
            
            // 四个角点
            geometry_msgs::Point p1, p2, p3, p4;
            p1.z = p2.z = p3.z = p4.z = pt.z - 0.01;
            
            p1.x = 0.5 * (l * std::cos(theta) - w * std::sin(theta)) + pt.x;
            p1.y = 0.5 * (l * std::sin(theta) + w * std::cos(theta)) + pt.y;
            
            p2.x = 0.5 * (-l * std::cos(theta) - w * std::sin(theta)) + pt.x;
            p2.y = 0.5 * (-l * std::sin(theta) + w * std::cos(theta)) + pt.y;
            
            p3.x = 0.5 * (-l * std::cos(theta) + w * std::sin(theta)) + pt.x;
            p3.y = 0.5 * (-l * std::sin(theta) - w * std::cos(theta)) + pt.y;
            
            p4.x = 0.5 * (l * std::cos(theta) + w * std::sin(theta)) + pt.x;
            p4.y = 0.5 * (l * std::sin(theta) - w * std::cos(theta)) + pt.y;

            // 添加线段 (p1->p2->p3->p4->p1)
            local_path_vis.points.push_back(p1); local_path_vis.colors.push_back(color);
            local_path_vis.points.push_back(p2); local_path_vis.colors.push_back(color);
            local_path_vis.points.push_back(p2); local_path_vis.colors.push_back(color);
            local_path_vis.points.push_back(p3); local_path_vis.colors.push_back(color);
            local_path_vis.points.push_back(p3); local_path_vis.colors.push_back(color);
            local_path_vis.points.push_back(p4); local_path_vis.colors.push_back(color);
            local_path_vis.points.push_back(p4); local_path_vis.colors.push_back(color);
            local_path_vis.points.push_back(p1); local_path_vis.colors.push_back(color);
        }
    }
    
    local_path_vis.pose.orientation.x = 0;
    local_path_vis.pose.orientation.y = 0;
    local_path_vis.pose.orientation.z = 0;
    local_path_vis.pose.orientation.w = 1;

    // 对应Python: 发布消息 (lines 150-152)
    local_path_vis_pub_.publish(local_path_vis);
    local_path_pub_.publish(local_path);
    local_plan_pub_.publish(local_plan);
}*/
