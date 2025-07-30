#include "local_planner.h"
#include "matplotlibcpp.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace plt = matplotlibcpp;

// 对应Python: def distance_global(c1, c2)
double LocalPlanner::distanceGlobal(const Eigen::Vector2d& c1, const Eigen::Vector2d& c2) {
    return std::sqrt(std::pow(c1(0) - c2(0), 2) + std::pow(c1(1) - c2(1), 2));
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

// 构造函数
LocalPlanner::LocalPlanner() {
    N_ = 25; // MPC预测步数
    z_ = 0.0;
    replan_period_ = 0.2;
    
    goal_state_ = Eigen::MatrixXd::Zero(N_, 3);
    
    curr_state_ = Eigen::Vector3d::Zero();
    mpc_success_ = false;
    curr_state_received_ = true;
    global_path_received_ = true;

    currPoseCallback();
    obsCallback();
    globalPathCallback();

    last_input_ = Eigen::MatrixXd::Zero(N_, 2);
    last_state_ = Eigen::MatrixXd::Zero(N_ + 1, 3);
}

LocalPlanner::~LocalPlanner() {}

// 重规划回调
void LocalPlanner::replanCallback() {
    std::cout << "Replan!!!" << std::endl;
    
    if (!chooseGoalState()) return;
    
    std::cout << "chooseGoalState success, about to call mpcEllip()" << std::endl;
    
    for (int i = 0; i < N_ - 1; ++i) {
        double y_diff = goal_state_(i+1, 1) - goal_state_(i, 1);
        double x_diff = goal_state_(i+1, 0) - goal_state_(i, 0);
        
        if (std::abs(x_diff) > 1e-6 || std::abs(y_diff) > 1e-6) {
            goal_state_(i, 2) = std::atan2(y_diff, x_diff);
        } else if (i != 0) {
            goal_state_(i, 2) = goal_state_(i-1, 2);
        } else {
            goal_state_(i, 2) = curr_state_(2);
        }
    }
    goal_state_(N_-1, 2) = goal_state_(N_-2, 2);
    
    for (int i = 0; i < N_; ++i) {
        if (i > 0) {
            double angle_diff = angleDifference(goal_state_(i, 2), goal_state_(i-1, 2));
            goal_state_(i, 2) = goal_state_(i-1, 2) + angle_diff;
        } else {
            double angle_diff = angleDifference(goal_state_(i, 2), curr_state_(2));
            goal_state_(i, 2) = curr_state_(2) + angle_diff;
        }
    }
    
    auto [states_sol, input_sol] = mpcEllip();

    if(mpc_success_) {
        std::cout << "mpcEllip() completed successfully." << std::endl;
    }
}

// 可视化函数
void LocalPlanner::visualizeResults() {
    // 1. 提取数据用于绘图
    std::vector<double> x_sol, y_sol, goal_x, goal_y, global_x, global_y;
    for (int i = 0; i < last_state_.rows(); ++i) {
        x_sol.push_back(last_state_(i, 0));
        y_sol.push_back(last_state_(i, 1));
    }
    for (int i = 0; i < goal_state_.rows(); ++i) {
        goal_x.push_back(goal_state_(i, 0));
        goal_y.push_back(goal_state_(i, 1));
    }
    for (int i = 0; i < global_path_.rows(); ++i) {
        global_x.push_back(global_path_(i, 0));
        global_y.push_back(global_path_(i, 1));
    }

    // 2. 清除旧图像并开始新图像
    plt::clf();
    
    // 3. 绘制轨迹
    plt::plot(global_x, global_y, {{"label", "Global Path"}, {"color", "gray"}, {"linestyle", ":"}});
    plt::plot(x_sol, y_sol, {{"label", "MPC Predicted Trajectory"}, {"color", "blue"}, {"marker", "o"}});
    plt::plot(goal_x, goal_y, {{"label", "Reference Trajectory"}, {"color", "green"}, {"linestyle", "--"}});

    // 4. 绘制障碍物
    if (!obstacles_.empty()) {
        int num_obs_sequences = obstacles_.size() / N_;
        for (int j = 0; j < num_obs_sequences; ++j) {
            // 只绘制第一个时间步的障碍物以避免混乱
            const auto& ob = obstacles_[j * N_];
            double ob_x = ob(0), ob_y = ob(1), a = ob(2), b = ob(3), theta = ob(4);
            
            std::vector<double> ell_x, ell_y;
            for (double ang = 0; ang <= 2 * M_PI + 0.1; ang += 0.1) {
                double x_unit = a * std::cos(ang);
                double y_unit = b * std::sin(ang);
                ell_x.push_back(ob_x + x_unit * std::cos(theta) - y_unit * std::sin(theta));
                ell_y.push_back(ob_y + x_unit * std::sin(theta) + y_unit * std::cos(theta));
            }
            std::string ob_color = (j == 0) ? "red" : "orange";
            plt::fill(ell_x, ell_y, {{"color", ob_color}, {"alpha", "0.5"}});
        }
    }

    // 5. 绘制起始点
    plt::scatter({x_sol.front()}, {y_sol.front()}, 100, {{"color", "red"}, {"zorder", "3"}, {"label", "Start Position"}});

    // 6. 设置图像属性
    plt::title("MPC Trajectory Visualization");
    plt::xlabel("X (m)");
    plt::ylabel("Y (m)");
    plt::legend();
    plt::axis("equal");
    plt::grid(true);

    // 7. 显示图像
    plt::show();
}


// 接受本车初始轨迹
void LocalPlanner::currPoseCallback() {
    std::lock_guard<std::mutex> lock(curr_pose_mutex_);
    curr_state_ << 0.0, 0.0, 0.0;
    curr_state_received_ = true;
}

// 创建障碍物
void LocalPlanner::obsCallback() {
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    obstacles_.clear();
    
    // 静态障碍物
    Eigen::VectorXd static_ob(5);
    static_ob << 3.0, 1.0, 0.5, 0.8, 0.0;
    for(int i = 0; i < N_; i++) obstacles_.push_back(static_ob);
    
    // 动态障碍物
    Eigen::VectorXd dynamic_ob(5);
    dynamic_ob << 4.0, -2.0, 0.5, 0.5, 0.0;
    for(int i = 0; i < N_; i++){
        obstacles_.push_back(dynamic_ob);
        dynamic_ob(0) -= 0.1 * 2.0; // vx = -2.0 m/s, T=0.1s
    }
}

// 创建参考线轨迹
void LocalPlanner::globalPathCallback() {
    std::lock_guard<std::mutex> lock(global_path_mutex_);
    int size = 100;
    global_path_ = Eigen::MatrixXd::Zero(size, 3);
    for (int i = 0; i < size; ++i) {
        global_path_(i, 0) = i * 0.2;
        global_path_(i, 1) = 2.0 * std::sin(global_path_(i, 0) / 2.0);
        global_path_(i, 2) = 0.0;
    }
    global_path_received_ = true;
}

// 选择目标状态
bool LocalPlanner::chooseGoalState() {
    std::lock_guard<std::mutex> curr_lock(curr_pose_mutex_);
    std::lock_guard<std::mutex> path_lock(global_path_mutex_);
    
    if (!global_path_received_ || !curr_state_received_ || global_path_.rows() == 0) return false;

    double min_dist = std::numeric_limits<double>::max();
    int num = 0;
    for (int i = 0; i < global_path_.rows(); ++i) {
        double dist = distanceGlobal(curr_state_.head<2>(), global_path_.row(i).head<2>());
        if (dist < min_dist) {
            min_dist = dist;
            num = i;
        }
    }

    for (int k = 0; k < N_; ++k) {
        int num_path = std::min((int)global_path_.rows() - 1, num + k * 1);
        goal_state_.row(k) = global_path_.row(num_path);
    }

    return true;
}

// 系统模型
casadi::MX LocalPlanner::systemModel(const casadi::MX& x, const casadi::MX& u) {
    return casadi::MX::vertcat({
        u(0) * casadi::MX::cos(x(2)),
        u(0) * casadi::MX::sin(x(2)),
        u(1)
    });
}

// 椭圆约束
casadi::MX LocalPlanner::ellipseConstraint(const casadi::MX& pos, const Eigen::VectorXd& ob) {
    double safe_dist = 0.5;
    double ob_x = ob(0), ob_y = ob(1), a = ob(2), b = ob(3), theta = ob(4);
    auto c = casadi::MX::cos(theta), s = casadi::MX::sin(theta);
    casadi::MX ob_vec = casadi::MX(casadi::DM({ob_x, ob_y}));
    casadi::MX center_vec = pos - ob_vec;
    auto term1 = (c*c/(a*a) + s*s/(b*b)) * center_vec(0) * center_vec(0);
    auto term2 = (s*s/(a*a) + c*c/(b*b)) * center_vec(1) * center_vec(1);
    auto term3 = 2*c*s*(1/(a*a) - 1/(b*b)) * center_vec(0) * center_vec(1);
    return b * (casadi::MX::sqrt(term1 + term2 + term3) - 1) - safe_dist;
}

// 二次型
casadi::MX LocalPlanner::quadratic(const casadi::MX& x, const Eigen::MatrixXd& A) {
    std::vector<double> A_data(A.data(), A.data() + A.size());
    casadi::DM A_dm = casadi::DM::reshape(casadi::DM(A_data), A.rows(), A.cols());
    return casadi::MX::mtimes({x, casadi::MX(A_dm), x.T()});
}

// 检查障碍物是否超出范围
bool LocalPlanner::exceedOb(const Eigen::VectorXd& ob) {
    if (!global_path_received_ || global_path_.rows() == 0) return true;
    double l_long = ob(2), l_short = ob(3);
    if (l_long <= 0 || l_short <= 0) return true;
    Eigen::Vector2d ob_vec(ob(0), ob(1));
    Eigen::Vector2d center_vec = goal_state_.row(N_-1).head<2>().transpose() - ob_vec;
    double dist_center = center_vec.norm();
    if (dist_center < 1e-6) return false;
    Eigen::Vector2d long_axis(std::cos(ob(4)) * l_long, std::sin(ob(4)) * l_long);
    double cos_ = center_vec.dot(long_axis) / (dist_center * l_long);
    double d = (std::abs(cos_) > 0.1) ? std::sqrt((l_long*l_long*l_short*l_short*(1 + (1/(cos_*cos_)-1))) / (l_short*l_short + l_long*l_long*((1/(cos_*cos_)-1)))) : l_short;
    Eigen::Vector2d cross_pt = ob_vec + d * center_vec / dist_center;
    return (goal_state_.row(N_-1).head<2>().transpose() - cross_pt).dot(curr_state_.head<2>() - cross_pt) > 0;
}

// MPC核心算法
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> LocalPlanner::mpcEllip() {
    std::lock_guard<std::mutex> curr_lock(curr_pose_mutex_);
    std::lock_guard<std::mutex> path_lock(global_path_mutex_);
    std::lock_guard<std::mutex> obs_lock(obstacle_mutex_);
    
    if (!curr_state_received_ || !global_path_received_ || goal_state_.rows() != N_) {
        return std::make_pair(last_state_, last_input_);
    }
    
    casadi::Opti opti;
    double T = 0.1, gamma_k = 0.3, v_max = 10.0, v_min = 0.0, omega_max = 1.2;
    
    try {
        auto opt_x0 = opti.parameter(3, 1);
        auto opt_states = opti.variable(N_ + 1, 3);
        auto opt_controls = opti.variable(N_, 2);
        auto v = opt_controls(casadi::Slice(), 0);
        auto omega = opt_controls(casadi::Slice(), 1);
        
        opti.subject_to(opt_states(0, casadi::Slice()) == opt_x0.T());
        
        double dist_to_goal = distanceGlobal(curr_state_.head<2>(), global_path_.bottomRows(1).leftCols(2).transpose());
        if (dist_to_goal > 1.0) {
            opti.subject_to(v >= v_min);
            opti.subject_to(v <= v_max);
        } else {
            opti.subject_to(v >= -v_min);
            opti.subject_to(v <= v_max);
        }
        opti.subject_to(omega >= -omega_max);
        opti.subject_to(omega <= omega_max);
        
        for (int i = 0; i < N_; ++i) {
            auto x_next = opt_states(i, casadi::Slice()) + T * systemModel(opt_states(i, casadi::Slice()).T(), opt_controls(i, casadi::Slice()).T()).T();
            opti.subject_to(opt_states(i + 1, casadi::Slice()) == x_next);
        }
        
        casadi::MX obj = 0;
        if (!obstacles_.empty()) {
            double slack_weight = 1000.0;
            int num_obs_sequences = obstacles_.size() / N_;
            for (int j = 0; j < num_obs_sequences; ++j) {
                int base_idx = j * N_;
                if (!exceedOb(obstacles_[base_idx])) {
                    auto slack_vars = opti.variable(N_ - 1);
                    opti.subject_to(slack_vars >= 0);
                    casadi::MX slack_penalty = 0;
                    for (int i = 0; i < N_ - 1; ++i) {
                        auto h_curr = ellipseConstraint(opt_states(i, casadi::Slice(0, 2)).T(), obstacles_[base_idx + i]);
                        auto h_next = ellipseConstraint(opt_states(i + 1, casadi::Slice(0, 2)).T(), obstacles_[base_idx + i + 1]);
                        opti.subject_to(h_next + slack_vars(i) >= (1 - gamma_k) * h_curr);
                        slack_penalty += slack_vars(i) * slack_vars(i);
                    }
                    obj += slack_weight * slack_penalty;
                }
            }
        }
        
        Eigen::Matrix2d R = Eigen::Vector2d(0.1, 0.02).asDiagonal();
        for (int i = 0; i < N_; ++i) {
            Eigen::Vector3d q_diag(1.0 + 0.05*i, 1.0 + 0.05*i, 0.02 + 0.005*i);
            Eigen::Matrix3d Q = q_diag.asDiagonal();
            casadi::MX state_error = opt_states(i, casadi::Slice()) - casadi::MX(casadi::DM(std::vector<double>(goal_state_.row(i).data(), goal_state_.row(i).data() + 3)));
            casadi::MX angle_error = opt_states(i, 2) - goal_state_(i, 2);
            obj += quadratic(state_error(casadi::Slice(0,2)), Q.block<2,2>(0,0));
            obj += Q(2,2) * (1 - casadi::MX::cos(angle_error));
            if (i < N_) {
                obj += quadratic(opt_controls(i, casadi::Slice()), R);
            }
        }
        
        opti.minimize(obj);
        
        casadi::Dict opts_setting;
        opts_setting["ipopt.max_iter"] = 2000;
        opts_setting["ipopt.print_level"] = 0;
        opts_setting["print_time"] = 0;
        opts_setting["ipopt.acceptable_tol"] = 1e-3;
        opts_setting["ipopt.acceptable_obj_change_tol"] = 1e-3;
        opti.solver("ipopt", opts_setting);
        
        opti.set_value(opt_x0, casadi::DM({curr_state_(0), curr_state_(1), curr_state_(2)}));
        
        auto sol = opti.solve();
        
        auto u_sol = sol.value(opt_controls);
        auto state_sol = sol.value(opt_states);
        
        Eigen::MatrixXd u_res(N_, 2);
        Eigen::MatrixXd state_res(N_ + 1, 3);
        
        for (int i = 0; i < N_; ++i) for(int j=0; j<2; ++j) u_res(i,j) = static_cast<double>(u_sol(i,j));
        for (int i = 0; i <= N_; ++i) for(int j=0; j<3; ++j) state_res(i,j) = static_cast<double>(state_sol(i,j));
        
        last_input_ = u_res;
        last_state_ = state_res;
        mpc_success_ = true;
        
        return std::make_pair(state_res, u_res);
        
    } catch (const std::exception& e) {
        mpc_success_ = false;
        if (last_input_.rows() >= N_) {
            for (int i = 0; i < N_ - 1; ++i) {
                last_input_.row(i) = last_input_.row(i + 1);
                last_state_.row(i) = last_state_.row(i + 1);
            }
            last_input_.row(N_ - 1).setZero();
        } else {
            last_input_ = Eigen::MatrixXd::Zero(N_, 2);
            last_state_ = Eigen::MatrixXd::Zero(N_ + 1, 3);
        }
        return std::make_pair(last_input_, last_state_);
    }
}

// Main function
int main() {
    LocalPlanner planner;
    planner.replanCallback();
    
    if (planner.mpc_success_) {
        std::cout << "Displaying visualization..." << std::endl;
        planner.visualizeResults();
    } else {
        std::cout << "MPC failed, no visualization to display." << std::endl;
    }
    
    return 0;
}
