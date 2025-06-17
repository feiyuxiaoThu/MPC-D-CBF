#include "../include/local_planner.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <cmath>

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
LocalPlanner::LocalPlanner() : nh_("~") {
    // 对应Python: self.replan_period = rospy.get_param('/local_planner/replan_period', 0.05)
    nh_.param("/local_planner/replan_period", replan_period_, 0.05);
    
    // 对应Python: self.N = 25, self.z = 0
    N_ = 25;
    z_ = 0.0;
    
    // 对应Python: self.goal_state = np.zeros([self.N, 3])
    goal_state_ = Eigen::MatrixXd::Zero(N_, 3);
    
    // 对应Python: self.curr_state = None等初始化
    curr_state_ = Eigen::Vector3d::Zero();
    mpc_success_ = false;
    curr_state_received_ = false;
    global_path_received_ = false;
    
    // 对应Python: self.last_input = [], self.last_state = []
    last_input_ = Eigen::MatrixXd::Zero(N_, 2);
    last_state_ = Eigen::MatrixXd::Zero(N_ + 1, 3);
    
    // 对应Python: rospy.Timer(rospy.Duration(self.replan_period), self.__replan_cb)
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
    
    ROS_INFO("Local Planner initialized with N=%d, replan_period=%.3f", N_, replan_period_);
}

LocalPlanner::~LocalPlanner() {}

// 对应Python: __replan_cb函数 (lines 46-65)
void LocalPlanner::replanCallback(const ros::TimerEvent& event) {
    // 对应Python: if self.choose_goal_state():
    ROS_INFO("Replan!!!");
    static int call_count = 0;
    call_count++;
    ROS_INFO("replanCallback called #%d at time %.3f", call_count, ros::Time::now().toSec());
    
    if (!chooseGoalState()) return;
    
    ROS_INFO("chooseGoalState success, about to call mpcEllip()");
    
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

    ROS_INFO("mpcEllip() completed successfully");
    
    
    
    // 对应Python: cmd_move发布 (lines 61-63)
    std_msgs::Bool cmd_move;
    {
        std::lock_guard<std::mutex> lock(global_path_mutex_);
        if (global_path_received_ && global_path_.rows() > 0) {
            Eigen::Vector2d curr_pos = curr_state_.head<2>();
            Eigen::Vector2d goal_pos = global_path_.bottomRows(1).leftCols(2).transpose();
            cmd_move.data = distanceGlobal(curr_pos, goal_pos) > 0.1;
        } else {
            cmd_move.data = false;
        }
    }
    cmd_move_pub_.publish(cmd_move);
    ROS_INFO("start local_plan_pub_ published");
    publishLocalPlan(input_sol, states_sol);
}

// 对应Python: __curr_pose_cb函数 (lines 67-73)
void LocalPlanner::currPoseCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(curr_pose_mutex_);
    if (msg->data.size() >= 3) {
        curr_state_(0) = msg->data[0];
        curr_state_(1) = msg->data[1];
        curr_state_(2) = normalizeAngle(msg->data[2]); // 归一化角度
        curr_state_received_ = true;
    }
}

// 对应Python: __obs_cb函数 (lines 75-81)
void LocalPlanner::obsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    obstacles_.clear();
    
    int size = static_cast<int>(msg->data.size()) / 5;
    for (int i = 0; i < size; ++i) {
        Eigen::VectorXd ob(5);
        for (int j = 0; j < 5; ++j) {
            ob(j) = msg->data[5*i + j];
        }
        obstacles_.push_back(ob);
    }
}

// 对应Python: __global_path_cb函数 (lines 83-90)
void LocalPlanner::globalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(global_path_mutex_);
    int size = msg->poses.size();
    if (size > 0) {
        global_path_ = Eigen::MatrixXd::Zero(size, 3);
        for (int i = 0; i < size; ++i) {
            global_path_(i, 0) = msg->poses[i].pose.position.x;
            global_path_(i, 1) = msg->poses[i].pose.position.y;
            global_path_(i, 2) = 0.0; // 如果需要角度信息可以从四元数提取
        }
        global_path_received_ = true;
    }
}

// 对应Python: choose_goal_state函数 (lines 154-175)
bool LocalPlanner::chooseGoalState() {
    std::lock_guard<std::mutex> curr_lock(curr_pose_mutex_);
    std::lock_guard<std::mutex> path_lock(global_path_mutex_);
    
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
    return casadi::MX::vertcat({
        u(0) * casadi::MX::cos(x(2)),  // ẋ = v*cos(θ)
        u(0) * casadi::MX::sin(x(2)),  // ẏ = v*sin(θ)
        u(1)                           // θ̇ = ω
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
    
    ROS_INFO("exceedOb: checking obstacle [%.3f, %.3f, %.3f, %.3f, %.3f]", 
             ob(0), ob(1), ob(2), ob(3), ob(4));
    
    double l_long_axis = ob(2);
    double l_short_axis = ob(3);
    
    // Add safety check for axis lengths
    if (l_long_axis <= 0 || l_short_axis <= 0) {
        ROS_WARN("exceedOb: invalid axis lengths, l_long=%.3f, l_short=%.3f", l_long_axis, l_short_axis);
        return true;
    }
    
    Eigen::Vector2d long_axis(std::cos(ob(4)) * l_long_axis, std::sin(ob(4)) * l_long_axis);

    Eigen::Vector2d ob_vec(ob(0), ob(1));
    Eigen::Vector2d center_vec = goal_state_.row(N_-1).head<2>().transpose() - ob_vec;
    double dist_center = center_vec.norm();
    
    ROS_INFO("exceedOb: dist_center=%.3f", dist_center);
    
    // Add safety check for zero distance
    if (dist_center < 1e-6) {
        ROS_WARN("exceedOb: dist_center too small: %.6f", dist_center);
        return false; // Obstacle at goal position
    }
    
    double cos_ = center_vec.dot(long_axis) / (dist_center * l_long_axis);
    ROS_INFO("exceedOb: cos_=%.3f", cos_);

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
    
    ROS_INFO("exceedOb: calculated d=%.3f", d);

    Eigen::Vector2d cross_pt = ob_vec + d * center_vec / dist_center;
    Eigen::Vector2d vec1 = goal_state_.row(N_-1).head<2>().transpose() - cross_pt;
    Eigen::Vector2d vec2 = curr_state_.head<2>() - cross_pt;
    double theta = vec1.dot(vec2);
    
    bool result = theta > 0;
    ROS_INFO("exceedOb: theta=%.3f, result=%s", theta, result ? "true" : "false");

    return result;
}

// 对应Python: MPC_ellip函数 (lines 177-365) - 核心MPC算法
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> LocalPlanner::mpcEllip() {
    ROS_INFO("=== mpcEllip() STARTED ===");
    
    std::lock_guard<std::mutex> curr_lock(curr_pose_mutex_);
    std::lock_guard<std::mutex> path_lock(global_path_mutex_);
    std::lock_guard<std::mutex> obs_lock(obstacle_mutex_);
    
    ROS_INFO("Acquired all locks, checking data status");
    ROS_INFO("curr_state_received_: %s", curr_state_received_ ? "true" : "false");
    ROS_INFO("global_path_received_: %s", global_path_received_ ? "true" : "false");
    ROS_INFO("curr_state_: [%.3f, %.3f, %.3f]", curr_state_(0), curr_state_(1), curr_state_(2));
    ROS_INFO("goal_state_ dimensions: %ld x %ld", goal_state_.rows(), goal_state_.cols());
    ROS_INFO("obstacles_ count: %zu", obstacles_.size());
    
    if (!curr_state_received_) {
        ROS_ERROR("Current state not received, cannot execute MPC");
        return std::make_pair(last_state_, last_input_);
    }
    
    if (!global_path_received_ || goal_state_.rows() != N_) {
        ROS_ERROR("Global path not received or goal_state dimension error, goal_state_.rows()=%ld, N_=%d", 
                  goal_state_.rows(), N_);
        return std::make_pair(last_state_, last_input_);
    }
    
    // 对应Python: opti = ca.Opti()
    casadi::Opti opti;
    ROS_INFO("Created casadi optimizer");
    
    // 对应Python: 参数设置 (lines 182-188)
    double T = 0.1;        // 时间步长
    double gamma_k = 0.3;  // 障碍物约束松弛因子
    double v_max = 1.2;    // 最大线速度
    double v_min = 0.1;    // 最小线速度  
    double omega_max = 1.2; // 最大角速度
    
    ROS_INFO("MPC parameters: T=%.3f, gamma_k=%.3f, v_max=%.3f, v_min=%.3f, omega_max=%.3f", 
             T, gamma_k, v_max, v_min, omega_max);
    
    try {
        // 对应Python: opt_x0 = opti.parameter(3)
        auto opt_x0 = opti.parameter(3, 1);
        ROS_INFO("Created initial state parameter opt_x0");
        
        // 对应Python: opt_states = opti.variable(self.N + 1, 3)
        auto opt_states = opti.variable(N_ + 1, 3);
        auto opt_controls = opti.variable(N_, 2);
        ROS_INFO("Created state variables opt_states(%d x 3) and control variables opt_controls(%d x 2)", N_+1, N_);
        
        // 对应Python: v = opt_controls[:, 0], omega = opt_controls[:, 1]
        auto v = opt_controls(casadi::Slice(), 0);
        auto omega = opt_controls(casadi::Slice(), 1);
        ROS_INFO("Extracted velocity and angular velocity control variables");
        
        // 对应Python: opti.subject_to(opt_states[0, :] == opt_x0.T) (line 291)
        opti.subject_to(opt_states(0, casadi::Slice()) == opt_x0.T());
        ROS_INFO("Added initial state constraint");
        
        // 对应Python: 速度约束 (lines 293-296)
        bool near_goal = false;
        if (global_path_received_ && global_path_.rows() > 0) {
            Eigen::Vector2d curr_pos = curr_state_.head<2>();
            Eigen::Vector2d goal_pos = global_path_.bottomRows(1).leftCols(2).transpose();
            double dist_to_goal = distanceGlobal(curr_pos, goal_pos);
            near_goal = dist_to_goal <= 1.0;
            ROS_INFO("Distance to goal: %.3f, near goal: %s", dist_to_goal, near_goal ? "yes" : "no");
        }
        
        if (!near_goal) {
            opti.subject_to(v >= v_min);
            opti.subject_to(v <= v_max);
            ROS_INFO("Added forward velocity constraint: [%.3f, %.3f]", v_min, v_max);
        } else {
            opti.subject_to(v >= -v_min);
            opti.subject_to(v <= v_max); 
            ROS_INFO("Added bidirectional velocity constraint: [%.3f, %.3f]", -v_min, v_max);
        }
        opti.subject_to(omega >= -omega_max);
        opti.subject_to(omega <= omega_max);
        ROS_INFO("Added angular velocity constraint: [%.3f, %.3f]", -omega_max, omega_max);
        
        // 对应Python: 系统模型约束 (lines 299-301)
        ROS_INFO("Starting to add system model constraints");
        for (int i = 0; i < N_; ++i) {
            auto x_curr = opt_states(i, casadi::Slice());
            auto u_curr = opt_controls(i, casadi::Slice());
            auto x_next = x_curr + T * systemModel(x_curr.T(), u_curr.T()).T();
            opti.subject_to(opt_states(i + 1, casadi::Slice()) == x_next);
            
            if (i < 3) { // Only print first few to avoid log spam
                ROS_INFO("Added system model constraint for step %d", i);
            }
        }
        ROS_INFO("Completed all %d system model constraints", N_);
        casadi::MX obj = 0;
        // 对应Python: 障碍物约束 (lines 303-309) - 修复索引问题
        int num_obs = 0;
        if (!obstacles_.empty()) {
            ROS_INFO("Starting to process obstacle constraints");
            ROS_INFO("obstacles_.size() = %zu", obstacles_.size());
            double slack_weight = 1000.0;  // 较大的权重以确保尽量满足约束
    
            // Fix: Calculate based on actual obstacle data structure
            // Check if obstacle data is sufficient for N_ step prediction
            if (obstacles_.size() >= N_) {
                num_obs = obstacles_.size() / N_; // Simplified processing, assume only one obstacle sequence
                ROS_INFO("Detected sufficient obstacle data, preparing to add constraints");
                
                for (int j = 0; j < num_obs; ++j) {
                    // Fix index problem: use safer index calculation
                    int base_idx = j * N_;
                    if (base_idx < static_cast<int>(obstacles_.size())) {
                        ROS_INFO("Checking obstacle sequence %d, base index: %d", j, base_idx);
                        
                        if (!exceedOb(obstacles_[base_idx])) {
                            ROS_INFO("Obstacle %d not out of range, adding ellipse constraints", j);
                             // 为这个障碍物序列创建松弛变量
                            auto slack_vars = opti.variable(N_ - 1);
                            
                            // 确保松弛变量非负
                            opti.subject_to(slack_vars >= 0);
                            
                            // 累积松弛变量的惩罚项
                            casadi::MX slack_penalty = 0;
                            for (int i = 0; i < N_ - 1; ++i) {
                                // Fix: Use N_ instead of hardcoded 25
                                int idx1 = base_idx + i;
                                int idx2 = base_idx + i + 1;
                                
                                // Fix: Ensure indices are within valid range
                                if (idx1 < static_cast<int>(obstacles_.size()) && 
                                    idx2 < static_cast<int>(obstacles_.size())) {
                                    
                                    if (i < 3) { // Only print first few
                                        ROS_INFO("Adding obstacle constraint for step %d, idx1=%d, idx2=%d", i, idx1, idx2);
                                    }
                                    
                                    auto h_curr = ellipseConstraint(
                                        opt_states(i, casadi::Slice(0, 2)).T(), 
                                        obstacles_[idx1]
                                    );
                                    auto h_next = ellipseConstraint(
                                        opt_states(i + 1, casadi::Slice(0, 2)).T(),
                                        obstacles_[idx2]
                                    );
                                    //opti.subject_to(h_next >= (1 - gamma_k) * h_curr);
                                     // 软约束：允许违反约束，但添加松弛变量
                                    opti.subject_to(h_next + slack_vars(i) >= (1 - gamma_k) * h_curr);
                                    
                                    // 累加松弛变量的惩罚
                                    slack_penalty = slack_penalty + slack_vars(i) * slack_vars(i);  // 二次惩罚
                                            
                                } else {
                                    ROS_WARN("Index out of bounds: idx1=%d, idx2=%d, obstacles_.size()=%zu", 
                                             idx1, idx2, obstacles_.size());
                                    break; // Stop adding more constraints
                                }
                            }
                             // 将松弛变量惩罚添加到目标函数
                            obj = obj + slack_weight * slack_penalty;
                        } else {
                            ROS_INFO("Obstacle %d out of range, skipping constraints", j);
                        }
                    } else {
                        ROS_WARN("Base index out of bounds: base_idx=%d, obstacles_.size()=%zu", 
                                 base_idx, obstacles_.size());
                    }
                }
            } else {
                ROS_WARN("Insufficient obstacle data: obstacles_.size()=%zu < N_=%d, skipping obstacle constraints", 
                         obstacles_.size(), N_);
            }
        } else {
            ROS_INFO("No obstacle data, skipping obstacle constraints");
        }
        
        // 对应Python: 目标函数 (lines 311-327)
        ROS_INFO("Starting to build objective function");
        //casadi::MX obj = 0;
        
        // R矩阵 (控制权重)
        Eigen::Matrix2d R = Eigen::Vector2d(0.1, 0.02).asDiagonal();
        ROS_INFO("R matrix diagonal elements: [%.3f, %.3f]", R(0,0), R(1,1));
        
        for (int i = 0; i < N_; ++i) {
            // Q矩阵 (状态权重)
            Eigen::Vector3d q_diag(1.0 + 0.05*i, 1.0 + 0.05*i, 0.02 + 0.005*i);
            Eigen::Matrix3d Q = q_diag.asDiagonal();
            
            // 状态误差 - 分别处理位置和角度
            // 位置误差 (x, y)
            casadi::MX pos_error = casadi::MX::vertcat({
                opt_states(i, 0) - goal_state_(i, 0),
                opt_states(i, 1) - goal_state_(i, 1)
            });
            
            // 角度误差 - 使用sin和cos来处理角度差以避免跳变
            casadi::MX angle_current = opt_states(i, 2);
            casadi::MX angle_goal = casadi::MX(goal_state_(i, 2));
            
            // 使用角度差的sin和cos来创建连续的角度误差
            casadi::MX angle_error_sin = casadi::MX::sin(angle_current - angle_goal);
            casadi::MX angle_error_cos = casadi::MX::cos(angle_current - angle_goal) - 1;
            
            // 位置部分的二次项
            casadi::MX pos_cost = pos_error(0) * pos_error(0) * Q(0,0) + 
                                  pos_error(1) * pos_error(1) * Q(1,1);
            
            // 角度部分的二次项 - 使用1-cos(θ)形式，它在θ=0附近是连续且平滑的
            casadi::MX angle_cost = Q(2,2) * (angle_error_sin * angle_error_sin + 
                                              angle_error_cos * angle_error_cos);
            
            if (i < N_ - 1) {
                // 控制误差  
                casadi::MX control_error = opt_controls(i, casadi::Slice());
                obj += 0.1 * (pos_cost + angle_cost) + quadratic(control_error, R);
            } else {
                obj += 0.1 * (pos_cost + angle_cost);
            }
            
            if (i < 3) { // Only print first few
                ROS_INFO("Added objective term for step %d, goal state: [%.3f, %.3f, %.3f]", 
                         i, goal_state_(i, 0), goal_state_(i, 1), goal_state_(i, 2));
            }
        }
        
        // 终端约束权重 - 修复角度误差计算
        Eigen::Matrix3d Q_terminal = Eigen::Vector3d(5.0, 5.0, 0.1).asDiagonal();
        
        // 终端位置误差
        casadi::MX terminal_pos_error = casadi::MX::vertcat({
            opt_states(N_-1, 0) - goal_state_(N_-1, 0),
            opt_states(N_-1, 1) - goal_state_(N_-1, 1)
        });
        
        // 终端角度误差
        casadi::MX terminal_angle_current = opt_states(N_-1, 2);
        casadi::MX terminal_angle_goal = casadi::MX(goal_state_(N_-1, 2));
        
        casadi::MX terminal_angle_error_sin = casadi::MX::sin(terminal_angle_current - terminal_angle_goal);
        casadi::MX terminal_angle_error_cos = casadi::MX::cos(terminal_angle_current - terminal_angle_goal) - 1;
        
        // 终端位置成本
        casadi::MX terminal_pos_cost = terminal_pos_error(0) * terminal_pos_error(0) * Q_terminal(0,0) + 
                                       terminal_pos_error(1) * terminal_pos_error(1) * Q_terminal(1,1);
        
        // 终端角度成本
        casadi::MX terminal_angle_cost = Q_terminal(2,2) * (terminal_angle_error_sin * terminal_angle_error_sin + 
                                                             terminal_angle_error_cos * terminal_angle_error_cos);
        
        obj += terminal_pos_cost + terminal_angle_cost;
        ROS_INFO("Added terminal cost term, terminal goal: [%.3f, %.3f, %.3f]", 
                 goal_state_(N_-1, 0), goal_state_(N_-1, 1), goal_state_(N_-1, 2));
        
        opti.minimize(obj);
        ROS_INFO("Set optimization objective function");
        
        // 对应Python: 求解器设置 (lines 329-332)
        casadi::Dict opts_setting;
        opts_setting["ipopt.max_iter"] = 2000;
        opts_setting["ipopt.print_level"] = 0;
        opts_setting["print_time"] = 0;
        opts_setting["ipopt.acceptable_tol"] = 1e-3;
        opts_setting["ipopt.acceptable_obj_change_tol"] = 1e-3;
        
        opti.solver("ipopt", opts_setting);
        ROS_INFO("Configured IPOPT solver");
        
        opti.set_value(opt_x0, casadi::DM({curr_state_(0), curr_state_(1), curr_state_(2)}));
        ROS_INFO("Set initial state parameter: [%.3f, %.3f, %.3f]", curr_state_(0), curr_state_(1), curr_state_(2));
        
        Eigen::MatrixXd u_res, state_res;
        
        // 对应Python: sol = opti.solve() (line 335)
        ROS_INFO("Starting to solve optimization problem...");
        auto sol = opti.solve();
        ROS_INFO("Optimization solved successfully!");
        
        // 提取解
        auto u_sol = sol.value(opt_controls);
        auto state_sol = sol.value(opt_states);
        ROS_INFO("Extracted optimization solution");
        
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
        
        ROS_INFO("Successfully converted solution to Eigen format");
        ROS_INFO("First control input: [%.3f, %.3f]", u_res(0, 0), u_res(0, 1));
        ROS_INFO("First state: [%.3f, %.3f, %.3f]", state_res(0, 0), state_res(0, 1), state_res(0, 2));
        
        last_input_ = u_res;
        last_state_ = state_res;
        mpc_success_ = true;
        
        ROS_INFO("=== mpcEllip() COMPLETED SUCCESSFULLY ===");
        return std::make_pair(state_res, u_res);
        
    } catch (const std::exception& e) {
        // 对应Python: except: 处理求解失败 (lines 343-354)
        ROS_ERROR("MPC solve failed, exception: %s", e.what());
        
        if (mpc_success_) {
            ROS_WARN("Using last successful solution but marking as failed");
            mpc_success_ = false;
        } else {
            ROS_WARN("Executing time shift strategy");
            // 时间平移策略
            if (last_input_.rows() >= N_) {
                for (int i = 0; i < N_ - 1; ++i) {
                    last_input_.row(i) = last_input_.row(i + 1);
                    last_state_.row(i) = last_state_.row(i + 1);
                }
                last_input_.row(N_ - 1) = Eigen::Vector2d::Zero();
                ROS_INFO("Completed time shift");
            } else {
                ROS_WARN("last_input_ dimensions insufficient, cannot perform time shift");
                // 初始化默认值
                last_input_ = Eigen::MatrixXd::Zero(N_, 2);
                last_state_ = Eigen::MatrixXd::Zero(N_ + 1, 3);
            }
        }
        
        Eigen::MatrixXd u_res = last_input_;
        Eigen::MatrixXd state_res = last_state_;
        
        ROS_INFO("=== mpcEllip() ENDED WITH EXCEPTION, returning backup solution ===");
        return std::make_pair(state_res, u_res);
    }
}

// 对应Python: __publish_local_plan函数 (lines 92-152)
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
}