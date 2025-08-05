#include "local_planner.h"
#include <algorithm>
#include <cmath>
#include <vector>
#include <fstream>
#include <iomanip>



void LocalPlanner::saveDataForVisualization3D() const {
    // 1. 保存 last_state_ (MPC 预测轨迹)
    std::ofstream mpc_file("../plot/3D/mpc_trajectory.txt");
    if (mpc_file.is_open()) {
        for (int i = 0; i < last_state_.rows(); ++i) {
            mpc_file << std::fixed << std::setprecision(5) << last_state_(i, 0) << " " << last_state_(i, 1) << std::endl;
        }
        mpc_file.close();
    }

    // 2. 保存 goal_state_ (参考轨迹)
    std::ofstream goal_file("../plot/3D/reference_trajectory.txt");
    if (goal_file.is_open()) {
        for (int i = 0; i < goal_state_.rows(); ++i) {
            goal_file << std::fixed << std::setprecision(5) << goal_state_(i, 0) << " " << goal_state_(i, 1) << std::endl;
        }
        goal_file.close();
    }

    // 3. 保存 global_path_ (全局路径)
    std::ofstream global_path_file("../plot/3D/global_path.txt");
    if (global_path_file.is_open()) {
        for (int i = 0; i < global_path_.rows(); ++i) {
            global_path_file << std::fixed << std::setprecision(5) << global_path_(i, 0) << " " << global_path_(i, 1) << std::endl;
        }
        global_path_file.close();
    }

    // 4. 保存 obstacles_ (所有时间步的障碍物)
    std::ofstream obs_file("../plot/3D/obstacles.txt");
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
    std::ofstream config_file("../plot/3D/config.txt");
    if (config_file.is_open()) {
        config_file << "N " << N_ << std::endl;
        config_file << "replan_period " << replan_period_ << std::endl;
        config_file.close();
    }

    std::cout << "Visualization data 3D saved to files." << std::endl;
}


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
    N_ = 50; // MPC预测步数
    z_ = 0.0;
    replan_period_ = 0.1;
    
    // 对应Python: self.goal_state = np.zeros([self.N, 3])
    goal_state_ = Eigen::MatrixXd::Zero(N_, 3);
    
    // 对应Python: self.curr_state = None等初始化
    curr_state_ = Eigen::Vector3d::Zero();
    mpc_success_ = false;
    curr_state_received_ = true;
    global_path_received_ = true;

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
    last_state_ = Eigen::MatrixXd::Zero(N_ + 1, 3);
    
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
        
        if (std::abs(x_diff) > 1e-6 || std::abs(y_diff) > 1e-6) {
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
    bool cmd_move;
    {
        // std::lock_guard<std::mutex> lock(global_path_mutex_); // Mutex removed for simplicity
        if (global_path_received_ && global_path_.rows() > 0) {
            Eigen::Vector2d curr_pos = curr_state_.head<2>();
            Eigen::Vector2d goal_pos = global_path_.bottomRows(1).leftCols(2).transpose();
            cmd_move = distanceGlobal(curr_pos, goal_pos) > 0.1;
        } else {
            cmd_move = false;
        }
    }
   
    //ROS_INFO("start local_plan_pub_ published");
    //publishLocalPlan(input_sol, states_sol);
}

//  接受本车初始轨迹，可以假设为 0 0
void LocalPlanner::currPoseCallback() {
    // std::lock_guard<std::mutex> lock(curr_pose_mutex_); // Mutex removed for simplicity
    bool use_ego_cor = true;
    if (use_ego_cor) {
        curr_state_(0) = 0.0;
        curr_state_(1) = 0.0;
        curr_state_(2) = 0.0; // 归一化角度
        curr_state_received_ = true;
    }
}

// 对应Python: __obs_cb函数 (lines 75-81)
void LocalPlanner::obsCallback() {
    // std::lock_guard<std::mutex> lock(obstacle_mutex_); // Mutex removed for simplicity
    obstacles_.clear();
    
    int size_static = 1; // 1 static obs
    int size_dynamic = 1; // 1 dynamic obs
    int size = N_*(size_static + size_dynamic);

    Eigen::VectorXd ob(5);
    ob << 3.0,1.0,0.5,0.8,0.0;
    for(int i =0; i< size_static*N_; i++){
        obstacles_.push_back(ob);
    }
    ob << 10.0,-0.5,1.0,1.0,0.0;
    obstacles_.push_back(ob);
    for(int i = 1; i< size_dynamic*N_; i++){
    double replan_period_ = 0.1;
        ob(0) = ob(0) + 2.0*replan_period_; // vx = 2.0
        obstacles_.push_back(ob);
    }
}

// 对应Python: __global_path_cb函数 (lines 83-90)
// 接受参考线轨迹
void LocalPlanner::globalPathCallback() {
    // std::lock_guard<std::mutex> lock(global_path_mutex_); // Mutex removed for simplicity
    int size = 100;// 100 个初始点
    if (size > 0) {
        global_path_ = Eigen::MatrixXd::Zero(size, 3);
        for (int i = 0; i < size; ++i) {
            global_path_(i, 0) = i*0.5; //msg->poses[i].pose.position.x;
            global_path_(i, 1) = 0.0; //msg->poses[i].pose.position.y;
            global_path_(i, 2) = 0.0; // 如果需要角度信息可以从四元数提取
        }
        global_path_received_ = true;
    }
}

// 对应Python: choose_goal_state函数 (lines 154-175)
bool LocalPlanner::chooseGoalState() {
    // std::lock_guard<std::mutex> curr_lock(curr_pose_mutex_); // Mutexes removed for simplicity
    // std::lock_guard<std::mutex> path_lock(global_path_mutex_);
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

/*
// MODIFICATION: Non-linear system model is no longer used for dynamics constraints.
// It is replaced by a linearized model inside the mpcEllip function.
casadi::MX LocalPlanner::systemModel(const casadi::MX& x, const casadi::MX& u) {
    return casadi::MX::vertcat({
        u(0) * casadi::MX::cos(x(2)),  // ẋ = v*cos(θ)
        u(0) * casadi::MX::sin(x(2)),  // ẏ = v*sin(θ)
        u(1)                           // θ̇ = ω
    });
}
*/

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
    
    // std::lock_guard<std::mutex> curr_lock(curr_pose_mutex_); // Mutexes removed for simplicity
    // std::lock_guard<std::mutex> path_lock(global_path_mutex_);
    // std::lock_guard<std::mutex> obs_lock(obstacle_mutex_);
    
    if (!curr_state_received_) {
        return std::make_pair(last_state_, last_input_);
    }
    
    if (!global_path_received_ || goal_state_.rows() != N_) {
        return std::make_pair(last_state_, last_input_);
    }
    
    casadi::Opti opti;
    
    double T = 0.1;        // 时间步长 (replan_period_)
    double gamma_k = 0.3;  // 障碍物约束松弛因子
    double v_max = 10;     // 最大线速度
    double v_min = 0.0;    // 最小线速度  
    double omega_max = 1.2; // 最大角速度
    
    try {
        auto opt_x0 = opti.parameter(3, 1);
        auto opt_states = opti.variable(N_ + 1, 3);
        auto opt_controls = opti.variable(N_, 2);
        
        auto v = opt_controls(casadi::Slice(), 0);
        auto omega = opt_controls(casadi::Slice(), 1);
        
        opti.subject_to(opt_states(0, casadi::Slice()) == opt_x0.T());
        
        bool near_goal = false;
        if (global_path_received_ && global_path_.rows() > 0) {
            Eigen::Vector2d curr_pos = curr_state_.head<2>();
            Eigen::Vector2d goal_pos = global_path_.bottomRows(1).leftCols(2).transpose();
            double dist_to_goal = distanceGlobal(curr_pos, goal_pos);
            near_goal = dist_to_goal <= 1.0;
        }
        
        if (!near_goal) {
            opti.subject_to(v >= v_min);
            opti.subject_to(v <= v_max);
        } else {
            opti.subject_to(v >= -v_min);
            opti.subject_to(v <= v_max); 
        }
        opti.subject_to(omega >= -omega_max);
        opti.subject_to(omega <= omega_max);
        
        // MODIFICATION: Replace non-linear dynamics with a linearized model
        //ROS_INFO("Starting to add LINEARIZED system model constraints");
        for (int i = 0; i < N_; ++i) {
            // 1. Get reference state and control for linearization
            Eigen::Vector3d x_ref = goal_state_.row(i);
            Eigen::Vector2d u_ref = Eigen::Vector2d::Zero();
            if (i < N_ - 1) {
                double dx = goal_state_(i+1, 0) - goal_state_(i, 0);
                double dy = goal_state_(i+1, 1) - goal_state_(i, 1);
                u_ref(0) = std::sqrt(dx*dx + dy*dy) / T; // v_ref
                double angle_diff = normalizeAngle(goal_state_(i+1, 2) - goal_state_(i, 2));
                u_ref(1) = angle_diff / T; // omega_ref
            } else {
                u_ref = (i > 0) ? last_input_.row(i-1) : Eigen::Vector2d::Zero();
            }

            // 2. Calculate Jacobian matrices at the reference point
            double v_ref = u_ref(0);
            double theta_ref = x_ref(2);
            double sin_theta_ref = std::sin(theta_ref);
            double cos_theta_ref = std::cos(theta_ref);

            Eigen::Matrix3d A_jac;
            A_jac << 0, 0, -v_ref * sin_theta_ref,
                     0, 0,  v_ref * cos_theta_ref,
                     0, 0,  0;

            Eigen::Matrix<double, 3, 2> B_jac;
            B_jac << cos_theta_ref, 0,
                     sin_theta_ref, 0,
                     0, 1;
            
            // 3. Form discrete-time linearized matrices
            Eigen::Matrix3d A_k = Eigen::Matrix3d::Identity() + T * A_jac;
            Eigen::Matrix<double, 3, 2> B_k = T * B_jac;
            
            // 4. Calculate correction term C_k
            Eigen::Vector3d x_ref_next = (i < N_ - 1) ? goal_state_.row(i+1).transpose() : goal_state_.row(N_-1).transpose();
            Eigen::Vector3d C_k = x_ref_next - A_k * x_ref - B_k * u_ref;

            // 5. Convert Eigen matrices to CasADi DM
            casadi::DM A_k_dm = casadi::DM::zeros(3,3);
            casadi::DM B_k_dm = casadi::DM::zeros(3,2);
            casadi::DM C_k_dm = casadi::DM::zeros(3,1);
            for(int r=0; r<3; ++r) {
                for(int c=0; c<3; ++c) A_k_dm(r,c) = A_k(r,c);
                for(int c=0; c<2; ++c) B_k_dm(r,c) = B_k(r,c);
                C_k_dm(r,0) = C_k(r);
            }
            
            // 6. Add the linear dynamics constraint
            casadi::MX x_curr = opt_states(i, casadi::Slice()).T();
            casadi::MX u_curr = opt_controls(i, casadi::Slice()).T();
            casadi::MX x_next_pred = casadi::MX::mtimes(A_k_dm, x_curr) + casadi::MX::mtimes(B_k_dm, u_curr) + C_k_dm;
            opti.subject_to(opt_states(i + 1, casadi::Slice()) == x_next_pred.T());
        }

        casadi::MX obj = 0;
        int num_obs = 0;
        if (!obstacles_.empty()) {
            double slack_weight = 1000.0;
            if (obstacles_.size() >= N_) {
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
                                if (idx1 < static_cast<int>(obstacles_.size()) && idx2 < static_cast<int>(obstacles_.size())) {
                                    auto h_curr = ellipseConstraint(opt_states(i, casadi::Slice(0, 2)).T(), obstacles_[idx1]);
                                    auto h_next = ellipseConstraint(opt_states(i + 1, casadi::Slice(0, 2)).T(), obstacles_[idx2]);
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
        
        Eigen::Matrix2d R = Eigen::Vector2d(0.1, 0.02).asDiagonal();
        for (int i = 0; i < N_; ++i) {
            Eigen::Vector3d q_diag(1.0 + 0.05*i, 1.0 + 0.05*i, 0.02 + 0.005*i);
            Eigen::Matrix3d Q = q_diag.asDiagonal();
            
            casadi::MX pos_error = casadi::MX::vertcat({
                opt_states(i, 0) - goal_state_(i, 0),
                opt_states(i, 1) - goal_state_(i, 1)
            });
            
            casadi::MX angle_current = opt_states(i, 2);
            casadi::MX angle_goal = casadi::MX(goal_state_(i, 2));
            casadi::MX angle_error_sin = casadi::MX::sin(angle_current - angle_goal);
            casadi::MX angle_error_cos = casadi::MX::cos(angle_current - angle_goal) - 1;
            
            casadi::MX pos_cost = pos_error(0) * pos_error(0) * Q(0,0) + 
                                  pos_error(1) * pos_error(1) * Q(1,1);
            
            casadi::MX angle_cost = Q(2,2) * (angle_error_sin * angle_error_sin + 
                                              angle_error_cos * angle_error_cos);
            
            if (i < N_ - 1) {
                casadi::MX control_error = opt_controls(i, casadi::Slice());
                obj += 0.1 * (pos_cost + angle_cost) + quadratic(control_error, R);
            } else {
                obj += 0.1 * (pos_cost + angle_cost);
            }
        }
        
        Eigen::Matrix3d Q_terminal = Eigen::Vector3d(5.0, 5.0, 0.1).asDiagonal();
        casadi::MX terminal_pos_error = casadi::MX::vertcat({
            opt_states(N_-1, 0) - goal_state_(N_-1, 0),
            opt_states(N_-1, 1) - goal_state_(N_-1, 1)
        });
        
        casadi::MX terminal_angle_current = opt_states(N_-1, 2);
        casadi::MX terminal_angle_goal = casadi::MX(goal_state_(N_-1, 2));
        casadi::MX terminal_angle_error_sin = casadi::MX::sin(terminal_angle_current - terminal_angle_goal);
        casadi::MX terminal_angle_error_cos = casadi::MX::cos(terminal_angle_current - terminal_angle_goal) - 1;
        
        casadi::MX terminal_pos_cost = terminal_pos_error(0) * terminal_pos_error(0) * Q_terminal(0,0) + 
                                       terminal_pos_error(1) * terminal_pos_error(1) * Q_terminal(1,1);
        casadi::MX terminal_angle_cost = Q_terminal(2,2) * (terminal_angle_error_sin * terminal_angle_error_sin + 
                                                             terminal_angle_error_cos * terminal_angle_error_cos);
        
        obj += terminal_pos_cost + terminal_angle_cost;
        opti.minimize(obj);
        
        casadi::Dict opts_setting;
        opts_setting["ipopt.max_iter"] = 2000;
        opts_setting["ipopt.print_level"] = 0;
        opts_setting["print_time"] = 0;
        opts_setting["ipopt.acceptable_tol"] = 1e-3;
        opts_setting["ipopt.acceptable_obj_change_tol"] = 1e-3;
        
        opti.solver("ipopt", opts_setting);
        
        opti.set_value(opt_x0, casadi::DM({curr_state_(0), curr_state_(1), curr_state_(2)}));
        
        Eigen::MatrixXd u_res, state_res;
        
        auto sol = opti.solve();
        
        auto u_sol = sol.value(opt_controls);
        auto state_sol = sol.value(opt_states);
        
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
        
        std::cout <<"Successfully converted solution to Eigen format" << std::endl;
        std::cout << "First control input: " << u_res(0, 0) << " " << u_res(0, 1) << std::endl;
        std::cout <<"First state:" << state_res(0, 0) << " " <<  state_res(0, 1) << " "<<  state_res(0, 2) << std::endl;
        
        last_input_ = u_res;
        last_state_ = state_res;
        mpc_success_ = true;
        
        return std::make_pair(state_res, u_res);
        
    } catch (const std::exception& e) {
        // ROS_ERROR("MPC solve failed, exception: %s", e.what());
        std::cerr << "MPC solve failed, exception: " << e.what() << std::endl;
        
        if (mpc_success_) {
            mpc_success_ = false;
        } else {
            if (last_input_.rows() >= N_) {
                for (int i = 0; i < N_ - 1; ++i) {
                    last_input_.row(i) = last_input_.row(i + 1);
                    last_state_.row(i) = last_state_.row(i + 1);
                }
                last_input_.row(N_ - 1) = Eigen::Vector2d::Zero();
            } else {
                last_input_ = Eigen::MatrixXd::Zero(N_, 2);
                last_state_ = Eigen::MatrixXd::Zero(N_ + 1, 3);
            }
        }
        
        Eigen::MatrixXd u_res = last_input_;
        Eigen::MatrixXd state_res = last_state_;
        
        return std::make_pair(state_res, u_res);
    }
}