#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <mutex>
#include <vector>
#include <memory>

class LocalPlanner {
private:
    // 对应Python: self.replan_period等参数
    double replan_period_;
    int N_;  // 预测步数，对应Python: self.N = 25
    double z_; // 高度，对应Python: self.z = 0
    
    // 对应Python: self.curr_state, self.global_path, self.ob
    Eigen::Vector3d curr_state_;
    Eigen::MatrixXd global_path_;
    std::vector<Eigen::VectorXd> obstacles_; // 对应Python: self.ob = []
    Eigen::MatrixXd goal_state_; // 对应Python: self.goal_state
    
    // 对应Python: self.last_input, self.last_state, self.mpc_success
    Eigen::MatrixXd last_input_;
    Eigen::MatrixXd last_state_;
    bool mpc_success_;
    
    // 对应Python的threading.Lock()
    std::mutex curr_pose_mutex_;
    std::mutex global_path_mutex_;
    std::mutex obstacle_mutex_;
    
    // 状态标志
    bool curr_state_received_;
    bool global_path_received_;
    
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
    casadi::MX quadratic(const casadi::MX& x, const Eigen::MatrixXd& A);
    
    // 对应Python中的f函数 - 系统模型
    casadi::MX systemModel(const casadi::MX& x, const casadi::MX& u);
};

#endif // LOCAL_PLANNER_H