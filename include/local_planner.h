#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H



#include <iostream>
#include <vector>
#include <mutex>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "osqp/osqp.h"


class LocalPlanner {
public:
    LocalPlanner();
    ~LocalPlanner();

    void replanCallback();
    void saveDataForVisualization2D() const;
    void saveDataForVisualization3D() const;
    double replan_period_;
    bool mpc_success_;

private:
    // ROS related members
    // ros::NodeHandle nh_;
    // ros::Subscriber curr_state_sub_;
    // ros::Subscriber obs_sub_;
    // ros::Subscriber global_path_sub_;
    // ros::Publisher local_path_pub_;
    // ros::Publisher local_plan_pub_;
    // ros::Publisher local_path_vis_pub_;
    // ros::Publisher cmd_move_pub_;

    // MPC parameters
    int N_;
    double z_;

    // State and Path data
    Eigen::Vector3d curr_state_;
    Eigen::MatrixXd global_path_;
    Eigen::MatrixXd goal_state_;
    std::vector<Eigen::VectorXd> obstacles_;

    // MPC solution
    Eigen::MatrixXd last_input_;
    Eigen::MatrixXd last_state_;

    // Flags and Mutexes
    bool curr_state_received_;
    bool global_path_received_;
    //std::mutex curr_pose_mutex_;
    //std::mutex obstacle_mutex_;
    //std::mutex global_path_mutex_;

    // Callback functions
    void currPoseCallback();
    void obsCallback();
    void globalPathCallback();

    // Helper functions
    bool chooseGoalState();
    double distanceGlobal(const Eigen::Vector2d& c1, const Eigen::Vector2d& c2);
    double normalizeAngle(double angle);
    double angleDifference(double angle1, double angle2);
    bool exceedOb(const Eigen::VectorXd& ob);

    // MPC core function
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> mpcEllip();

    // 辅助函数，用于将Eigen稀疏矩阵转换为OSQP可用的CSC格式
    void eigenToCSC(const Eigen::SparseMatrix<double>& eigen_mat, std::vector<c_float>& csc_values, std::vector<c_int>& csc_indices, std::vector<c_int>& csc_pointers);

    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> mpcEllipWithOsqp();

    // Publisher function
    // void publishLocalPlan(const Eigen::MatrixXd& input_sol, const Eigen::MatrixXd& state_sol)
    
    
};

#endif // LOCAL_PLANNER_H
