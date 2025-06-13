#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

class Controller {
public:
    Controller();
    ~Controller();

private:
    // 对应Python: __init__函数的变量
    ros::NodeHandle nh_;
    int N_;                              // 对应Python: self.N = 10
    ros::Rate rate_;                     // 对应Python: self.rate = rospy.Rate(50)
    
    // 对应Python的Subscriber和Publisher
    ros::Subscriber local_plan_sub_;     // 对应Python: self.local_plan_sub
    ros::Publisher vel_pub_;             // 对应Python: self.vel_pub  
    ros::Publisher curr_state_pub_;      // 对应Python: self.curr_state_pub
    
    // 对应Python: self.__timer_localization
    ros::Timer timer_localization_;

    ros::Timer control_timer_;
    
    // TF相关 (对应Python: self.listener = tf.TransformListener())
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // 控制变量 (对应Python: self.linear_speed, self.angular_speed)
    double linear_speed_;
    double angular_speed_;
    
    // 局部规划 (对应Python: self.local_plan = np.zeros([self.N, 2]))
    Eigen::MatrixXd local_plan_;
    
    // 对应Python的方法
    void quartToRpy(double x, double y, double z, double w, double& roll, double& pitch, double& yaw);
    void getCurrentState(const ros::TimerEvent& event);
    void pubVel();
    void controlCallback(const ros::TimerEvent& event);
    void localPlannerCb(const std_msgs::Float32MultiArray::ConstPtr& msg);
};

#endif // CONTROLLER_H