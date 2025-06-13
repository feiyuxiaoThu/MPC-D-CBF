#include "../include/controller.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// 对应Python: __init__函数 (lines 12-26)
Controller::Controller() : nh_("~"), rate_(50), tf_listener_(tf_buffer_) {
    // 对应Python: self.N = 10
    N_ = 10;
    
    // 对应Python: self.local_plan_sub = rospy.Subscriber('/local_plan', ...)
    local_plan_sub_ = nh_.subscribe("/local_plan", 10, 
                                   &Controller::localPlannerCb, this);
    
    // 对应Python: self.vel_pub = rospy.Publisher('/cmd_vel', ...)
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    // 对应Python: self.curr_state_pub = rospy.Publisher('/curr_state', ...)
    curr_state_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/curr_state", 10);
    
    // 对应Python: self.__timer_localization = rospy.Timer(rospy.Duration(0.01), ...)
    timer_localization_ = nh_.createTimer(ros::Duration(0.01), 
                                         &Controller::getCurrentState, this);

                                         // 添加控制定时器，替代无限循环
    control_timer_ = nh_.createTimer(ros::Duration(0.02), // 50Hz
                                    &Controller::controlCallback, this);
    
    // 对应Python: self.linear_speed = self.angular_speed = 0.0
    linear_speed_ = 0.0;
    angular_speed_ = 0.0;
    
    // 对应Python: self.local_plan = np.zeros([self.N, 2])
    local_plan_ = Eigen::MatrixXd::Zero(N_, 2);
    
    ROS_INFO("Controller initialized");
    
    // 对应Python: self.control_loop()
    //controlLoop();
}

Controller::~Controller() {}

// 对应Python: quart_to_rpy函数 (lines 28-32)
void Controller::quartToRpy(double x, double y, double z, double w, 
                           double& roll, double& pitch, double& yaw) {
    roll = std::atan2(2*(w*x+y*z), 1-2*(x*x+y*y));
    pitch = std::asin(2*(w*y-z*x));
    yaw = std::atan2(2*(w*z+x*y), 1-2*(z*z+y*y));
}

// 对应Python: get_current_state函数 (lines 34-41)
void Controller::getCurrentState(const ros::TimerEvent& event) {
    try {
        // 使用最新时间而不是ros::Time(0)
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            "world", "base_link", ros::Time::now(), ros::Duration(0.1));
        
        double roll, pitch, yaw;
        quartToRpy(transform.transform.rotation.x,
                   transform.transform.rotation.y,
                   transform.transform.rotation.z,
                   transform.transform.rotation.w,
                   roll, pitch, yaw);
        
        std_msgs::Float32MultiArray curr_state;
        curr_state.data.resize(3);
        curr_state.data[0] = transform.transform.translation.x;
        curr_state.data[1] = transform.transform.translation.y;
        curr_state.data[2] = yaw;
        
        curr_state_pub_.publish(curr_state);
        
        // 添加调试信息
        ROS_INFO_THROTTLE(2.0, "Published curr_state: [%.2f, %.2f, %.2f]", 
                         curr_state.data[0], curr_state.data[1], curr_state.data[2]);
        
    } catch (tf2::TransformException& ex) {
        ROS_ERROR_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
    }
}

// 对应Python: pub_vel函数 (lines 43-48)
void Controller::pubVel() {
    geometry_msgs::Twist control_cmd;
    control_cmd.linear.x = linear_speed_;
    control_cmd.angular.z = angular_speed_;
    
    ROS_INFO("Linear Speed: %.1f, Angular Speed: %.1f", linear_speed_, angular_speed_);
    vel_pub_.publish(control_cmd);
}

// 对应Python: control_loop函数 (lines 50-55)
// 新增控制回调函数
void Controller::controlCallback(const ros::TimerEvent& event) {
    linear_speed_ = local_plan_(0, 0);
    angular_speed_ = local_plan_(0, 1);
    pubVel();
}

// 对应Python: local_planner_cb函数 (lines 57-60)
void Controller::localPlannerCb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    // 对应Python: for i in range(self.N):
    for (int i = 0; i < N_ && i*2+1 < msg->data.size(); ++i) {
        // 对应Python: self.local_plan[i, 0] = msg.data[0+2*i]
        local_plan_(i, 0) = msg->data[0 + 2*i];
        // 对应Python: self.local_plan[i, 1] = msg.data[1+2*i]  
        local_plan_(i, 1) = msg->data[1 + 2*i];
    }
}