#include "../include/controller.h"
#include <ros/ros.h>

// 对应Python: if __name__ == '__main__': (lines 63-65)
int main(int argc, char** argv) {
    // 对应Python: rospy.init_node('control')
    ros::init(argc, argv, "controller_cpp");
    
    try {
        // 对应Python: controller = Controller()
        Controller controller;
        
        ROS_INFO("Controller C++ node started successfully");
        
        // 控制循环在构造函数中启动，这里只需要spin处理回调
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_ERROR("Controller failed to start: %s", e.what());
        return -1;
    }
    
    ROS_INFO("Controller node shutting down");
    return 0;
}