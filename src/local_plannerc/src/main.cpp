 #include "../include/local_planner.h"
#include <ros/ros.h>

// 对应Python: if __name__ == '__main__': (lines 363-365)
int main(int argc, char** argv) {
    // 对应Python: rospy.init_node("phri_planner") 
    ros::init(argc, argv, "local_planner_cpp");
    
    try {
        // 对应Python: phri_planner = Local_Planner()
        LocalPlanner planner;
        
        ROS_INFO("Local Planner C++ node started successfully");
        
        // 对应Python: rospy.spin()
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_ERROR("Local Planner failed to start: %s", e.what());
        return -1;
    }
    
    ROS_INFO("Local Planner node shutting down");
    return 0;
}