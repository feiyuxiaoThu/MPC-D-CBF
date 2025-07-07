 #include "../include/local_planner.h"

// 对应Python: if __name__ == '__main__': (lines 363-365)
int main() {

    std::cout << "we start mpc" << std::endl;
    LocalPlanner planner;
    planner.replanCallback();
        

    
    //ROS_INFO("Local Planner node shutting down");
    return 0;
}