 #include "../include/local_planner.h"

// 对应Python: if __name__ == '__main__': (lines 363-365)
// Main function
int main() {
    LocalPlanner planner;
    planner.replanCallback();
    
    std::cout << "Saving visualization data..." << std::endl;
    planner.saveDataForVisualization2D();
    planner.saveDataForVisualization3D();
    
    return 0;
}

