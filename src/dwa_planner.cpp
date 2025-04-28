#include "dwa_navigation/DwaPlanner.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "dwa_planner");
    DwaPlanner planner;
    planner.run();
    return 0;
}