#pragma once
#include "ParamState.h"
#include <vector>
#include "DwaPlanner.h"

using namespace ParamStateMachine;

bool StuckForSeconds(BaseState* current_state){
    return current_state->machine->dwa_planner->goal_received_ && current_state->time > 3;
}

bool SpeedUp(BaseState* current_state){
    double current_v = current_state->machine->dwa_planner->odom_data_.twist.twist.linear.x;
    return current_v > 0.8;
}

bool SpeedDown(BaseState* current_state){
    double current_v = current_state->machine->dwa_planner->odom_data_.twist.twist.linear.x;
    return current_v < 0.8;
}

bool RotateUp(BaseState* current_state){
    double current_w = current_state->machine->dwa_planner->odom_data_.twist.twist.angular.z;
    return abs(current_w) > 1.5;
}

bool RotateDown(BaseState* current_state){
    double current_w = current_state->machine->dwa_planner->odom_data_.twist.twist.angular.z;
    return abs(current_w) < 1.2;
}

bool Unstucked(BaseState* current_state){
    return RotateUp(current_state) || SpeedUp(current_state);
}

bool ReachGoal(BaseState* current_state){
    return current_state->machine->dwa_planner->goal_received_ == false || current_state->machine->dwa_planner->checkGoalReached();
}

bool NearlyHeadingTarget(BaseState* current_state){
    if(!current_state->machine->dwa_planner->goal_received_ || current_state->machine->dwa_planner->checkGoalReached()) return false;
    
    return abs(current_state->machine->dwa_planner->heading()) < 1;
}

std::vector<std::vector<float>> AllStates = {{1, 1, 1}, {3, 8, 1}, {1, 1, 1}, {4, 1.5, 3}, {0,10,0}};
std::vector<Condition> Conduits = {};
std::vector<Connection> Connections = {{0, 1, SpeedUp}, {1,0, SpeedDown}, {0,2, RotateDown}, {2,0, RotateUp}, {2,1, SpeedUp}, {2,3, StuckForSeconds}, {3, 0, Unstucked},
                                       {4, 0,NearlyHeadingTarget}, {0, 4, ReachGoal}, {1, 4, ReachGoal}, {2, 4, ReachGoal}, {3, 4, ReachGoal}};

int Entrance = 4;