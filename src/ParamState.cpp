#include "dwa_navigation/ParamState.h"

using namespace ParamStateMachine;

BaseState* BaseState::CheckStateJump()
{
    for(auto& pair : next_states){
        if((*pair.first)(this)){
            BaseState* result = pair.second;
            if(dynamic_cast<Conduit*>(pair.second)){
                result = pair.second->CheckStateJump();
            }

            if(result) return result;
        }
    }
    return nullptr;
}

void BaseState::OnEnter()
{
    time = 0;
}

void BaseState::OnExit()
{
}

void BaseState::ConnectTo(Condition condition, BaseState* state)
{
    next_states.emplace_back(condition, state);
    //return next_states.size() - 1;
}

BaseState* Conduit::CheckStateJump()
{
    if(condition(this)) return BaseState::CheckStateJump();
    return nullptr;
}

void ParamState::Update(float deltatime)
{
    time+=deltatime;
}

StateMachine::StateMachine(DwaPlanner* dwa_planner, const std::vector<std::vector<float>> &AllStates, const std::vector<Condition> &Conduits, const std::vector<Connection> &Connection, int entrance) : dwa_planner(dwa_planner)
{
    for(auto& param : AllStates){
        AddParamState(param[0], param[1], param[2]);
    }
    for(auto& condition : Conduits){
        AddConduit(condition);
    }
    for(auto& connect : Connection){
        ConnectStates(connect.from, connect.to, connect.condition);
    }
    current_state = dynamic_cast<ParamState*>(states[entrance]);
}

void StateMachine::Update(float deltatime)
{
    current_state->Update(deltatime);
    BaseState* next_state = current_state->CheckStateJump();
    ParamState* next_param_state = dynamic_cast<ParamState*>(next_state);
    if(next_param_state){
        SwitchTo(next_param_state);
    }
}

std::tuple<float, float, float> StateMachine::GetParams()
{
    return std::tuple<float, float, float>(current_state->kcollision, current_state->khead, current_state->kvel);
}

int StateMachine::AddParamState(float kcollision, float khead, float kvel)
{
    BaseState* state = new ParamState(this, kcollision, khead, kvel);
    states.push_back(state);
    return states.size() - 1;
}
int StateMachine::AddConduit(Condition condition)
{
    BaseState* state = new Conduit(this, condition);
    states.push_back(state);
    return states.size() - 1;
}

void StateMachine::ConnectStates(int from, int to, Condition condition)
{
    states[from]->ConnectTo(condition, states[to]);
}
void StateMachine::SwitchTo(ParamState *new_state)
{
    current_state->OnExit();
    current_state = new_state;
    current_state->OnEnter();
}