#pragma once
#include <vector>
#include <memory>

class DwaPlanner;

namespace ParamStateMachine{
    class BaseState;
    class StateMachine;

    typedef bool(*Condition)(BaseState*);

    class BaseState
    {
    public:
        BaseState(StateMachine* owner) : machine(owner){}

        //virtual void Update(float deltatime);
        virtual BaseState* CheckStateJump();

        virtual void OnEnter();
        virtual void OnExit();

        void ConnectTo(Condition condition, BaseState* state);

    //tool
    public:
        float time;
        StateMachine* machine;
    protected:
        std::vector<std::pair<Condition, BaseState*>> next_states;
    };

    class Conduit : public BaseState
    {
    public:
        Conduit(StateMachine* owner, Condition condition) : BaseState(owner), condition(condition){}
        virtual BaseState* CheckStateJump() override;
    protected:
        Condition condition;
    };

    class ParamState : public BaseState
    {
    public:
        ParamState(StateMachine* owner, float incollision, float inhead, float invel) : BaseState(owner), kcollision(incollision), khead(inhead), kvel(invel){}
        virtual void Update(float deltatime);
        float kcollision, khead, kvel;
    };

    struct Connection{
        float from, to;
        Condition condition;
    };

    class StateMachine{
    public:
        StateMachine(DwaPlanner* dwa_planner, const std::vector<std::vector<float>>& AllStates, const std::vector<Condition>& Conduits, const std::vector<Connection>& Connection, int entrance = 0);
        ~StateMachine(){
            for(BaseState* state : states) delete state;
        }

        void Update(float deltatime);
        std::tuple<float, float, float> GetParams();

        int AddParamState(float kcollision, float khead, float kvel);
        int AddConduit(Condition condition);
        void ConnectStates(int from, int to, Condition condition);

        DwaPlanner* dwa_planner;
    private:
        void SwitchTo(ParamState* new_state);
        ParamState* current_state;
        std::vector<BaseState*> states;
    };
}

