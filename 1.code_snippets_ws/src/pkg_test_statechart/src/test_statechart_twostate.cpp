#include <iostream>
#include <ctime>

#include "boost/statechart/state_machine.hpp"
#include "boost/statechart/simple_state.hpp"
#include "boost/statechart/transition.hpp"
#include "boost/statechart/event.hpp"

namespace sc = boost::statechart;

//定义事件    《根据事件 切换 状态》
class EvtStartStop : public sc::event< EvtStartStop > {};
class EvtReset : public sc::event< EvtReset > {};
class EvtGo : public sc::event< EvtGo > {};

class MainState;
class StopState;
class RunState;
class TwoState;

//定义状态机                             初始化状态 MainState
class Machine : public sc::state_machine< Machine, MainState > {
public:
  Machine()
  {
    std::cout << "Enter the state machine." << std::endl;
  }
  ~Machine()
  {
    std::cout << "Exit the state machine." << std::endl;
  }
};

//定义 MainState 状态 ， 它属于Machine， 它的子状态为 StopState
class MainState : public sc::simple_state< MainState, Machine, StopState >
{
public:
    typedef sc::transition< EvtReset, MainState > reactReset; //状态切换
    typedef sc::transition< EvtGo, TwoState > reactGo;            //状态切换

    typedef boost::mpl::list< reactReset, reactGo > reactions;  //reactions 切不可拼写错误
    //一个状态可以定义任意数量的动作。这就是为什么当多于一个时，我们不得不将它们放到一个mpl::list<> 里。

    MainState(void)
    {
        std::cout<<"Enter MainState"<<std::endl;
        mTime = 0;
    }

    ~MainState(void)
    {
        std::cout<<"Exit MainState"<<std::endl;
    }

    double mTime;
};


// 该状态属于无用状态，用于测试mpl::list的多transition用法
class TwoState : public sc::simple_state< TwoState, Machine >
{
public:
    typedef sc::transition< EvtGo, MainState > reactions; //状态切换

    TwoState(void)
    {
        std::cout<<"Enter TwoState"<<std::endl;
    }

    ~TwoState(void)
    {
        std::cout<<"Exit TwoState"<<std::endl;
    }
};


class StopState : public sc::simple_state< StopState, MainState >
{
public:
    typedef sc::transition< EvtStartStop, RunState > reactions; //状态切换

    StopState(void)
    {
        std::cout<<"Enter StopState"<<std::endl;
    }

    ~StopState(void)
    {
        std::cout<<"Exit StopState"<<std::endl;
    }
};

class RunState : public sc::simple_state< RunState, MainState >
{
public:
    typedef sc::transition< EvtStartStop, StopState > reactions;
    RunState(void)
    {
        std::cout<<"Enter RunState"<<std::endl;
        mStartTime = 0;
    }

    ~RunState(void)
    {
        std::cout<<"Exit RunState"<<std::endl;
        context<MainState>().mTime += std::difftime(std::time(0), mStartTime);
    }

    std::time_t mStartTime;
};


int main(int argc, char* argv[])
{
    Machine mc;
    mc.initiate();//进入主态，进入暂停态

//    mc.process_event(EvtStartStop());//离开暂停态，进入运行态
//    std::cout<<std::endl;
//    mc.process_event(EvtStartStop());//离开运行太，进入暂停态
//    std::cout<<std::endl;
//    mc.process_event(EvtReset());//离开暂停态，离开主态，进入主态，进入暂停态
//    std::cout<<std::endl;
//    mc.process_event(EvtGo());//离开暂停态，离开第一主态，进入第二主态
//    std::cout<<std::endl;
//    mc.process_event(EvtGo());//离开第二主态，进入第一主态，进入暂停态

    return 0;
}
