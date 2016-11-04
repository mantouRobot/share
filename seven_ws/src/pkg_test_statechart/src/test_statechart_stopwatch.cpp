//#include <boost/statechart/event.hpp>
//#include <boost/statechart/state_machine.hpp>
//#include <boost/statechart/simple_state.hpp>
//#include <boost/statechart/transition.hpp>

//namespace sc = boost::statechart;

//struct EvStartStop : sc::event< EvStartStop > {};
//struct EvReset : sc::event< EvReset > {};

//struct Active;
//struct StopWatch : sc::state_machine< StopWatch, Active > {};

//struct Stopped;

//// The simple_state class template accepts up to four parameters:
//// - The third parameter specifies the inner initial state, if
////   there is one. Here, only Active has inner states, which is
////   why it needs to pass its inner initial state Stopped to its
////   base
//// - The fourth parameter specifies whether and what kind of
////   history is kept

//// Active is the outermost state and therefore needs to pass the
//// state machine class it belongs to
////大状态里有两个小状态，大状态定义时指定大状态的初始状态
//struct Active : sc::simple_state< Active, StopWatch, Stopped >
//{
//  typedef sc::transition< EvReset, Active > reactions;
//};

//// Stopped and Running both specify Active as their Context,
//// which makes them nested inside Active
//struct Running : sc::simple_state< Running, Active >
//{
//  typedef sc::transition< EvStartStop, Stopped > reactions;
//};

//struct Stopped : sc::simple_state< Stopped, Active >
//{
//  typedef sc::transition< EvStartStop, Running > reactions;
//};

//// Because the context of a state must be a complete type (i.e.
//// not forward declared), a machine must be defined from
//// "outside to inside". That is, we always start with the state
//// machine, followed by outermost states, followed by the direct
//// inner states of outermost states and so on. We can do so in a
//// breadth-first or depth-first way or employ a mixture of the
//// two.

//int main()
//{
//  StopWatch myWatch;
//  myWatch.initiate();
//  myWatch.process_event( EvStartStop() );
//  myWatch.process_event( EvStartStop() );
//  myWatch.process_event( EvStartStop() );
//  myWatch.process_event( EvReset() );
//  return 0;
//}


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
class Machine : public sc::state_machine< Machine, MainState > {};

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


int _tmain(int argc, _TCHAR* argv[])
{
    Machine mc;
    mc.initiate();

    mc.process_event(EvtStartStop());
    std::cout<<std::endl;
    mc.process_event(EvtStartStop());
    std::cout<<std::endl;
    mc.process_event(EvtReset());
    std::cout<<std::endl;
    mc.process_event(EvtGo());
    std::cout<<std::endl;
    mc.process_event(EvtGo());

    return 0;
}
