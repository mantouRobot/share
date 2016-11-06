/***************************************
主体思想是状态机》大状态》小状态
状态机提供和程序其他部分的接口
大状态管理小状态的公共数据
state_cast由状态机调用，返回当前状态的引用
context由当前运行的小状态调用，返回大状态
另外，通过纯虚函数实现基类的引用指向派生类对象
***************************************/

#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/transition.hpp>
#include <iostream>

#include <ctime>
namespace  sc = boost::statechart;

struct Running;
struct Active;
struct Stopped;

//定义事件
struct EvReset : sc::event<EvReset>{};
struct EvStartStop : sc::event<EvStartStop>{};

//定义状态机
//基类的引用可以指向派生类
struct IElapsedTime
{
  virtual double ElapsedTime() const = 0;
};

struct StopWatch : sc::state_machine<StopWatch, Active>
{
  //state_cast返回指定状态的引用
  double ElapseTime() const
  {
    return state_cast<const IElapsedTime &>().ElapsedTime();
  }
};

//定义大状态，一方面接受reset事件，另一方面管理子状态的公共变量
struct Active : sc::simple_state<Active, StopWatch, Stopped>
{
  typedef sc::transition<EvReset, Active> reactions;
  Active() :
    elapsedTime_(0.0)
  {}

  double ElapsedTime() const {return elapsedTime_;}
  double &ElapsedTime() {return elapsedTime_;}

  private:
  double elapsedTime_;
};

//定义小状态
struct Stopped : IElapsedTime, sc::simple_state<Stopped, Active>
{
  typedef sc::transition<EvReset, Running> reactions;
  virtual double ElapsedTime() const
  {
    return context<Active>().ElapsedTime();
  }
};

struct Running : IElapsedTime, sc::simple_state<Running, Active>
{
  typedef sc::transition<EvReset, Stopped> reactions;
  Running() : startTime_(std::time(0)){}
  //context<>返回指定状态的引用,本状态活着，则上级状态也活着
  ~Running()
  {
    context<Active>().ElapsedTime() += std::difftime(std::time(0), startTime_);
  }
  virtual double ElapsedTime() const
  {
    return context<Active>().ElapsedTime() + std::difftime(std::time(0), startTime_);
  }
  private:
    std::time_t startTime_;
};

int main(int argc, char** argv)
{
  StopWatch myWatch;
  myWatch.initiate();
  std::cout << myWatch.ElapseTime() << std::endl;//当前态处于暂停，因此state_cast返回暂停态的引用
  myWatch.process_event(EvStartStop());;
  std::cout << myWatch.ElapseTime() << std::endl;//当前态处于运行，因此state_cast返回运行态的引用
  myWatch.process_event(EvStartStop());;
  std::cout << myWatch.ElapseTime() << std::endl;//当前态处于运行，因此state_cast返回运行态的引用
  myWatch.process_event(EvStartStop());;
  std::cout << myWatch.ElapseTime() << std::endl;//当前态处于运行，因此state_cast返回运行态的引用
  myWatch.process_event(EvStartStop());;
  std::cout << myWatch.ElapseTime() << std::endl;//当前态处于运行，因此state_cast返回运行态的引用
  return 0;
}
