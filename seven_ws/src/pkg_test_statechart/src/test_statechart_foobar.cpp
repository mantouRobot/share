#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/transition.hpp>
#include <ros/ros.h>

namespace sc = boost::statechart;

//定义事件
class EvtAtoB : public sc::event<EvtAtoB>{};
class EvtAtoEnd : public sc::event<EvtAtoEnd>{};
class EvtBtoA : public sc::event<EvtBtoA>{};

class Foo;
class Bar;
class Machine;

class Machine : public sc::state_machine< Machine, Foo >
{
public:
    Machine()
    {
      ROS_INFO("Enter state machine, initial state is Foo");
    }
    ~Machine()
    {
      ROS_WARN("Exit state machine.");
    }
};


class Foo : public sc::simple_state< Foo, Machine >
{
public:
  typedef sc::transition<EvtAtoB, Bar> reactions;
  Foo() :
    counter(0)
  {
    ROS_INFO("ENTER Foo");

  }
  ~Foo()
  {
    ROS_WARN("Exit Foo.");
  }
  int counter;
};

class Bar : public sc::simple_state<Bar, Machine>
{
public:
  Bar()
  {
    ROS_INFO("Enter the Bar.");
  }
  ~Bar()
  {
    ROS_WARN("Exit the Bar.");
  }
};
int main()
{
  Machine myMachine;
  myMachine.initiate();
  return 0;
}



























