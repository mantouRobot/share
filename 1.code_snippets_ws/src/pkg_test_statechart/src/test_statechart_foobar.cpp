#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>

#include <ros/ros.h>

namespace sc = boost::statechart;

//定义事件
class EvtStart : public sc::event<EvtStart>{};
class EvtAtoB : public sc::event<EvtAtoB>{};
class EvtAtoEnd : public sc::event<EvtAtoEnd>{};
class EvtBtoA : public sc::event<EvtBtoA>{};

class Foo;
class Bar;
class Machine;
class Active;

//对状态机来说，状态构建结束，事件队列没有任务，则状态机就析构，接着状态开始析构
//状态机先析构，构造析构不用做任何处理
class Machine : public sc::state_machine< Machine, Active >
{
public:
    Machine()
    {
      ROS_INFO("Enter state machine, initial state is Active.");
    }
    ~Machine()
    {
      ROS_WARN("Exit state machine.");
    }
};

//state构造特殊一点，其他一致
class Active : public sc::state<Active, Machine, Foo>
{
public:
  int count;
  int getCount() const { return count;}
  Active(my_context ctx) :
    my_base(ctx),
    count(0)
  {
    ROS_INFO("Enter Active, initial state is Foo.");
  }
  ~Active()
  {
    ROS_WARN("Exit the Active.");
  }
};

//post, process区别，transiz的用法在return custom reaction中
class Foo : public sc::state< Foo, Active >
{
public:
  typedef sc::transition<EvtAtoB, Bar> reactions;

  Foo(my_context ctx) : my_base(ctx)
  {
    ROS_INFO("ENTER Foo");
    if(context<Active>().count < 3)
    {
      context<Active>().count += 1;
      post_event(EvtAtoB());
//       transit<Bar>();
    }
  }

//  sc::result react( const EvtStart &a)
//  {
//    post_event(EvtAtoB());
//    return transit<Bar>();
//  }

//  void execute()
//  {
//    context<Active>().count += 1;
//    if(context<Active>().count < 3)
//      transit<Bar>();
//  }

  ~Foo()
  {
    ROS_WARN("Exit Foo.");
  }
};

class Bar : public sc::state<Bar, Active>
{
public:
  typedef sc::transition<EvtBtoA, Foo> reactions;
  Bar(my_context ctx) : my_base(ctx)
  {
    ROS_INFO("Enter the Bar.");
    post_event(EvtBtoA());
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
  myMachine.process_event(EvtStart());
  return 0;
}



























