#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <iostream>

namespace sc = boost::statechart;

// We are declaring all types as structs only to avoid having to
// type public. If you don't mind doing so, you can just as well
// use class.

// We need to forward-declare the initial state because it can
// only be defined at a point where the state machine is
// defined.
struct Greeting; //前向声明初始状态，先定义状态机再定义状态，故前向声明

// Boost.Statechart makes heavy use of the curiously recurring
// template pattern. The deriving class must always be passed as
// the first parameter to all base class templates.
//
// The state machine must be informed which state it has to
// enter when the machine is initiated. That's why Greeting is
// passed as the second template parameter.
struct Machine : sc::state_machine< Machine, Greeting > {};//第一个参数为状态机自身，第二个参数为初始状态

// For each state we need to define which state machine it
// belongs to and where it is located in the statechart. Both is
// specified with Context argument that is passed to
// simple_state<>. For a flat state machine as we have it here,
// the context is always the state machine. Consequently,
// Machine must be passed as the second template parameter to
// Greeting's base (the Context parameter is explained in more
// detail in the next example).
struct Greeting : sc::simple_state< Greeting, Machine >//第一个参数为状态自身，第二个参数为状态机
{
  // Whenever the state machine enters a state, it creates an
  // object of the corresponding state class. The object is then
  // kept alive as long as the machine remains in the state.
  // Finally, the object is destroyed when the state machine
  // exits the state. Therefore, a state entry action can be
  // defined by adding a constructor and a state exit action can
  // be defined by adding a destructor.
  //当状态机进入某个状态，状态机即创建和状态关联的类的对象。只要状态机
  //处于当前状态，这个对象就一直存在。当状态机退出该状态，这个对象销毁。
  //因此，一个状态进入的action通过构造函数来体现，退出的action通过析构体现。
  Greeting() { std::cout << "Hello World!\n"; } // entry
  ~Greeting() { std::cout << "Bye Bye World!\n"; } // exit
};

int main()
{
  Machine myMachine;
  // The machine is not yet running after construction. We start
  // it by calling initiate(). This triggers the construction of
  // the initial state Greeting
  //调用initiate触发初始状态的构造
  myMachine.initiate();
  // When we leave main(), myMachine is destructed what leads to
  // the destruction of all currently active states.
  return 0;
}
