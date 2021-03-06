#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

print 'hello, start smach test'

class Foo(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['outcome1','outcome2'])
    self.counter = 0

#此处counter不应该每次都初始化为0了吗？
#是不是不存在构造析构的状态类这一说？
  def execute(self, userdata):
    rospy.loginfo('Executing state FOO')
    if self.counter < 3:
      self.counter += 1
      return 'outcome1'
    else:
      return 'outcome2'

class Bar(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['outcome2'])

  def execute(self, userdata):
    rospy.loginfo('Executing state BAR')
    return 'outcome2'

def main():
  rospy.init_node('smach_test_node')
  sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])
  with sm:
    smach.StateMachine.add('FOO', Foo(),
                           transitions={'outcome1':'BAR',
                                        'outcome2':'outcome4'})
    smach.StateMachine.add('BAR', Bar(),
                           transitions={'outcome2':'FOO'})
  outcome = sm.execute()

if __name__ == '__main__':
  main()
