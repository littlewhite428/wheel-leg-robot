#!/usr/bin/python
'''

'''

import rospy
import sys
from enum import Enum
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32,Float64
class FSM_STATE(Enum):
    IDLE = 0
    INIT = 1
    SENDING = 2

class joint_linear_traj():
    def __init__(self,joint_numbers,joint_names,loop_rate,queue_size):
        self.state = FSM_STATE.IDLE
        self.joint_nums = joint_numbers
        self.joint_names = joint_names
        self.joint_states = {}
        self.goal_states = {}
        self.goal_pub = {}
        self.commands = {}
        self.loop_rate = loop_rate
        self.queue_size = queue_size
        self.cnt = 0
        

        if len(joint_names) == self.joint_nums:
            rospy.loginfo('joint_nums :'+str(self.joint_nums))
        else:
            rospy.logfatal('len(joint_names) != joint_nums ! exit !')
            sys.exit(-1)
        for name in self.joint_names:
            self.joint_states[name] = 0.0
            self.goal_states[name] = 0.0
            self.goal_pub[name] = rospy.Publisher('/wheel_leg/'+name.strip('joint_')+'_controller/command',Float64,queue_size=10)
        self.joint_subscriber = rospy.Subscriber('/wheel_leg/joint_states',JointState,self.joint_state_cb)
        self.fsm_subscriber = rospy.Subscriber('/wheel_leg/trigger',Int32,self.fsm_state_cb)

    # get the current value
    def joint_state_cb(self,msg):
        for i,each_name in enumerate(msg.name):
            if each_name in self.joint_states.keys():
                self.joint_states[each_name] = msg.position[i]
    # set goal value
    def fsm_state_cb(self,msg):
        if not self.state == FSM_STATE.IDLE:
            return
        name = ['joint_left_hip_left_leg1', 'joint_right_hip_right_leg1', 'joint_left_leg1_leg2', 'joint_right_leg1_leg2']
        if msg.data == 1.0:
            value = [-0.5236, -0.5236, 1.0472, 1.0472]
        else:
            value = [0.0, 0.0, 0.0, 0.0]
        # during_time = 2.0
        for i,each_name in enumerate(name):
            if each_name in self.goal_states.keys():
                self.goal_states[each_name] = value[i]
                now = self.joint_states[each_name]
                desire = self.goal_states[each_name]
                self.commands[each_name] = []
                # generate the control sequene
                for j in range(self.queue_size):
                    self.commands[each_name].append(now+(desire-now)/self.queue_size*(j+1))

        self.state = FSM_STATE.INIT

    def fsm_loop(self):
        rospy.loginfo_throttle(1,str(self.state))
        if self.state == FSM_STATE.IDLE:
            return
        elif self.state == FSM_STATE.INIT:
            # cnt zero jump to  SENDING
            self.state = FSM_STATE.SENDING
            self.cnt = 0
            return
        elif self.state == FSM_STATE.SENDING:
            # publish each goal command
            for each_name in self.joint_names:
                self.goal_pub[each_name].publish(Float64(self.commands[each_name][self.cnt]))
            self.cnt = self.cnt + 1
            if self.cnt == self.queue_size:
                self.cnt = 0
                self.state = FSM_STATE.IDLE
        else:
            rospy.logerr('No such FSM state !')

def main():
    rospy.init_node('traj')
    r = 10
    linear_traj = joint_linear_traj(4,['joint_left_hip_left_leg1', 'joint_right_hip_right_leg1', 'joint_left_leg1_leg2', 'joint_right_leg1_leg2'],r,30)
    rat = rospy.Rate(r)
    while not rospy.is_shutdown():
        linear_traj.fsm_loop()
        rat.sleep()

if __name__ == '__main__':
    main()