#!/usr/bin/python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import math
class roll_controller():
    def __init__(self):
        self.leg_length = 0.13

        self.height_des = 0.259
        self.height = 0.259
        
        self.left_height = 0.259
        self.right_height = 0.259

        self.roll_des = 0.0
        self.roll = 0.0
        self.last_last_height_err, self.last_height_err ,self.height_err, = 0,0,0
        self.last_last_roll_err, self.last_roll_err ,self.roll_err, = 0,0,0

        self.joint_names = ['joint_left_hip_left_leg1', 'joint_right_hip_right_leg1', 'joint_left_knee_leg2', 'joint_right_knee_leg2']
        self.joint_states = {}
        self.goal_states = {}
        self.goal_pub = {}
        for name in self.joint_names:
            self.joint_states[name] = 0.0
            self.goal_states[name] = 0.0
            self.goal_pub[name] = rospy.Publisher('/wheel_leg/'+name.strip('joint_')+'_controller/command',Float64,queue_size=10)


        self.Kp_h = 0.1
        self.Ki_h = 0.05
        self.Kp_r = 0.0
        self.Kd_r = 0.0

        self.roll_sub = rospy.Subscriber('/balance_state_roll',Float64,self.roll_cb)
        self.joint_sub = rospy.Subscriber('/wheel_leg/joint_states',JointState,self.joint_state_cb)

        self.des_roll_sub = rospy.Subscriber('/wheel_leg/des_roll',Float64,self.des_roll_cb)
        self.des_height_sub = rospy.Subscriber('/wheel_leg/des_height',Float64,self.des_height_cb)
    # get roll angle
    def roll_cb(self,msg):
        self.roll = msg.data
    # get des roll angle
    def des_roll_cb(self,msg):
        value = msg.data
        if abs(value) <= 0.2: # max roll angle
            self.roll_des = value
        else:
            rospy.logwarn('desire roll angle out of limits [-0.2,0.2]')
    # get des height angle
    def des_height_cb(self,msg):
        height = msg.data
        if height > 0.12 and height < 0.26:
            self.height_des = msg.data
        else:
            rospy.logwarn('desire height out of limits (0.12,0.26)')
    # get joint angle
    def joint_state_cb(self,msg):
        for i,each_name in enumerate(msg.name):
            if each_name in self.joint_states.keys():
                self.joint_states[each_name] = msg.position[i]
    def height_to_angle(self,height):
        return -math.acos(height/(2*self.leg_length))
    def angle_to_height(self,rad_angle):
        return 2*self.leg_length*math.cos(rad_angle)
    def clamp(self,value,maxValue,minValue):
        if value > maxValue:
            return maxValue
        elif value < minValue:
            return minValue
        else:
            return value
    def loop(self):
        self.left_height = self.angle_to_height(self.joint_states['joint_left_hip_left_leg1'])
        self.right_height = self.angle_to_height(self.joint_states['joint_right_hip_right_leg1'])
        self.height = max(self.left_height,self.right_height)
        self.height_err = self.height_des - self.height

        self.roll_err = self.roll_des -self.roll

        control_value_height = self.Kp_h * (self.height_err - self.last_height_err) + self.Ki_h * self.height_err
        control_value_roll = self.Kp_r * (self.roll_err - self.last_roll_err) + self.Kd_r * (self.roll_err-2*self.last_roll_err+self.last_last_roll_err)
        left_height_des = self.left_height + control_value_height + control_value_roll
        right_height_des = self.left_height + control_value_height - control_value_roll
        self.last_last_height_err = self.last_height_err
        self.last_height_err = self.height_err
        self.last_last_roll_err = self.last_roll_err
        self.last_roll_err = self.roll_err
        left_height_des = self.clamp(left_height_des,maxValue=0.26,minValue=0.12)
        right_height_des = self.clamp(right_height_des,maxValue=0.26,minValue=0.12)

        left_joint_angle = self.height_to_angle(left_height_des)
        right_joint_angle = self.height_to_angle(right_height_des)
        self.goal_pub['joint_left_hip_left_leg1'].publish(Float64(left_joint_angle))
        self.goal_pub['joint_left_knee_leg2'].publish(Float64(-2.0*left_joint_angle))
        self.goal_pub['joint_right_hip_right_leg1'].publish(Float64(right_joint_angle))
        self.goal_pub['joint_right_knee_leg2'].publish(Float64(-2.0*right_joint_angle))
def main():
    rospy.init_node("roll_controller")
    controller = roll_controller()
    rat = rospy.Rate(50)
    while not rospy.is_shutdown():
        controller.loop()
        rat.sleep()
        

if __name__ == "__main__":
    main()