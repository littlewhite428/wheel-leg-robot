#!/usr/bin/python
import rospy
from std_msgs.msg import Float64


velocity1 = 0.0
velocity2 = 0.0

def velocity1_cb(msg):
    global velocity1 
    velocity1 = msg.data

def velocity2_cb(msg):
    global velocity2 
    velocity2 = msg.data

rospy.Subscriber('/balance_command1',Float64,velocity1_cb,queue_size=1)
rospy.Subscriber('/balance_command2',Float64,velocity2_cb,queue_size=1)
right_wheel_command_pub = rospy.Publisher('/wheel_leg3/wheel1_velocity_controller/command',Float64,queue_size=10)
left_wheel_command_pub = rospy.Publisher('/wheel_leg3/wheel2_velocity_controller/command',Float64,queue_size=10)

rospy.init_node('control_assign')
r = rospy.Rate(200)

while not rospy.is_shutdown():
    velocity = velocity1 + velocity2
    right_cmd = Float64(velocity)
    left_cmd = Float64(velocity)
    right_wheel_command_pub.publish(right_cmd)
    left_wheel_command_pub.publish(left_cmd)
    r.sleep()