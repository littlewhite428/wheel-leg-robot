#!/usr/bin/python
import rospy
from std_msgs.msg import Float64


total_velocity = 0.0
total_roation = 0.0

def total_velocity_cb(msg):
    global total_velocity 
    total_velocity = msg.data

rospy.Subscriber('/balance_v_command',Float64,total_velocity_cb,queue_size=1)
right_wheel_command_pub = rospy.Publisher('/wheel_leg3/wheel1_velocity_controller/command',Float64,queue_size=10)
left_wheel_command_pub = rospy.Publisher('/wheel_leg3/wheel2_velocity_controller/command',Float64,queue_size=10)

rospy.init_node('control_assign')
r = rospy.Rate(100)

while not rospy.is_shutdown():
    right_cmd = Float64(total_velocity)
    left_cmd = Float64(total_velocity)
    right_wheel_command_pub.publish(right_cmd)
    left_wheel_command_pub.publish(left_cmd)
    r.sleep()