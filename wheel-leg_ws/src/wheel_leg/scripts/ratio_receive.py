#!/usr/bin/python
import socket

import rospy
from std_msgs.msg import Float64

def publish(msg):
    rx,ry,lx,ly = msg
    des_velocity = (50.0-ly)/50*0.3
    des_pusai = (50.0-lx)/50*0.8
    des_height = (100.0-ry)/100*(0.26-0.13)+0.13
    des_roll = (rx-50.0)/50*0.1
    velocity_pub.publish(Float64(des_velocity))
    pusai_pub.publish(Float64(des_pusai))
    height_pub.publish(Float64(des_height))
    roll_pub.publish(Float64(des_roll))
    rospy.loginfo_throttle(0.2,"%.2f %.2f %.2f %.2f" % (des_velocity,des_pusai,des_height,des_roll))

def parse_command(data):
    if len(data) < 6:
        return None
    for i in range(len(data)-1,-1,-1):
        if data[i] == ']' and i>=5:
            final_index = i
            break
    else:
        return None
    if data[final_index-5]=='[':
        return [ord(data[x]) for x in range(final_index-4,final_index)]
    else:
        return parse_command(data[:-1])

address=('192.168.10.160',1030)
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.bind(address)

roll_pub = rospy.Publisher('/wheel_leg/des_roll',Float64,queue_size=10)
height_pub = rospy.Publisher('/wheel_leg/des_height',Float64,queue_size=10)
velocity_pub = rospy.Publisher('/wheel_leg/des_velocity',Float64,queue_size=10)
pusai_pub = rospy.Publisher('/wheel_leg/des_pusai',Float64,queue_size=10)

rospy.init_node("ratio_receiver")
dur = rospy.Duration(0.05)
while not rospy.is_shutdown():
    data,addr=sock.recvfrom(1024)
    msg = parse_command(data)
    if msg:
        # rospy.loginfo_throttle(1,msg)
        publish(msg)
    rospy.sleep(dur)

sock.close()
