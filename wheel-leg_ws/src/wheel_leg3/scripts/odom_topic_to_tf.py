#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry


def odom_callback(odom_msg):
    time = odom_msg.header.stamp
    # print("time",time,type(time))
    pose = odom_msg.pose.pose.position
    x ,y ,z = pose.x , pose.y , pose.z
    # print("pose",pose,type(pose))
    quaternion = odom_msg.pose.pose.orientation
    q1, q2, q3, q4 = quaternion.x , quaternion.y, quaternion.z, quaternion.w
    # print("quaternion",quaternion,type(quaternion))
    br.sendTransform((x,y,z),(q1,q2,q3,q4),time,goal_frame,static_frame)


rospy.init_node("odom_to_tf")
odom_name = rospy.get_param("~odom_name")

static_frame = rospy.get_param("~static_frame")
goal_frame = rospy.get_param("~goal_frame")

odom_sub = rospy.Subscriber(odom_name,Odometry,odom_callback,queue_size=1)
br = tf.TransformBroadcaster()

rospy.spin()

