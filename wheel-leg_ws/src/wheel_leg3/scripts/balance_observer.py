#!/usr/bin/python
import rospy
import tf
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_msgs.msg import Header
from wheel_leg3.msg import Balance
import sys
import math

class balance_observer:
    def __init__(self,imu_topic,odom_topic,encoder1_topic,encoder2_topic,L,Wheel_Radius):
        self.imu_listener = rospy.Subscriber(imu_topic,Imu,self.imu_cb,queue_size=1)
        self.odom_listener = rospy.Subscriber(odom_topic,Odometry,self.odoom_cb,queue_size=1)
        self.encoder1_listener = rospy.Subscriber(encoder1_topic,Float64,self.encoder1_cb,queue_size=1)
        self.encoder2_listener = rospy.Subscriber(encoder2_topic,Float64,self.encoder2_cb,queue_size=1)

        self.velocity_publisher = rospy.Publisher('/balance_state_velocity',Float64,queue_size=10)
        self.angle_publisher = rospy.Publisher('/balance_state_angle',Float64,queue_size=10)
        self.angular_velocity_publisher = rospy.Publisher('/balance_state_angular',Float64,queue_size=10)
        self.balance_state_publisher = rospy.Publisher('/balance_state',Balance,queue_size=10)

        self.velocity = 0
        self.pusai = 0
        self.pitch = 0
        self.fake_velocity = 0
        self.fake_pusai = 0
        self.fai = 0
        
        self.right_roundps = 0
        self.last_right_roundps = 0
        self.left_roundps = 0
        self.last_left_roundps = 0

        self.L = L
        self.roundps_to_mps = 2 * 3.1415926 * Wheel_Radius # r/s to m/s Radius=0.04m
    def imu_cb(self,msg):
        quaternion = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
        r,p,y = tf.transformations.euler_from_quaternion(quaternion,axes='sxyz')
        self.pitch = p
        self.fai = msg.angular_velocity.y

        self.angle_publisher.publish(Float64(self.pitch))
        self.angular_velocity_publisher.publish(Float64(self.fai))
    '''
    odom velocity and angular velocity both in world frame
    '''
    def odoom_cb(self,msg):
        # problem here
        self.fake_velocity = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        self.fake_pusai = msg.twist.twist.angular.z

    def encoder1_cb(self,msg):
        self.right_roundps = 1*msg.data+(1-1)*self.last_right_roundps
        self.last_right_roundps = self.right_roundps
    def encoder2_cb(self,msg):
        self.left_roundps = 1*msg.data+(1-1)*self.last_left_roundps
        self.last_left_roundps = self.left_roundps
    def publish(self):
        balance_msg = Balance()
        header = Header()
        header.frame_id = "base_footprint"
        header.stamp = rospy.get_rostime()

        balance_msg.header = header
        balance_msg.pitch = self.pitch
        balance_msg.fai = self.fai
        # balance_msg.pusai = self.roundps_to_mps * (self.right_roundps - self.left_roundps) / self.L
        # balance_msg.velocity = self.roundps_to_mps * (self.left_roundps + self.right_roundps) / 2

        balance_msg.pusai = self.fake_pusai
        balance_msg.velocity = self.fake_velocity

        self.velocity_publisher.publish(Float64(self.roundps_to_mps * (self.left_roundps + self.right_roundps) / 2))
        self.balance_state_publisher.publish(balance_msg)

def main():
    rospy.init_node("balance_observer")
    # param get
    if rospy.has_param("~imu_topic"):
        imu_topic = rospy.get_param("~imu_topic")
    else:
        rospy.logerr("lack param of imu_topic ")
        sys.exit(-1)
    if rospy.has_param("~odom_topic"):
        odom_topic = rospy.get_param("~odom_topic")
    else:
        rospy.logerr("lack param of odom_topic ") 
        sys.exit(-1)   
    if rospy.has_param("~encoder1_topic"):
        encoder1_topic = rospy.get_param("~encoder1_topic")
    else:
        rospy.logerr("lack param of encoder1_topic ") 
        sys.exit(-1)   
    if rospy.has_param("~encoder2_topic"):
        encoder2_topic = rospy.get_param("~encoder2_topic")
    else:
        rospy.logerr("lack param of encoder2_topic ") 
        sys.exit(-1)
    if rospy.has_param("~wheel_distance"):
        wheel_distance = rospy.get_param("~wheel_distance")
    else:
        rospy.logerr("lack param of wheel_distance ") 
        sys.exit(-1)  
    if rospy.has_param("~wheel_radius"):
        wheel_radius = rospy.get_param("~wheel_radius")
    else:
        rospy.logerr("lack param of wheel_radius ") 
        sys.exit(-1)  

    obs = balance_observer(imu_topic,odom_topic,encoder1_topic,encoder2_topic,wheel_distance,wheel_radius)

    rat = rospy.Rate(100)
    while not rospy.is_shutdown():
        rat.sleep()
        obs.publish()

if __name__ == "__main__":
    main()