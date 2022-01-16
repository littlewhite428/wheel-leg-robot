#!/usr/bin/python
import rospy
import tf
from sensor_msgs.msg import Imu,JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_msgs.msg import Header
from wheel_leg.msg import Balance
import sys
import math

class balance_observer:
    def __init__(self,imu_topic,odom_topic,encoder1_topic,encoder2_topic,L,Wheel_Radius):
        self.velocity = 0
        self.pitch = 0
        self.fake_velocity = 0
        self.fake_pusai = 0
        self.last_fake_pusai = 0
        self.fai = 0
        self.last_fai = 0
        
        self.right_radps = 0
        self.last_right_radps = 0
        self.left_radps = 0
        self.last_left_radps = 0
        self.last_pitch = 0

        self.L = L
        self.radps_to_mps =  Wheel_Radius # r/s to m/s Radius=0.04m

        self.velocity_publisher = rospy.Publisher('/balance_state_velocity',Float64,queue_size=10)
        self.angle_publisher = rospy.Publisher('/balance_state_angle',Float64,queue_size=10)
        self.angular_velocity_publisher = rospy.Publisher('/balance_state_angular',Float64,queue_size=10)
        self.pusai_publisher = rospy.Publisher('/balance_state_pusai',Float64,queue_size=10)
        self.balance_state_publisher = rospy.Publisher('/balance_state',Balance,queue_size=10)

        self.imu_listener = rospy.Subscriber(imu_topic,Imu,self.imu_cb,queue_size=1)
        self.odom_listener = rospy.Subscriber(odom_topic,Odometry,self.odoom_cb,queue_size=1)
        self.encoder1_listener = rospy.Subscriber(encoder1_topic,JointState,self.encoder1_cb,queue_size=1)
        self.encoder2_listener = rospy.Subscriber(encoder2_topic,JointState,self.encoder2_cb,queue_size=1)


    def imu_cb(self,msg):
        quaternion = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
        r,p,y = tf.transformations.euler_from_quaternion(quaternion,axes='sxyz')
        self.pitch = 0.1 * p + 0.9 * self.last_pitch
        self.last_pitch = self.pitch
        self.fai = 0.1 * msg.angular_velocity.y + 0.9 * self.last_fai
        self.last_fai = self.fai

        self.angle_publisher.publish(Float64(self.pitch))
        self.angular_velocity_publisher.publish(Float64(self.fai))
    '''
    odom velocity and angular velocity both in world frame
    '''
    def odoom_cb(self,msg):
        # problem here
        symbo = 1 if (self.right_radps + self.left_radps) > 0 else -1
        self.fake_velocity = symbo * math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        self.fake_pusai = 0.1*msg.twist.twist.angular.z + (1-0.1) * self.last_fake_pusai
        self.last_fake_pusai = self.fake_pusai
        # self.pusai_publisher.publish(Float64(self.fake_pusai))

    def encoder1_cb(self,msg):
        self.right_radps = 0.3*msg.velocity[7]+(1-0.3)*self.last_right_radps
        self.last_right_radps = self.right_radps
    def encoder2_cb(self,msg):
        self.left_radps = 0.3*msg.velocity[4]+(1-0.3)*self.last_left_radps
        self.last_left_radps = self.left_radps
    def publish(self):
        balance_msg = Balance()
        header = Header()
        header.frame_id = "base_footprint"
        header.stamp = rospy.get_rostime()

        balance_msg.header = header
        balance_msg.pitch = self.pitch
        balance_msg.fai = self.fai
        # balance_msg.pusai = self.radps_to_mps * (self.right_radps - self.left_radps) / self.L
        # balance_msg.velocity = self.radps_to_mps * (self.left_radps + self.right_radps) / 2

        balance_msg.pusai = self.fake_pusai
        balance_msg.velocity = self.fake_velocity


        self.velocity = (self.radps_to_mps * (self.left_radps + self.right_radps) / 2)
        self.velocity_publisher.publish(Float64(self.velocity))
        # self.pusai_publisher.publish(Float64(self.radps_to_mps * (self.right_radps - self.left_radps) / self.L))
        self.pusai_publisher.publish(self.fake_pusai)
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

    rat = rospy.Rate(200)
    while not rospy.is_shutdown():
        rat.sleep()
        obs.publish()

if __name__ == "__main__":
    main()