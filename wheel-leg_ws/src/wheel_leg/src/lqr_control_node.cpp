#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "std_msgs/Float64.h"

double pitch_angle,pitch_angular,velocity,pusai,des_velocity,des_pusai; //sensor
double righht_cmd,left_cmd; //cmd 
double velocity_error,pusai_error;
double Ts = 0.01;
const double UPPER_MOMENT = 2.0;
const double LOWER_MOMENT = -2.0;
Eigen::Matrix<double,2,4> K;
Eigen::Matrix<double,2,2> Kc;
Eigen::Vector2d M(0.0,0.0); 
void pitch_angle_cb(const std_msgs::Float64::ConstPtr& msg)
{
    pitch_angle = msg->data;
}
void pitch_angular_cb(const std_msgs::Float64::ConstPtr& msg)
{
    pitch_angular = msg->data;
}
void velocity_cb(const std_msgs::Float64::ConstPtr& msg)
{
    velocity = msg->data;
}
void des_velocity_cb(const std_msgs::Float64::ConstPtr& msg)
{
    des_velocity = msg->data;
}
void pusai_cb(const std_msgs::Float64::ConstPtr& msg)
{
    pusai = msg->data;
}
void des_pusai_cb(const std_msgs::Float64::ConstPtr& msg)
{
    des_pusai = msg->data;
}

void init()
{
    K  <<    -17.9321,  -3.7491,   -5.8103,    2.2696,
             -17.9321,  -3.7491,   -5.8103,   -2.2696;
    Kc <<    -5.0000,    3.1623,
             -5.0000,   -3.1623; 
}
double clamp(double data,double upper,double lower)
{
    if(data > upper){
        return upper;
    }
    else if(data < lower){
        return lower;
    }
    else{
        return data;
    }
}
int main(int argc, char **argv)
{
  init();
  ros::init(argc, argv, "lqr_controller");

  ros::NodeHandle n;

  ros::Subscriber pitch_angle_sub = n.subscribe("/balance_state_angle", 10, pitch_angle_cb);
  ros::Subscriber pitch_angular_sub = n.subscribe("/balance_state_angular", 10, pitch_angular_cb);
  ros::Subscriber velocity_sub = n.subscribe("/balance_state_velocity", 10, velocity_cb);
  ros::Subscriber pusai_sub = n.subscribe("/balance_state_pusai", 10, pusai_cb);
  ros::Subscriber des_velocity_sub = n.subscribe("/wheel_leg/des_velocity", 10, des_velocity_cb);
  ros::Subscriber des_pusai_sub = n.subscribe("/wheel_leg/des_pusai", 10, des_pusai_cb);


  ros::Publisher right_wheel_pub = n.advertise<std_msgs::Float64>("/wheel_leg/right_wheel_effort_controller/command", 10);
  ros::Publisher left_wheel_pub = n.advertise<std_msgs::Float64>("/wheel_leg/left_wheel_effort_controller/command", 10);

  ros::Rate loop_rate(100); //100hz
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  while(ros::ok())
  {
    velocity_error += Ts*(des_velocity - velocity);
    pusai_error += Ts*(des_pusai - pusai);
    M = Kc * Eigen::Vector2d(velocity_error,pusai_error) - K * Eigen::Vector4d(pitch_angle,pitch_angular,velocity,pusai);

    std_msgs::Float64 msg;
    msg.data = clamp(M(0),UPPER_MOMENT,LOWER_MOMENT);
    right_wheel_pub.publish(msg);
    msg.data = clamp(M(1),UPPER_MOMENT,LOWER_MOMENT);
    left_wheel_pub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
  }
  std_msgs::Float64 msg;
  msg.data = 0.0;
  right_wheel_pub.publish(msg);
  left_wheel_pub.publish(msg);
  return 0;
}