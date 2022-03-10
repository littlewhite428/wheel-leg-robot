#include "kinematics_3dof.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "wheel_leg/Footprint.h"
#include "serial/serial.h"
#include "usb_can.h"

ros::Publisher right_hip_pub; 
ros::Publisher right_leg1_pub;
ros::Publisher right_leg2_pub;
ros::Publisher left_hip_pub;
ros::Publisher left_leg1_pub; 
ros::Publisher left_leg2_pub; 
ros::Subscriber footprint_sub;
void footprint_cb(const wheel_leg::Footprint & msg){
    const Eigen::Vector3d COM_2_Leg(0,0.1,0);
    Eigen::Vector3d COM(msg.COM.x,msg.COM.y,msg.COM.z);
    Eigen::Vector3d Left_foot(msg.Left_foot.x,msg.Left_foot.y,msg.Left_foot.z);
    Eigen::Vector3d relative_left = Left_foot - (COM + COM_2_Leg);
    relative_left(1) = 0.0;

    Eigen::Vector3d Right_foot(msg.Right_foot.x,msg.Right_foot.y,msg.Right_foot.z);
    Eigen::Vector3d relative_right = Right_foot - (COM - COM_2_Leg);
    relative_right(1) = 0.0;

    Eigen::Vector3d left_joint_angle = Kinematics::ik(relative_left,0.13,0.13);

    Eigen::Vector3d right_joint_angle = Kinematics::ik(relative_right,0.13,0.13);

    std_msgs::Float64 angle = std_msgs::Float64();
    angle.data = left_joint_angle(0);
    left_hip_pub.publish(angle);

    angle.data = right_joint_angle(0);
    right_hip_pub.publish(angle);

    angle.data = left_joint_angle(1);
    left_leg1_pub.publish(angle);  

    angle.data = right_joint_angle(1);
    right_leg1_pub.publish(angle);
    

    angle.data = left_joint_angle(2);
    left_leg2_pub.publish(angle);

    angle.data = right_joint_angle(2);   
    right_leg2_pub.publish(angle);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bipedal_controller");
    ros::NodeHandle n;
    right_hip_pub = n.advertise<std_msgs::Float64>("/wheel_leg/base_right_hip_controller/command", 1);
    right_leg1_pub = n.advertise<std_msgs::Float64>("/wheel_leg/right_hip_right_leg1_controller/command", 1);
    right_leg2_pub = n.advertise<std_msgs::Float64>("/wheel_leg/right_knee_leg2_controller/command", 1);
    left_hip_pub = n.advertise<std_msgs::Float64>("/wheel_leg/base_left_hip_controller/command", 1);
    left_leg1_pub = n.advertise<std_msgs::Float64>("/wheel_leg/left_hip_left_leg1_controller/command", 1);
    left_leg2_pub = n.advertise<std_msgs::Float64>("/wheel_leg/left_knee_leg2_controller/command", 1);
    footprint_sub = n.subscribe("/wheel_leg/footprint", 1, footprint_cb);

    const std::string sss("/dev/ttyACM0");
    Usb_can usb_can(sss,115200,serial::bytesize_t(8),serial::stopbits_one,serial::parity_odd,serial::Timeout::simpleTimeout(1e8));
    uint8_t id[2];
    id[0]=0x00;
    id[1]=0x00;
    uint8_t data[3];
    data[0] = 0xCC;
    data[1] = 0xEE;
    data[2] = 0xFF;
    usb_can.open();
    if(usb_can.isOpen()){
        usb_can.write(id,3,data);
        std::cout<<"send over"<<std::endl;
    }
    else{
        std::cout<<"open error"<<std::endl;
    }

    while(ros::ok()){
        size_t frame_num = usb_can.available();
        if(frame_num > 0){
            std::cout << "--new frame num: " << frame_num << std::endl;
            // each frame
            for(int i=0;i<frame_num;i++){
                std::cout << "  No. " << i << std::endl;
                can_frame* frame_ptr = usb_can.read();
                std::cout<<frame_ptr->getStr()<<std::endl;
                delete frame_ptr;
            }
        }
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }
    usb_can.close();


    // double x = 0,y = 0.0,z = -0.23;
    // while (ros::ok())
    // {
    //     for(x=-0.25;x<=0.25;x+=0.001){

    //         Eigen::Vector3d joint_angle = Kinematics::ik(Eigen::Vector3d(x,y,z),0.13,0.13);
    //         std_msgs::Float64 angle = std_msgs::Float64();
    //         angle.data = joint_angle(0);
    //         right_hip_pub.publish(angle);
    //         left_hip_pub.publish(angle);

    //         angle.data = joint_angle(1);
    //         right_leg1_pub.publish(angle);
    //         left_leg1_pub.publish(angle);

    //         angle.data = joint_angle(2);
    //         right_leg2_pub.publish(angle);
    //         left_leg2_pub.publish(angle);

    //         ros::Duration(0.02).sleep();
    //     }
    //     for(x=0.25;x>=-0.25;x-=0.001){

    //         Eigen::Vector3d joint_angle = Kinematics::ik(Eigen::Vector3d(x,y,z),0.13,0.13);
    //         std_msgs::Float64 angle = std_msgs::Float64();
    //         angle.data = joint_angle(0);
    //         right_hip_pub.publish(angle);
    //         left_hip_pub.publish(angle);

    //         angle.data = joint_angle(1);
    //         right_leg1_pub.publish(angle);
    //         left_leg1_pub.publish(angle);

    //         angle.data = joint_angle(2);
    //         right_leg2_pub.publish(angle);
    //         left_leg2_pub.publish(angle);

    //         ros::Duration(0.02).sleep();
    //     }
    // }
    // ros::shutdown();

}