#include <ros/ros.h>
#include <can_msgs/Frame.h>

#define MPH2KMH 1.60934
#define KIA_SOUL_STEERING_RATIO 15.7
void msgCallback(const can_msgs::Frame::ConstPtr& msg){
    unsigned char byte0;
    unsigned char byte1;
    unsigned char byte2;
    unsigned char byte3;
    unsigned char byte4;
    unsigned char byte5;
    unsigned char byte6;
    unsigned char byte7;

    short raw_speed_LF = 0;
    short raw_speed_RF = 0;
    short raw_speed_LB = 0;
    short raw_speed_RB = 0;
    short raw_angle = 0;

    double speed_LF;
    double speed_RF;
    double speed_LB;
    double speed_RB;
    double steering_wheel_angle;
    double yaw_angle;

    if(msg->id == 688){
        //Get Raw steering wheel angle data from can_tx msg
        byte0 = msg->data[0];
        byte1 = msg->data[1];
        raw_angle = byte1;
        raw_angle = (raw_angle << 8) + byte0;
        steering_wheel_angle = double(raw_angle) * 0.1;
        yaw_angle = steering_wheel_angle / KIA_SOUL_STEERING_RATIO;
        ROS_INFO("YAW ANGLE DATA %f degree", yaw_angle);
    }
    if(msg->id == 1200){
        //Get Raw wheels speed data from can_tx msg
        byte0 = msg->data[0];
        byte1 = msg->data[1];
        raw_speed_LF = byte1;
        raw_speed_LF = (raw_speed_LF << 8) + byte0;
        byte2 = msg->data[2];
        byte3 = msg->data[3];
        raw_speed_RF = byte3;
        raw_speed_RF = (raw_speed_RF << 8) + byte2;
        byte4 = msg->data[4];
        byte5 = msg->data[5];
        raw_speed_LB = byte5;
        raw_speed_LB = (raw_speed_LB << 8) + byte4;
        byte6 = msg->data[6];
        byte7 = msg->data[7];
        raw_speed_RB = byte7;
        raw_speed_RB = (raw_speed_RB << 8) + byte6;
        //Transform 'mph' to 'km/h'
        speed_LF = double(raw_speed_LF) * 0.02 * MPH2KMH;
        speed_RF = double(raw_speed_RF) * 0.02 * MPH2KMH;
        speed_LB = double(raw_speed_LB) * 0.02 * MPH2KMH;
        speed_RB = double(raw_speed_RB) * 0.02 * MPH2KMH;
        //ROS_INFO("WHEELS SPEED DATA LF : %f km/h, RF : %f km/h, LB : %f km/h, RB : %f km/h",
        //         speed_LF, speed_RF, speed_LB, speed_RB);
        ROS_INFO("AVERAGE WHEEL SPEED DATA : %f km/h",(speed_LF + speed_RF + speed_LB + speed_RB)/4);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "can_data_reader");
    ros::NodeHandle nh;
    ros::Subscriber ros_read_candata_sub = nh.subscribe("can_tx", 100, msgCallback);
    ros::spin();
    return 0;
}
