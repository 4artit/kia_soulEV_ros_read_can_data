#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>

#define KIA_SOUL_STEERING_RATIO 15.7
#define PI 3.14159265

double yaw_angle = 0.0;
double average_speed = 0.0;

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
    short raw_lat_accel = 0;
    short raw_long_accel = 0;
    short raw_cyl_pres = 0;
    short raw_yaw_rate = 0;

    double speed_LF;
    double speed_RF;
    double speed_LB;
    double speed_RB;
    double steering_wheel_angle;
    double lateral_accel;
    double longitudinal_accel;
    double cylinder_pressure;
    double yaw_rate;

    if(msg->id == 544){
	    byte0 = msg->data[0];
        byte1 = msg->data[1];
        raw_lat_accel = byte1;
        raw_lat_accel = (raw_lat_accel << 8) + byte0;
        byte2 = msg->data[2];
        byte3 = msg->data[3];
        raw_long_accel = byte3;
        raw_long_accel = (raw_long_accel << 8) + byte2;
        byte4 = msg->data[4];
        byte5 = msg->data[5];
        raw_cyl_pres = byte5;
        raw_cyl_pres = (raw_cyl_pres << 6) + (byte4 >> 2);
        byte6 = msg->data[6];
        byte7 = msg->data[7];
        raw_yaw_rate = byte7;
        raw_yaw_rate = (raw_yaw_rate << 8) + byte6;
        lateral_accel = double(raw_lat_accel) * 0.01 - 10.23; // m/s^2
        longitudinal_accel = double(raw_long_accel) * 0.01 - 10.23; // m/s^2
        cylinder_pressure = double(raw_long_accel) * 0.1; // bar
        yaw_rate = double(raw_yaw_rate) * 0.01 - 40.95; // rad/s
        ROS_INFO("LAT ACC : %f m/s^2  LONG ACC : %f m/s^2  CYLINDER PRESSURE : %f bar  YAW RATE : %f rad/s", lateral_accel, longitudinal_accel, cylinder_pressure, yaw_rate);
    }

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
        speed_LF = double(raw_speed_LF) * 0.03125;
        speed_RF = double(raw_speed_RF) * 0.03125;
        speed_LB = double(raw_speed_LB) * 0.03125;
        speed_RB = double(raw_speed_RB) * 0.03125;
        //ROS_INFO("WHEELS SPEED DATA LF : %f km/h, RF : %f km/h, LB : %f km/h, RB : %f km/h",
        //         speed_LF, speed_RF, speed_LB, speed_RB);
        ROS_INFO("AVERAGE WHEEL SPEED DATA : %f km/h",(speed_LF + speed_RF + speed_LB + speed_RB)/4);
        average_speed = (speed_LF + speed_RF + speed_LB + speed_RB)/4;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "can_data_reader");
    ros::NodeHandle nh;
    ros::Publisher stamped_pub = nh.advertise<geometry_msgs::TwistStamped>("/TwistStamped", 100);
    ros::Subscriber ros_read_candata_sub = nh.subscribe("can_tx", 100, msgCallback);
    ros::Rate loop_rate(60);
    ros::spinOnce();
    while(ros::ok()){
        geometry_msgs::TwistStamped vector_twist;
        vector_twist.header.stamp = ros::Time::now();
        vector_twist.twist.linear.x = cos(yaw_angle * PI / 180) * average_speed;
        vector_twist.twist.linear.y = sin(yaw_angle * PI / 180) * average_speed;
        vector_twist.twist.linear.z = 0.0;

        stamped_pub.publish(vector_twist);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
