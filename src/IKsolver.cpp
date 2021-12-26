#include <iostream>
#include <Eigen/Dense>

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include <HLrobotconfig.h>

int main(int argc, char **argv){

    double joint[6]={0};
    ros::init(argc, argv, "IKsolver");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/HL_controller/command", 1000);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        std_msgs::Float64MultiArray msg;
        HLRobot::SetRobotEndPos(-20.067, 33.435, 96.225, -15.926, 79.755, 267.17);
        HLRobot::GetJointAngles(joint[0],joint[1],joint[2],joint[3],joint[4],joint[5]);
        msg.data.push_back(joint[0]);
        msg.data.push_back(joint[1]);
        msg.data.push_back(joint[2]);
        msg.data.push_back(joint[3]);
        msg.data.push_back(joint[4]);
        msg.data.push_back(joint[5]);

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        }
    return 0;
}

