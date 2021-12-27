#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

#include <HLrobotconfig.h>

void readcsv(const std::string& filename,double* T);

int main(int argc, char **argv){

    double joint[6]={0};
    ros::init(argc, argv, "IKsolver");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/HL_controller/command", 1000);
    ros::Rate loop_rate(10);
    double TMatrix[16]={0}; 

    readcsv("/home/liuxiao/catkin_ws/src/HLRobot_gazebo/data/eye_hand/002_pose.csv",TMatrix);

    while (ros::ok()) {
        std_msgs::Float64MultiArray msg;
        //HLRobot::SetRobotEndPos(-20.067, 33.435, 96.225, -15.926, 79.755, 267.17);
        HLRobot::setTMatrix(TMatrix);
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

void readcsv(const std::string& filename, double* T){
    std::ifstream ifs(filename);
    if(!ifs){
        std::cerr << "failed to open " << filename << std::endl;
    }
        
    std::vector<std::vector<double>> matrix;
    while(!ifs.eof()) {
        std::string line;
        std::getline(ifs, line);

        if(line.empty()) {
            continue;
        }

        std::vector<double> row;
        std::stringstream sst(line);
        while(!sst.eof()) {
            double value;
            sst >> value;
            row.push_back(value);
        }

        matrix.push_back(row);
    }

    for(int i=0; i<matrix.size(); i++) {
        for(int j=0; j<matrix[i].size(); j++) {
            T[4*i+j] = matrix[i][j];
        }
    }

}