#ifndef ROBOTCONFIG_H_
#define ROBOTCONFIG_H_
#include "eigen3/Eigen/Dense"
#include <vector>
#include <iostream>
using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

const double PI = 3.1415926;
namespace HLRobot
{ 
	//Inverse kinematics solution
	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll);
	void GetJointAngles(double &angle1, double &angle2, double &angle3, double &angle4, double &angle5, double &angle6);
	
	//Forward kinematics solution
	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4, double angle5, double angle6);
	void GetJointEndPos(double &x, double &y, double &z, double &yaw, double &pitch, double &roll);

	//Inverse kinematics and Forward kinematics method function
	void robotBackward(const double* TransVector, bool* config, double* theta);
	void robotForward(const double* q, double* TransVector, bool* config);

    void setTMatrix(double* TransMatrix);
	
	Vector6d eigenGetJoints(Vector6d p);
	Vector3d RtoEulerAngles(Matrix3d r);

	Vector6d getJointVelocity(Vector6d q, Vector6d dv);
	Matrix6d getJacobian(const Vector6d& q);
}

#endif
