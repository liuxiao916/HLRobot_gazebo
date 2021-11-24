#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "HLrobotconfig.h"

using namespace std;
using namespace Eigen;

typedef Matrix<double, 8, 1> Vector8d;

Vector4d cubic(double qo, double dqo, double qf, double dqf, double td);

Vector8d multipoints_cubic(double qo, double dqo, double qv, double qf, double dqf, double td1, double td2);

Vector3d getEulerVelocity(double yaw, double pitch, double roll, double dv);

Matrix<double, 6, 4> getCofMatrix(const Vector6d& qo, const Vector6d& dqo, const Vector6d& qf, const Vector6d& dqf, double td);
Matrix<double, 6, 8> getCofMatrix2(const Vector6d& qo, const Vector6d& dqo, const Vector6d& vp, const Vector6d& qf, const Vector6d& dqf, double td);
