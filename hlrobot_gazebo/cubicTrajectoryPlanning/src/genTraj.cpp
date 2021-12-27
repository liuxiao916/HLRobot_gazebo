#include "genTraj.h"


Vector4d cubic(double qo, double dqo, double qf, double dqf, double td)
{
	Vector4d a;

	double h = qf - qo;

	a(3) = qo;
	a(2) = dqo;
	a(1) = (3 * h - (2 * dqo + dqf) * td) / td / td;
	a(0) = (-2 * h + (dqo + dqf) * td) / (td * td * td);

	return a;
}

Vector8d multipoints_cubic(double qo, double dqo, double qv, double qf, double dqf, double td1, double td2)
{
	double delta_q1 = qv - qo;
	double delta_q2 = qf - qv;

	double p1 = (delta_q1 / td1 - dqo) / td1;
	double p2 = delta_q2 / td2;
	double p3 = (-dqo - p1 * 2 * td1) / td1 / td1;
	
	Vector8d a;

	a(6) = ((-p1 / 2 / td1 - p3) * td1 * td1 - p2 + (dqf - p2) / 2) / (-td1 - td2) * 2;
	a(7) = ((dqf - p2) / td2 - a(6)) / (2 * td2);
	a(5) = p2 - td2 * td2 * a(7) - td2 * a(6);
	a(4) = qv;
	a(3) = p3 + a(5) / td1 / td1;
	a(2) = p1 - td1 * a(3);
	a(1) = dqo;
	a(0) = qo;
	
	// reverse a 
	for (int i = 0; i < 4; i++)	swap(a(i), a(7 - i));
	
	Vector4d b = a.head<4>();
	a.head<4>() = a.tail<4>();
	a.tail<4>() = b;

	return a;
}

Matrix<double, 6, 4> getCofMatrix(const Vector6d& qo, const Vector6d& dqo, const Vector6d& qf, const Vector6d& dqf, double td)
{
	Matrix<double, 4, 6> res;
	for (int i = 0; i < 6; i++)
	{
		res.col(i) = cubic(qo(i), dqo(i), qf(i), dqf(i), td);
	}

	return res.transpose();
}

Matrix<double, 6, 8> getCofMatrix2(const Vector6d& qo, const Vector6d& dqo, const Vector6d& vp, const Vector6d& qf, const Vector6d& dqf, double td)
{
	Matrix<double, 8, 6> res;
	for (int i = 0; i < 6; i++)
	{
		res.col(i) = multipoints_cubic(qo(i), dqo(i), vp(i), qf(i), dqf(i), td/2, td/2);
	}

	return res.transpose();
}

Vector3d getEulerVelocity(double yaw, double pitch, double roll, double dv)
{
	double cy = cos(yaw / 180 * PI);
	double sy = sin(yaw / 180 * PI);
	double cp = cos(pitch / 180 * PI);
	double sp = sin(pitch / 180 * PI);

	Matrix3d r;
	r << cy * cp, sy* cp, -sp,
		sy* sp, -cy * sp, 0,
		-cy, -sy, 0;

	// 返回角度每秒的速度
	return -1.0 / sp * (r * Vector3d(0, dv, 0));
}
