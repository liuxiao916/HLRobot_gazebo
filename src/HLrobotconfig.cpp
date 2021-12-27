#include "HLrobotconfig.h"

namespace HLRobot
{
	struct robotConfig {
		double L1, L2, L3, L4;
		Matrix<double, 3, 6> q;	// 3x6 matrix q we choose
		Matrix<double, 3, 6> w; // 3x6 w joint axis
		Matrix<double, 3, 6> v;
		Matrix4d g0; // inital configuration
		robotConfig(double l1, double l2, double l3, double l4) : L1(l1), L2(l2), L3(l3), L4(l4)
		{
			Matrix<double, 6, 3> tmp_q;
			tmp_q << 0, 0, 0, 0, 0, L1, 0, 0, L1 + L2, 0, 0, L1 + L2 + L3,
				0, 0, L1 + L2 + L3, 0, 0, L1 + L2 + L3 + L4;
			q = tmp_q.transpose();

			Matrix<double, 6, 3> tmp_w;
			tmp_w << 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1;
			w = tmp_w.transpose();

			for (int i = 0; i < 6; i++)
			{
				v.col(i) = -w.col(i).cross(q.col(i));
			}

			g0 << -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, L1 + L2 + L3 + L4, 0, 0, 0, 1;
		}
	};

	//初始化TransMatrix
	double mTransMatrix[16]{ 0 };
	double mJoint[6]{ 0 };

	//只使用一种姿态
	bool mConfig[3] = { 0, 0, 0 };

	robotConfig qkmRobot(491, 450, 450, 84);

	Matrix3d skew(Vector3d w) {
		Matrix3d so3;
		so3 << 0, -w(2), w(1),
			w(2), 0, -w(0),
			-w(1), w(0), 0;
		return so3;
	}

	Matrix4d R6toSE3(Vector3d v, Vector3d w, double theta)
	{
		// turn deg to rad
		theta = PI * theta / 180;

		Matrix3d so3 = skew(w);
		Matrix3d SO3 = Matrix3d::Identity() + sin(theta) * so3 + (1 - cos(theta)) * so3 * so3;

		// (eye(3) - SO3)* cross(w, v) + (w' * v * theta) * w;

		Vector3d p = (Matrix3d::Identity() - SO3) * (w.cross(v)) + theta * w.transpose() * v * w;

		Matrix4d res = Matrix4d::Identity();
		res.block<3, 3>(0, 0) = SO3;
		res.block<3, 1>(0, 3) = p;

		return res;
	}

	Matrix4d inverseSE3(Matrix4d g)
	{
		Matrix4d g_inv = Matrix4d::Identity();
		Vector3d p = g.block<3, 1>(0, 3);
		Matrix3d rot = g.block<3, 3>(0, 0);
		g_inv.block<3, 3>(0, 0) = rot.transpose();
		g_inv.block<3, 1>(0, 3) = -rot.transpose() * p;

		return g_inv;
	}

	Matrix6d Adjoint(const Matrix4d& g)
	{
		Matrix3d rot = g.block<3, 3>(0, 0);
		Vector3d p = g.block<3, 1>(0, 3);

		Matrix6d res = Matrix6d::Zero();
		res.block<3, 3>(0, 0) = rot;
		res.block<3, 3>(0, 3) = skew(p) * rot;
		res.block<3, 3>(3, 3) = rot;
		return res;
	}

	Matrix6d getJacobian(const Vector6d& q)
	{
		Matrix6d res;
		Matrix4d acm = Matrix4d::Identity();
		for (int i = 0; i < 6; i++)
		{
			if (i > 0) {
				acm = acm * R6toSE3(qkmRobot.v.col(i - 1), qkmRobot.w.col(i - 1), q(i - 1));
			}
			Vector6d twist;
			twist.head<3>() = qkmRobot.v.col(i);
			twist.tail<3>() = qkmRobot.w.col(i);
			res.col(i) = Adjoint(acm) * twist;
		}

		return res;
	}

	Vector6d getJointVelocity(Vector6d q, Vector6d dv)
	{
		Matrix6d J = getJacobian(q);

		return J.inverse() * dv;
	}

	void subProblem1(Vector3d r, Vector3d p, Vector3d q, Vector3d w, double& theta)
	{
		Vector3d u, v, u_, v_;
		u = p - r;
		v = q - r;

		u_ = u - w * w.transpose() * u;
		v_ = v - w * w.transpose() * v;

		theta = 180 / PI * atan2(w.transpose() * (u_.cross(v_)), u_.transpose() * v_);
	}

	void subProblem2(Vector3d r, Vector3d p, Vector3d q, Vector3d w1, Vector3d w2, vector<double>& theta1, vector<double>& theta2)
	{
		Vector3d u, v;
		u = p - r;
		v = q - r;
		double tmp_w12 = w1.transpose() * w2;
		double tmp_2u = w2.transpose() * u;
		double tmp_1v = w1.transpose() * v;

		double alpha = (tmp_w12 * tmp_2u - tmp_1v) / (tmp_w12 * tmp_w12 - 1);
		double beta = (tmp_w12 * tmp_1v - tmp_2u) / (tmp_w12 * tmp_w12 - 1);

		double u_norm = u.dot(u);
		Vector3d cross_w1w2 = w1.cross(w2);
		double y2 = (u_norm - alpha * alpha - beta * beta - 2 * alpha * beta * tmp_w12) / cross_w1w2.dot(cross_w1w2);

		if (y2 >= 0) {
			double y = sqrt(y2);
			Vector3d z1 = alpha * w1 + beta * w2 - y * cross_w1w2;
			Vector3d z2 = alpha * w1 + beta * w2 + y * cross_w1w2;
			Vector3d c1 = z1 + r;
			Vector3d c2 = z2 + r;

			double th1, th2;
			subProblem1(r, q, c1, w1, th1);
			theta1.emplace_back(-th1);
			subProblem1(r, q, c2, w1, th1);
			theta1.emplace_back(-th1);

			subProblem1(r, p, c1, w2, th2);
			theta2.emplace_back(th2);
			subProblem1(r, p, c2, w2, th2);
			theta2.emplace_back(th2);
		}
	}

	void subProblem3(Vector3d r, Vector3d p, Vector3d q, Vector3d w, double delta, vector<double>& theta)
	{
		Vector3d u, v, u_, v_;
		u = p - r;
		v = q - r;

		double tmp = w.transpose() * (p - q);
		double delta_2 = delta * delta - tmp * tmp;

		u_ = u - w.transpose() * u * w;
		v_ = v - w.transpose() * v * w;

		double th0 = atan2(w.transpose() * u_.cross(v_), u_.transpose() * v_);

		double u_n2 = u_.dot(u_);
		double v_n2 = v_.dot(v_);

		double th_ = (u_n2 + v_n2 - delta_2) / sqrt(4 * u_n2 * v_n2);
		if (abs(th_) <= 1) {
			theta.emplace_back(180 / PI * (th0 - acos(th_)));
			theta.emplace_back(180 / PI * (th0 + acos(th_)));
		}
	}

	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll)
	{
		Matrix4d g = Matrix4d::Identity();
		Vector3d p;
		p << x, y, z;
		// cout << "input: " << yaw << " " << pitch << " " << roll << endl;
		yaw = yaw * PI / 180;
		pitch = pitch * PI / 180;
		roll = roll * PI / 180;

		Eigen::AngleAxisd yawAngle(AngleAxisd(yaw, Vector3d::UnitZ()));
		Eigen::AngleAxisd pitchAngle(AngleAxisd(pitch, Vector3d::UnitY()));
		Eigen::AngleAxisd rollAngle(AngleAxisd(roll, Vector3d::UnitZ()));
		Matrix3d rot;
		rot = yawAngle * pitchAngle * rollAngle;
		
		//cout << "inverse: \n" << rot << endl;
		
		g.block<3, 3>(0, 0) = rot;
		g.block<3, 1>(0, 3) = p;

		for (int i = 0; i < 16; i++)
		{
			mTransMatrix[i] = g(i / 4, i % 4);
		}

	}

    void setTMatrix(double* TransMatrix){
        for (int i = 0; i < 16; i++)
            {
                mTransMatrix[i] = TransMatrix[i];
            }
    }

	void GetJointAngles(double& angle1, double& angle2, double& angle3, double& angle4, double& angle5, double& angle6)
	{
		double theta[6]{ 0 };
		robotBackward(mTransMatrix, mConfig, theta);
		angle1 = theta[0];
		angle2 = theta[1];
		angle3 = theta[2];
		angle4 = theta[3];
		angle5 = theta[4];
		angle6 = theta[5] + 360;
	}

	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4, double angle5, double angle6)
	{
		mJoint[0] = angle1;
		mJoint[1] = angle2;
		mJoint[2] = angle3;
		mJoint[3] = angle4;
		mJoint[4] = angle5;
		mJoint[5] = angle6;
	}

	Vector3d RtoEulerAngles(Matrix3d rot) {

		Vector3d res;
		double tmp = sqrt(rot(2, 0) * rot(2, 0) + rot(2, 1) * rot(2, 1));
		res(0) = atan2(rot(1, 2), rot(0, 2)) * 180 / PI;
		res(2) = atan2(rot(2, 1), -rot(2, 0)) * 180 / PI;
		res(1) = atan2(tmp, rot(2, 2)) * 180 / PI;
		return res;
	}

	void GetJointEndPos(double& x, double& y, double& z, double& yaw, double& pitch, double& roll)
	{
		robotForward(mJoint, mTransMatrix, mConfig);

		x = mTransMatrix[0 * 4 + 3];
		y = mTransMatrix[1 * 4 + 3];
		z = mTransMatrix[2 * 4 + 3];

		Matrix3d rot;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++) {
				rot(i, j) = mTransMatrix[i * 4 + j];
			}
		}

		cout << "forword: \n" << rot << endl;

		Vector3d zyz = RtoEulerAngles(rot);

		yaw = zyz(0);
		pitch = zyz(1);
		roll = zyz(2);

		/*cout << "my: " << yaw << " " << pitch << " " << roll << endl;
		Eigen::AngleAxisd yawAngle(AngleAxisd(yaw / 180 * PI, Vector3d::UnitZ()));
		Eigen::AngleAxisd pitchAngle(AngleAxisd(pitch / 180 * PI, Vector3d::UnitY()));
		Eigen::AngleAxisd rollAngle(AngleAxisd(roll / 180 * PI, Vector3d::UnitZ()));
		rot = yawAngle * pitchAngle * rollAngle;
		cout << "forwad2: \n" << rot << endl;*/

	}


	/********************************************************************
	ABSTRACT:	机器人逆运动学

	INPUTS:		T[16]:	位姿矩阵，其中长度距离为米

				config[3]：姿态，六轴机器人对应有8种姿态（即对应的逆运动学8个解），为了安全，
				实验室中我们只计算一种即可。config用来作为选解的标志数。

	OUTPUTS:    theta[6] 6个关节角, 单位为弧度

	RETURN:		<none>
	***********************************************************************/
	void robotBackward(const double* TransVector, bool* mconfig, double* theta)
	{
		vector<double> th1, th2, th3, th4, th5, th6;
		Matrix4d gd;
		for (int i = 0; i < 16; i++)
		{
			gd(i / 4, i % 4) = TransVector[i];
		}

		Matrix4d g1 = gd * inverseSE3(qkmRobot.g0);

		double l1, l2, l3, l4;
		l1 = qkmRobot.L1;
		l2 = qkmRobot.L2;
		l3 = qkmRobot.L3;
		l4 = qkmRobot.L4;
		// 1. select q1 on intersect point of 4, 5, 6
		Vector4d q1(0, 0, l1 + l2 + l3, 1);
		// 2. select q2 on intersect point of 1, 2
		Vector4d q2(0, 0, l1, 1);

		Vector4d tmp_1, tmp_2;
		tmp_1 = g1 * q1 - q2;
		double delta = tmp_1.norm();

		// get theta 3
		// pick an r on axis 3
		Vector3d r(0, 0, l1 + l2);
		subProblem3(r, q1.head(3), q2.head(3), qkmRobot.w.col(2), delta, th3);

		if (th3.size() == 0) {
			std::cout << "error with theta 3" << endl;
			return;
		}

		theta[2] = th3[0];

		Matrix4d SE_th3 = R6toSE3(qkmRobot.v.col(2), qkmRobot.w.col(2), th3[0]);
		tmp_1 = SE_th3 * q1;
		tmp_2 = g1 * q1;

		r = Vector3d(0, 0, l1);	// intersect of axis 1, 2
		subProblem2(r, tmp_1.head(3), tmp_2.head(3), qkmRobot.w.col(0), qkmRobot.w.col(1), th1, th2);

		if (th1.size() == 0) {
			std::cout << "error with theta 3" << endl;
			return;
		}

		Matrix4d SE_th1 = R6toSE3(qkmRobot.v.col(0), qkmRobot.w.col(0), th1[0]);
		Matrix4d SE_th2 = R6toSE3(qkmRobot.v.col(1), qkmRobot.w.col(1), th2[0]);
		g1 = inverseSE3(SE_th3) * inverseSE3(SE_th2) * inverseSE3(SE_th1) * g1;

		// select q3 on axis 6
		Vector4d q3(0, 0, l1 + l2 + l3 + l4, 1);
		tmp_1 = q3;
		tmp_2 = g1 * q3;
		r = Vector3d(0, 0, l1 + l2 + l3);
		subProblem2(r, tmp_1.head(3), tmp_2.head(3), qkmRobot.w.col(3), qkmRobot.w.col(4), th4, th5);

		if (th4.size() == 0) {
			std::cout << "error with theta 3" << endl;
			return;
		}

		Matrix4d SE_th4 = R6toSE3(qkmRobot.v.col(3), qkmRobot.w.col(3), th4[0]);
		Matrix4d SE_th5 = R6toSE3(qkmRobot.v.col(4), qkmRobot.w.col(4), th5[0]);

		g1 = inverseSE3(SE_th5) * inverseSE3(SE_th4) * g1;

		// select q4 not on axis 6
		Vector4d q4(0, 1, 0, 1);
		r = Vector3d(0, 0, 0);
		tmp_1 = q4;
		tmp_2 = g1 * q4;
		double tmp_val = 0;
		subProblem1(r, tmp_1.head(3), tmp_2.head(3), qkmRobot.w.col(5), tmp_val);
		th6.emplace_back(tmp_val);

		// organize 8 solution
		// th3 -> 2  th1 th2 -> 2 th4 th5 -> 2
		vector<Vector6d> sol;
		for (int i = 0; i < 2; i++) {
			Vector6d tmp;
			tmp(2) = th3[i];
			for (int j = 0; j < 2; j++) {
				tmp(0) = th1[j];
				tmp(1) = th2[j];
				for (int k = 0; k < 2; k++) {
					tmp(3) = th4[k];
					tmp(4) = th5[k];
					tmp(5) = th6[0];
					sol.emplace_back(tmp);
				}
			}
		}

		// get solution
		Vector6d res = sol[mconfig[2] * 4 + mconfig[1] * 2 + mconfig[0] * 1];
		for (int i = 0; i < 6; i++) { theta[i] = res(i); }

		//for (int i = 0; i < 8; i++) {
		//	cout << sol[i].transpose() << endl;
		//}

	}

	/********************************************************************
	ABSTRACT:	机器人正运动学

	INPUTS:		q[6]: 6个关节角, 单位为弧度

	OUTPUTS:	config[3]：姿态，六轴机器人对应有8种姿态，为了安全，
				实验室中我们只计算一种即可。config用来作为选解的标志数。

				TransVector[16] : 刚体变换矩阵，也就是末端的位姿描述，其中长度距离为米

	RETURN:		<none>
	***********************************************************************/
	void robotForward(const double* q, double* TransVector, bool* mconfig)
	{

		Matrix4d g = qkmRobot.g0;

		for (int i = 5; i >= 0; i--)
		{
			Vector3d w_t = qkmRobot.w.col(i);
			Vector3d q_t = qkmRobot.q.col(i);
			Vector3d v_t = -w_t.cross(q_t);
			g = R6toSE3(v_t, w_t, q[i]) * g;
		}

		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++) {
				TransVector[i * 4 + j] = g(i, j);
			}
		}

	}


	Vector6d eigenGetJoints(Vector6d p)
	{
		Vector6d res;
		SetRobotEndPos(p(0), p(1), p(2), p(3), p(4), p(5));
		GetJointAngles(res(0), res(1), res(2), res(3), res(4), res(5));
		return res;
	}
}
