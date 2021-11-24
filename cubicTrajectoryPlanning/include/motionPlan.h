#pragma once
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include "HLrobotconfig.h"
#include <vector>
#include "genTraj.h"

using namespace Eigen;
using namespace std;
using namespace HLRobot;

typedef Matrix<double, 6, 1> Vector6d;

class motionPlan
{
public:
	motionPlan();
	~motionPlan(){}
	
	void init_pos(string root, string p_up_fn, string p_down_fn, string q_up_fn, string q_down_fn, string Jfn);
	void loadConfig(string root, string p_up_fn, string p_down_fn, string q_up_fn, string q_down_fn, string Jfn);
	void test_knock(int idx);
	void test_fromOneToAnother(int from, int to);

	void up_and_down(double dth, double td, const Vector6d& po, const Vector6d& qo, const Vector6d& dqo, ofstream& out);
	void samplePoints(ofstream& out, Matrix<double, 6, 4>& cof, double t, double spt);

	void playSong(const vector<int>& note, const vector<double>& interval,
		const vector<double>& volume, string ppb);



private:
	const int num = 15;
	const double sample_time = 0.001;

	// for init pose
	Vector3d up, down, low, high;

	vector<Vector6d> p_up, p_down, q_up, q_down;
	vector<Matrix6d> Jvec;
};
