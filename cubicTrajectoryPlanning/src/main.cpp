#include <iostream>
#include <sstream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "motionPlan.h"

using namespace std;

void readNote(vector<int>& note, vector<double>& interval, vector<double>& volume)
{
	ifstream inf("./note/littlestar.txt");
	string tmp;
	stringstream ss;

	// get note
	getline(inf, tmp);
	ss = stringstream(tmp);
	int tmp_note;
	while (ss >> tmp_note)
	{
		note.emplace_back(tmp_note);
	}

	// get interval
	getline(inf, tmp);
	ss = stringstream(tmp);
	double tmp_interval;
	while (ss >> tmp_interval) {
		interval.emplace_back(tmp_interval);
	}

	// get volume
	getline(inf, tmp);
	ss = stringstream(tmp);
	double tmp_volume;
	while (ss >> tmp_volume) {
		volume.emplace_back(tmp_volume);
	}
	
	inf.close();

}

int main()
{
	
	motionPlan motion = motionPlan();
	motion.loadConfig("./", "p_up.txt", "p_down.txt", "q_up.txt",
		"q_down.txt", "Jfile.txt");
	//motion.test_knock(0);

	//motion.test_fromOneToAnother(0, 3);

	vector<int> note;
	vector<double> interval;
	vector<double> volume;

	readNote(note, interval, volume);



	//Vector3d p = (w.transpose() * v) * w;	// good
	//Vector3d p = 1.0 * (w.transpose() * v) * w;	// bug
	//Vector3d p = 1.0 * w.transpose() * v * w;	// good

	//init_pos();
	
	/*Vector6d v, vv;
	v << 0, 0, 0, 0, 10.0, 0;
	vv = Jvec[0] * v;
	cout << vv.transpose() << endl;
	cout << "p_down: " << p_down[0].transpose() << endl;
	up_and_down(3.0, 0.3, p_down[0], q_down[0], vv);*/

	/*double joint[6]{ 0 };
	HLRobot::SetRobotEndPos(509.031, 218.303, 508.222, -167.586, 150.59, -67.681);
	HLRobot::GetJointAngles(joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);

	for (int i = 0; i < 6; i++)
	{
		cout << joint[i] << endl;
	}*/

	/*double joints[6] = { -20.067, 33.435, 96.225, -15.926, 79.755, 267.17 };
	SetRobotJoint(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
	double xyzypr[6]{ 0 };
	
	GetJointEndPos(xyzypr[0], xyzypr[1], xyzypr[2], xyzypr[3], xyzypr[4], xyzypr[5]);

	for (int i = 0; i < 6; i++)
	{
		cout << xyzypr[i] << " ";
	}

	cout << endl;*/

	return 0;
}