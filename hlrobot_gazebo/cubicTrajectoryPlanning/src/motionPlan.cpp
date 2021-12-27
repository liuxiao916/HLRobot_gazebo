#include "motionPlan.h"

motionPlan::motionPlan()
{
  // 165 153 -99
	up << 154.434, 167.177, -109.327;
	down << 165.47, 151.645, -98.897;
	low << 464.5, -215.887, 542.236;
	high << 486.5, 216.373, 542.236;
	/*p_up.resize(num);
	p_down.resize(num);
	q_up.resize(num);
	q_down.resize(num);
	Jvec.resize(num);*/
}

void motionPlan::init_pos(string root, string p_up_fn, string p_down_fn, string q_up_fn, string q_down_fn, string Jfn)
{
	
	ofstream p_up_file(root + p_up_fn);
	ofstream p_down_file(root + p_down_fn);
	ofstream q_up_file(root + q_up_fn);
	ofstream q_down_file(root + q_down_fn);
	ofstream jacobian_file(root + Jfn);

	// 还可以预先计算好jacobian

	for (int i = 0; i < num; i++)
	{
		Vector6d tmp;
		double w = 1.0 * i / (num - 1);
		tmp.head<3>() = low + w * (high - low);
		tmp.tail<3>() = up;
		p_up.push_back(tmp);
		p_up_file << tmp.transpose() << endl;
		q_up.push_back(HLRobot::eigenGetJoints(tmp));
		q_up_file << q_up[i].transpose() << endl;
		tmp.tail<3>() = down;
		p_down.push_back(tmp);
		p_down_file << tmp.transpose() << endl;
		q_down.push_back(HLRobot::eigenGetJoints(tmp));
		q_down_file << q_down[i].transpose() << endl;

		Matrix6d J = getJacobian(q_down[i]);
		J = J.inverse();
		for (int j = 0; j < 6; j++)
		{
			jacobian_file << J.row(j) << endl;
		}
	}

	p_up_file.close();
	q_up_file.close();
	p_down_file.close();
	q_down_file.close();
	jacobian_file.close();
}

void motionPlan::loadConfig(string root, string p_up_fn, string p_down_fn, string q_up_fn, string q_down_fn, string Jfn)
{
	ifstream p_up_file(root + p_up_fn);
	ifstream p_down_file(root + p_down_fn);
	ifstream q_up_file(root + q_up_fn);
	ifstream q_down_file(root + q_down_fn);
	ifstream jacobian_file(root + Jfn);

    //if (jacobian_file.is_open()) cout << "good" << endl;
    //else    cout << "failed" << endl;

    auto readLine = [](ifstream& inf, int num) -> Vector6d {
		Vector6d res;
		for (int i = 0; i < num; i++)	inf >> res(i);
		return res;
	};
	for (int i = 0; i < num; i++)
	{
		Vector6d tmp;
		tmp = readLine(p_up_file, 6);
        p_up.push_back(tmp);
		tmp = readLine(p_down_file, 6);
		p_down.push_back(tmp);
		tmp = readLine(q_up_file, 6);
		q_up.push_back(tmp);
		tmp = readLine(q_down_file, 6);
		q_down.push_back(tmp);

		// read jacobian_inv
		Matrix6d Jtmp;
		for (int j = 0; j < 6; j++)
		{
			tmp = readLine(jacobian_file, 6);
			Jtmp.row(j) = tmp;
		}
		Jvec.push_back(Jtmp);
	}

	p_up_file.close();
	q_up_file.close();
	p_down_file.close();
	q_down_file.close();
	jacobian_file.close();
}

void motionPlan::test_knock(int idx)
{
	// 假设已经到达该点初始位置
	// debug parameter
	double td = 1.0;
	double dv = 10.0;

	// volume related
	double stop_time = 0.05;
	double dth = 3;

	ofstream ppbFile("C:/Users/Administrator/Desktop/rbpro/output/test_knock.txt");
	// 1. qo dqo qf dqf
	Vector6d qo = q_up[idx];
    Vector6d qf;
	Vector6d dqo = Vector6d::Zero();

    // minus some angle
    qf = minusPitchAngle(-dth, p_down[idx]);
	Vector6d dqf;
	dqf << 0, 0, 0, 0, dv, 0;
	dqf = Jvec[idx] * dqf;

	Matrix<double, 6, 4> cof_mat;
	cof_mat = getCofMatrix(qo, dqo, qf, dqf, td);
	samplePoints(ppbFile, cof_mat, td, sample_time);

    // 2. stop quickly
    qo = qf;
    dqo = dqf;
    qf = q_down[idx];
    dqf = Vector6d::Zero();
    cof_mat = getCofMatrix(qo, dqo, qf, dqf, stop_time);
    samplePoints(ppbFile, cof_mat, stop_time, sample_time);
	

    // 3. go up
	qo = qf;
	dqo = dqf;
	qf = q_up[idx];
	dqf = Vector6d::Zero();
	cof_mat = getCofMatrix(qo, dqo, qf, dqf, td);
	samplePoints(ppbFile, cof_mat, td, sample_time);

	/*string file = "E:/study/hit/机器人学/project/code/matlab/";
	ofstream outfile(file + "cubic.txt");

	for (int i = 0; i < 6; i++)
	{
		Vector4d cof = cubic(qo(i), dqo(i), qf(i), dqf(i), td);
		outfile << cof.transpose() << endl;
	}

	outfile.close();*/

	ppbFile.close();
}

void motionPlan::test_fromOneToAnother(int from, int to)
{
	// 假设已经到达该点初始位置
	// debug parameter
	double td = 1.0;
	double dv = 10.0;
	double interval = 2.0;

	// volume related
	double stop_time = 0.05;
	double dth = 3.0;

	ofstream ppbFile("C:/Users/Administrator/Desktop/rbpro/output/test_fromOneToAnother.txt");
	
	//1. from down and knock
	Vector6d po;
    Vector6d qo = q_up[from];
	Vector6d pf = p_down[from];
    Vector6d qf;
    // minus some angle
    qf = minusPitchAngle(-dth, p_down[from]);
	Vector6d dqo = Vector6d::Zero();
	Vector6d dqf;
	dqf << 0, 0, 0, 0, dv, 0;
	dqf = Jvec[from] * dqf;

	Matrix<double, 6, 4> cof_mat;
	cof_mat = getCofMatrix(qo, dqo, qf, dqf, td);
	samplePoints(ppbFile, cof_mat, td, sample_time);
	
    // 2. stop quickly
    qo = qf;
    dqo = dqf;
    qf = q_down[from];
    dqf = Vector6d::Zero();
    cof_mat = getCofMatrix(qo, dqo, qf, dqf, stop_time);
    samplePoints(ppbFile, cof_mat, stop_time, sample_time);
    

	// 2. goto to point through via point
	po = p_down[from];
	qo = qf;
	dqo = dqf;
	pf = p_down[to];
	qf = minusPitchAngle(-dth, p_down[to]);
	dqf << 0, 0, 0, 0, dv, 0;
	dqf = Jvec[to] * dqf;

	Vector6d via_p = (po + pf) / 2;
	via_p.tail<3>() = p_up[from].tail<3>();
	//cout << "po: " << po.transpose() << endl;
	//cout << "qo: " << qo.transpose() << endl;

	//cout << "via p: " << via_p.transpose() << endl;
	Vector6d via_q = eigenGetJoints(via_p);
	//cout << "via q: " << via_q.transpose() << endl;

	//cout << "pf: " << pf.transpose() << endl;
	//cout << "qf: " << qf.transpose() << endl;

	Matrix<double, 6, 8> tmp_cof = getCofMatrix2(qo, dqo, via_q, qf, dqf, interval);
	
	cof_mat = tmp_cof.block<6, 4>(0, 0);
	//cout << "cof1: " << cof_mat << endl;
	samplePoints(ppbFile, cof_mat, interval / 2, sample_time);
	cof_mat = tmp_cof.block<6, 4>(0, 4);
	//cout << "cof2: " << cof_mat << endl;
	samplePoints(ppbFile, cof_mat, interval / 2, sample_time);
	

    qo = qf;
    dqo = dqf;
    qf = q_down[to];
    dqf = Vector6d::Zero();
    cof_mat = getCofMatrix(qo, dqo, qf, dqf, stop_time);
    samplePoints(ppbFile, cof_mat, stop_time, sample_time);

	//3. get up
	qo = qf;
	dqo = dqf;
	qf = q_up[to];
	dqf = Vector6d::Zero();
	cof_mat = getCofMatrix(qo, dqo, qf, dqf, td);
	samplePoints(ppbFile, cof_mat, td, sample_time);

	ppbFile.close();
}


Vector6d  motionPlan::minusPitchAngle(double dth, const Vector6d& po)
{
    Vector3d zyz = po.tail<3>() / 180 * PI;

    Eigen::AngleAxisd yawAngle(AngleAxisd(zyz(0), Vector3d::UnitZ()));
    Eigen::AngleAxisd pitchAngle(AngleAxisd(zyz(1), Vector3d::UnitY()));
    Eigen::AngleAxisd rollAngle(AngleAxisd(zyz(2), Vector3d::UnitZ()));
    Matrix3d rot;
    rot = yawAngle * pitchAngle * rollAngle;
    //cout << "rot1 : \n" << rot << endl;
    rot = AngleAxisd(dth / 180 * PI, Vector3d::UnitY()).toRotationMatrix() * rot;
    //cout << "rot2 : \n" << rot << endl;
    Vector6d pf, qf;
    pf.head<3>() = po.head<3>();
    pf.tail<3>() = RtoEulerAngles(rot);
    //cout << "pf: " << pf.transpose() << endl;
    qf = eigenGetJoints(pf);

    return qf;
}


void motionPlan::up_and_down(double dth, double td, const Vector6d& po, const Vector6d& qo, const Vector6d& dqo, ofstream& out)
{
	// 假设弹性碰撞
	// yaw dth 转 zyz euler angle
	Vector3d zyz = po.tail<3>() / 180 * PI;

	Eigen::AngleAxisd yawAngle(AngleAxisd(zyz(0), Vector3d::UnitZ()));
	Eigen::AngleAxisd pitchAngle(AngleAxisd(zyz(1), Vector3d::UnitY()));
	Eigen::AngleAxisd rollAngle(AngleAxisd(zyz(2), Vector3d::UnitZ()));
	Matrix3d rot;
	rot = yawAngle * pitchAngle * rollAngle;
	//cout << "rot1 : \n" << rot << endl;
	rot = AngleAxisd(dth / 180 * PI, Vector3d::UnitY()).toRotationMatrix() * rot;
	//cout << "rot2 : \n" << rot << endl;
	Vector6d pf, qf;
	pf.head<3>() = po.head<3>();
	pf.tail<3>() = RtoEulerAngles(rot);
	//cout << "pf: " << pf.transpose() << endl;
	qf = eigenGetJoints(pf);
	//cout << "qf: " << qf.transpose() << endl;
	Vector6d dqf = Vector6d::Zero();

	// up and down 2 segment
	Matrix<double, 6, 4> cof_mat;
	cof_mat = getCofMatrix(qo, dqo, qf, dqf, td);
	samplePoints(out, cof_mat, td, sample_time);
	cof_mat = getCofMatrix(qf, dqf, qo, -dqo, td);
	samplePoints(out, cof_mat, td, sample_time);

	//string file = "E:/study/hit/机器人学/project/code/matlab/";
	//ofstream outfile(file + "updown.txt");
	//for (int i = 0; i < 6; i++)
	//{
	//	Vector4d cof = cubic(qo(i), dqo(i), qf(i), dqf(i), td);
	//	//outfile << cof.transpose() << endl;
	//}

	//outfile.close();
}

void motionPlan::samplePoints(ofstream& out, Matrix<double, 6, 4>& cof, double t, double spt)
{
	double at = 0;
	while (at <= t)
	{
		Vector4d tt;
		tt << at * at * at, at* at, at, 1;
		Vector6d q = cof * tt;
		//out << q.transpose() << endl;
        out << q(0) << " "
            << q(1) << " "
            << q(2) << " "
            << q(3) << " "
            << q(4) << " "
            << q(5) << endl;

		at += spt;
	}
}


// 尝试输出PPB
// note: 音符与位置所以索引的对应
// interval: 两个音符的时间间隔
// volume: 音符音量
// 假设已经移动到初始音符对应位置上方
// 采样时间假定为1e-3s
void motionPlan::playSong(const vector<int>& note, const vector<double>& interval,
	const vector<double>& volume, string ppb)
{
	ofstream ppbFile(ppb);

	int num_note = note.size();
	double t_start = 1.0;
	double stop_time = 0.05;
	double dth = 3.0;

	for (int i = 0; i <= num_note; i++)
	{	
		if (i % 3 == 0)	cout << "process: "<< 1.0 * i / num_note << endl;
		Vector6d po, pf, qo, qf, dqo, dqf;
		Matrix<double, 6, 4> cof_mat;
		if (i == 0) {
			pf = p_down[note[i]];
			qo = q_up[note[i]];
			qf = minusPitchAngle(-dth, p_down[note[i]]);
			dqo = Vector6d::Zero();
			dqf << 0, 0, 0, 0, volume[i], 0;
			dqf = Jvec[note[i]] * dqf;
			cof_mat = getCofMatrix(qo, dqo, qf, dqf, t_start);
			samplePoints(ppbFile, cof_mat, t_start, sample_time);
			
            // stop quickly
            qo = qf;
            dqo = dqf;
            qf = q_down[note[i]];
            dqf = Vector6d::Zero();
            cof_mat = getCofMatrix(qo, dqo, qf, dqf, stop_time);
            samplePoints(ppbFile, cof_mat, stop_time, sample_time);
			continue;
		}
		if (i == num_note)
		{
			qo = qf;
			dqo = dqf;
			qf = q_up[note[i - 1]];
			dqf = Vector6d::Zero();
			cof_mat = getCofMatrix(qo, dqo, qf, dqf, t_start);
			samplePoints(ppbFile, cof_mat, t_start, sample_time);
			break;
		}

		po = p_down[note[i-1]];
		qo = qf;
		dqo = dqf;
		pf = p_down[note[i]];
		qf = minusPitchAngle(-dth, p_down[note[i]]);
		dqf << 0, 0, 0, 0, volume[i], 0;
		dqf = Jvec[note[i]] * dqf;

		// find via point
		Vector6d via_p = (po + pf) / 2;
		via_p.tail<3>() = p_up[note[i]].tail<3>();
		Vector6d via_q = eigenGetJoints(via_p);
		Matrix<double, 6, 8> tmp_cof = getCofMatrix2(qo, dqo, via_q, qf, dqf, interval[i - 1]);
		cof_mat = tmp_cof.block<6, 4>(0, 0);
		samplePoints(ppbFile, cof_mat, interval[i - 1] / 2, sample_time);
		cof_mat = tmp_cof.block<6, 4>(0, 4);
		samplePoints(ppbFile, cof_mat, interval[i - 1] / 2, sample_time);
		
        qo = qf;
        dqo = dqf;
        qf = q_down[note[i]];
        dqf = Vector6d::Zero();
        cof_mat = getCofMatrix(qo, dqo, qf, dqf, stop_time);
        samplePoints(ppbFile, cof_mat, stop_time, sample_time);
	}

	ppbFile.close();
}


