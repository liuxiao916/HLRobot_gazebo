
#include<iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include<winsock.h>
#include <conio.h>
#include <stack>
#include "HLrobotconfig.h"
#include "eigen3/Eigen/Dense"
#include "motionPlan.h"
#include "FtpControl.h"

#pragma comment(lib,"ws2_32.lib")

#define SENDCMD 4
#define Move 5
#define READFILE 6
#define SUCK 7

#define OPEN
#define PPB

using namespace std;
using namespace Eigen;
using namespace HLRobot;
typedef Matrix<double, 6, 1> Vector6d;

enum MoveOperation
{
	Line, Joint
};

void initialization();
#pragma comment(lib, "WS2_32.lib")


void readNote(string notefile, vector<int>& note, vector<double>& interval, vector<double>& volume)
{
    ifstream inf(notefile);
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

void runPPB(SOCKET& ser, string dst)
{
	
	int send_len, recv_len;
	char recv_buf[100] = {};
    char cmd[100] = {};
 
	//PPB读取文件
    sprintf_s(cmd, sizeof(cmd), "[%d# PPB.ReadFile 1,/data/%s]", READFILE, dst.c_str());
    cout << cmd << endl;
	send_len = send(ser, cmd, 100, 0);
	recv_len = recv(ser, recv_buf, 100, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
    memset(cmd, '\0', sizeof(cmd));
	Sleep(500);
	// 到达起始点
	send_len = send(ser, "[1# PPB.J2StartPoint 1,0,1]", 100, 0);
	recv_len = recv(ser, recv_buf, 100, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(2000);
	// PPB run
	send_len = send(ser, "[1# PPB.Run 1]", 100, 0);
	recv_len = recv(ser, recv_buf, 100, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(1000);

}

// 全局要首先设置一个p变量
void sendCmd(SOCKET& ser, Vector6d &j, int move_type) {

	char cmd[128] = {0};
	sprintf_s(cmd, sizeof(cmd), "[%d# q=%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]", SENDCMD, j(0), j(1), j(2), j(3), j(4), j(5));
    cout << cmd << endl;
	int send_len, recv_len;
	char recv_buf[200] = {};
	send_len = send(ser, cmd, 200, 0);
	recv_len = recv(ser, recv_buf, 200, 0);
	cout << recv_buf << endl;

	memset(cmd, 0, sizeof(cmd));
	memset(recv_buf, 0, sizeof(recv_buf));
    Sleep(100);
	if (move_type == Line) {
		sprintf_s(cmd, sizeof(cmd), "[%d# Move.Line p]", Move);
		send_len = send(ser, cmd, 200, 0);
		recv_len = recv(ser, recv_buf, 200, 0);
		cout << recv_buf << endl;
	}
	else if (move_type == Joint) {
        cout << "moving!" << endl;
		sprintf_s(cmd, sizeof(cmd), "[%d# Move.Joint q]", Move);
		send_len = send(ser, cmd, 200, 0);
		recv_len = recv(ser, recv_buf, 200, 0);
		cout << recv_buf << endl;
	}
	else {
		cout << "invalid move type" << endl;
	}
}



int main()
{   
    
#ifdef OPEN

	//定义长度变量
	int send_len = 0;
	int recv_len = 0;
	//定义发送缓冲区和接受缓冲区
    char send_buf[100] = {};
	char recv_buf[200] = {};
    string recvstr;
	//定义服务端套接字，接受请求套接字
	SOCKET s_server;
	//服务端地址客户端地址
	SOCKADDR_IN server_addr;
	initialization();
	//填充服务端信息
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.10.120");
	server_addr.sin_port = htons(2090);
	//创建套接字
	s_server = socket(AF_INET, SOCK_STREAM, 0);
	if (connect(s_server, (SOCKADDR *)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
		cout << "服务器连接失败！" << endl;
		WSACleanup();
	}
	else {
		cout << "服务器连接成功！" << endl;
	}

	//登录
    send_len = send(s_server, "[0# System.Login 0]", 100, 0);
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << recv_buf << endl;
    memset(recv_buf,'\0',sizeof(recv_buf));
    Sleep(100);
	//使能
    send_len = send(s_server, "[0# Robot.PowerEnable 1,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(100);

	send_len = send(s_server, "[0# System.Abort 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(100);
    
	send_len = send(s_server, "[6# System.Start 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "start: " << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(100);

	send_len = send(s_server, "[9# Robot.Home 1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 200, 0);
	cout << "home: " << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(100);

    send_len = send(s_server, "[8# System.Auto 1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << "auto: " << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(100);

    // set point 
    //send_len = send(s_server, "[1# LocationJ q]", 100, 0);
    //recv_len = recv(s_server, recv_buf, 100, 0);
    //cout << recv_buf << endl;
    //memset(recv_buf, '\0', sizeof(recv_buf));
    //Sleep(10);


#ifdef PPB

	//PPB
	send_len = send(s_server, "[1# PPB.Enable 1,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(500);
	//关节坐标系
	send_len = send(s_server, "[1# Robot.Frame 1,1]", 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(500);

#endif // PPB

	// write your own code
	


#endif // OPEN

    motionPlan motion = motionPlan();
    string root = "C:/Users/Administrator/Desktop/rbpro/";
    motion.loadConfig(root, "p_up.txt", "p_down.txt", "q_up.txt", "q_down.txt", "Jfile.txt");
    //motion.test_knock(3);
    //motion.test_fromOneToAnother(0, 3);

    vector<int> note;
    vector<double> interval;
    vector<double> volume;
    
    
   string notefile = "C:/Users/Administrator/Desktop/rbpro/solar.txt";
    //string notefile = "C:/Users/Administrator/Desktop/rbpro/hitsong.txt";

    readNote(notefile, note, interval, volume);
   
    for (int& e : note) {
        e += 2;
    }
    //for (int e : note) cout << e << " " << endl;
    for (double& e : interval)
    {
        e *= 0.8;
    }
        //for (int e : note)   cout << e << " " << endl;
    //cout << note.size() << "  " << interval.size() << endl;

    for (int i = 0; i < note.size(); i++)    volume.push_back(10);

    motion.playSong(note, interval, volume, "C:/Users/Administrator/Desktop/rbpro/output/solar.txt");
    //motion.playSong(note, interval, volume, "C:/Users/Administrator/Desktop/rbpro/output/hitsong.txt");
    //string dst = "test_knock.txt";
    ////string dst = "test_from.txt";
    string dst = "solar.txt";
   // string dst = "hitsong.txt";
    //string dst = "littlestar";

#ifdef PPB

    FtpControl::Upload("192.168.10.101", "data" , root + "output/solar.txt", dst);
    //FtpControl::Upload("192.168.10.101", "data", root + "output/hitsong.txt", dst);

    //FtpControl::Upload("192.168.10.101", "data" , root + "output/test_fromOneToAnother.txt", dst);
    //FtpControl::Upload("192.168.10.101", "data", root + "output/solar.txt", dst);
    Sleep(400);
    runPPB(s_server, dst);

#endif // PPB
    // move J
    //Vector6d q;
    //q << -21.2185, 28.2899, 102.869, -14.4835, 48.9455, 265.769;
    ////q << -20.5356, 32.5688, 97.2993, -15.9786, 75.987, 266.835;
    //sendCmd(s_server, q, Joint);

#ifdef OPEN
    closesocket(s_server);
    //释放DLL资源
    WSACleanup();
#endif // OPEN

	return 0;
}
void initialization() {
	//初始化套接字库
	WORD w_req = MAKEWORD(2, 2);//版本号
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0) {
		cout << "初始化套接字库失败！" << endl;
	}
	else {
		cout << "初始化套接字库成功！" << endl;
	}
	//检测版本号
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "套接字库版本号不符！" << endl;
		WSACleanup();
	}
	else {
		cout << "套接字库版本正确！" << endl;
	}
}
