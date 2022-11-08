#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <chrono>
#include <sys/ioctl.h>
#include <error.h>
#include <errno.h>

#include <thread>
#include <mutex>

#include "Clutch.h"

class Rotary
{
public:
	Rotary();
	~Rotary();

	int init();
	void recv();
	void send();

private:
	static void interrupt();
	void calcVelocity();

private:
	int sock;
	struct sockaddr_in addr;

	int sock_recv;
	struct sockaddr_in addr_recv;

	std::thread *th_calcVelo;
	std::mutex mtx;
	bool isCalcVelocity;

	static int pulsecnt[2];
	static std::chrono::system_clock::time_point chrono_t;
	static double t;
	static double v;
};

namespace ROTARY
{
	const int SEND_PORT = 12346;
	const int RECV_PORT = 22346;

	namespace PIN_CONFIG {
		const int IN = 17;
	}
	const double TIRE_R = 0.45;			//?^?C?????a[m]
	const double SENSOR_R = 0.12;		//?Z???T????_???a[m]
	const double RESOLUTION = 0.345;	//1?p???X?????????s????[m]

	const int RESET_CODE = 5;	//?p???X?J?E???g???Z?b?g?R?[?h

	namespace CALC_VELO
	{
		const int dt = 100;	//[ms]
		const int lim_T = 1500.0;	//when over it, v=0.0
	}
}
