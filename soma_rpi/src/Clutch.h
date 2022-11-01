#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <error.h>
#include <errno.h>

namespace CLUTCH
{
	const int SEND_PORT = 12345;
	const int RECV_PORT = 22345;

	namespace PIN_CONFIG
	{
		const int RELAY_ON_OFF = 6;		//ON or OFF
		const int RELAY_F_R = 5;		//Forward or Reverse

		const int STATE_ON = HIGH;
		const int STATE_OFF = LOW;
		const int STATE_FOWARD = HIGH;
		const int STATE_REVERSE = LOW;
	}

	namespace STATE
	{
		const int FORWARD = 1;
		const int BACKWARD = 2;
		const int FREE = 3;
	}

	const int REPLY_STATE = 15;
}


extern int global_clutch;

class Clutch
{
public:
	Clutch();
	~Clutch();

	int init();
	void recv();
	void send();


	int setClutchOn();
	int setClutchOff();

	int setFoward();
	int setBackward();
	int setFree();

private:
	int sock;
	struct sockaddr_in addr;

	int sock_recv;
	struct sockaddr_in addr_recv;

	int state;
};

