#include "Clutch.h"
int global_clutch = CLUTCH::STATE::FREE;

Clutch::Clutch()
{
	state = CLUTCH::STATE::FREE;
}

Clutch::~Clutch()
{
}

int Clutch::init()
{
	printf("[Clutch] initialize\n");

	if (wiringPiSetupSys() < 0) {
		printf("[CLUTCH] wiringPi Setup Failed\n");
		return -1;
	}

	//UDP recieve socket setup
	sock_recv = socket(AF_INET, SOCK_DGRAM, 0);

	if (sock_recv < 0) {
		printf("%s:Socket error:%d", __FUNCTION__, sock);
		return -1;
	}

	addr_recv.sin_family = AF_INET;
	addr_recv.sin_port = htons(CLUTCH::RECV_PORT);
	addr_recv.sin_addr.s_addr = INADDR_ANY;

	if (bind(sock_recv, (struct sockaddr*)&addr_recv, sizeof(addr_recv)) < 0) {
		perror("bind");
		return -1;
	}

	//nonblocking mode
	int val = 1;
	ioctl(sock_recv, FIONBIO, &val);

	//UDP socket setup
	sock = socket(AF_INET, SOCK_DGRAM, 0);

	if (sock < 0) {
		printf("%s:Socket error:%d", __FUNCTION__, sock);
		return -1;
	}

	addr.sin_family = AF_INET;
	addr.sin_port = htons(CLUTCH::SEND_PORT);
	addr.sin_addr.s_addr = inet_addr("192.168.1.255");

	int yes = 1;
	if (setsockopt(sock, SOL_SOCKET,
		SO_BROADCAST, (char*)&yes, sizeof(yes)) == -1) {
		printf("Broad cast setup failed\n");
		return -1;
	}


	//GPIOÝ’è
	pinMode(CLUTCH::PIN_CONFIG::RELAY_ON_OFF, OUTPUT);
	pinMode(CLUTCH::PIN_CONFIG::RELAY_F_R, OUTPUT);
	pullUpDnControl(CLUTCH::PIN_CONFIG::RELAY_ON_OFF, PUD_DOWN);
	pullUpDnControl(CLUTCH::PIN_CONFIG::RELAY_F_R, PUD_DOWN);

	digitalWrite(CLUTCH::PIN_CONFIG::RELAY_ON_OFF, LOW);
	digitalWrite(CLUTCH::PIN_CONFIG::RELAY_F_R, LOW);

	digitalWrite(CLUTCH::PIN_CONFIG::RELAY_ON_OFF, HIGH);
	delay(500);
	digitalWrite(CLUTCH::PIN_CONFIG::RELAY_ON_OFF, LOW);

	return 0;
}

void Clutch::recv()
{
	int recv;
	struct sockaddr_in from;
	socklen_t sin_size;

	if (recvfrom(sock_recv, (char*)&recv, sizeof(int), 0,
		(struct sockaddr*)&from, &sin_size) < 0) {
		if (errno == EAGAIN) {
			return; //not reached data
		}
		perror("recv");
		return;
	}

	switch (recv) {
	case CLUTCH::STATE::FORWARD:
		setFoward();
		break;
	case CLUTCH::STATE::BACKWARD:
		setBackward();
		break;
	case CLUTCH::STATE::FREE:
		setFree();
		break;
	default:
		break;
	}

	return;
}

void Clutch::send()
{
	int send = state;

	int s = sendto(sock, (char*)&send, sizeof(int), 0,
		(struct sockaddr*)&addr, sizeof(addr));
	if (s == -1) { //local error
		perror("socket");
		return;
	}

	//printf("[Clutch]:sent state\n");

	return;
}

int Clutch::setClutchOn()
{
	digitalWrite(CLUTCH::PIN_CONFIG::RELAY_ON_OFF,
		CLUTCH::PIN_CONFIG::STATE_ON);
	return 0;
}

int Clutch::setClutchOff()
{
	digitalWrite(CLUTCH::PIN_CONFIG::RELAY_ON_OFF,
		CLUTCH::PIN_CONFIG::STATE_OFF);
	return 0;
}

int Clutch::setFoward()
{
	if (state == CLUTCH::STATE::FORWARD) {
		return 0;	//Already
	}

	setClutchOff();
	delay(200);

	digitalWrite(CLUTCH::PIN_CONFIG::RELAY_F_R,
		CLUTCH::PIN_CONFIG::STATE_FOWARD);

	setClutchOn();



	//check
	if (digitalRead(CLUTCH::PIN_CONFIG::RELAY_ON_OFF)
		!= CLUTCH::PIN_CONFIG::STATE_ON) {
		return -1;
	}
	if (digitalRead(CLUTCH::PIN_CONFIG::RELAY_F_R)
		!= CLUTCH::PIN_CONFIG::STATE_FOWARD) {
		return -1;
	}

	//Success
	state = CLUTCH::STATE::FORWARD;
	global_clutch = CLUTCH::STATE::FORWARD;
	return 0;
}

int Clutch::setBackward()
{
	if (state == CLUTCH::STATE::BACKWARD) {
		return 0;	//Already
	}

	setClutchOff();
	delay(200);

	digitalWrite(CLUTCH::PIN_CONFIG::RELAY_F_R,
		CLUTCH::PIN_CONFIG::STATE_REVERSE);

	setClutchOn();

	if (digitalRead(CLUTCH::PIN_CONFIG::RELAY_ON_OFF)
		!= CLUTCH::PIN_CONFIG::STATE_ON) {
		return -1;
	}
	if (digitalRead(CLUTCH::PIN_CONFIG::RELAY_F_R)
		!= CLUTCH::PIN_CONFIG::STATE_REVERSE) {
		return -1;
	}

	//Success
	state = CLUTCH::STATE::BACKWARD;
	global_clutch = CLUTCH::STATE::BACKWARD;

	return 0;
}

int Clutch::setFree()
{
	if (state == CLUTCH::STATE::FREE) {
		return 0;	//Already
	}

	setClutchOff();

	if (digitalRead(CLUTCH::PIN_CONFIG::RELAY_ON_OFF)
		!= CLUTCH::PIN_CONFIG::STATE_OFF) {
		return -1;
	}

	//Success
	state = CLUTCH::STATE::FREE;
	global_clutch = CLUTCH::STATE::FREE;

	return 0;
}
