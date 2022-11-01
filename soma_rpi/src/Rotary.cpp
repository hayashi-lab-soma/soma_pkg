#include "Rotary.h"

int Rotary::pulsecnt[2] = { 0, 0 };
std::chrono::system_clock::time_point Rotary::chrono_t;
double Rotary::t = -1.0;
double Rotary::v = 0.0;

Rotary::Rotary()
{
}

Rotary::~Rotary()
{
}

int Rotary::init()
{
	printf("[Rotary] Initialize\n");

	//wiringPi setup
	if (wiringPiSetupSys() < 0) {
		printf("[Rotary] wiringPi Setup Failed\n");
		return -1;
	}

	//UDP recieve socket setup
	sock_recv = socket(AF_INET, SOCK_DGRAM, 0);

	if (sock_recv < 0) {
		printf("%s:Socket error:%d", __FUNCTION__, sock);
		return -1;
	}

	addr_recv.sin_family = AF_INET;
	addr_recv.sin_port = htons(ROTARY::RECV_PORT);
	addr_recv.sin_addr.s_addr = INADDR_ANY;

	if (bind(sock_recv, (struct sockaddr*)&addr_recv, sizeof(addr_recv)) < 0) {
		perror("bind");
		return -1;
	}

	//nonblocking mode
	int val = 1;
	ioctl(sock_recv, FIONBIO, &val);

	//UDP socket setup
	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	if (sock < 0) {
		printf("%s:Socket error:%d", __FUNCTION__, sock);
		return -1;
	}

	addr.sin_family = AF_INET;
	addr.sin_port = htons(ROTARY::SEND_PORT);
	addr.sin_addr.s_addr = inet_addr("192.168.1.11");

	int yes = 1;
	if (setsockopt(sock, SOL_SOCKET,
		SO_BROADCAST, (char*)&yes, sizeof(yes)) == -1) {
		printf("Broad cast setup failed\n");
		return -1;
	}
	if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1) {
		return -1;
	}

//	ioctl(sock, FIONBIO, &yes);

	pinMode(ROTARY::PIN_CONFIG::IN, INPUT);
	pullUpDnControl(ROTARY::PIN_CONFIG::IN, PUD_DOWN);

	if (wiringPiISR(ROTARY::PIN_CONFIG::IN,
		INT_EDGE_RISING,
		&Rotary::interrupt) < 0) {
		printf("[Rotary] ISR Setup Failed\n");
		return -1;
	}
	printf("%s:Success ISR setup\n", __FUNCTION__);

	//velocity calculation thread start
	isCalcVelocity = true;
	th_calcVelo = new std::thread(&Rotary::calcVelocity, this);

	chrono_t = std::chrono::system_clock::now();
	return 0;
}

void Rotary::interrupt()
{
	//printf("%s:Interrupt!\n", __FUNCTION__);
	int v = digitalRead(ROTARY::PIN_CONFIG::IN);
	if (v != HIGH) return;

	//現在時間取得
	std::chrono::system_clock::time_point n = std::chrono::system_clock::now();
	//前回との差分計算
	t = std::chrono::duration_cast<std::chrono::milliseconds>(n - chrono_t).count();

	if (t < 10.0) {
		printf("chataling?\n");
		return; //chataling protection
	}

	//pulsecnt[1] = pulsecnt[0];
	pulsecnt[0]++;

	//シフト
	chrono_t = n;
	return;
}

void Rotary::calcVelocity()
{
	isCalcVelocity = true;
	double lim_T = 2000;	//[ms]
	int dt = 33;
	long timecnt = 1;

	while (isCalcVelocity) {
		//printf("call\n");

		//calc increased pulse count
		long incr = pulsecnt[0] - pulsecnt[1];

		if (incr > 0) {	//positive
			double delta = (double)(incr)*(ROTARY::SENSOR_R*3.1415 / 4.0); // m/s
			v = delta/((double)timecnt*(double)(dt)/1000.0);
			v *= ROTARY::TIRE_R / ROTARY::SENSOR_R;

			if (global_clutch == CLUTCH::STATE::BACKWARD) {
				v = -1.0 * v;
			}

			pulsecnt[1] = pulsecnt[0];
			timecnt = 1;
		}
		else if (incr == 0) { //増分なし
			timecnt++;
		}
		else if (incr < 0) { //カウンタがリセットされている
			pulsecnt[1] = 0;
			timecnt = 1;
		}


		if (timecnt >= (lim_T / (double)dt)) v = 0.0;

		delay(dt);
	}
}

void Rotary::recv()
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

	if (recv == ROTARY::RESET_CODE) {
		pulsecnt[1] = pulsecnt[0] = 0;
	}
}

void Rotary::send()
{
	struct Send_t
	{
		unsigned long pulse_cnt;
		double velocity;
	} send;

	send.pulse_cnt = pulsecnt[0];
	send.velocity = v;

	int s = sendto(sock, (char*)&send, sizeof(Send_t), 0,
		(struct sockaddr*)&addr, sizeof(addr));
	
	if (s == -1) { //local error
		perror("socket");
		return;
	}

	//printf("sent:%dbyte, (%d)\n", s, send.pulse_cnt);
	return;
}
