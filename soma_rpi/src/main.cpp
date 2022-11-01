#include <stdio.h>
#include <stdlib.h>

#include "Rotary.h"
#include "Clutch.h"

int main(void)
{
	printf(" --- ATV Raspberry Pi Program Ver.2 --- \n");

	Clutch clutch;
	Rotary rot;

	if (clutch.init() == -1) return -1;
	if (rot.init() == -1) return -1;

	printf("--- Success initialize ---\n");

	while (1) {
		clutch.recv();

		rot.send();
		clutch.send();

		usleep(100000);
	}

	return 0;
}