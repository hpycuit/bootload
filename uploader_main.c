#include <stdio.h>
#include <string.h>
#include "stdbool.h"
#include "upload.h"

int
main(int argc,char *argv[])
{
	int ret;
	char *fn;
	const char *uart_name = "/dev/ttyUSB0";
	int baud = 115200;
	fn="./Firmware.bin";
	int step;
	int progress;
	bool complete;

	system("clear");

	if(!strcmp(argv[1], "start")) {
		complete = false;
		ret = autopilot_upgrade_start(uart_name, baud, fn);	
		while(!complete)
		{
			ret = get_upgrade_progress(&complete, &step, &progress);
			printf("\033[2;0H\033[2K\rstep:%d  progress:%d\n", step, progress);
			printf("\033[3;0H\033[2K\rstderr:%s\n", strerror(ret));
			sleep(1);
			if ((step == 3) && (progress == 100)) {
				break;
			}
		}
		return 0;
	}

	errx(1, "unrecognized command, try 'start'");
}
