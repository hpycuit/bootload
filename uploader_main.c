#include "upload.h"

int
main()
{
	int ret;
	char *fn;
	const char *uart_name = "/dev/ttyUSB0";
	int baud = 115200;
	fn="./firmware.bin";

	ret = autopilot_upgrade_start(uart_name, baud, fn);	
	
	return ret;
}

