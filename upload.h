#ifndef _HIFLY_UPLOADER_H
#define _HIFLY_UPLOADER_H

#include <stdio.h>
#include <stdbool.h>

/**
	Serial firmware uploader for the HIFLY bootloader
	
	param1:serial port
	param2:baudrate
	param3:firmware Bin file path 
**/
int autopilot_upgrade_start(const char *dev, const int baudrate, const char *filenames);
/**
	autopilot upgrade progress

	param1:ERASE:step=1;PROGRAM:step=2;VERIFY:step=3;
	param2:Upgrade progress:0~100,complete is 100
**/
void autopilot_upgrade_progress(int *step, int *progress);

#endif
