#ifndef _HIFLY_UPLOADER_H
#define _HIFLY_UPLOADER_H

#include <stdio.h>
#include <stdbool.h>

#define OK						0
#define PROTO_NOP			 0x00
#define PROTO_OK			 0x10
#define PROTO_FAILED		 0x11
#define PROTO_INSYNC		 0x12
#define PROTO_EOC			 0x20
#define PROTO_GET_SYNC		 0x21
#define PROTO_GET_DEVICE	 0x22
#define PROTO_CHIP_ERASE	 0x23
#define PROTO_CHIP_VERIFY	 0x24
#define PROTO_PROG_MULTI	 0x27
#define PROTO_READ_MULTI	 0x28
#define PROTO_GET_CRC		 0x29
#define PROTO_REBOOT		 0x30

#define INFO_BL_REV			 1		/**< bootloader protocol revision */
#define BL_REV				 4		/**< supported bootloader protocol  */
#define INFO_BOARD_ID		 2		/**< board type */
#define INFO_BOARD_REV		 3		/**< board revision */
#define INFO_FLASH_SIZE		 4		/**< max firmware size in bytes */

#define PROG_MULTI_MAX		 252		/**< protocol max is 255, must be multiple of 4 */

#define ERASE				1
#define PROGRAM				2
#define VERIFY				3

uint8_t nsh_init[] = {0x0d, 0x0d, 0x0d};
char nsh_reboot_bl[] = "reboot -b \n";
char nsh_reboot[] = "reboot\n";
uint8_t mavlink_reboot_id1[] = {0xfe, 0x21, 0x72, 0xff, 0x00, 0x4c, 0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf6, 0x00, 0x01, 0x00, 0x00, 0x48, 0xf0}; 
uint8_t mavlink_reboot_id0[] = {0xfe, 0x21, 0x45, 0xff, 0x00, 0x4c, 0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf6, 0x00, 0x00, 0x00, 0x00, 0xd7, 0xac};

int _boot_fd = -1;
int _fw_fd = -1;

int firmware_step;
int firmware_progress;

struct termios uart_config_original;

uint32_t		bl_rev; /**< bootloader revision */

int			recv_byte_with_timeout(uint8_t *c, unsigned timeout);
int			recv_bytes(uint8_t *p, unsigned count);
void			drain();
int			send(uint8_t c);
int			send_buf(uint8_t *p, unsigned count);
int			get_sync(unsigned timeout);
int			__sync();
uint8_t			get_info(int param, uint32_t *val);
int			erase();
int			program(size_t fw_size);
int			verify_rev3(size_t fw_size);
int			reboot();
int			upload(const char *filenames);
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
