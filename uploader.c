#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include "crc32.h"
#include "upload.h"

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


int err = 0;
bool start = false;
const char *device;
int baud;
const char *file;
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
void		*upgrade_start(void *arg);

void
drain()
{
	uint8_t c;
	int ret;

	do {
		// the small recv_bytes timeout here is to allow for fast
		// drain when rebooting the io board for a forced
		// update of the fw without using the safety switch
		ret = recv_byte_with_timeout(&c, 40);

		if (ret == OK) {
			printf("\033[4;0H\033[2K\rdiscard 0x%02x\n", c);
			fflush(stdout);
		}
	} while (ret == OK);
}

int 
send(uint8_t c)
{
	if(write(_boot_fd, &c, 1) != 1)
	{
		return -1;
	}
	return OK;
}

int send_buf(uint8_t *p, unsigned count)
{
	int ret;
	while (count--) {
		ret = send(*p++);
		if (ret != OK)
			break;
	}
	return ret;
}

void
send_reboot()
{
	send_buf(nsh_reboot, sizeof(nsh_reboot));
	send_buf(mavlink_reboot_id1, sizeof(mavlink_reboot_id1));
	send_buf(mavlink_reboot_id0, sizeof(mavlink_reboot_id0));
}

int
recv_byte_with_timeout(uint8_t *c, unsigned timeout)
{
	struct pollfd fds;

	fds.fd = _boot_fd;
	fds.events = POLLIN;
	
	/* wait <timout> ms for a character */
	int ret = poll(&fds, 1, timeout);

	if (ret < 1) {
		printf("\033[4;0H\033[2K\rpoll timeout %d\n", ret);
		fflush(stdout);
		return -1;
	}
	
	ret = read(_boot_fd, c, 1);
	return OK;
}

int
recv_bytes(uint8_t *p, unsigned count)
{
	int ret = OK;
	
	while(count--)
	{
		ret = recv_byte_with_timeout(p++, 5000);
		if(ret != OK)
		{
			break;
		}
	}
	
	return ret;
}

int
get_sync(unsigned timeout)
{
	uint8_t c[2];
	int ret;

	ret = recv_byte_with_timeout(c, timeout);

	if (ret != OK)
	{
		printf("\033[4;0H\033[2K\rget_sync timeout\n");
		fflush(stdout);
		return ret;
	}

	ret = recv_byte_with_timeout(c + 1, timeout); 
	if (ret != OK)
	{
		return ret;
	}

	if ((c[0] != PROTO_INSYNC) || (c[1] != PROTO_OK)) {
		printf("\033[4;0H\033[2K\rbad sync 0x%02x,0x%02x\n", c[0], c[1]);
		fflush(stdout);
		return -1;
	}

	return ret;
}
/*make sure we are in sync before starting*/
int 
__sync()
{
	drain();

	send(PROTO_GET_SYNC);
	send(PROTO_EOC);
	return get_sync(40);
}

uint8_t
get_info(int param, uint32_t *val)
{ 
	uint32_t ret; 
	
	send(PROTO_GET_DEVICE);
	send(param);
	send(PROTO_EOC);

	ret = recv_bytes((uint8_t *)val, sizeof(*val));

	if (ret != OK)
		return ret;
	return get_sync(40);
}

int
erase()
{
	int ret = -1;
	time_t erase_start, deadline;
	firmware_step = 1;
	
	printf("\033[4;0H\033[2K\rerase...\n");
	fflush(stdout);
	erase_start = time(NULL);
	
	printf("\033[4;0H\033[2K\rerase...\n");
	fflush(stdout);
	send(PROTO_CHIP_ERASE);
	send(PROTO_EOC);
	
	deadline = erase_start + 20;
	while (time(NULL) < deadline)
	{
		ret = get_sync(100);
		if (ret == OK) {
			firmware_progress = 100;
			break;
		} else {
			firmware_progress = (int)((20-(deadline-time(NULL)))/20.0*100);
			printf("\033[4;0H\033[2K\rerase : %d\n", firmware_progress);
			fflush(stdout);
		}
	}

	if (ret != OK) {
		printf("\033[4;0H\033[2K\rerase timeout");
		fflush(stdout);
	}
	return ret;	
}

int
read_with_retry(int fd, void *buf, size_t n)
{
	int ret;
	uint8_t retries = 0;
	do {
		ret = read(fd, buf, n);
	} while (ret == -1 && retries++ < 100);
	if (retries != 0) {
		printf("\033[4;0H\033[2K\rread of %u bytes needed %u retries\n",
		       (unsigned)n,
		       (unsigned)retries);
		fflush(stdout);
	}
	return ret;
}

int
program(size_t fw_size)
{
	ssize_t count;
	int ret;
	size_t sent = 0;

	uint8_t file_buf[PROG_MULTI_MAX];
	firmware_step = 2;

	memset(file_buf, 0, sizeof(file_buf));

	if (!file_buf) {
		printf("\033[4;0H\033[2K\rCan't allocate program buffer\n");
		fflush(stdout);
		return -1;
	}

	printf("\033[4;0H\033[2K\rprogramming %u bytes...\n", (unsigned)fw_size);
	fflush(stdout);

	ret = lseek(_fw_fd, 0, SEEK_SET);

	if (ret != OK) {
		printf("\033[4;0H\033[2K\rlseek fail\n");
		fflush(stdout);
		return ret;
	}

	while (sent < fw_size) {
		/* get more bytes to program */
		size_t n = fw_size - sent;
		if (n > PROG_MULTI_MAX) {
			n = PROG_MULTI_MAX;
		}
		count = read_with_retry(_fw_fd, file_buf, n);

		if (count != (ssize_t)n) {
			printf("\033[4;0H\033[2K\rfirmware read of %u bytes at %u failed -> %d errno %d\n", 
			    (unsigned)n,
			    (unsigned)sent,
			    (int)count,
			    (int)errno);
			fflush(stdout);
			ret = -errno;
			break;
		}

		sent += count;

		send(PROTO_PROG_MULTI);
		send(count);
		send_buf(file_buf, count);
		send(PROTO_EOC);

		ret = get_sync(1000);

		if (ret != OK) {
			break;
		}
		firmware_progress = (int)((float)sent/(float)fw_size*100.0);
		printf("\033[4;0H\033[2K\rprogram: %d \n", firmware_progress);
		fflush(stdout);
	}
	return ret;
}

int
verify_rev3(size_t fw_size_local)
{
	int ret;
	uint8_t	file_buf[4];
	ssize_t count;
	uint32_t sum = 0;
	uint32_t bytes_read = 0;
	uint32_t crc = 0;
	uint32_t fw_size_remote;
	uint8_t fill_blank = 0xff;

	firmware_step = 3;

	printf("\033[4;0H\033[2K\rverify...\n");
	fflush(stdout);
	ret = lseek(_fw_fd, 0, SEEK_SET);

	if (ret != OK) {
		printf("\033[4;0H\033[2K\rlseek fail\n");
		fflush(stdout);
		return ret;
	}

	ret = get_info(INFO_FLASH_SIZE, &fw_size_remote);
	send(PROTO_EOC);

	if (ret != OK) {
		printf("\033[4;0H\033[2K\rcould not read firmware size\n");
		fflush(stdout);
		return ret;
	}

	/* read through the firmware file again and calculate the checksum*/
	while (bytes_read < fw_size_local) {
		size_t n = fw_size_local - bytes_read;
		if (n > sizeof(file_buf)) {
			n = sizeof(file_buf);
		}
		count = read_with_retry(_fw_fd, file_buf, n);

		if (count != (ssize_t)n) {
			printf("\033[4;0H\033[2K\rfirmware read of %u bytes at %u failed -> %d errno %d\n", 
			    (unsigned)n,
			    (unsigned)bytes_read,
			    (int)count,
			    (int)errno);
			fflush(stdout);
		}

		/* set the rest to ff */
		if (count == 0) {
			break;
		}
		/* stop if the file cannot be read */
		if (count < 0)
			return -1;

		/* calculate crc32 sum */
		sum = crc32part((uint8_t *)&file_buf, sizeof(file_buf), sum);

		bytes_read += count;
		firmware_progress = (int)((float)bytes_read/(float)fw_size_local*100.0);
		printf("\033[4;0H\033[2K\rverify : %d \n", firmware_progress);
		fflush(stdout);
	}

	/* fill the rest with 0xff */
	while (bytes_read < fw_size_remote) {
		sum = crc32part(&fill_blank, sizeof(fill_blank), sum);
		bytes_read += sizeof(fill_blank);
	}

	/* request CRC from hifly */
	send(PROTO_GET_CRC);
	send(PROTO_EOC);

	ret = recv_bytes((uint8_t*)(&crc), sizeof(crc));

	if (ret != OK) {
		printf("\033[4;0H\033[2K\rdid not receive CRC checksum\n");
		fflush(stdout);
		return ret;
	}

	/* compare the CRC sum from the IO with the one calculated */
	if (sum != crc) {
		printf("\033[4;0H\033[2K\rCRC wrong: received: %d, expected: %d\n", crc, sum);
		fflush(stdout);
		return -1;
	}

	return OK;
}

int
reboot()
{
	send(PROTO_REBOOT);
	usleep(100*1000); // Ensure the farend is in wait for char.
	send(PROTO_EOC);

	return OK;
}

int
open_uart(const char *uart_name, int baud)
{
	int ret;
	int speed;
			
	switch (baud) {
	case 0:      speed = B0;      break;

	case 50:     speed = B50;     break;

	case 75:     speed = B75;     break;

	case 110:    speed = B110;    break;

	case 134:    speed = B134;    break;

	case 150:    speed = B150;    break;

	case 200:    speed = B200;    break;

	case 300:    speed = B300;    break;

	case 600:    speed = B600;    break;

	case 1200:   speed = B1200;   break;

	case 1800:   speed = B1800;   break;

	case 2400:   speed = B2400;   break;

	case 4800:   speed = B4800;   break;

	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	case 460800: speed = B460800; break;

	case 921600: speed = B921600; break;

	default:
		break;
	}
	
	/* open uart*/
	_boot_fd = open(uart_name, O_RDWR | O_NOCTTY);
	
	if(_boot_fd < 0)
	{
		printf("\033[4;0H\033[2K\rERR OPEN :%s\n", uart_name);
		fflush(stdout);
		return _boot_fd;
	}
	
	/* Try to set baud rate*/
	struct termios uart_config;
	int termios_state;

	/* Back up the original uart configuration to restore it after exit*/
	if((termios_state = tcgetattr(_boot_fd, &uart_config_original)) < 0)
	{
		printf("\033[4;0H\033[2K\rERR GET CONF %s: %d\n", uart_name, termios_state);
		fflush(stdout);
		close(_boot_fd);
		_boot_fd = -1;
		return -1;
	}	
	
	/* Fill the struct for the new configuration*/
	tcgetattr(_boot_fd, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF)*/
	uart_config.c_oflag &= ~ONLCR;
	uart_config.c_iflag &= ~ONLCR;
	/* close hardware flow control*/
	uart_config.c_cflag &= ~CRTSCTS;
	/*close mask and set 8 data bits*/
	uart_config.c_cflag &= ~CSIZE;
	uart_config.c_cflag |= CS8;
	/*set 1 stop bits*/
	uart_config.c_cflag &= ~CSTOPB;
	/*disable parity bit*/
	uart_config.c_cflag &= ~PARENB;
	
	/* Put in raw mode */
	cfmakeraw(&uart_config);

	/* set baud rate*/
	if(cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0)
	{
		printf("\033[4;0H\033[2K\rERR SET BAUD %s: %d\n", uart_name, termios_state);
		fflush(stdout);
		close(_boot_fd);
		_boot_fd = -1;
		return -1;
	}
	
	if((termios_state = tcsetattr(_boot_fd, TCSANOW, &uart_config)) < 0)
	{
		printf("\033[4;0H\033[2K\rERR SET CONF %s\n", uart_name);
		fflush(stdout);
		close(_boot_fd);
		return -1;
	}
	
	printf("\033[4;0H\033[2K\ropen %s successed\n", uart_name);
	fflush(stdout);

	return OK;
}


int
upload(const char *filename)
{
	int ret = -1;
	size_t fw_size;
	if (filename == NULL) {
		printf("\033[4;0H\033[2K\rno file name\n");
		fflush(stdout);
		goto upload_fail;
	}
	
	_fw_fd = open(filename, O_RDONLY);

	if (_fw_fd < 0) {
		printf("\033[4;0H\033[2K\rfailed to open %s\n", filename);
		fflush(stdout);
		goto upload_fail;
	}

	printf("\033[4;0H\033[2K\rusing firmware from %s\n", filename);
	fflush(stdout);

	int i;
	/* look for the bootloader*/
	for (i = 0; i < 15; i++) {
		ret = __sync();
		if (ret == OK) {
			break;
		} else {
			send_reboot();
			usleep(1000000);
		}
	}
	
	if (ret != OK) {
		/* this is immediately fatal */
		printf("\033[4;0H\033[2K\rbootloader not responding\n");
		fflush(stdout);
		goto upload_fail;
	}

	struct stat st;
	if (stat(filename, &st) != 0) {
		printf("\033[4;0H\033[2K\rFailed to stat %s - %d\n", filename, (int)errno);
		fflush(stdout);
		goto upload_fail;
	}
	fw_size = st.st_size;

	ret = get_info(INFO_BL_REV, &bl_rev);
	
	if (ret != OK) {
		goto upload_fail;
	}

	if (ret == OK) {
		printf("\033[4;0H\033[2K\rbl_rev\n");
		if (bl_rev <= BL_REV) {
			printf("\033[4;0H\033[2K\rfound bootloader revision: %d\n", bl_rev);
			fflush(stdout);
		} else {
			printf("\033[4;0H\033[2K\rfound unsupported bootloader revision %d, exiting\n", bl_rev);
			fflush(stdout);
			goto upload_fail;
		}
	}

	ret = erase();

	if (ret != OK) {
		printf("\033[4;0H\033[2K\rerase failed\n");
		fflush(stdout);
		goto upload_fail;
	}

	ret = program(fw_size);

	if (ret != OK) {
		printf("\033[4;0H\033[2K\rprogram failed\n");
		fflush(stdout);
		goto upload_fail;
	}

	ret = verify_rev3(fw_size);

	if (ret != OK) {
		printf("\033[4;0H\033[2K\rverify failed\n");
		fflush(stdout);
		goto upload_fail;
	}

	ret = reboot();

	if (ret != OK) {
		printf("\033[4;0H\033[2K\rreboot failed\n");
		fflush(stdout);
		goto upload_fail;
	}

	printf("\033[4;0H\033[2K\rupdate complete\n");
	fflush(stdout);
	
upload_fail:
	if(_fw_fd > 0) {
		close(_fw_fd);
		_fw_fd = -1;
	}

	err = ret;

	usleep(100*1000);

	return ret;
}

void *upgrade_start(void *arg)
{
	int ret;
	ret = open_uart(device, baud);
	if(ret == OK) {
		ret = upload(file);
	}
		
	tcsetattr(_boot_fd, TCSANOW, &uart_config_original);
	close(_boot_fd);
	_boot_fd = -1;
}

int autopilot_upgrade_start(const char *dev, const int baudrate, const char *filenames)
{
	int ret;
	pthread_t upgrade_thread;
	start = true;

	device = dev;
	baud = baudrate;
	file = filenames;

	ret = pthread_create(&upgrade_thread, NULL, upgrade_start, NULL);
	
	pthread_detach(upgrade_thread);

	return ret;
}

int get_upgrade_progress(bool *complete, int *step, int *progress)
{
	if (start != true)
	{
		printf("can't start upgrade, please try ""start""");
		return -1;
	}
	*step = firmware_step;
	*progress = firmware_progress;
	if((firmware_step == VERIFY) && (firmware_progress == 100))
	{
		*complete = true;
	} else {
		*complete = false;
	}

	return err;
}

