#include <stdio.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <linux/socket.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <termios.h>
#include <pthread.h>

#include "vtterminal.h"

int main(void) {
	int rc;
	int fd_can0;
	int fd_ble_serial;

	pthread_t thread_can0;
	pthread_t thread_ble_serial;

	// open devices
	fd_can0 = can0_init();
	if (fd_can0 < 0)
		puts("can0 open failed\n");

	fd_ble_serial = ble_serial_init();
	if (fd_ble_serial < 0)
		puts("ble_serial open failed\n");

	// data read thread
	rc = pthread_create(&thread_can0, NULL, can0_read_data, &fd_can0);
	if (rc)
		puts("can0 thread create failed!");

	rc = pthread_create(&thread_ble_serial, NULL, ble_serial_read_data,
			&fd_ble_serial);
	if (rc)
		puts("ble_serial thread create failed!");

	// thread join
	pthread_join(thread_can0, NULL);
	pthread_join(thread_ble_serial, NULL);

	puts("in main");

	// release
	can0_release(fd_can0);
	ble_serial_release(fd_ble_serial);

	puts("Device closed, ble_serial & can0");

	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////
/// can0

int can0_init() {
	int fd;
	int ret;
	struct sockaddr_can addr;
	struct ifreq ifr;

	srand(time(NULL));
	fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (fd < 0) {
		perror("socket PF_CAN failed");
		return -1;
	}

	strcpy(ifr.ifr_name, CAN_DEV_NAME);
	ret = ioctl(fd, SIOCGIFINDEX, &ifr);
	if (ret < 0) {
		perror("ioctl failed");
		return -1;
	}

	addr.can_family = PF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	ret = bind(fd, (struct sockaddr *) &addr, sizeof(addr));
	if (ret < 0) {
		perror("bind failed");
		return -1;
	}

	ret = set_can_filter(fd);
	if (ret < 0) {
		perror("setsockopt failed");
		return -1;
	}

	printf("Device opened : socket can0,  fd : %d\n", fd);
	puts("Device attribute : default");

	return fd;
}

void *can0_read_data(void * _fd) {
	int fd;
	int ret, i;
	struct can_frame fr, frdup;
	struct timeval tv;
	fd_set rset;

	fd = *(int*) _fd;

	while (1) {
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		FD_ZERO(&rset);
		FD_SET(fd, &rset);

		printf("======================\n");

		ret = read(fd, &frdup, sizeof(frdup));
		if (ret < sizeof(frdup)) {
			myerr("read failed");
			return NULL;
		}

		if (frdup.can_id & CAN_ERR_FLAG) {
			handle_err_frame(&frdup);
			myerr("CAN device error");
			continue;
		}

		handle_read_frame(&frdup);

		sleep(4);
	}
}

void can0_write_data(int fd) {
	int ret, i;
	struct can_frame fr, frdup;
	struct timeval tv;
	fd_set rset;

	tv.tv_sec = 1;
	tv.tv_usec = 0;
	FD_ZERO(&rset);
	FD_SET(fd, &rset);

	frdup.can_id = 0x1fffffff;
	frdup.can_id &= CAN_EFF_MASK;
	frdup.can_id |= CAN_EFF_FLAG;
	frdup.can_dlc = 8;

	printf("send data = ");
	int j = 0xDD;
	for (i = 0; i < frdup.can_dlc; i++) {
		frdup.data[i] = j--;
		printf("%02x ", frdup.data[i]);
	}

	ret = write(fd, &frdup, sizeof(frdup));
	if (ret < 0) {
		myerr("write failed");
		return;
	}

	return;
}

void can0_release(int fd) {
	close(fd);
}

int set_can_filter(int fd) {
	int ret;

	struct can_filter filter[3];
	filter[0].can_id = 0x200 | CAN_EFF_FLAG;
	filter[0].can_mask = 0xFFF;

	filter[1].can_id = 0x20F | CAN_EFF_FLAG;
	filter[1].can_mask = 0xFFF;

	filter[2].can_id = 0x1F0 | CAN_EFF_FLAG;
	filter[2].can_mask = 0xFFF;

	ret = setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));

	return ret;
}

void handle_read_frame(struct can_frame *fr) {
	int i;
	printf("%08x\n", fr->can_id & CAN_EFF_MASK);
	//printf("%08x\n", fr->can_id);
	printf("dlc = %d\n", fr->can_dlc);
	printf("data = ");
	for (i = 0; i < fr->can_dlc; i++)
		printf("%02x ", fr->data[i]);
	printf("\n");
}

void handle_err_frame(const struct can_frame *fr) {
	if (fr->can_id & CAN_ERR_TX_TIMEOUT) {
		errout("CAN_ERR_TX_TIMEOUT");
	}
	if (fr->can_id & CAN_ERR_LOSTARB) {
		errout("CAN_ERR_LOSTARB");
		errcode(fr->data[0]);
	}
	if (fr->can_id & CAN_ERR_CRTL) {
		errout("CAN_ERR_CRTL");
		errcode(fr->data[1]);
	}
	if (fr->can_id & CAN_ERR_PROT) {
		errout("CAN_ERR_PROT");
		errcode(fr->data[2]);
		errcode(fr->data[3]);
	}
	if (fr->can_id & CAN_ERR_TRX) {
		errout("CAN_ERR_TRX");
		errcode(fr->data[4]);
	}
	if (fr->can_id & CAN_ERR_ACK) {
		errout("CAN_ERR_ACK");
	}
	if (fr->can_id & CAN_ERR_BUSOFF) {
		errout("CAN_ERR_BUSOFF");
	}
	if (fr->can_id & CAN_ERR_BUSERROR) {
		errout("CAN_ERR_BUSERROR");
	}
	if (fr->can_id & CAN_ERR_RESTARTED) {
		errout("CAN_ERR_RESTARTED");
	}
}

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

///// ble serial
int ble_serial_init() {
	int fd;
	int len, ret;

	fd = open(DEV_NAME, O_RDWR | O_NOCTTY/* | O_NDELAY*/);
	if (fd < 0) {
		perror("Open device error\n");
	}
	printf("Device opened : %s\n", DEV_NAME);

	ret = set_port_attr(fd, B9600, 8, "1", 'N', 0, 0);
	if (ret < 0) {
		printf("Set attribute failed \n");
		exit(-1);
	}
	puts("Device attribute : (B9600 8n1)  (vtime 0)  (vmin 0)\n");

	return fd;
}

void *ble_serial_read_data(void*_fd) {
	char buf[8];
	int len;
	int fd;

	fd = *(int*) _fd;

	while (1) {
		memset(buf, 0, sizeof(buf));
		len = read(fd, buf, sizeof(buf));
		if (len < 0) {
			printf("Read data error!\n");
			return NULL;
		}

		printf("++++++++++++++++++++++++++++++++++++++\n");

		printf("Read length : %d \n", len);
		printf("Read data : %s \n", buf);

		ble_serial_write_data(fd, buf);
	}
}

void ble_serial_write_data(int fd, char* buf) {
	int len;

	len = write(fd, buf, sizeof(buf));
	if (len < 0) {
		printf("BT serial write data error!\n");
		return;
	}
	printf("BT serial output data : %s \n", buf);
	return;
}

void ble_serial_release(int fd) {
	close(fd);
}

int set_port_attr(int fd, int baudrate, int databit, const char *stopbit,
		char parity, int vtime, int vmin) {
	struct termios opt;

	tcgetattr(fd, &opt);

	set_baudrate(&opt, baudrate);

	// enable the receiver and set local mode ...
	opt.c_cflag |= CLOCAL | CREAD;

	// (our slave device may not support)
	// Hardware flow control off
	opt.c_cflag &= ~CRTSCTS;
	// Hardware flow control on
	//  opt.c_cflag |= CRTSCTS;

	/* | CRTSCTS */
	set_data_bit(&opt, databit);
	set_parity(&opt, parity);
	set_stopbit(&opt, stopbit);

	// Original, no need CR
	// opt.c_cflag |= ~(ICANON | ECHO | ECHOE | ISIG);
	// Classical, designed to line, need CR or LF
	// opt.c_cflag |= (ICANON | ECHO | ECHOE);

	opt.c_oflag = 0;
	opt.c_lflag |= 0;

	// Raw output
	opt.c_oflag &= ~OPOST;
	// Classical output
	// opt.c_oflag |= OPOST;

	opt.c_cc[VTIME] = vtime;
	opt.c_cc[VMIN] = vmin;

	tcflush(fd, TCIFLUSH);

	return (tcsetattr(fd, TCSANOW, &opt));
}

void set_baudrate(struct termios *opt, unsigned int baudrate) {
	cfsetispeed(opt, baudrate);
	cfsetospeed(opt, baudrate);
}

void set_data_bit(struct termios *opt, unsigned int databit) {
	opt->c_cflag &= ~CSIZE;
	switch (databit) {
	case 8:
		opt->c_cflag |= CS8;
		break;
	case 7:
		opt->c_cflag |= CS7;
		break;
	case 6:
		opt->c_cflag |= CS6;
		break;
	case 5:
		opt->c_cflag |= CS5;
		break;
	default:
		opt->c_cflag |= CS8;
		break;
	}
}

void set_parity(struct termios *opt, char parity) {
	switch (parity) {
	case 'N':
		/* 无校验 */
		/* 偶校验 */
		/* 奇校验 */
	case 'n':
		opt->c_cflag &= ~PARENB;
		break;
	case 'E':
	case 'e':
		opt->c_cflag |= PARENB;
		opt->c_cflag &= ~PARODD;
		break;
	case 'O':
	case 'o':
		opt->c_cflag |= PARENB;
		opt->c_cflag |= ~PARODD;
		break;
		/* 其它选择为无校验 */
	default:
		opt->c_cflag &= ~PARENB;
		break;
	}
}

void set_stopbit(struct termios *opt, const char *stopbit) {
	if (0 == strcmp(stopbit, "1")) {
		opt->c_cflag &= ~CSTOPB;
		/* 1 位停止位 t */
		/* 1.5 位停止位 */
		/* 2 位停止位 */
		/* 1 位停止位 */
	} else if (0 == strcmp(stopbit, "1.5")) {
		opt->c_cflag &= ~CSTOPB;
	} else if (0 == strcmp(stopbit, "2")) {
		opt->c_cflag |= CSTOPB;
	} else {
		opt->c_cflag &= ~CSTOPB;
	}
}
