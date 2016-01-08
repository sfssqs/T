/*
 * vtterminal.h
 *
 *  Created on: 2015年12月10日
 *      Author: summer
 */

#ifndef VTTERMINAL_H_
#define VTTERMINAL_H_

#ifndef AF_CAN
#define AF_CAN 29
#endif
#ifndef PF_CAN
#define PF_CAN AF_CAN
#endif

#define DEV_NAME "/dev/ttymxc1"
#define CAN_DEV_NAME "can0"

#define myerr(str)	fprintf(stderr, "%s, %s, %d: %s\n", __FILE__, __func__, __LINE__, str)
#define errout(_s)	fprintf(stderr, "error class: %s\n", (_s))
#define errcode(_d) fprintf(stderr, "error code: %02x\n", (_d))

typedef void (*callback)(int fb, char*);

void *ble_serial_read_data(void* fd);
void *can0_read_data( void* fd);
void *startUDPServer(void* fd);

void ble_serial_write_data(int fd, char* buf);
void can0_write_data(int fd, unsigned char cmd);
void net_udp_register();
void net_udp_read_data( int fd);
void net_udp_write_data(int fd, char* cmd, int len);

void set_stopbit(struct termios *opt, const char *stopbit);
void set_parity(struct termios *opt, char parity);
void set_data_bit(struct termios *opt, unsigned int databit);
void set_baudrate(struct termios *opt, unsigned int baudrate);
int set_port_attr(int fd, int baudrate, int databit, const char *stopbit,
		char parity, int vtime, int vmin);

void handle_err_frame(const struct can_frame *fr);
void handle_read_frame(struct can_frame *fr);
int set_can_filter(int fd);

void can0_release(int fd);
void ble_serial_release(int fd);
void net_udp_release(int fd);

#endif /* VTTERMINAL_H_ */
