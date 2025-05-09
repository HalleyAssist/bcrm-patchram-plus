// TODO: Integrate BCM support into Bluez hciattach

#define _GNU_SOURCE         /* See feature_test_macros(7) */
#include <stdio.h>
#include <getopt.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <stdlib.h>

#include <sys/termios.h>
#include <sys/ioctl.h>
#include <limits.h>

#include <unistd.h>

#include <stdnoreturn.h>
#include <string.h>
#include <signal.h>
#include <stdbool.h>



#define HCIUARTSETPROTO		_IOW('U', 200, int)
#define HCIUARTGETPROTO		_IOR('U', 201, int)
#define HCIUARTGETDEVICE	_IOR('U', 202, int)
#define HCIUARTSETFLAGS		_IOW('U', 203, int)
#define HCIUARTGETFLAGS		_IOR('U', 204, int)

#define HCI_UART_H4	0
#define HCI_UART_BCSP	1
#define HCI_UART_3WIRE	2
#define HCI_UART_H4DS	3
#define HCI_UART_LL	4
#define HCI_UART_ATH3K  5
#define HCI_UART_INTEL	6
#define HCI_UART_BCM	7
#define HCI_UART_QCA	8
#define HCI_UART_AG6XX	9
#define HCI_UART_NOKIA	10
#define HCI_UART_MRVL	11

typedef unsigned char uchar;

int uart_fd = -1;
int hcdfile_fd = -1;
int termios_baudrate = 0;
int bdaddr_flag = 0;
int debug = 0;
int tosleep = 0;

struct termios termios;
uchar buffer[512];
uchar buffer2[512];

uchar hci_download_minidriver[] = { 0x01, 0x2e, 0xfc, 0x00 };

uchar hci_update_baud_rate[] = { 0x01, 0x18, 0xfc, 0x06, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00 };

int
parse_patchram(char *optarg)
{
	char *p;

	if (!(p = strrchr(optarg, '.'))) {
		fprintf(stderr, "file %s not an HCD file\n", optarg);
		exit(3);
	}

	p++;

	if (strcasecmp("hcd", p) != 0) {
		fprintf(stderr, "file %s not an HCD file\n", optarg);
		exit(4);
	}

	if ((hcdfile_fd = open(optarg, O_RDONLY)) == -1) {
		fprintf(stderr, "file %s could not be opened, error %d\n", optarg, errno);
		exit(5);
	}

	return(0);
}

void
BRCM_encode_baud_rate(uint baud_rate, uchar *encoded_baud)
{
	if(baud_rate == 0 || encoded_baud == NULL) {
		fprintf(stderr, "Baudrate not supported!");
		return;
	}

	encoded_baud[0] = (uchar)(baud_rate & 0xFF);
	encoded_baud[1] = (uchar)(baud_rate >> 8);
	encoded_baud[2] = (uchar)(baud_rate >> 16);
	encoded_baud[3] = (uchar)(baud_rate >> 24);
}

typedef struct {
	int baud_rate;
	int termios_value;
} tBaudRates;

int
validate_baudrate(int baud_rate, int *value)
{
	const tBaudRates baud_rates [] = {
		{ 115200, B115200 },
		{ 230400, B230400 },
		{ 460800, B460800 },
		{ 500000, B500000 },
		{ 576000, B576000 },
		{ 921600, B921600 },
		{ 1000000, B1000000 },
		{ 1152000, B1152000 },
		{ 1500000, B1500000 },
		{ 2000000, B2000000 },
		{ 2500000, B2500000 },
		{ 3000000, B3000000 },
	#ifndef __CYGWIN__
		{ 3500000, B3500000 },
		{ 4000000, B4000000 }
	#endif
	};


	unsigned int i;

	for (i = 0; i < (sizeof(baud_rates) / sizeof(tBaudRates)); i++) {
		if (baud_rates[i].baud_rate == baud_rate) {
			*value = baud_rates[i].termios_value;
			return(1);
		}
	}

	return(0);
}

int
parse_debug(char *optarg)
{
	debug = atoi(optarg);

	return(0);
}

int
parse_baudrate(char *optarg)
{
	int baudrate = atoi(optarg);
	fprintf(stderr, "Baudrate %d\n", baudrate);

	if (!validate_baudrate(baudrate, &termios_baudrate)) {
		fprintf(stderr, "Baudrate %d not supported!\n", baudrate);
		exit(1);
	}
	BRCM_encode_baud_rate(baudrate, &hci_update_baud_rate[6]);

	return(0);
}

int
parse_enable_hci(char *optarg)
{
	enable_hci = 1;
	return(0);
}

int
parse_tosleep(char *optarg)
{
	tosleep = atoi(optarg);

	if (tosleep <= 0) {
		return(1);
	}

	return(0);
}

int
parse_cmd_line(int argc, char **argv)
{
	int c;
	int ret = 0;

	typedef int (*PFI)();

	PFI parse[] = { parse_patchram, parse_debug, parse_baudrate, parse_enable_hci, parse_tosleep};

	while (1) {
		int this_option_optind = optind ? optind : 1;
		int option_index = 0;

		static struct option long_options[] = {
			{"patchram", 1, 0, 0},
			{"debug", 1, 0, 0},
			{"baudrate", 1, 0, 0},
			{"enable_hci", 0, 0, 0},
			{"tosleep", 1, 0, 0},
			{0, 0, 0, 0}
		};

		c = getopt_long_only (argc, argv, "d", long_options,
				&option_index);

		if (c == -1) {
			break;
		}

		switch (c) {
			case 0:
				if (debug) {
					printf ("option %s",
						long_options[option_index].name);
					if (optarg)
						printf (" with arg %s", optarg);
					printf ("\n");
				}

				ret = (*parse[option_index])(optarg);

				break;
			case 'd':
				debug = 1;
				break;

		}
	}

	if (ret) {
		return(1);
	}

	if (optind < argc) {
		if (debug)
			printf ("%s \n", argv[optind]);
		if ((uart_fd = open(argv[optind], O_RDWR | O_NOCTTY)) == -1) {
			fprintf(stderr, "port %s could not be opened, error %d\n",
					argv[2], errno);
		}
	}

	return(0);
}

void
init_uart()
{
	int ld;

	tcflush(uart_fd, TCIOFLUSH);
	tcgetattr(uart_fd, &termios);

	cfmakeraw(&termios);

	termios.c_cflag |= CRTSCTS | CLOCAL;
	tcsetattr(uart_fd, TCSANOW, &termios);
	
	cfsetospeed(&termios, B115200);
	cfsetispeed(&termios, B115200);
	tcsetattr(uart_fd, TCSANOW, &termios);

	tcflush(uart_fd, TCIOFLUSH);

	ld = N_TTY;
	if (ioctl(uart_fd, TIOCSETD, &ld) < 0) {
		perror("Can't restore line discipline");
		exit(1);
	}
}

const char format_strings [][50] = {
	"%02x\n", // 1
	"%02x %02x\n", // 2
	"%02x %02x %02x\n", // 3
	"%02x %02x %02x %02x\n", // 4
	"%02x %02x %02x %02x %02x\n", // 5
	"%02x %02x %02x %02x %02x %02x\n", // 6
	"%02x %02x %02x %02x %02x %02x %02x\n", // 7
	"%02x %02x %02x %02x %02x %02x %02x %02x\n", // 8	
};

void
dump(const uchar *out, int len)
{
	int i, f;
	const char *fmt;

	if(debug <= 1) return;

	for (i = 0; i < len; i+=8) {
		f = len - i;
		if (f > 8) {
			f = 8;
		}
		fmt = format_strings[f - 1];

		fprintf(stderr, fmt, out[i], out[i + 1], out[i + 2],
			out[i + 3], out[i + 4], out[i + 5],
			out[i + 6], out[i + 7]);
	}
}

void
read_event(int fd, uchar *b)
{
	int i = 0;
	int len = 3;
	int count;

	// read first 3 bytes of event (HCI_EVENT_PKT, EVENT_CODE, LEN)
	while ((count = read(fd, &b[i], len)) < len) {
		i += count;
		len -= count;
	}

	i += count;
	len = b[2];

	while ((count = read(fd, &b[i], len)) < len) {
		i += count;
		len -= count;
	}

	if (debug) {
		count += i;

		fprintf(stderr, "received %d\n", count);
		dump(b, count);
	}
}

void
hci_send_cmd(const uchar *buf, int len)
{
	if (debug) {
		fprintf(stderr, "writing\n");
		dump(buf, len);
	}

	write(uart_fd, buf, len);
}

void
expired(int sig)
{
	const uchar hci_reset[] = { 0x01, 0x03, 0x0c, 0x00 };

	hci_send_cmd(hci_reset, sizeof(hci_reset));
	alarm(4);
}

void
proc_reset()
{
	signal(SIGALRM, expired);

	expired(0);

	read_event(uart_fd, buffer);

	alarm(0);
}

static bool hci_is_event(uchar *buf)
{
	if (buf[0] != 0x04 || buf[1] != 0x0e) { // EVT_CMD_COMPLETE = 0x0e
		return false;
	}

	return true;
}

void
expired_exit(int sig)
{
	fprintf(stderr, "Error: Timeout waiting for event\n");
	exit(1);
}

void
proc_patchram()
{
	int len;

	signal(SIGALRM, expired_exit);

	alarm(4);
	hci_send_cmd(hci_download_minidriver, sizeof(hci_download_minidriver));

	read_event(uart_fd, buffer2);
	alarm(0);

	if(!hci_is_event(buffer2)) {
		fprintf(stderr, "Error: No event received to hci_download_minidriver\n");
		exit(1);
	}

	if(buffer2[6] != 0x00) {
		fprintf(stderr, "Error: hci_download_minidriver failed\n");
		exit(1);
	}

	if (tosleep) {
		usleep(tosleep);
	}

	while (read(hcdfile_fd, &buffer[1], 3)) {
		buffer[0] = 0x01;

		len = buffer[3];
	
		if(len > sizeof(buffer) - 4) {
			fprintf(stderr, "Error: CMD length %d is too long\n", len);
			exit(1);
		}

		read(hcdfile_fd, &buffer[4], len);

		alarm(4);
		hci_send_cmd(buffer, len + 4);

		read_event(uart_fd, buffer2);

		if(!hci_is_event(buffer2)) {
			fprintf(stderr, "Error: No event received to hcdfile_fd (retry recv)\n");
			read_event(uart_fd, buffer2);
			
			if(!hci_is_event(buffer2)) {
				fprintf(stderr, "Error: No event received to hcdfile_fd\n");
				exit(1);
			}
		}

		alarm(0);

		if(buffer2[6] != 0x00) {
			fprintf(stderr, "Error: hcdfile_fd failed\n");
			exit(1);
		}
	}

	proc_reset();
}

void
proc_baudrate()
{
	tcflush(uart_fd, TCIOFLUSH);
	
	hci_send_cmd(hci_update_baud_rate, sizeof(hci_update_baud_rate));

	read_event(uart_fd, buffer);

	if(!hci_is_event(buffer)) {
		fprintf(stderr, "Error: No event received to hci_update_baud_rate\n");
		exit(1);
	}

	if(buffer[6] != 0x00) {
		fprintf(stderr, "Error: hci_update_baud_rate failed\n");
		exit(1);
	}

	cfsetospeed(&termios, termios_baudrate);
	cfsetispeed(&termios, termios_baudrate);
	tcsetattr(uart_fd, TCSANOW, &termios);

	if (debug) {
		fprintf(stderr, "Done setting baudrate\n");
	}
}

noreturn void
proc_enable_hci()
{
	int i = N_HCI;
	
	if (ioctl(uart_fd, TIOCSETD, &i) < 0) {
		fprintf(stderr, "Can't set line discipline\n");
		return;
	}

	i = HCI_UART_H4;
	if (ioctl(uart_fd, HCIUARTSETPROTO, i) < 0) {
		fprintf(stderr, "Can't set hci protocol\n");
		return;
	}
	fprintf(stderr, "Done setting line discpline\n");
		
	while (1) {
		sleep(UINT_MAX);
	}
}


int
main (int argc, char **argv)
{
	int err;

	if (parse_cmd_line(argc, argv)) {
		exit(1);
	}

	if (uart_fd < 0) {
		exit(2);
	}

	init_uart();

	proc_reset();

	if (hcdfile_fd > 0) {
		proc_patchram();
		close(hcdfile_fd);
	}

	proc_reset();

	proc_baudrate();

	proc_enable_hci();

	exit(0);
}
