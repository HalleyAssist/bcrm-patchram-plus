/*******************************************************************************
 *
 *  Copyright (C) 2009-2011 Broadcom Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/*****************************************************************************
**                                                                           
**  Name:          brcm_patchram_plus.c
**
**  Description:   This program downloads a patchram files in the HCD format
**                 to Broadcom Bluetooth based silicon and combo chips and
**				   and other utility functions.
**
**                 It can be invoked from the command line in the form
**						<-d> to print a debug log
**						<--patchram patchram_file>
**						<--baudrate baud_rate>
**						<--bd_addr bd_address>
**						<--enable_lpm>
**						<--enable_hci>
**						<--use_baudrate_for_download>
**						<--scopcm=sco_routing,pcm_interface_rate,frame_type,
**							sync_mode,clock_mode,lsb_first,fill_bits,
**							fill_method,fill_num,right_justify>
**
**							Where
**
**							sco_routing is 0 for PCM, 1 for Transport,
**							2 for Codec and 3 for I2S,
**
**							pcm_interface_rate is 0 for 128KBps, 1 for
**							256 KBps, 2 for 512KBps, 3 for 1024KBps,
**							and 4 for 2048Kbps,
**
**							frame_type is 0 for short and 1 for long,
**
**							sync_mode is 0 for slave and 1 for master,
**
**							clock_mode is 0 for slabe and 1 for master,
**
**							lsb_first is 0 for false aand 1 for true,
**
**							fill_bits is the value in decimal for unused bits,
**
**							fill_method is 0 for 0's and 1 for 1's, 2 for
**								signed and 3 for programmable,
**
**							fill_num is the number or bits to fill,
**
**							right_justify is 0 for false and 1 for true
**
**						<--i2s=i2s_enable,is_master,sample_rate,clock_rate>
**
**							Where
**
**							i2s_enable is 0 for disable and 1 for enable,
**
**							is_master is 0 for slave and 1 for master,
**
**							sample_rate is 0 for 8KHz, 1 for 16Khz and
**								2 for 4 KHz,
**
**							clock_rate is 0 for 128KHz, 1 for 256KHz, 3 for
**								1024 KHz and 4 for 2048 KHz.
**
**						<--no2bytes skips waiting for two byte confirmation
**							before starting patchram download. Newer chips
**                          do not generate these two bytes.>
**						<--tosleep=number of microsseconds to sleep before
**							patchram download begins.>
**						uart_device_name
**
**                 For example:
**
**                 brcm_patchram_plus -d --patchram  \
**						BCM2045B2_002.002.011.0348.0349.hcd /dev/ttyHS0
**
**                 It will return 0 for success and a number greater than 0
**                 for any errors.
**
**                 For Android, this program invoked using a 
**                 "system(2)" call from the beginning of the bt_enable
**                 function inside the file 
**                 system/bluetooth/bluedroid/bluetooth.c.
**
**                 If the Android system property "ro.bt.bcm_bdaddr_path" is
**                 set, then the bd_addr will be read from this path.
**                 This is overridden by --bd_addr on the command line.
**  
******************************************************************************/

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

#include <string.h>
#include <signal.h>
#include <stdbool.h>
#include <poll.h>



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
int enable_hci = 0;
int use_baudrate_for_download = 0;
int debug = 0;
int scopcm = 0;
int no2bytes = 0;
int tosleep = 0;

struct termios termios;
uchar buffer[512];
uchar buffer2[512];

uchar hci_download_minidriver[] = { 0x01, 0x2e, 0xfc, 0x00 };

uchar hci_update_baud_rate[] = { 0x01, 0x18, 0xfc, 0x06, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00 };

uchar hci_write_bd_addr[] = { 0x01, 0x01, 0xfc, 0x06,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

uchar hci_write_sco_pcm_int[] =
	{ 0x01, 0x1C, 0xFC, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00 };

uchar hci_write_pcm_data_format[] =
	{ 0x01, 0x1e, 0xFC, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00 };

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
parse_bdaddr(char *optarg)
{
	int bd_addr[6];
	int i;

	sscanf(optarg, "%02X:%02X:%02X:%02X:%02X:%02X",
		&bd_addr[5], &bd_addr[4], &bd_addr[3],
		&bd_addr[2], &bd_addr[1], &bd_addr[0]);

	for (i = 0; i < 6; i++) {
		hci_write_bd_addr[4 + i] = bd_addr[i];
	}

	bdaddr_flag = 1;

	return(0);
}

int
parse_use_baudrate_for_download(char *optarg)
{
	use_baudrate_for_download = 1;
	return(0);
}

int
parse_enable_hci(char *optarg)
{
	enable_hci = 1;
	return(0);
}

int
parse_scopcm(char *optarg)
{
	int param[10];
	int ret;
	int i;

	ret = sscanf(optarg, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
		&param[0], &param[1], &param[2], &param[3], &param[4],
		&param[5], &param[6], &param[7], &param[8], &param[9]);

	if (ret != 10) {
		return(1);
	}

	scopcm = 1;

	for (i = 0; i < 5; i++) {
		hci_write_sco_pcm_int[4 + i] = param[i];
	}

	for (i = 0; i < 5; i++) {
		hci_write_pcm_data_format[4 + i] = param[5 + i];
	}

	return(0);
}

int
parse_no2bytes(char *optarg)
{
	no2bytes = 1;
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

void
usage(char *argv0)
{
	printf("Usage %s:\n", argv0);
	printf("\t<-d>/<--debug n> to print a debug log\n");
	printf("\t<--patchram patchram_file>\n");
	printf("\t<--baudrate baud_rate>\n");
	printf("\t<--bd_addr bd_address>\n");
	printf("\t<--enable_hci>\n");
	printf("\t<--use_baudrate_for_download> - Uses the\n");
	printf("\t\tbaudrate for downloading the firmware\n");
	printf("\t<--scopcm=sco_routing,pcm_interface_rate,frame_type,\n");
	printf("\t\tsync_mode,clock_mode,lsb_first,fill_bits,\n");
	printf("\t\tfill_method,fill_num,right_justify>\n");
	printf("\n\t\tWhere\n");
	printf("\n\t\tsco_routing is 0 for PCM, 1 for Transport,\n");
	printf("\t\t2 for Codec and 3 for I2S,\n");
	printf("\n\t\tpcm_interface_rate is 0 for 128KBps, 1 for\n");
	printf("\t\t256 KBps, 2 for 512KBps, 3 for 1024KBps,\n");
	printf("\t\tand 4 for 2048Kbps,\n");
	printf("\n\t\tframe_type is 0 for short and 1 for long,\n");
	printf("\t\tsync_mode is 0 for slave and 1 for master,\n");
	printf("\n\t\tclock_mode is 0 for slabe and 1 for master,\n");
	printf("\n\t\tlsb_first is 0 for false aand 1 for true,\n");
	printf("\n\t\tfill_bits is the value in decimal for unused bits,\n");
	printf("\n\t\tfill_method is 0 for 0's and 1 for 1's, 2 for\n");
	printf("\t\tsigned and 3 for programmable,\n");
	printf("\n\t\tfill_num is the number or bits to fill,\n");
	printf("\n\t\tright_justify is 0 for false and 1 for true\n");
	printf("\n\t<--i2s=i2s_enable,is_master,sample_rate,clock_rate>\n");
	printf("\n\t\tWhere\n");
	printf("\n\t\ti2s_enable is 0 for disable and 1 for enable,\n");
	printf("\n\t\tis_master is 0 for slave and 1 for master,\n");
	printf("\n\t\tsample_rate is 0 for 8KHz, 1 for 16Khz and\n");
	printf("\t\t2 for 4 KHz,\n");
	printf("\n\t\tclock_rate is 0 for 128KHz, 1 for 256KHz, 3 for\n");
	printf("\t\t1024 KHz and 4 for 2048 KHz.\n\n");
	printf("\t<--no2bytes skips waiting for two byte confirmation\n");
	printf("\t\tbefore starting patchram download. Newer chips\n");
	printf("\t\tdo not generate these two bytes.>\n");
	printf("\t<--tosleep=microseconds>\n");
	printf("\tuart_device_name\n");
}

int
parse_cmd_line(int argc, char **argv)
{
	int c;
	int ret = 0;

	typedef int (*PFI)();

	PFI parse[] = { parse_patchram, parse_debug, parse_baudrate,
		parse_bdaddr, parse_enable_hci,
		parse_use_baudrate_for_download,
		parse_scopcm, parse_no2bytes, parse_tosleep};

	while (1) {
		int this_option_optind = optind ? optind : 1;
		int option_index = 0;

		static struct option long_options[] = {
			{"patchram", 1, 0, 0},
			{"debug", 1, 0, 0},
			{"baudrate", 1, 0, 0},
			{"bd_addr", 1, 0, 0},
			{"enable_hci", 0, 0, 0},
			{"use_baudrate_for_download", 0, 0, 0},
			{"scopcm", 1, 0, 0},
			{"no2bytes", 0, 0, 0},
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

			case '?':
				//nobreak
			default:
				usage(argv[0]);
				break;
		}

		if (ret) {
			usage(argv[0]);
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

	if (!no2bytes) {
		read(uart_fd, &buffer[0], 2);
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

	if (use_baudrate_for_download) {
		cfsetospeed(&termios, B115200);
		cfsetispeed(&termios, B115200);
		tcsetattr(uart_fd, TCSANOW, &termios);
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

void
proc_bdaddr()
{
	hci_send_cmd(hci_write_bd_addr, sizeof(hci_write_bd_addr));

	read_event(uart_fd, buffer);
}

void
proc_scopcm()
{
	hci_send_cmd(hci_write_sco_pcm_int,
		sizeof(hci_write_sco_pcm_int));

	read_event(uart_fd, buffer);

	hci_send_cmd(hci_write_pcm_data_format,
		sizeof(hci_write_pcm_data_format));

	read_event(uart_fd, buffer);
}

void
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
	return;
}


int
main (int argc, char **argv)
{
	struct pollfd p;
	int err;
	sigset_t sigs;

	if (parse_cmd_line(argc, argv)) {
		exit(1);
	}

	if (uart_fd < 0) {
		exit(2);
	}

	init_uart();

	proc_reset();

	if (use_baudrate_for_download) {
		if (termios_baudrate) {
			proc_baudrate();
		}
	}

	if (hcdfile_fd > 0) {
		proc_patchram();
	}

	if (bdaddr_flag) {
		proc_bdaddr();
	}

	if (scopcm) {
		proc_scopcm();
	}

	if (enable_hci) {
		proc_reset();

		proc_baudrate();

		proc_enable_hci();

		p.fd = uart_fd;
		p.events = POLLERR | POLLHUP;

		sigfillset(&sigs);
		sigdelset(&sigs, SIGCHLD);
		sigdelset(&sigs, SIGPIPE);
		sigdelset(&sigs, SIGTERM);
		sigdelset(&sigs, SIGINT);
		sigdelset(&sigs, SIGHUP);

		// poll for events
		while (1) {
			p.revents = 0;
			err = ppoll(&p, 1, NULL, &sigs);
			if (err < 0 && errno == EINTR)
				continue;
			if (err)
				break;
		}

	} else {
		if (termios_baudrate) {
			proc_baudrate();
		}
	}

	exit(0);
}
