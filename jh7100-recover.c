/*
 * This simple tool developed for the StarFive JH7100 SoC. The BootROM
 * XMODEM protocol implimentation has a bug that prevents standard
 * lrzsz tools to send files over XMODEM.
 *
 * Copyright (c) 2021 Kali Prasad <kprasadvnsi@protonmail.com>
 *
 * License: MIT
 */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>


//#define DEBUG

#ifdef DEBUG
#define debug(format, ...) printf("Debug:" format "\n", ##__VA_ARGS__);
#else
#define debug(format, ...)	;
#endif

#define SOH 0x01
#define ACK 0x06
#define NAK 0x15 
#define EOT 0x04

#define min(a, b)   ((a) < (b) ? (a) : (b))

#define PBSTR "########################################"
#define PBWIDTH 40

#define DEBUG_BAUD 9600

#define XMODEM_PAYLOAD_LEN	128	/* the length of xmodem packet payload*/
#define BUFF_SIZE		128	/* the length of buffer*/

struct xmodem_packet {
	uint8_t start;
	uint8_t block;
	uint8_t block_neg;
	uint8_t payload[XMODEM_PAYLOAD_LEN];
	uint16_t crc;
} __attribute__((packed));

static const char bootrom_str[] = "(C)SiFive\r\n";
static const char success_str[] = "updata success\r\n";
static const char xmodem_str[] = "send a file by xmodem\r\n";

static const char *serial_device;
static const char *progname;

static void progress_bar(double percentage, size_t len, size_t len_f)
{
	int val = (int) (percentage * 100);
	int lpad = (int) (percentage * PBWIDTH);
	int rpad = PBWIDTH - lpad;

	printf("\r[%.*s%*s] %3d%%", lpad, PBSTR, rpad, "", val);
	printf("  %zu/%zu Bytes", len, len_f);
	fflush(stdout);
}

/* for CRC */
static uint16_t crc_update(uint16_t crc_in, int incr)
{
	uint16_t xor = crc_in >> 15;
	uint16_t out = crc_in << 1;

	if (incr)
		out++;

	if (xor)
		out ^= 0x1021;

	return out;
}

static uint16_t swap16(uint16_t in)
{
	return (in >> 8) | ((in & 0xff) << 8);
}

static uint16_t crc16(const uint8_t *data, uint16_t size)
{
	uint16_t crc, i;

	for (crc = 0; size > 0; size--, data++)
		for (i = 0x80; i; i >>= 1)
			crc = crc_update(crc, *data & i);

	for (i = 0; i < 16; i++)
		crc = crc_update(crc, 0);

	return crc;
}
/* for CRC */

static int xmodem_send(int serial_f, const char *filename)
{
	int ret, fd;
	size_t len, len_f;
	uint8_t response;
	struct stat stat;
	uint8_t eot = EOT;
	const uint8_t *buf;
	struct xmodem_packet packet;
	double progress;

	fd = open(filename, O_RDWR | O_SYNC);
	if (fd < 0) {
		fprintf(stderr, "Can NOT open file: %s", filename);
		return -errno;
	}

	fstat(fd, &stat);
	len = len_f = stat.st_size;
	buf = mmap(NULL, len, PROT_READ, MAP_PRIVATE, fd, 0);
	if (!buf) {
		perror("mmap");
		return -errno;
	}

	printf("\tWaiting for XMODEM request[C]...\n");
	fflush(stdout);

	debug("[probing C]:");
	do {
		ret = read(serial_f, &response, sizeof(response));
		if (ret != sizeof(response)) {
			perror("read");
			return -errno;
		}
		debug("%c", response);
	} while (response != 'C');
	debug("Got C");
	printf("\tSending %s \n", filename);

	packet.block = 1;
	packet.start = SOH;

	while (len) {
		size_t z = 0;
		int next = 0;
//		char status;

		z = min(len, sizeof(packet.payload));
		memcpy(packet.payload, buf, z);
		memset(packet.payload + z, 0xff, sizeof(packet.payload) - z);

		packet.crc = swap16(crc16(packet.payload, sizeof(packet.payload)));
		packet.block_neg = 0xff - packet.block;

		ret = write(serial_f, &packet, sizeof(packet));
		if (ret != sizeof(packet))
			return -errno;

		ret = read(serial_f, &response, sizeof(response));
		if (ret != sizeof(response))
			return -errno;

		switch (response) {
		case NAK:
//			status = 'N';
			break;

		case ACK:
//			status = '.';
			next = 1;
			break;

		default:
//			status = '?';
			next = 1;
			break;
		}

		progress = (len_f - len) / (len_f / 100);
		if(len<XMODEM_PAYLOAD_LEN)
			progress_bar(1, len_f, len_f);
		else
			progress_bar(progress / 100, (len_f - len), len_f);

		if (next) {
			packet.block++;
			len -= z;
			buf += z;
		}
	}

	ret = write(serial_f, &eot, sizeof(eot));
	if (ret != sizeof(eot))
		return -errno;

	printf("\n");
	close(fd);
	return 0;
}

static int open_serial(const char *path, int baud, int canonical)
{
	int fd;
	struct termios tty;

	fd = open(path, O_RDWR | O_SYNC);
	if (fd < 0) {
		perror("Can NOT open serial port.");
		return -errno;
	}

	memset(&tty, 0, sizeof(tty));
	if (tcgetattr(fd, &tty) != 0) {
		perror("tcgetattr");
		return -errno;
	}

	cfsetospeed(&tty, baud);
	cfsetispeed(&tty, baud);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
	if(canonical)
		tty.c_lflag |= ICANON;
	else
		tty.c_lflag = 0;

	tty.c_oflag = 0;
	tty.c_cc[VMIN]  = 1;
	tty.c_cc[VTIME] = 10;
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~(PARENB | PARODD);
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		perror("tcsetattr");
		return -errno;
	}

	return fd;
}

static void check_success(void)
{
	int ret, serial_f;
	char buf[BUFF_SIZE];

	printf("Awaiting confirmation...\n");

	serial_f = open_serial(serial_device, DEBUG_BAUD, 1);
	if (serial_f < 0)
		exit(EXIT_FAILURE);

	do {
		ret = read(serial_f, buf, sizeof(buf));
		buf[ret] = '\0';

		debug("GOT[%d]:%s", ret, buf);
	} while(strcmp(success_str, buf));
	debug("Hit: updata success");

	printf("done.\n\n");
	close(serial_f);
}

static void initialize(void)
{
	int ret, serial_f;
	char buf[BUFF_SIZE];

	serial_f = open_serial(serial_device, DEBUG_BAUD, 1);
	if (serial_f < 0)
		exit(EXIT_FAILURE);

	printf("Waiting for bootloader mode on %s...\n", serial_device);

	do {
		ret = read(serial_f, buf, BUFF_SIZE);
		buf[ret] = '\0';

		debug("GOT[%d]:%s", ret, buf);
	} while(strcmp(bootrom_str, buf));
	debug("Hit: (C)SiFive");

	printf("Bootloader mode active\n\n");
	close(serial_f);
}

static void send_recovery(const char *filename)
{
	int ret, serial_f;
	char cmd1[] = "load 0x18000000\n";
	char cmd2[] = "do 0x18000000\n";

	serial_f = open_serial(serial_device, DEBUG_BAUD, 0);
	if (serial_f < 0)
		exit(EXIT_FAILURE);

	ret = write(serial_f, &cmd1, sizeof(cmd1));
	if (ret != sizeof(cmd1))
		exit(EXIT_FAILURE);

	ret = xmodem_send(serial_f, filename);
	if (ret < 0)
		exit(EXIT_FAILURE);

	ret = write(serial_f, &cmd2, sizeof(cmd2));
	if (ret != sizeof(cmd2))
		exit(EXIT_FAILURE);

	fflush(stdout);
	close(serial_f);
}

static void select_update_option(int option)
{
	int ret, serial_f;
	char buf[BUFF_SIZE];
	uint8_t new_line[] = {13,10};
	char o1[] = "0\r\n";
	char o2[] = "1\r\n";

	serial_f = open_serial(serial_device, DEBUG_BAUD, 1);
	if (serial_f < 0)
		exit(EXIT_FAILURE);

	if(option) {
		ret = write(serial_f, &o2, sizeof(o2));
		if (ret != sizeof(o2))
			exit(EXIT_FAILURE);
		debug("SEND[%lu]:%s", sizeof(o2), o2);
	} else {
		ret = write(serial_f, &o1, sizeof(o1));
		if (ret != sizeof(o1))
			exit(EXIT_FAILURE);
		debug("SEND[%lu]:%s", sizeof(o1), o1);
	}

	ret = write(serial_f, &new_line, sizeof(new_line));
	if (ret != sizeof(new_line))
		exit(EXIT_FAILURE);
	debug("SEND[%lu]:CR LF", sizeof(new_line));

	do {
		ret = read(serial_f, buf, BUFF_SIZE);
		buf[ret] = '\0';

		debug("GOT[%d]:%s", ret, buf);
	} while(strcmp(xmodem_str, buf));
	debug("Hit: send a file by xmodem");

	close(serial_f);
}

static void update_firmware(const char *filename)
{
	int ret, serial_f;

	serial_f = open_serial(serial_device, DEBUG_BAUD, 0);
	if (serial_f < 0)
		exit(EXIT_FAILURE);

	ret = xmodem_send(serial_f, filename);
	if (ret < 0)
		exit(EXIT_FAILURE);

	close(serial_f);
	check_success();
}

static void usage(void)
{
	fprintf(stderr,
		"Usage: %s [OPTION]... \n"
		"-D, --device <tty device>	: Serial tty device path.\n"
		"-r, --recovery <filename>	: Bootloader recovery firmware.\n"
		"-b, --bootloader <filename>	: Second stage bootloader.\n"
		"-d, --ddrinit <filename>	: DRAM initialization firmware.\n"
		"-h, --help			: Show this help.\n", progname);
}

static const char *optstring = "-hD:r:b:d:";
static const struct option long_options[] = {
	{ "device", 1, NULL, 'D' },
	{ "recovery", 1, NULL, 'r' },
	{ "bootloader", 1, NULL, 'b' },
	{ "ddrinit", 1, NULL, 'd' },
	{ "help", 0, NULL, 'h' },
	{ NULL, 0, NULL, 0 },
};

int main(int argc, char **argv)
{
	char c, *recovery_f = NULL, *bootloader_f = NULL, *ddr_init_f = NULL;

	progname = argv[0];

	while ((c = getopt_long(argc, argv, optstring, long_options, NULL)) >= 0) {
		switch (c) {
		case 'r':
			recovery_f = optarg;
			break;
		case 'b':
			bootloader_f = optarg;
			break;
		case 'd':
			ddr_init_f = optarg;
			break;
		case 'D':
			serial_device = optarg;
			break;
		case 'h':
			usage();
			exit(EXIT_SUCCESS);
			break;
		default:
			goto error;
		}
	}

	if(serial_device) {
		initialize();
	} else {
		fprintf(stderr, "Need serial device path.\n");
		goto error;
	}

	if(recovery_f) {
		printf("Uploading recovery binary...\n");
		send_recovery(recovery_f);
		printf("\n----------Enter recovery mode----------\n");
	} else {
		fprintf(stderr, "Need recovery file.\n");
		goto error;
	}

	if (bootloader_f || ddr_init_f) {
		if(bootloader_f) {
			printf("Updating bootloader...\n");
			select_update_option(0);
			update_firmware(bootloader_f);
		}

		if(ddr_init_f) {
			printf("Updating dduinit...\n");
			select_update_option(1);
			update_firmware(ddr_init_f);
		}
		printf("\nFirmware update completed!\n");
		exit(EXIT_SUCCESS);
	}

error:
	usage();
	exit(EXIT_FAILURE);
}
