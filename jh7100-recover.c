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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <termios.h>
#include <unistd.h>

#define SOH 0x01
#define ACK 0x06
#define NAK 0x15 
#define EOT 0x04

#define min(a, b)   ((a) < (b) ? (a) : (b))

#define PBSTR "########################################"
#define PBWIDTH 40

char *serial_device;

struct xmodem_packet {
    uint8_t start;
    uint8_t block;
    uint8_t block_neg;
    uint8_t payload[128];
    uint16_t crc;
} __attribute__((packed));

void progress_bar(double percentage, size_t len, size_t len_f) {
    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\r[%.*s%*s] %3d%%", lpad, PBSTR, rpad, "", val);
    printf("  %zu/%zu Bytes", len, len_f);
    fflush(stdout);
}

uint16_t crc_update(uint16_t crc_in, int incr)
{
    uint16_t xor = crc_in >> 15;
    uint16_t out = crc_in << 1;

    if (incr)
            out++;

    if (xor)
            out ^= 0x1021;

    return out;
}

uint16_t swap16(uint16_t in)
{
    return (in >> 8) | ((in & 0xff) << 8);
}

uint16_t crc16(const uint8_t *data, uint16_t size)
{
    uint16_t crc, i;

    for (crc = 0; size > 0; size--, data++)
        for (i = 0x80; i; i >>= 1)
            crc = crc_update(crc, *data & i);

    for (i = 0; i < 16; i++)
        crc = crc_update(crc, 0);

    return crc;
}

int xmodem_send(int serial_f, const char *filename)
{
    size_t len, len_f;
    int ret, fd;
    uint8_t response;
    struct stat stat;
    const uint8_t *buf;
    uint8_t eot = EOT;
    struct xmodem_packet packet;
    double progress;

    fd = open(filename, O_RDONLY);
    if (fd < 0) {
        perror("open");
        return -errno;
    }

    fstat(fd, &stat);
    len = len_f = stat.st_size;
    buf = mmap(NULL, len, PROT_READ, MAP_PRIVATE, fd, 0);
    if (!buf) {
        perror("mmap");
        return -errno;
    }

    printf("\nWaiting for XMODEM request...");
    fflush(stdout);

    do {
        ret = read(serial_f, &response, sizeof(response));
        if (ret != sizeof(response)) {
            perror("read");
            return -errno;
        }
    } while (response != 'C');

    printf("done.\n");
    printf("Sending %s \n", filename);

    packet.block = 1;
    packet.start = SOH;

    while (len) {
        size_t z = 0;
        int next = 0;
        char status;

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
                status = 'N';
                break;
        case ACK:
                status = '.';
                next = 1;
                break;
        default:
                status = '?';
                next = 1;
                break;
        }

        progress = (len_f - len)/(len_f/100);
        if(len<128)
            progress_bar(1, len_f, len_f);
        else
            progress_bar(progress/100, (len_f-len), len_f);

        if (next) {
                packet.block++;
                len -= z;
                buf += z;
        }
    }

    printf("\nAwaiting confirmation...");
    ret = write(serial_f, &eot, sizeof(eot));
    if (ret != sizeof(eot))
        return -errno;

    printf("\n");
    close(fd);
    return 0;
}

int open_serial(const char *path, int baud, int canonical)
{
    int fd;
    struct termios tty;

    fd = open(path, O_RDWR | O_SYNC);
    if (fd < 0) {
            perror("open");
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

int check_success(int success_text_len)
{
    int i, ret, serial_f;
    char buf[128];

    serial_f = open_serial(serial_device, 9600, 1);
    if (serial_f < 0)
            return -errno;

    while(1) {
        ret = read(serial_f, buf, sizeof(buf));
        // printf("bytes received: %d\n", ret);
        // for (i = 0; i < ret; i++) {
        //     printf("%c", buf[i]);
        // }
        if(ret == success_text_len){
            printf("done.\n");
            break;
        }
    }

    close(serial_f);
    return 0;
}

int initialize()
{
    int i, ret, serial_f;
    char buf[128];

    serial_f = open_serial(serial_device, 9600, 1);
    if (serial_f < 0)
            return -errno;

    printf("Waiting for bootloader mode...\n");
    while(1) {

        ret = read(serial_f, buf, sizeof(buf));

        for (i = 0; i < ret; i++) {
            printf("%c", buf[i]);
        }
        if(ret == 10){
            printf("Bootloader mode active\n");
            break;
        }
    }
    close(serial_f);
    return 0;
}

int send_recovery(const char *filename)
{
    int ret, serial_f;
    char cmd1[] = "load 0x18000000\n";
    char cmd2[] = "do 0x18000000\n";

    serial_f = open_serial(serial_device, 9600, 0);
    if (serial_f < 0)
            return -errno;
    
    ret = write(serial_f, &cmd1, sizeof(cmd1));
        if (ret != sizeof(cmd1))
                return -errno;

    ret = xmodem_send(serial_f, filename);
        if (ret < 0)
                return ret;

    ret = write(serial_f, &cmd2, sizeof(cmd2));
        if (ret != sizeof(cmd2))
                return -errno;

    fflush(stdout);
    close(serial_f);
    return 0;
}

int select_update_option(int option)
{
    int i, ret, serial_f;
    char buf[128];
    uint8_t new_line[] = {13,10};
    char o1[] = "0\r\n";
    char o2[] = "1\r\n";

    serial_f = open_serial(serial_device, 9600, 1);
    if (serial_f < 0)
            return -errno;

    if(option)
    {
        ret = write(serial_f, &o2, sizeof(o2));
        if (ret != sizeof(o2))
                return -errno;
    } else {
        ret = write(serial_f, &o1, sizeof(o1));
        if (ret != sizeof(o1))
                return -errno;
    }

    ret = write(serial_f, &new_line, sizeof(new_line));
        if (ret != sizeof(new_line))
                return -errno;

    while(1) {

        ret = read(serial_f, buf, sizeof(buf));
        // for (i = 0; i < ret; i++) {
        //     printf("%c", buf[i]);
        // }
        if(ret == 22){
            break;
        }
    }

    close(serial_f);
    return 0;
}

int update_firmware(const char *filename)
{
    int ret, serial_f;

    serial_f = open_serial(serial_device, 9600, 0);
    if (serial_f < 0)
            return -errno;

    ret = xmodem_send(serial_f, filename);
        if (ret < 0)
                return ret;
    
    close(serial_f);
    check_success(15);

    return 0;
}

void show_help(void) {
    printf(
        "-D, --device <tty device>      : Serial tty device path.\n"
        "-r, --recovery <filename>      : Bootloader recovery firmware.\n"
        "-b, --bootloader <filename>    : Second stage bootloader.\n"
        "-d, --ddrinit <filename>       : DRAM initialization firmware.\n"
        "-h, --help                     : Show this help.\n"
    );
}

int main(int argc, char **argv)
{
    int i, recovery = 0, bootloader = 0, ddr_init = 0, device = 0;
    const char *recovery_f, *bootloader_f, *ddr_init_f;

    for (i = 1; i < argc; i++) {
        int more = i+1 < argc; /* There are more arguments. */

        if (!strcmp(argv[i],"--device") || !strcmp(argv[i],"-D") && more) {
            device = 1;
            serial_device = strdup(argv[++i]);
        } else if (!strcmp(argv[i],"--recovery") || !strcmp(argv[i],"-r") && more) {
            recovery = 1;
            recovery_f = argv[++i];
        } else if (!strcmp(argv[i],"--bootloader") || !strcmp(argv[i],"-b") && more) {
            bootloader = 1;
            bootloader_f = argv[++i];
        } else if (!strcmp(argv[i],"--ddrinit") || !strcmp(argv[i],"-d") && more) {
            ddr_init = 1;
            ddr_init_f = argv[++i];
        } else if (!strcmp(argv[i],"--help") || !strcmp(argv[i],"-h")) {
            show_help();
            exit(0);
        } else {
            printf("Unknown or not enough arguments for option '%s'.\n", argv[i]);
            show_help();
            exit(1);
        }
    }

    if(device)
    {
        initialize();
    } else {
        printf("No serial device path. Quiting...\n");
        return 1;
    }

    if(recovery)
    {
        send_recovery(recovery_f);
    } else {
        printf("No recovery file. Quiting...\n");
        return 1;
    }
    
    if(bootloader)
    {
        select_update_option(0);
        update_firmware(bootloader_f);
    }
    
    if(ddr_init) {
        select_update_option(1);
        update_firmware(ddr_init_f);
    }

    return 0;
}