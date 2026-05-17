#include "serial.h"
#include "app_state.h"
#include "logger.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>

static int set_interface_attribs(int fd, int speed, int databits, int stopbits, const char *parity)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        log_error("tcgetattr: %s", strerror(errno));
        return -1;
    }

    cfmakeraw(&tty);

    speed_t baud;
    switch (speed) {
        case 9600: baud = B9600; break;
        case 19200: baud = B19200; break;
        case 38400: baud = B38400; break;
        case 57600: baud = B57600; break;
        case 115200: baud = B115200; break;
        default: baud = B115200; break;
    }
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag &= ~CSIZE;
    switch (databits) {
        case 5: tty.c_cflag |= CS5; break;
        case 6: tty.c_cflag |= CS6; break;
        case 7: tty.c_cflag |= CS7; break;
        case 8:
        default: tty.c_cflag |= CS8; break;
    }

    if (stopbits == 2)
        tty.c_cflag |= CSTOPB;
    else
        tty.c_cflag &= ~CSTOPB;

    tty.c_cflag &= ~(PARENB | PARODD);
    if (parity && strcmp(parity, "none") != 0) {
        if (strcmp(parity, "even") == 0) {
            tty.c_cflag |= PARENB;
        } else if (strcmp(parity, "odd") == 0) {
            tty.c_cflag |= PARENB | PARODD;
        }
    }

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        log_error("tcsetattr: %s", strerror(errno));
        return -1;
    }

    return 0;
}

void *serial_thread(void *arg)
{
    Config *cfg = (Config *)arg;

    log_info("Serial opening %s baud=%d", cfg->serial_port, cfg->baudrate);

    int fd = open(cfg->serial_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        log_error("open serial: %s", strerror(errno));
        return NULL;
    }

    if (set_interface_attribs(fd, cfg->baudrate, cfg->databits,cfg->stopbits, cfg->parity) != 0) {
        close(fd);
        return NULL;
    }

    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    char buf[256];

    while (g_running) {
        ssize_t n = read(fd, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';

            pthread_mutex_lock(&g_data_mutex);
            snprintf(g_last_serial_data, sizeof(g_last_serial_data), "%s", buf);
            g_serial_data_available = 1;
            pthread_mutex_unlock(&g_data_mutex);

            log_debug("Serial read: %s", buf);

            // do not write (yet)
            //ssize_t w = write(fd, buf, n);
            //if (w < 0) {
            //    log_error("serial write: %s", strerror(errno));
            //}
        } else if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                continue;
            log_error("serial read: %s", strerror(errno));
            break;
        } else {
            usleep(1000);
        }
    }

    close(fd);
    log_info("Serial thread exiting");
    return NULL;
}
