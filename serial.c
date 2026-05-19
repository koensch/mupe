#include "serial.h"
#include "app_state.h"
#include "logger.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
//#include <asm/termios.h>
#include <string.h>
#include <errno.h>

static int set_interface_attribs(int fd, int speed, int databits, int stopbits, const char *parity)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        log_error("tcgetattr: %s", strerror(errno));
        return -1;
    }

    cfmakeraw(&tty); // set to default

    speed_t baud = 0;
    switch (speed) {
        case 9600: baud = B9600; break;
        case 19200: baud = B19200; break;
        case 38400: baud = B38400; break;
        case 57600: baud = B57600; break;
        case 115200: baud = B115200; break;
        default: break;
    }
    //#define BOTHER 0x000010000
    #define XCBAUD   0x0000100F
    #define XBOTHER  0x00001000
    if (baud==0) {
        // custom baudrates
        tty.c_cflag &= ~XCBAUD; // clear CBAUD bits
        tty.c_cflag |= XBOTHER; // set 'other' bit
        tty.c_ispeed = speed;
        tty.c_ospeed = speed; 
        log_info("Serial: setting custom baudrate: %d", speed);
    } else {
        // standard baudrates
        cfsetispeed(&tty, baud);
        cfsetospeed(&tty, baud);
    }

    tty.c_cflag &= ~CSIZE;
    switch (databits) {
        case 5: tty.c_cflag |= CS5; break;
        case 6: tty.c_cflag |= CS6; break;
        case 7: tty.c_cflag |= CS7; break;
        case 8:
        default: tty.c_cflag |= CS8; break;
    }

    if (stopbits == 2)
        tty.c_cflag |= CSTOPB;  // set stopbit bit = 2 stopbits
    else
        tty.c_cflag &= ~CSTOPB; // reset stopbit bit = 1 stopbit

    tty.c_cflag &= ~(PARENB | PARODD);
    if (parity && strcmp(parity, "none") != 0) {
        if (strcmp(parity, "even") == 0) {
            tty.c_cflag |= PARENB;
        } else if (strcmp(parity, "odd") == 0) {
            tty.c_cflag |= PARENB | PARODD;
        }
    }

    tty.c_cflag |= CLOCAL | CREAD;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        log_error("tcsetattr: %s", strerror(errno));
        return -1;
    }
    log_debug("Serial: c_cflag = %X", tty.c_cflag);

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

    unsigned char rawbuf[256];
    ssize_t n = 0;

    // this serial handler is part of a reverse-engineering project. At the start, the dynamic behaviour is
    // undefined. To get a understanding how it works and be able to adjust the process without knowing everything
    // up front, the following approach is used.
    // - in this case we are dealing with a bytes stream representing binary values (and not ASCII). On the
    //   protocol level we should seperate frames by timing and not by null-characters.
    // - we read from the serial port non-blocking; we do not wait for a serial packet to complete.
    // - we loop around the 'read' continuously. when there is no input we use a usleep to prevent racing
    // - when there is input, we check the buffer. If it's empty (0), we start building a new frame. We set a
    //   counter to a start value of 100 and keep on adding characters, as long as there is data. Everytime
    //   we have data, we set to counter to 100.
    // - there is a timer-process (running in a different thread) that counts-down this counter, once every 1msec
    //   when the counter is >0 we know that 10*10msec have not yet passed since the last data was received.
    //   At this moment we think/guess that 100ms separates the frames.
    // - The timer-process will continue the count down. When a new frame is being received (and the
    //   counter set to 100) it will give us an idea how long there is between the new frame and the previous one.
    // - When the receive buffer is empty and there is data present in the output buffer, any output message
    //   that is waiting to be send, is written to the port to avoid a collision with others on the same bus.

    // initialize for a clean start
    pthread_mutex_lock(&g_data_mutex);
    g_serial_data_len = 0;
    g_serial_data_counter = SerialCounter_Reset;
    pthread_mutex_unlock(&g_data_mutex);
    
    while (g_running) {
        // get input (if any) into our local buffer
        n = read(fd, rawbuf, sizeof(rawbuf));
        if (n > 0) {
            // if we receive more that we can handle, let it be known
            if (n > (ssize_t)(sizeof(rawbuf)-1)) {
                log_debug("Serial read: possible overflow of receive buffer");
            } else {
                // if any input received; append to buffer
                pthread_mutex_lock(&g_data_mutex);
                memcpy(&g_last_serial_data[g_serial_data_len], rawbuf, n);
                g_serial_data_len = g_serial_data_len + n;
                g_serial_data_counter = SerialCounter_Set;
                pthread_mutex_unlock(&g_data_mutex);
            }
        } else if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                continue;
            log_error("serial read: %s", strerror(errno));
            break;
        } else {
            // if there is no input (n=0) and the counter is quiet, we may send someting
            // realise that when we send someting, this is echoed back into our read (Tx
            // and Rx signals are together. That means we have to write 'blocking'
            if (g_serial_data_counter > SerialCounter_Reset) {
                // someting is still going on
            } else {
                // do not write (yet)
                //ssize_t w = write(fd, buf, n);
                //if (w < 0) {
                //    log_error("serial write: %s", strerror(errno));
                //}
            }
        }
        usleep(MSEC1);
    }
    close(fd);
    log_info("Serial thread exiting");
    return NULL;
}
