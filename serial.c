#include "serial.h"
#include "app_state.h"
#include "logger.h"

#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
//#include <termios.h>
#include <string.h>
#include <errno.h>
#include "/usr/include/asm-generic/termbits.h"
#include "/usr/include/asm-generic/ioctls.h"

#include "bgnprotocol.h"

//#define PROTOCOL_DEBUG

static int set_interface_attribs(int fd, int speed, int databits, int stopbits, const char *parity)
{
    struct termios2 tty;
    int r;

    // get existing Terminal Control block
    r = ioctl(fd, TCGETS2, &tty);
    if (r == 0) {
    } else {
        log_error("Serial settings: ioctl error %d", r);
        return -1;
    }

    tty.c_iflag &= ~IGNBRK;                 // disable break processing
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // disable xon/xoff control
    tty.c_lflag = 0;                        // no signalling characters, no echo
    tty.c_oflag = 0;                        // no remapping, no delays
    tty.c_cc[VMIN] = 0;                     // read doesn't block
    tty.c_cc[VTIME] = 1;                    // 0.1 sec read timeout

    tty.c_cflag |= (CLOCAL | CREAD);  // ignore modem status, enable receiver
    // custom baudrate
    tty.c_cflag &= ~CBAUD; // clear CBAUD bits
    tty.c_cflag |= BOTHER; // set 'other' bit
    tty.c_ispeed = speed;
    tty.c_ospeed = speed;

    // set 'databits'
    tty.c_cflag &= ~CSIZE;
    switch (databits) {
        case 5: tty.c_cflag |= CS5; break;
        case 6: tty.c_cflag |= CS6; break;
        case 7: tty.c_cflag |= CS7; break;
        case 8:
        default: tty.c_cflag |= CS8; break;
    }

    // set 'stopbits'
    if (stopbits == 2)
        tty.c_cflag |= CSTOPB;  // set stopbit bit = 2 stopbits
    else
        tty.c_cflag &= ~CSTOPB; // reset stopbit bit = 1 stopbit

    // set parity
    tty.c_cflag &= ~(PARENB | PARODD);
    if (parity && strcmp(parity, "none") != 0) {
        if (strcmp(parity, "even") == 0) {
            tty.c_cflag |= PARENB;
        } else if (strcmp(parity, "odd") == 0) {
            tty.c_cflag |= PARENB | PARODD;
        }
    }

    r = ioctl(fd, TCSETS2, &tty);
    if (r == 0) {
        log_info("Serial settings: %d baud, %d databits, %d stopbits, %s parity", speed, databits, stopbits, parity);
    } else {
        log_error("Serial settings: ioctl error %d", r);
        return -1;
    }

    return 0;
}

void *serial_thread(void *arg)
{
    Config *cfg = (Config *)arg;
#ifdef PROTOCOL_DEBUG
    char asciibuf[2048];
#endif

    log_info("Serial opening %s", cfg->serial_port);

    int fd = open(cfg->serial_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        log_error("Serial: open(): %s", strerror(errno));
        return NULL;
    }

    if (set_interface_attribs(fd, cfg->baudrate, cfg->databits,cfg->stopbits, cfg->parity) != 0) {
        log_error("Serial: set_interface_attribs(): %s", strerror(errno));
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
    //   when the counter is >0 we know that 100*1msec have not yet passed since the last data was received.
    //   At this moment we think/guess that 85ms separates the frames.
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
#ifdef PROTOCOL_DEBUG
            // convert frame into prinatble text, byte spaced
            byte2ascii(asciibuf, sizeof(asciibuf), rawbuf, n, BYTE2ASCII_SPACED);
            log_debug("Serial %d msec, %d bytes: %s", g_serial_data_counter, n, asciibuf);
#endif
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
