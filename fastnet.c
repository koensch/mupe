#include "fastnet.h"
#include "app_state.h"
#include "logger.h"

#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>

// to enable non-standard baudrates; you need other includefiles and not the standard one
// you migh need to fiddle arount to het in working on your system. Google is your friend.
//#include <termios.h>
#include "/usr/include/asm-generic/termbits.h"
#include "/usr/include/asm-generic/ioctls.h"

#include <string.h>
#include <errno.h>

#include "bgnet.h"

static int set_interface_attribs(char *cPort, int fd, int speed, int databits, int stopbits, const char *parity, const char *mode)
{
    struct termios2 tty;
    int r;

    // get existing Terminal Control block
    r = ioctl(fd, TCGETS2, &tty);
    if (r == 0) {
    } else {
        log_error("Fastnet serial settings: ioctl error %d", r);
        return -1;
    }

    tty.c_iflag &= ~IGNBRK;                 // disable break processing
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // disable xon/xoff control
    tty.c_lflag = 0;                        // no signalling characters, no echo
    tty.c_oflag = 0;                        // no remapping, no delays
    tty.c_cc[VMIN] = 0;                     // read doesn't block
    tty.c_cc[VTIME] = 1;                    // 0.1 sec read timeout

    tty.c_cflag |= (CLOCAL | CREAD);  // ignore modem status, enable receiver

    // set baudrate to a custom value
    tty.c_cflag &= ~CBAUD; // clear CBAUD bits
    tty.c_cflag |= BOTHER; // set 'other' bit
    tty.c_ispeed = speed;
    tty.c_ospeed = speed;

    // set 'databits'
    tty.c_cflag &= ~CSIZE; // clear the CS bits
    switch (databits) {    // set the CS bits accordingly
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
        log_info("Fastnet serial settings: port %s, %d baud, %d databits, %d stopbits, %s parity, %s mode", cPort, speed, databits, stopbits, parity, mode);
    } else {
        log_error("Fastnet serial settings: ioctl error %d", r);
        return -1;
    }

    return 0;
}

void *fastnet_thread(void *arg)
{
    BGNET_CTRL *pCtrl= (BGNET_CTRL *)arg;

    log_info("Fastnet opening %s", pCtrl->pCfg->fastnet.port);

    int fd = open(pCtrl->pCfg->fastnet.port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        log_error("Fastnet serial: open(): %s", strerror(errno));
        return NULL;
    }

    if (set_interface_attribs(pCtrl->pCfg->fastnet.port, fd, pCtrl->pCfg->fastnet.baudrate, pCtrl->pCfg->fastnet.databits,pCtrl->pCfg->fastnet.stopbits, pCtrl->pCfg->fastnet.parity, pCtrl->pCfg->fastnet.mode) != 0) {
        log_error("Fastnet serial: set_interface_attribs(): %s", strerror(errno));
        close(fd);
        return NULL;
    }

    // identify selected protocol
    pCtrl->pCfg->fastnet.handler = BGNET_HANDLER_UNDEF;
    if (strcmp(pCtrl->pCfg->fastnet.protocol, BGNET_HANDLER_TYPE_NETIO) == 0) {
        pCtrl->pCfg->fastnet.handler = BGNET_HANDLER_NETIO;
    }
    // validate the protocol
    switch (pCtrl->pCfg->fastnet.handler) {
        case BGNET_HANDLER_NETIO:
            break;
        default:
            log_error("Fastnet serial: protocol '%s' is not supported. Implemented protocol(s): 'netio'", pCtrl->pCfg->fastnet.protocol);
            close(fd);
            g_running = 0;
            return NULL;
            break;
    }
    // note: protocol handler is triggered from timer thread when data was received (by the fastnet thread). We only want to
    // proceed here if a valid protocol handler is defined, 

    // we allow readonly/passive/readwrite on this communication. This allows us to be a 'monitor-only' application
    // if you enter something wrong, you get RO mode
    if (strcmp(pCtrl->pCfg->fastnet.mode, "readwrite") == 0) {
        g_fastnet_mode = BGN_FASTNET_MODE_RW; // write actively
    } else {
        if (strcmp(pCtrl->pCfg->fastnet.mode, "passive") == 0) {
            g_fastnet_mode = BGN_FASTNET_MODE_PASSIVE; // only write when explicitly requested
        } else {
            g_fastnet_mode = BGN_FASTNET_MODE_RO; // do not write anything
        }
    }

    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    // we could use a smaller size for RxBuf because we read unblocked, but again, to write we need blockin and have
    // space for the full frame. We keep them the same size for consistency
    unsigned char RxBuf[BGNET_RXTXRAW_SIZE];
    unsigned char TxBuf[BGNET_RXTXRAW_SIZE];
    ssize_t nRx, nTx, nWx;

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
    g_fastnet_data_lenRx = 0;
    g_fastnet_data_counter = SerialCounter_Reset;
    pthread_mutex_unlock(&g_data_mutex);

    nRx = 0;
    nTx = 0;    
    while (g_running) {
        // get input (if any) into our local buffer
        nRx = read(fd, RxBuf, sizeof(RxBuf));
        if (nRx > 0) {
            // if we receive more that we can handle, let it be known. With a known baudrate and the 1msec interval you can estimate
            // how much space we need in the receive buffer.
            // 11000 baud, 12bits/character (8 data, 1 start, 1 parity, 2 stop) => ~1000 bytes per second => 1 byte/msec (with 1024
            // bytes we are totalloy safe and the overflow should never occur)
            if (nRx > (ssize_t)(sizeof(RxBuf) - 1)) {
                log_debug("Fastnet serial read: possible overflow of receive buffer");
            } else {
                // if any input received; append to buffer; contents is processed by routine kick-off by timer process
                pthread_mutex_lock(&g_data_mutex);
                memcpy(&g_last_fastnet_dataRx[g_fastnet_data_lenRx], RxBuf, nRx);
                g_fastnet_data_lenRx = g_fastnet_data_lenRx + nRx;
                g_fastnet_data_counter = SerialCounter_Set;
                pthread_mutex_unlock(&g_data_mutex);
            }
        } else if (nRx < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                continue;
            log_error("Fastnet serial read: %s", strerror(errno));
            break;
        } else {
            // If there is no input (n=0) and the counter is below the mark (ie. there was some silence), we may send someting.
            // Realise that when we send someting, this is echoed back into our read (TxD and RxD signals are connected).
            // That means we have to write 'blocking' to avoid being interrupted by our output coming back in.
            // Realise that when we send somthing, this may coincide with another station on the fastnet bus doing the same
            // This is not something we can prevent. Collisions are observed by CS errors on received frames. If this happens
            // frequently, we might need alter our timing to improve peaceful co-excistence with the other devices on the network.
            if (g_fastnet_data_counter > SerialCounter_Reset) {
                // someting is still going on
            } else {
                // if there is someting to send
                if (g_fastnet_data_lenTx > 0) {
                        
                    // get message from common storage into local storage
                    pthread_mutex_lock(&g_data_mutex);
                    memcpy(&TxBuf, &g_last_fastnet_dataTx, g_fastnet_data_lenTx);
                    nTx = g_fastnet_data_lenTx;
                    g_fastnet_data_lenTx = 0; // release common storage
                    pthread_mutex_unlock(&g_data_mutex);

                    // write content
                    nWx = write(fd, TxBuf, nTx);
                    if (nWx == nTx) {
                        // nothing wrong here, everything was written
                        //log_info("Fastnet serial write: %d bytes output", nTx);
                    } else {
                        // for some obscure reason we we unable to write all data
                        log_error("Fastnet serial write: %s (bytes to send= %d, actually send= %d)", strerror(errno), nTx, nWx);
                    }
                }
            }
        }
        usleep(MSEC1);
    }
    close(fd);
    log_info("Fastnet Serial: thread exiting");
    return NULL;
}
