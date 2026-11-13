#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>

//#include <time.h>
//#include <limits.h>
//#include <string.h>
//#include <stdlib.h>
//#include <arpa/inet.h>

#include "app_state.h"
#include "config.h"
#include "logger.h"
#include "bgnet.h"

#include "nmea0183.h"


// return a string with the recognized and supported NMEA sentences
char *bgnet_nmea_sentences(char *cBuffer, uint8_t ucEnable) {
    int i, j;    

    // supported sentences by PILOT
    char pilot_sentence[BGNET_NMEA0183_NOS][4] = {"APA", "APB", "RMB", "XTE", "VTG", "BWR", "BWC", "VHW"};  
    uint8_t pilot_mask[BGNET_NMEA0183_NOS] = {BGNET_NMEA0183_APA, BGNET_NMEA0183_APB, BGNET_NMEA0183_RMB, BGNET_NMEA0183_XTE, BGNET_NMEA0183_VTG, BGNET_NMEA0183_BWR, BGNET_NMEA0183_BWC, BGNET_NMEA0183_VHW};

    strcpy(cBuffer, ""); // clear return buffer
    j = 0;
    for (i=0; i<BGNET_NMEA0183_NOS; i++) {
        if (ucEnable & pilot_mask[i]) {
            if (j>0) {
                strcat(cBuffer, ",");
            }
            strcat(cBuffer, pilot_sentence[i]);
            j++;
        } else {
            // not present
        }
    }
    return(cBuffer);
}

static unsigned char set_interface_pilot(char *cNav)
{
    int i;
    unsigned char r; 

    // supported sentences by PILOT
    char pilot_sentence[BGNET_NMEA0183_NOS][4] = {"APA", "APB", "RMB", "XTE", "VTG", "BWR", "BWC", "VHW"};  
    uint8_t pilot_mask[BGNET_NMEA0183_NOS] = {BGNET_NMEA0183_APA, BGNET_NMEA0183_APB, BGNET_NMEA0183_RMB, BGNET_NMEA0183_XTE, BGNET_NMEA0183_VTG, BGNET_NMEA0183_BWR, BGNET_NMEA0183_BWC, BGNET_NMEA0183_VHW};  

    // identify selected sentences
    r  = BGNET_NMEA0183_OFF;
    for (i=0; i<BGNET_NMEA0183_NOS; i++) {
        if (strstr(cNav, pilot_sentence[i]) == NULL) {
            // not present
        } else {
            r |= pilot_mask[i]; // set bit for this type of sentence
        }
    }
    return(r);
}

// initialize the NMEA control block
void bgnet_nmea_ctrl_init(BGNET_NMEA_CTRL *pNMEA, uint8_t enable, time_t t) {
    int i;
    uint8_t pilot_mask[BGNET_NMEA0183_NOS] = {BGNET_NMEA0183_APA, BGNET_NMEA0183_APB, BGNET_NMEA0183_RMB, BGNET_NMEA0183_XTE, BGNET_NMEA0183_VTG, BGNET_NMEA0183_BWR, BGNET_NMEA0183_BWC, BGNET_NMEA0183_VHW};  

    // initialize counter
    CounterInit(&pNMEA->Counter, t);
    // for every possible sentence set the typebits in the typelist
    // (this allows us, at runtime, to parse through the typelist and select the next configured sentence for this port)
    for (i=0; i<BGNET_NMEA0183_NOS; i++) {
        if (enable & pilot_mask[i]) {
            pNMEA->TypeList[i] = pilot_mask[i]; // enabled sentence for this port
        } else {
            pNMEA->TypeList[i] = 0; // disabled sentence for this port
        }
    }
}
// pop the next sentence type from the list
uint8_t bgnet_nmea_ctrl_pop(BGNET_NMEA_CTRL *pNMEA, time_t t) {
    size_t i;
    uint8_t r;
    // get start counter
    i = pNMEA->Counter.iCounter;
    r = BGNET_NMEA0183_OFF;
    while (1) { // endless loop
        if (i < BGNET_NMEA0183_NOS) {
            // should be ok
        } else {
            // wrap-back to 0
            i = 0;
        }
        if (pNMEA->TypeList[i] == BGNET_NMEA0183_OFF) {
            // skip this one, try next
            i++;
        } else {
            // got one! save current counter and return sentencetype from list
            pNMEA->Counter.iCounter = i;
            pNMEA->Counter.tTimestamp = t;
            r = pNMEA->TypeList[i];
            break; // exit loop
        }
    }
    return(r);
}


static int set_interface_attribs(unsigned char ucEnable, char *chPort, int *fd, struct termios *ptty, int speed, int databits, int stopbits, const char *parity)
{
    int r;
    char strWork[128];

    // get existing Terminal Control block
    r = ioctl(*fd, TCGETS, ptty);
    if (r == 0) {
    } else {
        log_error("NMEA0183 serial settings: ioctl error %d", r);
        return -1;
    }
    ptty->c_iflag &= ~IGNBRK;                 // disable break processing
    ptty->c_iflag &= ~(IXON | IXOFF | IXANY); // disable xon/xoff control
    ptty->c_lflag = 0;                        // no signalling characters, no echo
    ptty->c_oflag = 0;                        // no remapping, no delays
    ptty->c_cc[VMIN] = 0;                     // read doesn't block
    ptty->c_cc[VTIME] = 1;                    // 0.1 sec read timeout

    ptty->c_cflag |= (CLOCAL | CREAD);  // ignore modem status, enable receiver

    // set baudrate to a standard value
    ptty->c_cflag &= ~CBAUD; // clear CBAUD bits
    switch (speed) {
        case 4800:
            cfsetispeed(ptty, B4800);
            break;
        case 9600:
            cfsetispeed(ptty, B9600);
            break;
        default:
            log_error("NMEA0183 serial settings: unsupported baudrate: %d", speed);
            break;
    }
 
    // set 'databits'
    ptty->c_cflag &= ~CSIZE; // clear the CS bits (= undefined)
    switch (databits) {    // set the CS bits accordingly
        case 5: ptty->c_cflag |= CS5; break;
        case 6: ptty->c_cflag |= CS6; break;
        case 7: ptty->c_cflag |= CS7; break;
        case 8:
        default: ptty->c_cflag |= CS8; break;
    }

    // set 'stopbits'
    ptty->c_cflag &= ~CSTOPB; // reset stop bit (= 1 stopbit)
    switch (stopbits) {
        case 2: ptty->c_cflag |= CSTOPB; break; // set stop bit (= 2 stopbits)
        case 1:
        default: break;
    }

    // set parity
    ptty->c_cflag &= ~(PARENB | PARODD); // reset parity bits (= none)
    if (parity && strcmp(parity, "none") == 0) {
        // leave bits reset
    } else {
        if (strcmp(parity, "even") == 0) {
            ptty->c_cflag |= PARENB;
        } else if (strcmp(parity, "odd") == 0) {
            ptty->c_cflag |= PARENB | PARODD;
        }
    }

    r = ioctl(*fd, TCSETS, ptty);
    if (r == 0) {
        log_info("NMEA0183 serial settings: port %s, %d baud, %d databits, %d stopbits, %s parity (%s)", chPort, speed, databits, stopbits, parity, bgnet_nmea_sentences(strWork, ucEnable));
    } else {
        log_error("NMEA0183 serial settings: ioctl error %d", r);
        return -1;
    }

    // we read non-blocking, because we do not expect and RxD data; PILOT only has the TxD signal wired
    int flags = fcntl(*fd, F_GETFL, 0);
    fcntl(*fd, F_SETFL, flags & ~O_NONBLOCK);

    return 0;
}

static void interface_handler(int fd, int *g_lenRx, unsigned char *g_dataRx, int *g_lenTx, unsigned char *g_dataTx)
{
    unsigned char RxBuf[BGNET_RXTXRAW_SIZE];
    unsigned char TxBuf[BGNET_RXTXRAW_SIZE];
    ssize_t nRx, nTx, nWx;

    log_debug("NMEA0183 - interface_handler(),fd= %X, g_lenRx= %d, g_lenTx= %d", fd, *g_lenRx, *g_lenTx);

    // get input (if any) into our local buffer. we do not expect RxD data, but we leave
    // this structure intact for other situations
    nRx = read(fd, RxBuf, sizeof(RxBuf));
    log_debug("NMEA0183 - nRx= %d", nRx);
    if (nRx > 0) {
        // we do not expect any RxD data, we do this to prevent compiler warnings because of unused arguments
        if (*g_lenRx > 0) {
            // get message from common storage into local storage
            pthread_mutex_lock(&g_data_mutex);
            memcpy(&RxBuf, g_dataRx, *g_lenRx);
            nRx = *g_lenRx;
            *g_lenRx = 0; // release common storage
            pthread_mutex_unlock(&g_data_mutex);
        }
    } else if (nRx < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            log_error("NMEA0183 serial read: %s", strerror(errno));
        } else {
            if (*g_lenTx > 0) {
                // get message from common storage into local storage
                pthread_mutex_lock(&g_data_mutex);
                memcpy(&TxBuf, g_dataTx, *g_lenTx);
                nTx = *g_lenTx;
                *g_lenTx = 0; // release common storage
                pthread_mutex_unlock(&g_data_mutex);

                // write content
                nWx = write(fd, TxBuf, nTx);
                if (nWx == nTx) {
                    // nothing wrong here, everything was written
                    //log_info("NMEA0183 serial write: %d bytes output", nTx);
                } else {
                    // for some obscure reason we we unable to write all data
                    log_error("NMEA0183 serial write: %s (bytes to send= %d, actually send= %d)", strerror(errno), nTx, nWx);
                }
            }
        }
    }
}

void *nmea0183_thread(void *arg)
{
    BGNET_CTRL *pCtrl= (BGNET_CTRL *)arg;
    int fd1 = 0, fd2 = 0;
    struct termios tty1, tty2;

    // configure port1
    pCtrl->pCfg->nmea0183.port1.enable = BGNET_NMEA0183_OFF;
    if (strlen(pCtrl->pCfg->nmea0183.port1.port) > 0) {
        log_info("NMEA0183 opening port 1: %s", pCtrl->pCfg->nmea0183.port1.port);

        fd1 = open(pCtrl->pCfg->nmea0183.port1.port, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd1 < 0) {
            log_error("NMEA0183 serial port1: open(): %s", strerror(errno));
        }

        // enable NMEA1083 sentences for this port
        pCtrl->pCfg->nmea0183.port1.enable = set_interface_pilot(pCtrl->pCfg->nmea0183.port1.nav);
        if (pCtrl->pCfg->nmea0183.port1.enable == BGNET_NMEA0183_OFF) {
            // no sentences selected
            log_error("NMEA0183 serial port1: no valid NMEA0183 sentences selected");
            close(fd1);
        } else {
            if (set_interface_attribs(pCtrl->pCfg->nmea0183.port1.enable, pCtrl->pCfg->nmea0183.port1.port, &fd1, &tty1, pCtrl->pCfg->nmea0183.baudrate, pCtrl->pCfg->nmea0183.databits,pCtrl->pCfg->nmea0183.stopbits, pCtrl->pCfg->nmea0183.parity) != 0) {
                log_error("NMEA0183 serial port1: set_interface_attribs(): %s", strerror(errno));
                close(fd1);
            }
        }
    }
    log_debug("NMEA0183 serial port1: fd1= 0x%X", fd1);

    // configure port2
    pCtrl->pCfg->nmea0183.port2.enable = BGNET_NMEA0183_OFF;
    if (strlen(pCtrl->pCfg->nmea0183.port2.port) > 0) {
        log_info("NMEA0183 opening port 2: %s", pCtrl->pCfg->nmea0183.port2.port);

        fd2 = open(pCtrl->pCfg->nmea0183.port2.port, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd2 < 0) {
            log_error("NMEA0183 serial port2: open(): %s", strerror(errno));
        }

        // enable NMEA1083 sentences for this port
        pCtrl->pCfg->nmea0183.port2.enable = set_interface_pilot(pCtrl->pCfg->nmea0183.port2.nav);
        if (pCtrl->pCfg->nmea0183.port2.enable == BGNET_NMEA0183_OFF) {
            // no sentences selected
            log_error("NMEA0183 serial port2: no valid NMEA0183 sentences selected");
            close(fd2);
        } else {
            if (set_interface_attribs(pCtrl->pCfg->nmea0183.port2.enable, pCtrl->pCfg->nmea0183.port2.port, &fd2, &tty2, pCtrl->pCfg->nmea0183.baudrate, pCtrl->pCfg->nmea0183.databits,pCtrl->pCfg->nmea0183.stopbits, pCtrl->pCfg->nmea0183.parity) != 0) {
                log_error("NMEA0183 serial port2: set_interface_attribs(): %s", strerror(errno));
                close(fd2);
            }
        }
    }
    log_debug("NMEA0183 serial port1: fd2= 0x%X", fd2);

    // only proceed if we have port1 and/or port2 configured
    if ((pCtrl->pCfg->nmea0183.port1.enable == BGNET_NMEA0183_OFF) & (pCtrl->pCfg->nmea0183.port2.enable == BGNET_NMEA0183_OFF)) {
        log_error("NMEA0183 serial: no port configured, exit thread");
    } else {
        // initialize for a clean start
        pthread_mutex_lock(&g_data_mutex);
        g_nmea0183_data_lenRx1 = 0;
        g_nmea0183_data_lenTx1 = 0;
        g_nmea0183_data_lenRx2 = 0;
        g_nmea0183_data_lenTx2 = 0;
        pthread_mutex_unlock(&g_data_mutex);

        while (g_running) {
            if (fd1 == 0) {
                // port not configured
            } else {
                interface_handler(fd1, &g_nmea0183_data_lenRx1, g_last_nmea0183_dataRx1, &g_nmea0183_data_lenTx1, g_last_nmea0183_dataTx1);
            }
            if (fd2 == 0) {
                // port not configured
            } else {
                interface_handler(fd2, &g_nmea0183_data_lenRx2, g_last_nmea0183_dataRx2, &g_nmea0183_data_lenTx2, g_last_nmea0183_dataTx2);
            }
            sleep(SEC2); // 2 seconds between sentences
        }
    }
    close(fd1);
    close(fd2);
    log_info("NMEA0183 serial: thread exiting");
    return NULL;
}
