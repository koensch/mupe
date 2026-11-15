#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include "app_state.h"
#include "config.h"
#include "logger.h"
#include "bgnet.h"

#include "nmea0183.h"


// return a string with the recognized and by PILOT supported NMEA sentences
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

static unsigned char set_interface_pilot(BGNET_NMEA_CTRL *pNMEA, char *cNav)
{
    int i;
    unsigned char r; 

    // supported sentences by PILOT
    char pilot_sentence[BGNET_NMEA0183_NOS][4] = {"APA", "APB", "RMB", "XTE", "VTG", "BWR", "BWC", "VHW"};  
    uint8_t pilot_mask[BGNET_NMEA0183_NOS] = {BGNET_NMEA0183_APA, BGNET_NMEA0183_APB, BGNET_NMEA0183_RMB, BGNET_NMEA0183_XTE, BGNET_NMEA0183_VTG, BGNET_NMEA0183_BWR, BGNET_NMEA0183_BWC, BGNET_NMEA0183_VHW};  

    // identify selected sentences and put data in the TypeList
    r  = BGNET_NMEA0183_OFF;
    for (i = 0; i < BGNET_NMEA0183_NOS; i++) {
        if (strstr(cNav, pilot_sentence[i]) == NULL) {
            // not present
            pNMEA->TypeList[i] = BGNET_NMEA0183_OFF;
        } else {
            pNMEA->TypeList[i] = pilot_mask[i];
            r |= pilot_mask[i]; // set bit for this type of sentence
        }
    }
    return(r);
}

// pop the next sentence type from the list
uint8_t bgnet_nmea_ctrl_pop(BGNET_NMEA_CTRL *pNMEA, time_t t) {
    uint8_t r;
    r = BGNET_NMEA0183_OFF;
    while (1) { // endless loop
        if (pNMEA->Counter.iCounter < BGNET_NMEA0183_NOS) {
            // should be ok (in range between 0-NOS)
        } else {
            // wrap-back to 0
            pNMEA->Counter.iCounter = 0;
        }
        if (pNMEA->TypeList[pNMEA->Counter.iCounter] == BGNET_NMEA0183_OFF) {
            // skip this one, try next
            pNMEA->Counter.iCounter++;
        } else {
            // got one! save current counter and return sentencetype from list
            r = pNMEA->TypeList[pNMEA->Counter.iCounter];
            pNMEA->Counter.iCounter++;
            pNMEA->Counter.tTimestamp = t;
            break; // exit loop
        }
    }
    return(r);
}
// a typical NMEA0183 sentence looks like this, where the checksum varies by the contents between $ and * (not included):
//  "$GPAPA,A,A,0.10,R,N,V,V,011,M,DEST,011,M*zz"
// 1 - first character must be "$" (is skipped for checksum calculation)
// 2 - the separator between the sentence and the checksum is a "*" (is skipped for checksum calculation)
// 3 - maximum length of the whole sentence (including the "$" and the "*zz") shall not be bigger that 128 characters
uint16_t bgnet_checksum_nmea0183_calc(char *cNMEA, int iStart, int iLen) {
    uint16_t uCS = 0x00;
    int i;
    for (i = iStart; i < (iStart+iLen); i ++) {
        uCS ^= cNMEA[i];
    }
    return(uCS);
}
char *bgnet_checksum_nmea0183(char *cNMEA) {
    uint16_t uiCS = 0xFF, ucCS;
    int iLen = 0;
    // we need 6 characters to add, make sure our sentence remains within the 128 character limit
    iLen = strlen(cNMEA);
    if ((cNMEA[0] == BGNET_ASC_DOLLAR) & (iLen < 128-6) & (cNMEA[iLen-1] == BGNET_ASC_ASTERIX)) {
        // calculate CS, skip the 1st character ($) and the last character (*)
        uiCS = bgnet_checksum_nmea0183_calc(cNMEA, 1, iLen-1-1);
        // convert the CS (0-0xFF) to 2-bytes HEX/ASCII
        ucCS = bin2hex(uiCS);
        // append the CS characters
        cNMEA[iLen-1+0] = BGNET_ASC_ASTERIX;
        cNMEA[iLen-1+1] = (char)((ucCS & 0xFF00) >> 8);
        cNMEA[iLen-1+2] = (char)(ucCS & 0x00FF);
        //cNMEA[iLen-1+3] = BGNET_ASC_CR;
        cNMEA[iLen-1+3] = BGNET_ASC_LF;
        cNMEA[iLen-1+4] = BGNET_ASC_NULL; // string terminator
    } else {
        // not a valid NMEA sentence
        strcpy(cNMEA, "$GPZZZ,not,a,valid,nmea,string*ZZ");
    }
    return(cNMEA);
}
// return the 'Steer To RIGHT/LEFT character, depending on the value
char *strSteerTo(char *cBuffer, double dValue) {
    if (dValue < 0) {
        cBuffer[0] = BGNET_ASC_R;
        cBuffer[1] = BGNET_ASC_NULL;
    } else {
        cBuffer[0] = BGNET_ASC_L;
        cBuffer[1] = BGNET_ASC_NULL;
    }
    return(cBuffer);
}
// return a string with the NMEA sentence of your choice (single sentence only!)
// we shall only use EXTERNAL data (from NMEA 2000, SignalK or API) as a source for these sentences.
// realise that via PILOT, these same values will end up in the INTERNAL data structures (via DATA
// command send by PILOT). Do not make a circular loop with this data!
//
// FAA mode indicator for:  RMC, RMB, VTG, GLL, BWC, XTE, we always use 'E' (=Estimate) to distingish from other fields using A/V
// GLL was introduced with NMEA 3.0
// we use "II" as "Talker Identifier"
size_t intMakeSentenceNMEA(BGNET_CTRL *pCtrl, char *cBuffer, unsigned char ucMask) {
    char szWork[128];
    switch (ucMask) {
        case BGNET_NMEA0183_APA: // the APA sentence is replaced by the APB; we use the same variables (but just some less variables)
            snprintf(cBuffer, BGNET_NMEA0183_SIZE, "$IIAPA,A,A,%.2f,R,N,V,V,011,T,WP03*", pCtrl->BGNET.PILOT.NMEA.RMB.XTE.value.dValue);
            break;
        case BGNET_NMEA0183_APB: // APB is the 'fixed' version of APA. You should not be using APA
            snprintf(cBuffer, BGNET_NMEA0183_SIZE, "$IIAPB,A,A,%.2f,L,N,A,A,184.2,T,WP04,194.2,T,174.2,T*", pCtrl->BGNET.PILOT.NMEA.RMB.XTE.value.dValue);
            break;
        case BGNET_NMEA0183_VTG: // PILOT only uses SOG [KT] $AGVTG,28.3,T,28.3,M,4.6,N,8.5,K*50
            //snprintf(cBuffer, BGNET_NMEA0183_SIZE, "$IIVTG,,,,,%.1f,N,,,*", pCtrl->BGNET.PILOT.NMEA.VTG.SOG.value.dValue);
            snprintf(cBuffer, BGNET_NMEA0183_SIZE, "$IIVTG,,,,,%.1f,N,,*", g_n2kdata.pos.sog.value.dValue);
            break;
        case BGNET_NMEA0183_RMB:
            snprintf(cBuffer, BGNET_NMEA0183_SIZE, "$IIRMB,A,%.2f,%s,M11,M12,3526.624914,N,13241.976383,E,11.22,276.5,8.6,A,E*",
                                                    fabs(g_n2kdata.pos.xte.value.dValue),  // should be a absolute value
                                                    strSteerTo(szWork, g_n2kdata.pos.xte.value.dValue)
                                                    );
            break;
        case BGNET_NMEA0183_XTE:
            snprintf(cBuffer, BGNET_NMEA0183_SIZE, "$IIXTE,A,A,%.2f,R,N,E*", pCtrl->BGNET.PILOT.NMEA.RMB.XTE.value.dValue);
            break;
        case BGNET_NMEA0183_BWC:
            snprintf(cBuffer, BGNET_NMEA0183_SIZE, "$IIBWC,125444,4917.24,N,02309.57,W,051.9,T,031.6,M,001.3,N,M21,E*");
            break;
        case BGNET_NMEA0183_BWR:
            snprintf(cBuffer, BGNET_NMEA0183_SIZE, "$IIBWR,225444,4917.24,N,01309.57,E,051.9,T,031.6,M,001.3,N,M31,E4*");
            break;
        case BGNET_NMEA0183_VHW: // not documented in the user manual, but it appears to work
            snprintf(cBuffer, BGNET_NMEA0183_SIZE, "$IIVHW,15.8,T,17.3,M,5.0,N,9.3,K*");
            break;
        default:
            // this should never happen
            log_error("Unknown NMEA0183 idetifier used: 0x%02X (should never happen)", ucMask);
            snprintf(cBuffer, BGNET_NMEA0183_SIZE, "$GPERR,unsupported sentence type 0x%02X*", ucMask);
            break;
    }
    bgnet_checksum_nmea0183(cBuffer); // append Checksum to sentence
    //strcpy(cBuffer, "$GNRMB,A,1.56,R,M1,M2,3526.624914,N,13241.976383,E,11.22,276.5,-22.2,*73");
    return(strlen(cBuffer)); // strlen does not include the NULL terminator
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
            cfsetospeed(ptty, B4800);
            break;
        case 9600:
            cfsetispeed(ptty, B9600);
            cfsetospeed(ptty, B9600);
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

static void interface_handler(int fd, size_t *pRx, char *RxBuf, size_t *pTx, char *TxBuf) {
    // get input (if any) into our local buffer. we do not expect RxD data, but we leave
    // this structure intact for other situations
    int nRx = *pRx;
    size_t nTx = *pTx;
    nRx = read(fd, RxBuf, BGNET_NMEA0183_SIZE);
    if (nRx > 0) {
        // we do not expect any RxD data
        if (nRx > 0) {
            // we have received someting. This should not be possible, because our PILOT is readonly (so it won't write to us)
            // do not process incoming data, just reset buffer. Note: if you ever want to re-purpose this handler to handle 
            // incoming NMEA 0183 sentences, this is the place to do that.
            log_error("NMEA0183 serial read: unexpected input %s (%d characters)", RxBuf, nRx);
            nRx = 0; 
        }
    } else if (nRx < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            log_error("NMEA0183 serial read: %s", strerror(errno));
        }
    } else {
        // nothing read, no error, check if there is someting to send
        if (*pTx > 0) {
            // there is someting in the output buffer, write it out
            nTx = write(fd, TxBuf, *pTx);
            if (nTx == *pTx) {
                // nothing wrong here, everything was written
                *pTx = 0; // reset output buffer; data was send
            } else {
                // for some obscure reason we we unable to write all data
                log_error("NMEA0183 serial write: %s (bytes to send= %d, actually send= %d)", strerror(errno), *pTx, nTx);
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
        log_info("NMEA0183 opening port1: %s", pCtrl->pCfg->nmea0183.port1.port);
        fd1 = open(pCtrl->pCfg->nmea0183.port1.port, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd1 < 0) {
            log_error("NMEA0183 serial port1: open(): %s", strerror(errno));
        } else {
            // enable NMEA1083 sentences for this port
            pCtrl->pCfg->nmea0183.port1.enable = set_interface_pilot(&pCtrl->BGNET.PILOT.NMEA.PORT1, pCtrl->pCfg->nmea0183.port1.nav);
            if (pCtrl->pCfg->nmea0183.port1.enable == BGNET_NMEA0183_OFF) {
                // no sentences selected
                log_error("NMEA0183 serial port1: no valid NMEA0183 sentences selected");
                close(fd1);
            } else {
                if (set_interface_attribs(pCtrl->pCfg->nmea0183.port1.enable, pCtrl->pCfg->nmea0183.port1.port, &fd1, &tty1, pCtrl->pCfg->nmea0183.baudrate, pCtrl->pCfg->nmea0183.databits,pCtrl->pCfg->nmea0183.stopbits, pCtrl->pCfg->nmea0183.parity) != 0) {
                    log_error("NMEA0183 serial port1: set_interface_attribs(): %s", strerror(errno));
                    close(fd1);
                    pCtrl->pCfg->nmea0183.port1.enable = BGNET_NMEA0183_OFF;
                } else {
                    log_info("NMEA0183 port1 %s ready, enable= 0x%02X", pCtrl->pCfg->nmea0183.port1.port, pCtrl->pCfg->nmea0183.port1.enable);
                }
            }
        }
    }

    // configure port2
    pCtrl->pCfg->nmea0183.port2.enable = BGNET_NMEA0183_OFF;
    if (strlen(pCtrl->pCfg->nmea0183.port2.port) > 0) {
        log_info("NMEA0183 opening port2: %s", pCtrl->pCfg->nmea0183.port2.port);
        fd2 = open(pCtrl->pCfg->nmea0183.port2.port, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd2 < 0) {
            log_error("NMEA0183 serial port2: open(): %s", strerror(errno));
        } else {
            // enable NMEA1083 sentences for this port
            pCtrl->pCfg->nmea0183.port2.enable = set_interface_pilot(&pCtrl->BGNET.PILOT.NMEA.PORT2, pCtrl->pCfg->nmea0183.port2.nav);
            if (pCtrl->pCfg->nmea0183.port2.enable == BGNET_NMEA0183_OFF) {
                // no sentences selected
                log_error("NMEA0183 serial port2: no valid NMEA0183 sentences selected");
                close(fd2);
            } else {
                if (set_interface_attribs(pCtrl->pCfg->nmea0183.port2.enable, pCtrl->pCfg->nmea0183.port2.port, &fd2, &tty2, pCtrl->pCfg->nmea0183.baudrate, pCtrl->pCfg->nmea0183.databits,pCtrl->pCfg->nmea0183.stopbits, pCtrl->pCfg->nmea0183.parity) != 0) {
                    log_error("NMEA0183 serial port2: set_interface_attribs(): %s", strerror(errno));
                    close(fd2);
                    pCtrl->pCfg->nmea0183.port2.enable = BGNET_NMEA0183_OFF;
                } else {
                    log_info("NMEA0183 port2 %s ready, enable= 0x%02X", pCtrl->pCfg->nmea0183.port2.port, pCtrl->pCfg->nmea0183.port2.enable);
                }
            }
        }
    }

    // if both ports are not configured, we exit the thread. It has no use of continue
    if ((pCtrl->pCfg->nmea0183.port1.enable == BGNET_NMEA0183_OFF) & (pCtrl->pCfg->nmea0183.port2.enable == BGNET_NMEA0183_OFF)) {
        log_error("NMEA0183 serial: no port configured, exit thread");
    } else {

        uint8_t uiSentenceID;
        char NMEA_Rx1[BGNET_NMEA0183_SIZE], NMEA_Tx1[BGNET_NMEA0183_SIZE], NMEA_Rx2[BGNET_NMEA0183_SIZE], NMEA_Tx2[BGNET_NMEA0183_SIZE];
        memset(NMEA_Rx1, 0, BGNET_NMEA0183_SIZE);
        memset(NMEA_Tx1, 0, BGNET_NMEA0183_SIZE);
        memset(NMEA_Rx2, 0, BGNET_NMEA0183_SIZE);
        memset(NMEA_Tx2, 0, BGNET_NMEA0183_SIZE);
        size_t nRx1 = 0, nTx1 = 0, nRx2 = 0, nTx2 = 0;

        while (g_running) {
            if (fd1 == 0) {
                // port not configured
            } else {
                interface_handler(fd1, &nRx1, NMEA_Rx1, &nTx1, NMEA_Tx1);
                if (nTx1 == 0) {
                    uiSentenceID = bgnet_nmea_ctrl_pop(&pCtrl->BGNET.PILOT.NMEA.PORT1, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
                    nTx1 = intMakeSentenceNMEA(pCtrl, NMEA_Tx1, uiSentenceID);
                    log_debug("NMEA port1: %s, sentenceID: 0x%02X, %d characters written", NMEA_Tx1, uiSentenceID, nTx1);
                }
            }
            if (fd2 == 0) {
                // port not configured
            } else {
                interface_handler(fd2, &nRx2, NMEA_Rx2, &nTx2, NMEA_Tx2);
                if (nTx2 == 0) {
                    uiSentenceID = bgnet_nmea_ctrl_pop(&pCtrl->BGNET.PILOT.NMEA.PORT2, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
                    nTx2 = intMakeSentenceNMEA(pCtrl, NMEA_Tx2, uiSentenceID);
                    log_debug("NMEA port2: %s, sentenceID: 0x%02X, %d characters written", NMEA_Tx2, uiSentenceID, nTx2);
                }
            }
            sleep(SEC1); // 2 seconds between sentences
        }
    }
    close(fd1);
    close(fd2);
    log_info("NMEA0183 serial: thread exiting");
    return NULL;
}
