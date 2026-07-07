#ifndef BGNPROTOSUB_H
#define BGNPROTOSUB_H

#include <stdio.h>
#include <time.h>
#include <limits.h>
#include <string.h>
#include <arpa/inet.h>

#include "app_state.h"
#include "config.h"
#include "logger.h"
#include "bgnprotocol.h"

// calculate the checksum over a bytes sequence. Used generate a CR for sending messages, and
// also checking the received CR vs a calculated CR
unsigned char calculate_checksum(unsigned char *buf, unsigned char init, int start, int max) {
    int i;
    unsigned char c = -init;
    for (i=start; i<(start+max); i++) {
        c -= buf[i];
    }
    return (c);
}

// get the index for the devicetype we are looking for
int bgn_FindDeviceIndex(BGNET_CTRL* pCtrl, unsigned char ucDevAddr) {
    int i,r;
    r=0; // no match is not found (return index 0, first device. Lookup is done from index 1)
    for (i=1; i<BGNET_TYPE_MAX; i++) {
        // devicetype in table first match with deviceaddress/type
        if ((pCtrl->pDEVDT[i].iDeviceType & BGNET_TYPE_MASK) == (ucDevAddr & BGNET_TYPE_MASK)) {
            r = i; // first match
            break;
        }
    }
    return(r);
}

void CounterInit(BGNET_COUNT *Counter, time_t t) {
    Counter->iCounter = 0; // 0 means 'never touched'
    Counter->tTimestamp = t;
    Counter->tCounterReset = t;
}
void CounterUp(BGNET_COUNT *Counter, time_t t) {
    Counter->tTimestamp = t;
    // update counter (with wrap-around)
    if (Counter->iCounter < SHRT_MAX) {
        Counter->iCounter++;
    } else {
        Counter->iCounter = 1; // start at 1 because 0 means 'never touched'
        Counter->tCounterReset = t;
    }
}

// a reset is usually done:
// - at startup of this driver
// - when we observe "POWERUP" message(s) from the NETWORK
void bgn_net_initialize_reset(BGNET_CTRL *pCtrl) {
    //pthread_mutex_lock(&g_data_mutex);
    pCtrl->iInitialized = BGNET_CTRL_INIT_RESET; // reset initialized state
    //pthread_mutex_unlock(&g_data_mutex);
}

void bgn_net_initialize(BGNET_CTRL *pCtrl) {
    int i;
    time_t t;

    if (pCtrl->iInitialized != BGNET_CTRL_INIT_RESET){
        // controlblock is already initialized. This means that this initialisation only happens if the 'initialized state' was reset
        // prior this function.
    } else {
        // note: we do NOT iniliaize the pointers to the data structures. These are set upon start and never change

        // note: the counters are initialized at 0, when roll-over set to 1. This way
        // we keep track of devices/commands/channels that were never set; these remain
        // at 0. I know that we are in error with the exact count, but i do not mind.

        time(&t); // save timestamp to we use same timestamp everywhere
        pCtrl->iInitialized = BGNET_CTRL_INIT; // mark as initialized
        pCtrl->iDeviceCounter = BGNET_CTRL_DEVCNTR_INIT;
        pCtrl->iDeviceRun = 0; //reset run counter (0-1000)

        // stage 1 - ACP initialize
        pCtrl->ACP.iNewACP = 0;
        pCtrl->ACP.tTimestamp = t; // time of startup
        pCtrl->ACP.PILOT.value.nLong = BGN_PILOT_SET;
        pCtrl->ACP.PILOT.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.PILOT.Counter, t);

        pCtrl->ACP.STATE.value.nByte = BGN_AP_MODE1_AUTO;
        pCtrl->ACP.STATE.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.STATE.Counter, t);

        pCtrl->ACP.MODE.value.nByte = BGN_AP_MODE2_COMP;
        pCtrl->ACP.MODE.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.MODE.Counter, t);

        pCtrl->ACP.Joystick.value.nByte = BGN_JOYSTICK_OFF; // overrides PILOT control
        pCtrl->ACP.Joystick.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.Joystick.Counter, t);

        pCtrl->ACP.BackLight.value.nByte = 0; // backlight (0=OFF, 3-1=ON)
        pCtrl->ACP.BackLight.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.BackLight.Counter, t);

        pCtrl->ACP.CourseTarget.value.nLong = 123; // Course Target, calculated by the Autopilot (0-359)
        pCtrl->ACP.CourseTarget.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.CourseTarget.Counter, t);

        // we do NOT use a floating point value; otherwise we get all kind or rounding errors. Just keep it as handled within the
        // B&G system: as an integer*10 (so we get 1 decimal)
        pCtrl->ACP.StoredLogNM.value.nLong = 12345*10+6; // channel CD/STORELOGNM (on the real system this is stored in non-volitile memory)
        pCtrl->ACP.StoredLogNM.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.StoredLogNM.Counter, t);

        pCtrl->ACP.TripLogNM.value.nLong = 1234*10+5; // channel CD/STORELOGNM (on the real system this is stored in non-volitile memory)
        pCtrl->ACP.TripLogNM.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.TripLogNM.Counter, t);

        // stage 1 - initialize device list
        pCtrl->iNewDevice = BGN_NEW_RESET;
        for (i=0; i<BGN_DEVICE_MAX; i++) {
            pCtrl->DEV[i].iNewDevice = BGN_NEW_RESET;
            // error counters Checksum
            pCtrl->DEV[i].iCSError_Header = 0;
            pCtrl->DEV[i].iCSError_Frame = 0;
            CounterInit(&pCtrl->DEV[i].CounterRx, t);
            memset(pCtrl->DEV[i].ucFrameRx, 0, BGN_FRAME_SIZE);
            CounterInit(&pCtrl->DEV[i].CounterTx, t);
            memset(pCtrl->DEV[i].ucFrameTx, 0, BGN_FRAME_SIZE);
            pCtrl->DEV[i].source = BGN_SOURCE_NONE;
        }

        // stage 2 - initialize command list
        pCtrl->iNewCommand = BGN_NEW_RESET; // no new data available
        for (i=0; i<BGN_COMMAND_MAX; i++) {
            pCtrl->CMD[i].iNewCommand = BGN_NEW_RESET;
            CounterInit(&pCtrl->CMD[i].CounterRx, t);
            memset(pCtrl->CMD[i].ucFrameRx, 0, BGN_FRAME_SIZE);
            CounterInit(&pCtrl->CMD[i].CounterTx, t);
            memset(pCtrl->CMD[i].ucFrameTx, 0, BGN_FRAME_SIZE);
        }

        // stage 3 - initialize channel list
        pCtrl->iNewChannel = BGN_NEW_RESET; // no new data available
        for (i=0; i<BGN_CHANNEL_MAX; i++) {
            pCtrl->CHN[i].iNewChannel = BGN_NEW_RESET;

            pCtrl->CHN[i].x01.ucFROM = 0;
            memset(pCtrl->CHN[i].x01.ucRAW, 0, sizeof(pCtrl->CHN[i].x01.ucRAW));
            strcpy(pCtrl->CHN[i].x01.cValue, "");
            CounterInit(&pCtrl->CHN[i].x01.CounterRx, t);

            memset(pCtrl->CHN[i].x02.ucRAW, 0, sizeof(pCtrl->CHN[i].x02.ucRAW));
            strcpy(pCtrl->CHN[i].x02.cValue, "");
            CounterInit(&pCtrl->CHN[i].x02.CounterRx, t);

            pCtrl->CHN[i].x06.ucTO = 0x00;
            pCtrl->CHN[i].x06.ucFROM = 0x00;
            memset(pCtrl->CHN[i].x06.ucRAW, 0, sizeof(pCtrl->CHN[i].x06.ucRAW));
            CounterInit(&pCtrl->CHN[i].x06.CounterRx, t);
        }
        log_debug("bgn_net_initialize: Done");
    }
};

// this routine we use to generate a output message and append the payload (if any)
size_t bgn_net_makemessage(unsigned char *pBuf, int iTo, int iFrom, int iCmd, unsigned char *payload, int payloadlength) {
    int i;
    size_t xSize;
    pBuf[BGNET_FRAME_HEAD_TO] = iTo;
    pBuf[BGNET_FRAME_HEAD_FROM] = iFrom;
    pBuf[BGNET_FRAME_HEAD_SIZE] = 0; // assume command without payload
    pBuf[BGNET_FRAME_HEAD_CMD] = iCmd;
    switch (iCmd) {
        case BGNET_FRAME_CMD_POWERUP: // no payload
            if (payloadlength == 0) {
                pBuf[BGNET_FRAME_HEAD_CS] = calculate_checksum(pBuf, BGNET_FRAME_CR_HEAD, 0, BGNET_FRAME_HEAD-1);
                xSize = BGNET_FRAME_HEAD;
            } else {
                xSize = 0;
            }
            break;
        case BGNET_FRAME_CMD_BACKLIGHT: // always 1-byte payload
            if (payloadlength == 1) {
                pBuf[BGNET_FRAME_HEAD_SIZE] = 1; 
                pBuf[BGNET_FRAME_HEAD_CS] = calculate_checksum(pBuf, BGNET_FRAME_CR_HEAD, 0, BGNET_FRAME_HEAD-1);
                pBuf[BGNET_FRAME_HEAD + 0] = payload[0]; // 1-byte payload
                xSize = BGNET_FRAME_HEAD + pBuf[BGNET_FRAME_HEAD_SIZE] + BGNET_FRAME_TAIL_CS;
                pBuf[BGNET_FRAME_HEAD + 1] = calculate_checksum(pBuf, BGNET_FRAME_CR_FRAME, 0, xSize-1);
            } else {
                xSize = 0;
            }
            break;
        case BGNET_FRAME_CMD_DATA:  // variable payload length, payload mandatory
            if (payloadlength == 0) {
                xSize = 0; // always must have some payload
            } else {
                pBuf[BGNET_FRAME_HEAD_SIZE] = payloadlength;
                pBuf[BGNET_FRAME_HEAD_CS] = calculate_checksum(pBuf, BGNET_FRAME_CR_HEAD, 0, BGNET_FRAME_HEAD-1);
                for (i=0; i<payloadlength; i++) {
                    pBuf[BGNET_FRAME_HEAD + i] = payload[i]; // copy whole payload
                }
                xSize = BGNET_FRAME_HEAD + pBuf[BGNET_FRAME_HEAD_SIZE] + BGNET_FRAME_TAIL_CS;
                pBuf[BGNET_FRAME_HEAD + i] = calculate_checksum(pBuf, BGNET_FRAME_CR_FRAME, 0, xSize-1);
            }
            break;
        case BGNET_FRAME_CMD_QUAD_RESET:
        case BGNET_FRAME_CMD_QUAD_RESET_R:
        case BGNET_FRAME_CMD_PILOT_AM:
        case BGNET_FRAME_CMD_PILOT_MODE:
        case BGNET_FRAME_CMD_PILOT_SPEED:
        case BGNET_FRAME_CMD_PILOT_STEP:
            if (payloadlength == 0) {
                xSize = 0; // always must have some payload
            } else {
                pBuf[BGNET_FRAME_HEAD_SIZE] = payloadlength;
                pBuf[BGNET_FRAME_HEAD_CS] = calculate_checksum(pBuf, BGNET_FRAME_CR_HEAD, 0, BGNET_FRAME_HEAD-1);
                for (i=0; i<payloadlength; i++) {
                    pBuf[BGNET_FRAME_HEAD + i] = payload[i]; // copy whole payload
                }
                xSize = BGNET_FRAME_HEAD + pBuf[BGNET_FRAME_HEAD_SIZE] + BGNET_FRAME_TAIL_CS;
                pBuf[BGNET_FRAME_HEAD + i] = calculate_checksum(pBuf, BGNET_FRAME_CR_FRAME, 0, xSize-1);
            }
            break;
        case  BGNET_FRAME_CMD_ALARMOFF: // always 2 bytes payload
            if (payloadlength == 2) {
                pBuf[BGNET_FRAME_HEAD_SIZE] = payloadlength;
                pBuf[BGNET_FRAME_HEAD_CS] = calculate_checksum(pBuf, BGNET_FRAME_CR_HEAD, 0, BGNET_FRAME_HEAD-1);
                for (i=0; i<payloadlength; i++) {
                    pBuf[BGNET_FRAME_HEAD + i] = payload[i]; // copy whole payload
                }
                xSize = BGNET_FRAME_HEAD + pBuf[BGNET_FRAME_HEAD_SIZE] + BGNET_FRAME_TAIL_CS;
                pBuf[BGNET_FRAME_HEAD + i] = calculate_checksum(pBuf, BGNET_FRAME_CR_FRAME, 0, xSize-1);
            } else {
                xSize = 0; // always must have some payload
            }
            break;
        case  BGNET_FRAME_CMD_ACK: // alway 1 byte payload
            if (payloadlength == 1) {
                pBuf[BGNET_FRAME_HEAD_SIZE] = payloadlength;
                pBuf[BGNET_FRAME_HEAD_CS] = calculate_checksum(pBuf, BGNET_FRAME_CR_HEAD, 0, BGNET_FRAME_HEAD-1);
                for (i=0; i<payloadlength; i++) {
                    pBuf[BGNET_FRAME_HEAD + i] = payload[i]; // copy whole payload
                }
                xSize = BGNET_FRAME_HEAD + pBuf[BGNET_FRAME_HEAD_SIZE] + BGNET_FRAME_TAIL_CS;
                pBuf[BGNET_FRAME_HEAD + i] = calculate_checksum(pBuf, BGNET_FRAME_CR_FRAME, 0, xSize-1);
            } else {
                xSize = 0; // always must have some payload
            }
            break;
        default:
            xSize = 0;
            break;
    }
    log_info("bgn_net_makemessage: %ld (bytes) for cmd=%02X, payload= %d", xSize, iCmd, payloadlength);
    return(xSize);
}
// this one usually sends 8 channels
int makeQUADresponse2(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r=0, c;
    union CH* puCH;

    c = BGN_CNR_41_BOATSPEEDKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(125);
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_42_BOATSPEEDRAW;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(120);
    r = r + ChannelByteLength(pCtrl, puCH);

    // the next 3 values are 'rotated' by the QUAD: CD every other time, 64/65 every 4'th time
    // but that is too complex at this moment. Later i will build a rotating scheme.

    c = BGN_CNR_CD_STORLOGNM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uL[0] = htonl(112233);
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_64_SPEEDAVGKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(111); 
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_65_SPEEDMAXKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(222);
    r = r + ChannelByteLength(pCtrl, puCH);

    // the next values are always present 

    c = BGN_CNR_81_DRDNM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(333); 
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_CF_TRIPNM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(55); 
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_D3_DR_COURSE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(-20); 
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_20_SEATEMPQ;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(12); 
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_75_TIMER;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uB[0] = 23; // HH
    puCH->s.cv.uB[1] = 59; // MM
    puCH->s.cv.uB[2] = 59; // SS
    r = r + ChannelByteLength(pCtrl, puCH);

    return(r);
}
// this one usually sends 4 channels
int makeQUADresponse1(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r = 0, c;
    union CH* puCH;

    c = BGN_CNR_31_ECHO_SNR;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uC[0] = '\0'; 
    puCH->s.cv.uC[0] = ' '; 
    puCH->s.cv.uC[0] = 'F'; 
    puCH->s.cv.uC[0] = ' ';
    r = r + ChannelByteLength(pCtrl, puCH);
    
    // the next 3 values are 'rotated' by the QUAD: FE every other time, FC/FD every 4'th time
    // but that is too complex at this moment. Later i will build a rotating scheme.

    c = BGN_CNR_FC_SHALLOWALARM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uC[0] = '\0'; 
    puCH->s.cv.uC[0] = 'O'; 
    puCH->s.cv.uC[0] = 'F'; 
    puCH->s.cv.uC[0] = 'F';
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_FD_DEPTHALARM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(5); 
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_FE_ANCHORALARM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uC[0] = '\0'; 
    puCH->s.cv.uC[0] = 'O'; 
    puCH->s.cv.uC[0] = 'F'; 
    puCH->s.cv.uC[0] = 'F';
    r = r + ChannelByteLength(pCtrl, puCH);

    // the next values are always present 

    c = BGN_CNR_FF_DEPTH_CAL;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(4); 
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_C4_DEPTHM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(6); 
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_C1_DEPTHM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(2); 
    r = r + ChannelByteLength(pCtrl, puCH);

    return(r);
}

int makeACPresponseSTATUS(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r = 0,c;
    union CH* puCH;

    c = BGN_CNR_B5_AP_STATUS;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uB[0] = pCtrl->ACP.STATE.value.nByte; // AUTO/STBY
    puCH->s.cv.uB[1] = pCtrl->ACP.MODE.value.nByte;  // COMP/NAV/VANE
    r = r + ChannelByteLength(pCtrl, puCH);

    if (puCH->s.cv.uB[0] == BGN_AP_MODE1_AUTO) {
        c = BGN_CNR_A6_AP_CTARGET;
        puCH = (union CH*)&(payload[r]);
        puCH->s.CH = c;
        puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
        puCH->s.cv.uW[0] = htons(pCtrl->ACP.CourseTarget.value.nLong);
        r = r + ChannelByteLength(pCtrl, puCH);
    }
    return(r);
}

int makeACPresponseTARGETONLY(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r = 0,c;
    union CH* puCH;

    // when in AUTO, the diplay shows the COURSE TARGET (set of the autopilot)
    c = BGN_CNR_A6_AP_CTARGET;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(pCtrl->ACP.CourseTarget.value.nLong);
    r = r + ChannelByteLength(pCtrl, puCH);

    return(r);
}


int makeACPresponseMINI(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r = 0,c;
    union CH* puCH;

    c = BGN_CNR_B5_AP_STATUS;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uB[0] = pCtrl->ACP.STATE.value.nByte; // AUTO/STBY
    puCH->s.cv.uB[1] = pCtrl->ACP.MODE.value.nByte;  // COMP/NAV/VANE
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_0B_RUDDER;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(-7);
    r = r + ChannelByteLength(pCtrl, puCH);

    // when in STBY, the display shows the HEADING (compass reading)
    c = BGN_CNR_49_HEADING;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(221);
    r = r + ChannelByteLength(pCtrl, puCH);

    // when in AUTO, the diplay shows the COURSE TARGET (set of the autopilot)
    c = BGN_CNR_A6_AP_CTARGET;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(pCtrl->ACP.CourseTarget.value.nLong);
    r = r + ChannelByteLength(pCtrl, puCH);

    return(r);
}

// When the PILOT sends a 0E/KEEPALIVE command to the ACP, and the PILOT does not
// receive a reply, the message "No Pilot" is displayed.
// At minimum, it should receive the AP_STATUS reply (via a DATA command) and
// PILOT come alive. It needs more data to be functional (but that is
// another ballgame)
int makeACPresponseFAST(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r = 0,c;
    union CH* puCH;
    time_t t;
    time(&t);

    c = BGN_CNR_B5_AP_STATUS;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uB[0] = pCtrl->ACP.STATE.value.nByte; // AUTO/STBY
    puCH->s.cv.uB[1] = pCtrl->ACP.MODE.value.nByte;  // COMP/NAV/VANE
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_0B_RUDDER;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(-8+(t&0x01F));
    r = r + ChannelByteLength(pCtrl, puCH);

    // when in STBY, the display shows the HEADING (compass reading)
    c = BGN_CNR_49_HEADING;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(222);
    r = r + ChannelByteLength(pCtrl, puCH);

    // when in AUTO, the diplay shows the COURSE TARGET (set of the autopilot)
    c = BGN_CNR_A6_AP_CTARGET;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(pCtrl->ACP.CourseTarget.value.nLong);
    r = r + ChannelByteLength(pCtrl, puCH);
 
    c = BGN_CNR_51_AWINDANGLE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(100);
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_59_TWINDANGLE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(102);
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_EE_XTE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(-57);
    r = r + ChannelByteLength(pCtrl, puCH);
 
    // depending on the MODE (COMP/NAV/VANE)
    switch(puCH->s.cv.uB[1]) {
        case BGN_AP_MODE2_COMP:
            break;
        case BGN_AP_MODE2_NAV:
            c = BGN_CNR_A7_AP_WPB;
            puCH = (union CH*)&(payload[r]);
            puCH->s.CH = c;
            puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
            puCH->s.cv.sW[0] = htons(123);
            r = r + ChannelByteLength(pCtrl, puCH);

            c = BGN_CNR_E8_WPD;
            puCH = (union CH*)&(payload[r]);
            puCH->s.CH = c;
            puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
            puCH->s.cv.sW[0] = htons(333);
            r = r + ChannelByteLength(pCtrl, puCH);
            break;
        case BGN_AP_MODE2_VANE:
            // AWA signal shall be provided by WIND
            break;      
    }
    switch(puCH->s.cv.uB[0]) {
        case BGN_AP_MODE1_AUTO:
            break;
        case BGN_AP_MODE1_STBY:
            break;
    }
    return (r);
}

// scheduled response from ACP to PILOT (and others)
int makeACPresponse(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r = 0,c;
    union CH* puCH;
    time_t t;
    time(&t);  // we use the timeo create some dynamics

    c = BGN_CNR_0B_RUDDER;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(-8+(t&0x01F));
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_41_BOATSPEEDKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(118);
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_49_HEADING;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(77);
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_51_AWINDANGLE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(-95);
    r = r + ChannelByteLength(pCtrl, puCH);

    return (r);
}

// timer channel (FMT_03, channel 0x75 only)
union CH* encode_value_timer(union CH* pCH, unsigned char ucCH, unsigned char ucFMT, uint8_t HH, uint8_t MM, uint8_t SS) {
    pCH->s.CH = ucCH;
    pCH->s.FMT = ucFMT;
    switch(ucFMT) {
        case BGN_FMT_03:
            pCH->s.cv.uB[0] = HH;
            pCH->s.cv.uB[1] = MM;
            pCH->s.cv.uB[2] = SS;
            break;
    }
    return(pCH);
}
// ascii channel (FMT_04, various channels) please be so kind to specify atleast 3 characters
union CH* encode_value_ascii(union CH* pCH, unsigned char ucCH, unsigned char ucFMT, char* pcText) {
    pCH->s.CH = ucCH;
    pCH->s.FMT = ucFMT;
    switch(ucFMT) {
        case BGN_FMT_04:
            pCH->s.cv.uB[0] = 0;
            pCH->s.cv.uB[1] = pcText[0];
            pCH->s.cv.uB[2] = pcText[1];
            pCH->s.cv.uB[3] = pcText[2];
            break;
    }
    return(pCH);
}

// with host numerical variables (FMT 01, 02, 08, various channels)
union CH* encode_value_double(union CH* pCH, unsigned char ucCH, unsigned char ucFMT, double dV) {
    uint8_t iD;
    uint16_t iW;
    uint32_t iL;
    pCH->s.CH = ucCH;
    if ((ucFMT & BGN_FMT_MASK_DIVPAD) == 0) {
        // do auto-scale, scale it before rounding/converting to int
        if (dV < 1.0) {
            iW = DTOIROUND(dV*1000.0);
            iD = 3;
        } else {
            if (dV < 10.0) {
                iW = DTOIROUND(dV*100.0);
                iD = 2;
            } else {
                if (dV < 100.0) {
                    iW = DTOIROUND(dV*10.0);
                    iD = 1;
                } else {
                    if (dV < 1000.0) {
                        iW = DTOIROUND(dV);
                        iD = 0;
                    } else {
                        iW = 999;
                        iD = 0;
                    }
                }
            }
        }
        // shift divisor bits into FMT byte
        pCH->s.FMT = ucFMT | ((iD<<6) & BGN_FMT_MASK_DIVPAD);
    } else {
        // skip autoscale (value is already scaled)
        pCH->s.FMT = ucFMT;
        iW = DTOIROUND(dV);
        iL = DTOIROUND(dV);
    }
    switch(ucFMT) {
        case BGN_FMT_01: // signed 16bit register
            pCH->s.cv.sW[0] = (int16_t)htons(iW);
            break;
        case BGN_FMT_08: // unsigned 16bit register 
            pCH->s.cv.uW[0] = htons(iW);
            break;
        case BGN_FMT_02: // unsigned 32bit register (only used by 0xCD, STORLOGNM, value *10)
            pCH->s.cv.uL[0] = htonl(iL);
            break;
        default:
            // for every other type we do nothing, we should never come here
    }
    return(pCH);
}

// encode a value to channel format
int encode_channel(BGNET_CTRL *pCtrl, union CH* pCH) {
    // check that CH and FMT were correctly set. Then it's safe to assume that also the
    // data is present and correcly fonrmatted.
    // if CH is not set, or our DataType is different than the default DataType it seems not right
    if ((pCH->s.CH == BGN_CNR_00_NODERESET) | ((pCH->s.FMT & BGN_FMT_MASK_DT) != (pCtrl->pCHNDT[pCH->s.CH].ucFMT & BGN_FMT_MASK_DT))) {
        log_error("encode_channel: Channel 0x%02X seems incorrect", pCH->s.CH);
        pCH->s.FMT = BGN_FMT_04;
        pCH->s.cv.uB[0] = 0;
        pCH->s.cv.uB[1] = 'E';
        pCH->s.cv.uB[2] = 'R';
        pCH->s.cv.uB[3] = 'R';
    }
    return(ChannelByteLength(pCtrl, pCH));
}

int makeSPEEDresponse(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r = 0,c;
    union CH* puCH;
    time_t t;
    time(&t);  // we use the time to create some dynamics

    c = BGN_CNR_46_AP_SPEEDKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(6 + (t&0x07));
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_41_BOATSPEEDKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(16 + (t&0x07));
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_EB_SOGKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(26 + (t&0x07));
    r = r + ChannelByteLength(pCtrl, puCH);

    return (r);
}

#ifdef KOEN
// update timestamp and increment counter (with wrap-around)
void CounterUpDATA(BGN_CHANNEL *pCH, time_t t) {
    pCH->x01.CounterRx.tTimestamp = t;
    // update counter (with wrap-around)
    if (pCH->x01.CounterRx.iCounter < SHRT_MAX) {
        pCH->x01.CounterRx.iCounter++;
    } else {
        pCH->x01.CounterRx.iCounter = 1;
        pCH->x01.CounterRx.tCounterReset = t;
    }
}
void CounterRxUpCMD(BGN_COMMAND *pCMD, time_t t) {
    pCMD->CounterRx.tTimestamp = t;
    // update counter (with wrap-around)
    if (pCMD->CounterRx.iCounter < SHRT_MAX) {
        pCMD->CounterRx.iCounter++;
    } else {
        pCMD->CounterRx.iCounter = 1;
        pCMD->CounterRx.tCounterReset = t;
    }
}
void CounterTxUpCMD(BGN_COMMAND *pCMD, time_t t) {
    pCMD->CounterTx.tTimestamp = t;
    // update counter (with wrap-around)
    if (pCMD->CounterTx.iCounter < SHRT_MAX) {
        pCMD->CounterTx.iCounter++;
    } else {
        pCMD->CounterTx.iCounter = 1;
        pCMD->CounterTx.tCounterReset = t;
    }
}
void CounterRxUpDEV(BGN_DEVICE *pDEV, time_t t) {
    pDEV->CounterRx.tTimestamp = t;
    // update counter (with wrap-around)
    if (pDEV->CounterRx.iCounter < SHRT_MAX) {
        pDEV->CounterRx.iCounter++;
    } else {
        pDEV->CounterRx.iCounter = 1;
        pDEV->CounterRx.tCounterReset = t;
    }
}
void CounterTxUpDEV(BGN_DEVICE *pDEV, time_t t) {
    pDEV->CounterTx.tTimestamp = t;
    // update counter (with wrap-around)
    if (pDEV->CounterTx.iCounter < SHRT_MAX) {
        pDEV->CounterTx.iCounter++;
    } else {
        pDEV->CounterTx.iCounter = 1;
        pDEV->CounterTx.tCounterReset = t;
    }
}
#endif
// This is the "device processor".
// We scan through the data stored by the frame decoder.
// If we want to generate some output for transmission, put the frame in the transmission buffer
// (at this moment we see no need to do such thing)
size_t bgn_net_process(BGNET_CTRL *pCtrl, unsigned char* buffer, size_t size) {
    int i;
    size_t r;
    time_t t;
    time(&t); // save the time

    // erase output buffer (to prevent compiler warnings about unused arguments)
    memset(buffer, 0, size);
    r = 0;

    // device received new data
     if ((pCtrl->iNewCommand == BGN_NEW_RESET)) {
        // nothing to do
     } else {
        log_debug("bgn_net_process: Resetting CMD");
        for (i=0; i<BGN_COMMAND_MAX; i++) {            if (pCtrl->CMD[i].iNewCommand == BGN_NEW_RESET) {
                // nothing to do
            } else {
                log_info("bgn_net_process: Command 0x%02X processing", i);
                switch(i) {
                    case BGNET_FRAME_CMD_PILOT_HELLO:
                    case BGNET_FRAME_CMD_DATA:
                    case BGNET_FRAME_CMD_QUAD_RESET:
                    case BGNET_FRAME_CMD_QUAD_RESET_R:
                    case BGNET_FRAME_CMD_PILOT_AM:
                    case BGNET_FRAME_CMD_PILOT_MODE:
                    case BGNET_FRAME_CMD_PILOT_SPEED:
                    case BGNET_FRAME_CMD_PILOT_STEP:
                        break;
                }
                pCtrl->CMD[i].iNewCommand = BGN_NEW_RESET; // reset new data flag for this command
            }
        }
        pCtrl->iNewCommand = BGN_NEW_RESET;
    }

    if ((pCtrl->iNewChannel == BGN_NEW_RESET) & (1==2)) {
        // nothing to do
    } else {
        log_debug("bgn_net_process: Resetting DATA");
        for (i=0; i<BGN_CHANNEL_MAX; i++) {
            if (pCtrl->CHN[i].iNewChannel == BGN_NEW_RESET) {
                // nothing to do
            } else {
                // process new channel data (to be added later)
                pCtrl->CHN[i].iNewChannel = BGN_NEW_RESET; // reset newdata flag
            }
        }
        pCtrl->iNewChannel = BGN_NEW_RESET; // reset
    }

    if ((pCtrl->iNewDevice == BGN_NEW_RESET)) {
        // nothing to do
    } else {
        log_debug("bgn_net_process: Resetting DEV");
        for (i=0; i<BGN_DEVICE_MAX; i++) {
            if (pCtrl->DEV[i].iNewDevice == BGN_NEW_RESET) {
                // nothing to do
            } else {
                // process new channel data (to be added later)
                pCtrl->DEV[i].iNewDevice = BGN_NEW_RESET; // reset newdata flag
            }
        }
        pCtrl->iNewDevice = BGN_NEW_RESET; // reset

//#define BGN_ENABLE_QUAD_TX
#ifdef BGN_ENABLE_QUAD_TX
        len_pl = makeQUADresponse1(pCtrl, payload);
        len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);

        // put it for transmission
        memcpy(&buffer[r], &tmp_buffer, len_msg); // put output message in write buffer
        r = r + len_msg; // increment byte counter in TX buffer

        // update statistics
        //memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].ucFrameTx, &tmp_buffer, len_msg);
        CounterTxUpCMD(&pCtrl->CMD[BGNnET_FRAME_CMD_DATA], t);

        len_pl = makeQUADresponse2(pCtrl, payload);
        len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
        
        // put it for transmission
        //memcpy(&buffer[r], &tmp_buffer, len_msg); // put output message in write buffer
        r = r + len_msg; // increment byte counter in TX buffer

        // update statistics
        memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].ucFrameTx, &tmp_buffer, len_msg);
        CounterTxUpCMD(&pCtrl->CMD[BGNET_FRAME_CMD_DATA], t);
#endif
    }

    // stage 5 - check if we are allowed to send output
    // check if we are in RW mode; if not, destroy our output. I'd rather do it here that making the logic complexer
    // by integrating the readonly/readwrite mode into the logic
    // realise that readonly does NOT mean 'no output'. The application will send a output message upon request (otherwise
    // the PILOT unit would be stuck in 'No Pilot')
    if (g_serial_mode == BGN_SERIAL_MODE_RW) {
        // leave output data
    } else {
        r = 0; // destroy what we have in the buffer
    }
    // if you want to send data, retrun the number of bytes in the output buffer
    return(r);
};

// returns the length of a channel block; it all depends on the FMT specifier (in the FMT Definition Table).
int ChannelByteLength(BGNET_CTRL* pCtrl, union CH* pCH) {
    int r,i,n;
    unsigned char ucFMT;
    // get FMT byte from target
    ucFMT = pCH->s.FMT;
    // make FMT into DataType, get length from Definition Table
    r = pCtrl->pFMTDT[(ucFMT & BGN_FMT_MASK_DT)].iLen;
    if (r > 0) {
        // fixed length, use it as-is 
    } else {
        if (r == 0) {
            // variable length, look for the null byte
            n = -1;
            for (i=0;i<12;i++) { 
                if (pCH->s.cv.uC[i] == '\0') {
                    n = i;
                    break;
                }
            }
            // n can be anything between 0-11 when a null-terminator was found
            if (n < 0) {
                // null-terminator was not found
                r = 4; // just a guess
                log_error("FMT specifier 0x%02X, invalid length %d, no terminator found", ucFMT & BGN_FMT_MASK_DT, r);
            } else {
                r = 2 + n + 1; // 2 bytes header, n characters, 1 null-terminator
            }
        } else {
            // is the value is unknown, i ju*(pBuf + 3)st make a guess. 4 is better than zero or -1
            r = 4; // just a guess
            log_error("FMT specifier 0x%02X, invalid length %d, unknown datatype", ucFMT & BGN_FMT_MASK_DT, r);
        }
    }
    // note: this return is used to advance though a frame/subframe. if this value is not positive,
    // we will never complete the parsing (and create chaos).
    return(r);
}

// returns the length of a frame block
// some commands have no payload, just the header. Others do have a payload (with a extra CS).
int CommandByteLength(unsigned char* ucFrame) {
    int r;
    if (ucFrame[BGNET_FRAME_HEAD_SIZE] == 0) {
        r = BGNET_FRAME_HEAD;
    } else {
        r = BGNET_FRAME_HEAD + ucFrame[BGNET_FRAME_HEAD_SIZE] + BGNET_FRAME_TAIL_CS;
    }
    return(r);
}

// copy command frame into a ascii buffer (for display purposes). It's used for logging purposes, 
// but also to paint frames into the webpages
// it converts to a string like "0F00AC", "0F 00 AC" or "0x0F 0x00 0x0AC". Realise that the produced
// string my be much longer that the original byte sequence.
char* CommandToAscii(char* asciibuf, size_t size, unsigned char* ucFrame) {
    int n, y;
    size_t FormatSpecifier = BYTE2ASCII_SPACED;
    char cWork[BGN_CMD_ASCII];
    cWork[0] = '\0'; // string terminator at the beginning
    if (ucFrame[BGNET_FRAME_HEAD_CMD] == 0) {
        // invalid command, buffer must be empty
        strcpy(asciibuf,"---");
    } else {
        y = CommandByteLength(ucFrame);
        if (y*FormatSpecifier < size) {
            // it fits, just copy it all
            byte2ascii(asciibuf, size, ucFrame, y, FormatSpecifier);
        } else {
            // we use a 7-character separation string: " /.../ " to avoid excessive long output
            // we also copy the first 10 (5 header with CS + 5 first data) and the tail (5 last data + CS = 6 bytes)
            // 6 bytes tail = 18 ascii characters
            n = (BGN_CMD_ASCII - strlen(BGN_CMD_ASCII_SEP))/FormatSpecifier; // we have ascii space for 'n' databytes
            byte2ascii(asciibuf, size, ucFrame, n-BGN_CMD_DATA_TAIL, FormatSpecifier);
            strcat(asciibuf, BGN_CMD_ASCII_SEP); // append separator
            // local buffer cWork is only used fot the tail section. If we keep it short (6 bytes, max 12/18/30 characters) we'll never get a buffer overrun
            byte2ascii(cWork, sizeof(cWork), &ucFrame[y-BGN_CMD_DATA_TAIL], BGN_CMD_DATA_TAIL, FormatSpecifier);
            // append the tail
            strcat(asciibuf, cWork);
        }
    }
    // return a null terminated ascii string
    return(asciibuf);
}

// returns a string representation of a channel value
// numeric: do not call this against non-numeric FMT's. Returns a numeric you can use in calculations
// string: use this to display values (ascii, numeric, time, whatever)
int decode_channel_numeric(union CH* puCH) {
    int iRawValue;
    switch(puCH->s.FMT & BGN_FMT_MASK_DT) {
        case BGN_FMT_01: // signed 16bit register (-999 to +999)
            iRawValue = (int16_t)ntohs(puCH->s.cv.sW[0]);
            break;
        case BGN_FMT_02: // unsigned 32bit register (0xCD, storedlog)
            iRawValue = ntohl(puCH->s.cv.uL[0]);
            break;
        case BGN_FMT_08: // unsigned 16bit register
            iRawValue = ntohs(puCH->s.cv.uW[0]);
            break;
        default:
            iRawValue = -0;
            break;
    }
    return(iRawValue);
}
char* decode_channel_string(char* pStr, union CH* puCH) {
    int iRawValue;
    int iValid; // 0= invalid DataType, set to "???"
                // 1= Numerical, convert RawValue byte sequence (with optional scaling)
                // 2= ASCII, already set (text, timer, hex)
    unsigned int d;
    const double fDiv[4]    = { 1.0,   10.0,  100.0, 1000.0};
    const char cPrint[4][6] = {"%d", "%.1f", "%.2f", "%.3f"};
    switch(puCH->s.FMT & BGN_FMT_MASK_DT) {
        case BGN_FMT_00: // single character
            strcpy(pStr,"{null}");
            iValid = 2;
            break;
        case BGN_FMT_01: // signed 16bit register (-999 to +999)
            iRawValue = decode_channel_numeric(puCH);
            iValid = 1;
            break;
        case BGN_FMT_02: // unsigned 32bit register (0xCD, storedlog)
            iRawValue = decode_channel_numeric(puCH);
            iValid = 1;
            break;
        case BGN_FMT_03: // 3 bytes timer value (0x75, timer)
                         // note: QUAD displays H:MM:SS maximum (left digit is not displayed)
                         //       DATA displays H:MM, SS is displayed in the topline
            sprintf(pStr,"%02d:%02d:%02d", puCH->s.cv.uB[0], puCH->s.cv.uB[1], puCH->s.cv.uB[2]);
            iValid = 2;
            break;
        case BGN_FMT_04: // 3 bytes ASCII, byte 0 is unused
            pStr[0] = puCH->s.cv.uB[1];
            pStr[1] = puCH->s.cv.uB[2];
            pStr[2] = puCH->s.cv.uB[3];
            pStr[3] = '\0';
            iValid = 2;
            break;
        case BGN_FMT_07: // variable length ASCII string (null-terminated)
            strcpy(pStr, puCH->s.cv.uC);
            iValid = 2;
            break;
        case BGN_FMT_08: // unsigned 16bit register
            if (puCH->s.CH == BGN_CNR_B5_AP_STATUS) {
                sprintf(pStr,"0x%02X 0x%02X", puCH->s.cv.uB[0], puCH->s.cv.uB[1]);
                iValid = 2;
            } else {
                iRawValue = decode_channel_numeric(puCH);
                iValid = 1;
            }
            break;
        case BGN_FMT_0C: // 3 bytes ASCII, byte 0 is unused
            pStr[0] = puCH->s.cv.uB[0];
            pStr[1] = puCH->s.cv.uB[1];
            pStr[2] = puCH->s.cv.uB[2];
            pStr[3] = '\0';
            iValid = 2;
            break;
        default:
            pStr[0] = '?';
            pStr[1] = '?';
            pStr[2] = '?';
            pStr[3] = '\0';
            iValid = 0;
    }
    if (iValid == 1) {
        // get divisor value (0-3) from FMT byte
        d = (puCH->s.FMT & BGN_FMT_MASK_DIVPAD_DIV)>>6;
        switch (d) {
            case 1:  // scaled value is printed with a decimals
            case 2:
            case 3:
                // convert to a real value, apply divisor (1.0-1000.0)
                sprintf(pStr, cPrint[d], ITOD(iRawValue)/fDiv[d]);
                break;
            default: // non-scaled value is printed a integer
                sprintf(pStr, cPrint[d], iRawValue);
                break;
        }
    }
    return(pStr);
}

// routine to process values in the body. A body might contain multiple values
int decode_command_DATA(BGNET_CTRL* pCtrl, unsigned char *buffer) {
    int y,z;
    unsigned char *pBuf,*pEnd;
    union CH* pCH;
    time_t t;  // used to timestamp incoming channel

    pBuf = buffer+BGNET_FRAME_HEAD;
    pEnd = buffer+BGNET_FRAME_HEAD + buffer[BGNET_FRAME_HEAD_SIZE];
    z = 0; // number of channels in this command
    time(&t);
    // scan through frame/subframe
    while (pBuf < pEnd) {
        pCH = (union CH*)(pBuf + BGN_VALUE_CNR);
        pCtrl->CHN[pCH->s.CH].x01.ucFROM = *(buffer + BGNET_FRAME_HEAD_FROM); // originator of the frame (that included this channel)

        // we need the channel length to advance to the next in this frame
        y = ChannelByteLength(pCtrl, pCH);

        // check if existing channel (stored previous time) and the new channel (in the received frame) are equal
        if (memcpy(((union CH*)pCtrl->CHN[pCH->s.CH].x01.ucRAW), pCH, y) == 0) {
            // no chnage, do no process this channel
        } else {

            // decode channel, get displayed value (this is what we use in the webserver)
            decode_channel_string(pCtrl->CHN[pCH->s.CH].x01.cValue, pCH);
            // copy channel byte sequence (2 purposes: to display via the webserver and to compare new/present data)
            memcpy(pCtrl->CHN[pCH->s.CH].x01.ucRAW, pCH, y);

            pCtrl->CHN[pCH->s.CH].iNewChannel = BGN_NEW_SET; // signal this channel as new/changed
            CounterUp(&pCtrl->CHN[pCH->s.CH].x01.CounterRx, t);

            // sometimes we have to do something when some data comes in
            switch(pCH->s.CH) {
                case BGN_CNR_B5_AP_STATUS:
                    pCtrl->ACP.STATE.value.nByte = (pCH->s.cv.uB[0] & BGN_AP_MODE1_MASK_STATE);  // get AUTO/STBY state into ACP
                    pCtrl->ACP.STATE.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->ACP.STATE.Counter, t);

                    pCtrl->ACP.MODE.value.nByte  = (pCH->s.cv.uB[1] & BGN_AP_MODE2_MASK_MODE);   // get COMP/NAV/VANE mode into ACP
                    pCtrl->ACP.MODE.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->ACP.MODE.Counter, t);
                    break;
                case BGN_CNR_CD_STORLOGNM:
                    pCtrl->ACP.StoredLogNM.value.nLong = ntohl(pCH->s.cv.uL[0]);
                    pCtrl->ACP.StoredLogNM.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->ACP.StoredLogNM.Counter, t);
                    break;
                case BGN_CNR_81_DRDNM:
                    pCtrl->ACP.TripLogNM.value.nLong = ntohs(pCH->s.cv.uW[0]);
                    pCtrl->ACP.TripLogNM.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->ACP.TripLogNM.Counter, t);
                    break;
                case BGN_CNR_A6_AP_CTARGET:
                    pCtrl->ACP.CourseTarget.value.nLong = ntohs(pCH->s.cv.uW[0]);
                    pCtrl->ACP.CourseTarget.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->ACP.CourseTarget.Counter, t);
                    break;
            }
            if (pCtrl->pCHNDT[y].iImpExpEnable) {

            }
            switch(pCtrl->pCHNDT[pCH->s.CH].iImpExpEnable) {
                case BGN_CNR_IMPEXP_NONE:
                    // value not designed to be import/exportable
                    break;
                case BGN_CNR_IMPEXP_NMEA:
                    // because we are here processing channels from the BGN interface, we are going to export it
                    // (importing does not make sense because we already have it)
                    // if (later) NMEA has this data incoming, it will overwrite it (and we will not write it again)
                    // we do not need to lock/unlock it, as NMEA already does that and will cancel us out anyway
                    switch(pCH->s.CH) {
                        case BGN_CNR_0B_RUDDER:
                            if ((g_n2kdata.boat.rudder.source == BGN_SOURCE_NMEA) | (g_n2kdata.boat.rudder.source == BGN_SOURCE_SIGNALK)) {
                                // is was already written by NMEA/SIGNALK interface; do not overwrite
                            } else {
                                // BGN sends rudder position in angle/degrees (signed, -45/+45). NMEA stores rudder
                                // position as radian. Multiply by PI/180 to get radians
                                g_n2kdata.boat.rudder.value.dValue = ITOD(ntohs(pCH->s.cv.sW[0])) * BGNET_PI/180;
                                g_n2kdata.boat.rudder.source = BGN_SOURCE_BGNET;
                                CounterUp(&g_n2kdata.boat.rudder.Counter, t);
                            }
                            break;
                    }
                    break;
                case BGN_CNR_IMPEXP_SIGNALK:
                    // because we are here processing channels from the BGN interface, we are going to export it
                    // (importing does not make sense because we already have it)
                    // if (later) SIGNAL has this data incoming, it will overwrite it (and we will not write it again)
                    // we do not need to lock/unlock it, as SIGNAL already does that and will cancel us out anyway
                    switch(pCH->s.CH) {
                        case BGN_CNR_0B_RUDDER:
                            if ((g_n2kdata.boat.rudder.source == BGN_SOURCE_NMEA) | (g_n2kdata.boat.rudder.source == BGN_SOURCE_SIGNALK)) {
                                // is was already written by NMEA/SIGNAL interface; do not overwrite
                            } else {
                                // BGN sends rudder position in angle/degrees (signed, -45/+45). NMEA stores rudder
                                // position as radian. Multiply by PI/180 to get radians
                                g_n2kdata.boat.rudder.value.dValue = ITOD(ntohs(pCH->s.cv.sW[0])) * BGNET_PI/180;
                                g_n2kdata.boat.rudder.source = BGN_SOURCE_BGNET;
                                CounterUp(&g_n2kdata.boat.rudder.Counter, t);
                            }
                            break;
                    }
                    break;
                    break;
            }

            log_debug("Channel #%d, ID=0x%02X, FMT=0x%02X, label=%s, value=%s", z
                                                                        , pCH->s.CH
                                                                        , pCH->s.FMT
                                                                        , pCtrl->pCHNDT[pCH->s.CH].cLabel
                                                                        , pCtrl->CHN[pCH->s.CH].x01.cValue
                                                                    );
        }
        pBuf = pBuf + y; // advance to next channel in this frame
        z++; // count channels in this frame
    }
    return(z); // return number of channels
}
// routine to process values in the body. A body might contain multiple values
void decode_command_SUBSCRIBE(BGNET_CTRL* pCtrl, unsigned char *buffer) {
    unsigned char ucCHN;
    time_t t;
    time(&t);
    // this type of frame shall have a payload of 1 byte
    if (buffer[BGNET_FRAME_HEAD_SIZE] == 1) {
        ucCHN = buffer[BGNET_FRAME_HEAD + 0]; // first payoad byte
        pCtrl->CHN[ucCHN].x06.ucTO = buffer[BGNET_FRAME_HEAD_TO];
        pCtrl->CHN[ucCHN].x06.ucFROM = buffer[BGNET_FRAME_HEAD_FROM];
        CounterUp(&pCtrl->CHN[ucCHN].x06.CounterRx, t);
        memcpy(pCtrl->CHN[ucCHN].x06.ucRAW, buffer, (BGNET_FRAME_HEAD + buffer[BGNET_FRAME_HEAD_SIZE] + BGNET_FRAME_TAIL_CS));
        log_debug("    Ch=%02X, Subscribed by %02X on %02X", ucCHN, buffer[BGNET_FRAME_HEAD_FROM], buffer[BGNET_FRAME_HEAD_TO]);
    }
    return;
}
// routine to advertise values that can be subscribed
void decode_command_LABEL(BGNET_CTRL* pCtrl, unsigned char *buffer) {
    unsigned char string[255];
    unsigned char ucCHN;
    size_t len;
    time_t t;
    time(&t);

    // this type of frame has a variable payload: Ch#, 3 bytes data, nbytes ascii
    ucCHN = buffer[BGNET_FRAME_HEAD + 0]; // first payload byte is channel ID
    pCtrl->CHN[ucCHN].iNewChannel = BGN_NEW_SET;
    if ((buffer[BGNET_FRAME_HEAD_SIZE] > 4) & (buffer[BGNET_FRAME_HEAD_SIZE] < sizeof(pCtrl->CHN[ucCHN].x02.ucRAW))) {
        pCtrl->CHN[ucCHN].x02.CounterRx.iCounter++;
        pCtrl->CHN[ucCHN].x02.CounterRx.tTimestamp = t;
        // copy the frame
        len = BGNET_FRAME_HEAD + buffer[BGNET_FRAME_HEAD_SIZE] + BGNET_FRAME_TAIL_CS;
        memcpy(pCtrl->CHN[ucCHN].x02.ucRAW, buffer, len);
        // copy the label
        len = buffer[BGNET_FRAME_HEAD_SIZE] - 4;
        strncpy(pCtrl->CHN[ucCHN].x02.cValue, (char*)&(pCtrl->CHN[ucCHN].x02.ucRAW[BGNET_FRAME_HEAD + 4]), len);
        pCtrl->CHN[ucCHN].x02.cValue[len] = '\0';

        log_debug("  1st=%02X, len= %ld", buffer[BGNET_FRAME_HEAD+4], len);
        memcpy(string, &buffer[BGNET_FRAME_HEAD+4], len);
        string[len] = 0x00;
        log_debug("    Ch=%02X, advertised by %02X to %02X, value= %s", buffer[BGNET_FRAME_HEAD+0], buffer[BGNET_FRAME_HEAD_FROM], buffer[BGNET_FRAME_HEAD_TO], string);
    } else {
        strcpy(pCtrl->CHN[ucCHN].x02.cValue, "???");
    }
    return;
}

// decode the received buffer, possibly multiple subframes, return number of sub-frames
size_t decode_frame(BGNET_CTRL* pCtrl, unsigned char *buffer, size_t len) {
    unsigned char cs; // checksum calculated
    unsigned char *pBuf, *pEnd; // pointer to start and end of buffer
    int RxSize, x, z, n;
    int iIsValid, iValue;
    unsigned char ucCMD, ucFROM;
    time_t t;
    unsigned char payload[1024], TxBuf[1024];
    int len_pl, len_msg, TxSize;
    union CH *pCH;

    pBuf = buffer; // gets incremented as we go
    pEnd = buffer+len; // end of buffer
    z = 0; // subframe counter

    // process multiple subframes in our buffer
    while (pBuf < pEnd) {
        // update the time
        time(&t);
        // initialize some variables
        TxSize = 0;
        len_pl = 0;
        len_msg = 0;
        iIsValid = 1; // tricky: if we hit a 'default' in a switch we reset this, so statistics is not updated
        // the header is fixed size
        ucFROM = pBuf[BGNET_FRAME_HEAD_FROM];
        cs = calculate_checksum(pBuf, BGNET_FRAME_CR_HEAD, BGNET_FRAME_HEAD_TO, BGNET_FRAME_HEAD-1); // calculate the checksum over the header (without the CR)
        if (*(pBuf+BGNET_FRAME_HEAD_CS) == cs) {
            // check if this subframe is adressed TO us and not FROM us
            if (ucFROM == BGNET_ADDRESS_ME) {
                RxSize = pEnd - pBuf; // dump the remainder of the frame; it's a readback of my own transmission
            } else {
                // register FROM as 'active' device on the BGN interface (to prevent other interface writing conflicting channel data)
                pCtrl->DEV[ucFROM].source = BGN_SOURCE_BGNET;
                // update DEV statistics
                CounterUp(&pCtrl->DEV[ucFROM].CounterRx, t);
                // The HEADER is OK, process it. Note that some CMD's do not have a body and in such case, we should skip the body & CS part.
                // update DEVICE structure (statistics)

                x = bgn_FindDeviceIndex(pCtrl, *(pBuf+BGNET_FRAME_HEAD_FROM)); // lookup by device address/type, so we can print the device name                          
                ucCMD = *(pBuf + BGNET_FRAME_HEAD_CMD);
                if (*(pBuf+BGNET_FRAME_HEAD_SIZE) == 0) {  // No payload (no frame CS)
                    RxSize = BGNET_FRAME_HEAD;
                    switch(ucCMD) {
                        case BGNET_FRAME_CMD_PILOT_HELLO:
                            // when we receive this message, we know that there is atleast 1 active PILOT on the network.
                            // We set the counter; timer thread counts it down and if this message is no longer received
                            // we (the ACP) know that there is no longer user-input possible because no PILOT is connected.
                            log_info("  Command %02X (header only) KEEPALIVE by %02X %s", ucCMD, ucFROM, pCtrl->pDEVDT[x].sModelName);

                            // set the counter (count down done by timer thread)
                            pthread_mutex_lock(&g_data_mutex);
                            pCtrl->ACP.PILOT.value.nLong = BGN_PILOT_SET;
                            pCtrl->ACP.PILOT.source = BGN_SOURCE_BGNET;
                            CounterUp(&pCtrl->ACP.PILOT.Counter, t);
                            pthread_mutex_unlock(&g_data_mutex);

                            len_pl = makeACPresponseMINI(pCtrl, payload);
                            len_msg = bgn_net_makemessage(&TxBuf[TxSize], BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                            memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].ucFrameTx, &TxBuf[TxSize], len_msg);
                            CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].CounterTx, t);
                            memcpy(pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                            CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                            TxSize = TxSize + len_msg;

                            // put it for transmission
                            pthread_mutex_lock(&g_data_mutex);
                            memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &TxBuf, TxSize); // put output message in write buffer
                            g_serial_data_lenTx = g_serial_data_lenTx + TxSize; // increment byte counter in TX buffer
                            pthread_mutex_unlock(&g_data_mutex);

                            break;
                        case BGNET_FRAME_CMD_POWERUP:
                            // when we receive this message one (or more) devices whent though the startup process (ie. power was
                            // switch ON while we (the ACP) were still running.
                            // At this moment i see no reason to do something special at this occasion. Realise that most networks
                            // contain multiple units. You might expect a burst of these messages. Some units (like WIND) take some
                            // more time then others. Also units that are normally passive (not broadcasting channels, like DATA)
                            // will send this message.
                            log_info("  Command %02X (header only) POWERUP by %02X %s", ucCMD, ucFROM, pCtrl->pDEVDT[x].sModelName);
                            bgn_net_initialize_reset(pCtrl);
                            break;
                        default:
                            iIsValid = 0; // do not update device/command statistics
                            x = bgn_FindDeviceIndex(pCtrl, *(pBuf+BGNET_FRAME_HEAD_FROM)); // lookup by device address/type, so we can print the device name
                            log_info("  Command %02X (header only) IGNORED (from %02X %s)", *(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName);
                            break;
                    }
                    if (iIsValid > 0) {
                        // update DEV & CMD structure
                        pCtrl->iNewDevice = BGN_NEW_SET; // signal this device
                        pCtrl->DEV[ucFROM].iNewDevice = BGN_NEW_SET;
                        memcpy(pCtrl->DEV[ucFROM].ucFrameRx, pBuf, RxSize);
                        CounterUp(&pCtrl->DEV[ucFROM].CounterRx, t);

                        pCtrl->iNewCommand = BGN_NEW_SET; // signal possible CMD change
                        pCtrl->CMD[ucCMD].iNewCommand = BGN_NEW_SET; // signal this command
                        memcpy(pCtrl->CMD[ucCMD].ucFrameRx, pBuf, RxSize);
                        CounterUp(&pCtrl->CMD[ucCMD].CounterRx, t);
                    }
                } else {
                    RxSize = BGNET_FRAME_HEAD + *(pBuf+BGNET_FRAME_HEAD_SIZE) + BGNET_FRAME_TAIL_CS; // expected framesize = head (including CR), body, and frame CR
                    //iIsValid = 1;
                    cs = calculate_checksum(pBuf, BGNET_FRAME_CR_FRAME, BGNET_FRAME_HEAD_TO, RxSize-1);
                    if (*(pBuf + RxSize-1) == cs) { 
                        //iIsValid = 1;
                        switch(ucCMD) {
                            case BGNET_FRAME_CMD_DATA:
                                // process DATA command (broadcast)
                                n = decode_command_DATA(pCtrl, pBuf);
                                log_info("  Command %02X (with payload) DATA, %d channels", ucCMD, n);
                                break;
                            case BGNET_FRAME_CMD_LABEL:
                                // process message (only at startup, only by DATA)
                                decode_command_LABEL(pCtrl, pBuf);
                                log_info("  Command %02X (with payload) LABEL", ucCMD);
                                break;
                            case BGNET_FRAME_CMD_SUBSCRIBE:
                                // process message (only at startup, only by DATA)
                                decode_command_SUBSCRIBE(pCtrl, pBuf);
                                log_info("  Command %02X (with payload) SUBSCRIBE", ucCMD);
                                break;
                            case BGNET_FRAME_CMD_BACKLIGHT:
                                // some station, LIGHTS button was pressed; 1 byte payload (intensity 0-3)
                                pCtrl->ACP.BackLight.value.nByte = *(pBuf+BGNET_FRAME_HEAD+0); // first byte is the LIGHTS state
                                pCtrl->ACP.BackLight.source = BGN_SOURCE_BGNET;
                                CounterUp(&g_n2kdata.boat.rudder.Counter, t);

                                log_info("  Command %02X (with payload) BACKLIGHT (BY: %02X, %s, LEVEL: %02X)",
                                                    *(pBuf+BGNET_FRAME_HEAD_CMD),
                                                    ucFROM,
                                                    pCtrl->pDEVDT[x].sModelName,
                                                    pCtrl->ACP.BackLight.value.nByte);
                                break;
                            case BGNET_FRAME_CMD_QUAD_RESET:
                            case BGNET_FRAME_CMD_QUAD_RESET_R:
                                // this command is send by QUAD if a reset (TRIPLOGNM, DR_COURSE/DR_DIST) must be performed
                                // still to be implemented.
                                // TRIPLOG is maintained by SPEED or QUAD unit, not in ACP, so we do nothing
                                break;
                            case BGNET_FRAME_CMD_PILOT_AM:
                                // set AUTO/STBY state. pressing the AUTO while in AUTO, no message is send by PILOT.
                                // The buttons AUTO and OFF generate the same message and effectively the buttons are a toggle;
                                // message has 5 bytes payload:
                                // 0 = ACK sequence number (to be returned vua ACK message)
                                // 1 = 0xB5/AP_STATUS, channel number
                                // 2 = 0x01 - FMT
                                // 3 = 0x00 - 00
                                // 4 = 0x01 - 01
                                log_info("  Command %02X AUTO/OFF BUTTON PRESSED (BY: %02X, %s, state= %d)", *(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName, *(pBuf+BGNET_FRAME_HEAD+4));
                                // handle the contents of the COMMAND
                                switch(*(pBuf+BGNET_FRAME_HEAD+1)) {
                                    case BGN_CNR_B5_AP_STATUS:
                                        // we go from AUTO to STBY or VV
                                        if (pCtrl->ACP.STATE.value.nByte == BGN_AP_MODE1_STBY) {
                                            // our current state is STBY; this means we go from STBY to AUTO
                                            // In AUTO, the CTARGET is used by the AP to steer. To prevent a
                                            // drastic manouvre when switching to AUTO, we initialize CTARGET
                                            // against our current heading.
                                            // After switching to AUTO, the AP can (slowly) get the boat on our
                                            // desired course.
                                            pCtrl->ACP.CourseTarget.value.nLong = ntohs(((union CH*)(pCtrl->CHN[BGN_CNR_49_HEADING].x01.ucRAW))->s.cv.uW[0]);
                                            pCtrl->ACP.CourseTarget.source = BGN_SOURCE_ACP;
                                            CounterUp(&pCtrl->ACP.CourseTarget.Counter, t);
                                        }

                                        pCtrl->ACP.STATE.value.nByte = (1 - pCtrl->ACP.STATE.value.nByte);
                                        pCtrl->ACP.STATE.source = BGN_SOURCE_BGNET;
                                        CounterUp(&pCtrl->ACP.STATE.Counter, t);
                                        break;
                                }

                                // create ACK response; the 1st byte contains the byte to reply with
                                payload[0] = *(pBuf + BGNET_FRAME_HEAD + 0);
                                len_pl = 1;
                                //msg_offset = 0;

                                // make ACK frame, update statistics
                                len_msg = bgn_net_makemessage(&TxBuf[TxSize], *(pBuf + BGNET_FRAME_HEAD_FROM), BGNET_ADDRESS_ME, BGNET_FRAME_CMD_ACK, payload, len_pl);
                                memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_ACK].ucFrameTx, &TxBuf[TxSize], len_msg);
                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_ACK].CounterTx, t);
                                memcpy(&pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                TxSize = TxSize + len_msg;

                                // make DATA command
                                len_pl = makeACPresponseSTATUS(pCtrl, payload);
                                // make DATA frame, update statistics
                                len_msg = bgn_net_makemessage(&TxBuf[TxSize], BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                                memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].ucFrameTx, &TxBuf[TxSize], len_msg);
                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].CounterTx, t);
                                memcpy(&pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                TxSize =TxSize + len_msg;

                                // put it for transmission
                                pthread_mutex_lock(&g_data_mutex);
                                memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &TxBuf, TxSize); // put output message in write buffer
                                g_serial_data_lenTx = g_serial_data_lenTx + TxSize; // increment byte counter in TX buffer
                                pthread_mutex_unlock(&g_data_mutex);

                                break;
                            case BGNET_FRAME_CMD_PILOT_MODE:
                                // set CONTROL MODE (COMP/NAV/VANE). pressing the MODE will send this message (in AUTO or STBY)
                                // This button advances from one the the next.
                                // message has 5 bytes payload:
                                // 0 = ACK sequence number (to be returned vua ACK message)
                                // 1 = 0xA6/AP_CTARGET channel number
                                // 2 = 0x01 - ??
                                // 3 = 0x00 - ??
                                // 4 = 0x03 - ??
                                log_info("  Command %02X MODE BUTTON PRESSED (BY: %02X, %s, state= %d)", *(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName, *(pBuf+BGNET_FRAME_HEAD+4));
                                if (pCtrl->ACP.Joystick.value.nByte == BGN_JOYSTICK_OFF) {
                                    // only toggle is JOYSTICK is OFF
                                    switch (pCtrl->ACP.MODE.value.nByte) {
                                        case BGN_AP_MODE2_VANE:
                                            pCtrl->ACP.MODE.value.nByte = BGN_AP_MODE2_NAV;
                                            break;
                                        case BGN_AP_MODE2_NAV:
                                            pCtrl->ACP.MODE.value.nByte = BGN_AP_MODE2_COMP;
                                            break;
                                        case BGN_AP_MODE2_COMP:
                                            pCtrl->ACP.MODE.value.nByte = BGN_AP_MODE2_VANE;
                                            break;
                                        default:
                                            pCtrl->ACP.MODE.value.nByte = BGN_AP_MODE2_COMP;
                                            break;
                                    }
                                    pCtrl->ACP.MODE.source = BGN_SOURCE_BGNET;
                                    CounterUp(&pCtrl->ACP.MODE.Counter, t);
                                } else {
                                    // Joystick engaged, disengage it
                                    pCtrl->ACP.Joystick.value.nByte = BGN_JOYSTICK_OFF;
                                    pCtrl->ACP.Joystick.source = BGN_SOURCE_BGNET;
                                    CounterUp(&pCtrl->ACP.Joystick.Counter, t);
                                }

                                // create ACK response
                                payload[0] = *(pBuf + BGNET_FRAME_HEAD + 0); // get 1st payload byte and return it
                                len_pl = 1;
                                //msg_offset = 0;

                                // make ACK frame, update statistics
                                len_msg = bgn_net_makemessage(&TxBuf[TxSize], *(pBuf + BGNET_FRAME_HEAD_FROM), BGNET_ADDRESS_ME, BGNET_FRAME_CMD_ACK, payload, len_pl);
                                memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_ACK].ucFrameTx, &TxBuf[TxSize], len_msg);
                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_ACK].CounterTx, t);
                                memcpy(&pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                TxSize = TxSize + len_msg;

                                // make DATA command
                                len_pl = makeACPresponseSTATUS(pCtrl, payload);
                                // make DATA frame, update statistics
                                len_msg = bgn_net_makemessage(&TxBuf[TxSize], BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                                memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].ucFrameTx, &TxBuf[TxSize], len_msg);
                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].CounterTx, t);
                                memcpy(&pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                TxSize = TxSize + len_msg;

                                // put it for transmission
                                pthread_mutex_lock(&g_data_mutex);
                                memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &TxBuf, TxSize); // put output message in write buffer
                                g_serial_data_lenTx = g_serial_data_lenTx + TxSize; // increment byte counter in TX buffer
                                pthread_mutex_unlock(&g_data_mutex);

                                break;
                            case BGNET_FRAME_CMD_PILOT_SPEED:
                                // the SPEED key was pressed
                                // set display the various SPEED variables
                                // message has 2 bytes payload:
                                // 0 = ACK sequence number (to be returned vua ACK message)
                                // 1 = 0x46/AP_SPEEDKT channel number
                                log_info("  Command %02X SPEED BUTTON PRESSED (BY: %02X, %s, state= %d)", *(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName, *(pBuf+BGNET_FRAME_HEAD+4));
                                
                                // create ACK response
                                payload[0] = *(pBuf + BGNET_FRAME_HEAD + 0); // get 1st payload byte and return it
                                len_pl = 1;
                                //msg_offset = 0;

                                // make ACK frame, update statistics
                                len_msg = bgn_net_makemessage(&TxBuf[TxSize], *(pBuf + BGNET_FRAME_HEAD_FROM), BGNET_ADDRESS_ME, BGNET_FRAME_CMD_ACK, payload, len_pl);
                                memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_ACK].ucFrameTx, &TxBuf[TxSize], len_msg);
                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_ACK].CounterTx, t);
                                memcpy(&pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                TxSize = TxSize + len_msg;

                                // make DATA command
                                len_pl = makeSPEEDresponse(pCtrl, payload);
                                // make DATA frame, update statistics
                                len_msg = bgn_net_makemessage(&TxBuf[TxSize], BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                                memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].ucFrameTx, &TxBuf[TxSize], len_msg);
                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].CounterTx, t);
                                memcpy(&pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                TxSize = TxSize + len_msg;

                                // put it for transmission
                                pthread_mutex_lock(&g_data_mutex);
                                memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &TxBuf, TxSize); // put output message in write buffer
                                g_serial_data_lenTx = g_serial_data_lenTx + TxSize; // increment byte counter in TX buffer
                                pthread_mutex_unlock(&g_data_mutex);

                                break;
                            case BGNET_FRAME_CMD_PILOT_STEP:
                                // 1 of the 4 +/- keys was pressed
                                // message has 2 bytes payload:
                                // 0 = ACK sequence number (to be returned vua ACK message)
                                // 1 = 0xA6/AP_CTARGET channel number
                                log_info("  Command %02X 1-of-4 STEP BUTTONS PRESSED (BY: %02X, %s, state= %d)", *(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName, *(pBuf+BGNET_FRAME_HEAD+4));

                                // create ACK response
                                payload[0] = *(pBuf + BGNET_FRAME_HEAD + 0); // get 1st payload byte and return it
                                len_pl = 1;
                                //msg_offset = 0;

                                // make ACK frame, update statistics
                                len_msg = bgn_net_makemessage(&TxBuf[TxSize], *(pBuf + BGNET_FRAME_HEAD_FROM), BGNET_ADDRESS_ME, BGNET_FRAME_CMD_ACK, payload, len_pl);
                                memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_ACK].ucFrameTx, &TxBuf[TxSize], len_msg);
                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_ACK].CounterTx, t);
                                memcpy(&pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                TxSize = TxSize + len_msg;

                                // put it for transmission
                                pthread_mutex_lock(&g_data_mutex);
                                memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &TxBuf, TxSize); // put output message in write buffer
                                g_serial_data_lenTx = g_serial_data_lenTx + TxSize; // increment byte counter in TX buffer
                                pthread_mutex_unlock(&g_data_mutex);
                                
                                break;
                            case BGNET_FRAME_CMD_PILOT_ADJUST:
                                // 1 of the 4 +/- keys was pressed
                                // message has 5 bytes payload:
                                // 0 = ACK sequence number (to be returned vua ACK message)
                                // 1 = 0xA6/AP_CTARGET channel number
                                // 2 = 0x01 FMT_01
                                // 3 = 0xFF makes -10
                                // 4 = 0xF6
                                log_info("  Command %02X STEP ADJUST (BY: %02X, %s, state= %d)", *(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName, *(pBuf+BGNET_FRAME_HEAD+4));
                                // channel starts at 2nd byte of the payload
                                pCH = (union CH*)(pBuf + BGNET_FRAME_HEAD + 1);
                                iValue = decode_channel_numeric(pCH);
                                log_info("  Command: Channel 0x%02X -> %d", pCH->s.CH, iValue);
                                switch(pCH->s.CH) {
                                    case BGN_CNR_A6_AP_CTARGET:
                                            pCtrl->ACP.CourseTarget.value.nLong = pCtrl->ACP.CourseTarget.value.nLong + iValue;
                                        break;
                                    default:
                                        break;
                                }

                                // create ACK response
                                payload[0] = *(pBuf + BGNET_FRAME_HEAD + 0); // get 1st payload byte and return it
                                len_pl = 1;
                                //msg_offset = 0;

                                // make ACK frame, update statistics
                                len_msg = bgn_net_makemessage(&TxBuf[TxSize], *(pBuf + BGNET_FRAME_HEAD_FROM), BGNET_ADDRESS_ME, BGNET_FRAME_CMD_ACK, payload, len_pl);
                                memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_ACK].ucFrameTx, &TxBuf[TxSize], len_msg);
                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_ACK].CounterTx, t);
                                memcpy(&pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                TxSize = TxSize + len_msg;

                                // make DATA frame for PILOT
                                len_pl = makeACPresponseFAST(pCtrl, payload);
                                len_msg = bgn_net_makemessage(&TxBuf[TxSize], *(pBuf + BGNET_FRAME_HEAD_FROM), BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                                memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].ucFrameTx, &TxBuf[TxSize], len_msg);
                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].CounterTx, t);
                                memcpy(&pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                TxSize = TxSize + len_msg;

                                // put it for transmission
                                pthread_mutex_lock(&g_data_mutex);
                                memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &TxBuf, TxSize); // put output message in write buffer
                                g_serial_data_lenTx = g_serial_data_lenTx + TxSize; // increment byte counter in TX buffer
                                pthread_mutex_unlock(&g_data_mutex);
                                
                                break;
                            default:
                                iIsValid = 0; // do not update device/command statistics
                                x = bgn_FindDeviceIndex(pCtrl, *(pBuf+BGNET_FRAME_HEAD_FROM)); // lookup by device address/type, so we can print the device name
                                log_info("  Command %02X (with body) IGNORED (from %02X %s)", *(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName);
                                break;
                        }
                        if (iIsValid > 0) {
                            // update DEV & CMD structure
                            pCtrl->iNewDevice = BGN_NEW_SET; // signal this device
                            pCtrl->DEV[ucFROM].iNewDevice = BGN_NEW_SET;
                            memcpy(pCtrl->DEV[ucFROM].ucFrameRx, pBuf, RxSize);
                            CounterUp(&pCtrl->DEV[ucFROM].CounterRx, t);

                            pCtrl->iNewCommand = BGN_NEW_SET; // signal possible CMD change
                            pCtrl->CMD[ucCMD].iNewCommand = BGN_NEW_SET; // signal this command
                            memcpy(pCtrl->CMD[ucCMD].ucFrameRx, pBuf, RxSize);
                            CounterUp(&pCtrl->CMD[ucCMD].CounterRx, t);
                        }   
                    } else {
                        // CS is bad (only skip this sub-frame, the length is known)
                        log_info("    Frame Checksum BAD, framesize: %d", RxSize);
                        pCtrl->DEV[ucFROM].iCSError_Frame++;
                    }
                }
            }
        } else {
            // header CS invalid, we can not process this subframe, we can not establish a subframe length unambigiously, so we dump the whole buffer
            pCtrl->DEV[ucFROM].iCSError_Header++;
            log_error("  Header Checksum ERR, dumping RX frame");
            break;
        }
        pBuf = pBuf + RxSize;
        z = z + 1;
    }
    return(z);
}

#endif
