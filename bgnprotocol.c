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
unsigned char bgnet_checksum(unsigned char *buf, unsigned char init, int start, int max) {
    int i;
    unsigned char c = -init;
    for (i=start; i<(start+max); i++) {
        c -= buf[i];
    }
    return (c);
}

// get the index for the devicetype we are looking for
int bgnet_deviceindex(BGNET_CTRL* pCtrl, unsigned char ucDevAddr) {
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
void NMEAinit(NMEA2K_VALUE* pNMEA, time_t t) {
    pNMEA->source = BGN_SOURCE_NONE;
    pNMEA->value.nLong = 0;
    pNMEA->value.dValue = 0.0;
    CounterInit(&pNMEA->Counter, t);
    return;
}
void log_buffer(unsigned char *pBuf, size_t y) {
    char asciibuf[120];
    byte2ascii(asciibuf, sizeof(asciibuf), pBuf, y, BYTE2ASCII_SPACED);
    log_debug("log_buffer: Command 0x= %s, (%d bytes)", asciibuf, y);
}
void log_frame(unsigned char *pBuf) {
    int y;
    char asciibuf[120];
    y = bgnet_frame_length(pBuf);
    byte2ascii(asciibuf, sizeof(asciibuf), pBuf, y, BYTE2ASCII_SPACED);
    log_debug("log_frame: Command 0x%02X -> %s", pBuf[BGNET_FRAME_HEAD_CMD], asciibuf);
}

unsigned char ACPAckSeqGet(BGNET_CTRL *pCtrl, FRAME_H* pF, time_t t) {
    unsigned char r;
    r = (unsigned char)(pCtrl->ACP.CommandACK.Counter.iCounter & 0xFF);
    pCtrl->ACP.CommandACK.value.nByte = pF->CMD;
    CounterUp(&pCtrl->ACP.CommandACK.Counter, t);
    return(r);
}

// a reset is usually done:
// - at startup of this driver
// - when we observe "POWERUP" message(s) from the NETWORK (of any of the units)
void bgnet_initialize_reset(BGNET_CTRL *pCtrl) {
    //pthread_mutex_lock(&g_data_mutex);
    pCtrl->iInitialized = BGNET_CTRL_INIT_RESET; // reset initialized state
    //pthread_mutex_unlock(&g_data_mutex);
}

void bgnet_initialize(BGNET_CTRL *pCtrl) {
    int i;
    time_t t;

    if (pCtrl->iInitialized != BGNET_CTRL_INIT_RESET){
        // controlblock is already initialized. This means that this initialisation only happens if the 'initialized state' was reset
        // prior this function.
    } else {
        // note: we do NOT iniliaize the pointers to the data structures. These are set upon start and never change

        // stage 1: set state 'initialized'
        time(&t); // save timestamp to we use same timestamp everywhere
        pCtrl->iInitialized = BGNET_CTRL_INIT; // mark as initialized
        pCtrl->iDeviceCounter = BGNET_CTRL_DEVCNTR_INIT;
        pCtrl->iDeviceRun = 0; //reset run counter (0-1000)

        // stage 2 - ACP initialize data structure
        pCtrl->ACP.iNewACP = 0;
        pCtrl->ACP.tTimestamp = t;

        // PILOT watchdog (set by PILOT_HELLO, count-down by timer)
        pCtrl->ACP.PILOT.value.nLong = BGN_PILOT_SET;
        pCtrl->ACP.PILOT.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.PILOT.Counter, t);

        // CMD counter (used to send ACKSEQ values to PILOT)
        pCtrl->ACP.CMD.value.nLong = 0;
        pCtrl->ACP.CMD.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.CMD.Counter, t);

        // initialize AutoPilot to STBY/COMP; joystick OFF
        pCtrl->ACP.STATE.value.nByte = BGN_AP_MODE1_STBY;
        pCtrl->ACP.STATE.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.STATE.Counter, t);

        pCtrl->ACP.MODE.value.nByte = BGN_AP_MODE2_COMP;
        pCtrl->ACP.MODE.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.MODE.Counter, t);

        pCtrl->ACP.Joystick.value.nByte = BGN_JOYSTICK_OFF; // overrides PILOT control
        pCtrl->ACP.Joystick.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.Joystick.Counter, t);

        // PILOT SETUP parameters
        pCtrl->ACP.SpeedSelect.value.nByte = 2; //0=Transmitter, 1=Manual, 2=SOG
        pCtrl->ACP.SpeedSelect.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.SpeedSelect.Counter, t);

        pCtrl->ACP.Response.value.nByte = 2;  //0=Normal, 1=Downwind, 2=Eco
        pCtrl->ACP.Response.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.Response.Counter, t);

        pCtrl->ACP.WA.value.nLong = 14; // default value is 4 minutes, range 1-360 (so it must be a 16 bits register)
        pCtrl->ACP.WA.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.WA.Counter, t);

        pCtrl->ACP.OC.value.nLong = 25; // default value is 15degrees; range 1-99
        pCtrl->ACP.OC.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.OC.Counter, t);

        pCtrl->ACP.SETUP_JSM.value.nLong = 1; // Joystick Steering Mode (Normal, Proportional)
        pCtrl->ACP.SETUP_JSM.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.SETUP_JSM.Counter, t);

        pCtrl->ACP.SETUP_DMP.value.nLong = 44; // Compass Damping, 1-99 sec, 4= default
        pCtrl->ACP.SETUP_DMP.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.SETUP_DMP.Counter, t);

        pCtrl->ACP.SETUP_FR.value.nLong = 1; // Fault Reporting (On/Off)
        pCtrl->ACP.SETUP_FR.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.SETUP_FR.Counter, t);

        // other parameters
        pCtrl->ACP.BackLight.value.nByte = 0; // backlight (0=OFF, 3-1=ON)
        pCtrl->ACP.BackLight.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.BackLight.Counter, t);

        pCtrl->ACP.CourseTarget.value.nLong = 123; // Course Target, calculated by the Autopilot (0-359)
        pCtrl->ACP.CourseTarget.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.CourseTarget.Counter, t);

        // we do NOT use a floating point value; otherwise we get all kind or rounding errors. Just keep it as handled within the
        // B&G system: as an integer*10 (so we get 1 decimal)
        pCtrl->ACP.StoredLogNM.value.nLong = 12345*10+6; // channel CD/STORELOGNM (on the real system this is stored in non-volitile memory by QUAD/SPEED, no reset)
        pCtrl->ACP.StoredLogNM.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.StoredLogNM.Counter, t);

        pCtrl->ACP.TripLogNM.value.nLong = 1234*10+5; // channel CF/TRIPLOGNM (on the real system this is stored in non-volitile memory by QUAD/SPEED, resetable)
        pCtrl->ACP.TripLogNM.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.TripLogNM.Counter, t);

        // stage 3 - initialize device structure
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

        // stage 4 - initialize command structure
        pCtrl->iNewCommand = BGN_NEW_RESET; // no new data available
        for (i=0; i<BGN_COMMAND_MAX; i++) {
            pCtrl->CMD[i].iNewCommand = BGN_NEW_RESET;
            CounterInit(&pCtrl->CMD[i].CounterRx, t);
            memset(pCtrl->CMD[i].ucFrameRx, 0, BGN_FRAME_SIZE);
            CounterInit(&pCtrl->CMD[i].CounterTx, t);
            memset(pCtrl->CMD[i].ucFrameTx, 0, BGN_FRAME_SIZE);
        }

        // stage 5 - initialize channel structure
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

            pCtrl->CHN[i].x00.ucTO = 0x00;
            pCtrl->CHN[i].x00.ucFROM = 0x00;
            pCtrl->CHN[i].x00.ucCMD = 0x00;
            memset(pCtrl->CHN[i].x00.ucRAW, 0, sizeof(pCtrl->CHN[i].x06.ucRAW));
            strcpy(pCtrl->CHN[i].x00.cValue, "");
            CounterInit(&pCtrl->CHN[i].x00.CounterRx, t);
        }

        log_debug("bgnet_initialize: Done");
    }
};

// get the command label from the Command Definition Table
char* strCommandLabel(BGNET_CTRL *pCtrl, char *pBuf, unsigned char ucCMD) {
    if (pCtrl->pCMDDT[ucCMD].ucCommandIndex == ucCMD) {
        strcpy(pBuf, pCtrl->pCMDDT[ucCMD].cLabel);
    } else {
        strcpy(pBuf, "undefined command");
    }
    return(pBuf);
};

// encode a frame (with or without payload)
size_t bgnet_encode_frame(unsigned char *pBuf, int iTo, int iFrom, int iCmd, unsigned char *payload, int payloadlength) {
    FRAME_H* pFH;
    pFH = (FRAME_H*)pBuf;

    pFH->TO = iTo;
    pFH->FROM = iFrom;
    pFH->PL = payloadlength;
    pFH->CMD = iCmd;
    pFH->CSH = bgnet_checksum(pBuf, BGNET_FRAME_CR_HEAD, 0, BGNET_FRAME_HEAD-1);
    if (payloadlength == 0) {
        // nothing to do
    } else {
        memcpy(pFH->payload, payload, payloadlength);
        pFH->payload[payloadlength] = bgnet_checksum(pFH->payload, BGNET_FRAME_CR_FRAME, 0, payloadlength);
    }
    return(bgnet_frame_length(pBuf));
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
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_42_BOATSPEEDRAW;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(225);
    r = r + bgnet_channel_length(pCtrl, puCH);

    // the next 3 values are 'rotated' by the QUAD: CD every other time, 64/65 every 4'th time
    // but that is too complex at this moment. Later i will build a rotating scheme.

    c = BGN_CNR_CD_STORLOGNM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uL[0] = htonl(112233);
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_64_SPEEDAVGKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(111); 
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_65_SPEEDMAXKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(222);
    r = r + bgnet_channel_length(pCtrl, puCH);

    // the next values are always present 

    c = BGN_CNR_81_DRDNM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(333); 
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_CF_TRIPNM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(55); 
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_D3_DR_COURSE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(-20); 
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_20_SEATEMPQ;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(12); 
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_75_TIMER;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uB[0] = 23; // HH
    puCH->s.cv.uB[1] = 59; // MM
    puCH->s.cv.uB[2] = 59; // SS
    r = r + bgnet_channel_length(pCtrl, puCH);

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
    r = r + bgnet_channel_length(pCtrl, puCH);
    
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
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_FD_DEPTHALARM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(5); 
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_FE_ANCHORALARM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uC[0] = '\0'; 
    puCH->s.cv.uC[0] = 'O'; 
    puCH->s.cv.uC[0] = 'F'; 
    puCH->s.cv.uC[0] = 'F';
    r = r + bgnet_channel_length(pCtrl, puCH);

    // the next values are always present 

    c = BGN_CNR_FF_DEPTH_CAL;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(4); 
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_C4_DEPTHM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(6); 
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_C1_DEPTHM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(2); 
    r = r + bgnet_channel_length(pCtrl, puCH);

    return(r);
}

int makeACPresponseCTARGET(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r = 0,c;
    union CH* puCH;

    c = BGN_CNR_A6_AP_CTARGET;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(pCtrl->ACP.CourseTarget.value.nLong);
    r = r + bgnet_channel_length(pCtrl, puCH);

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
    r = r + bgnet_channel_length(pCtrl, puCH);

    // CTARGET is displayed by PILOT in large digits. If we switch to AUTO, and we omit
    // the CTARGET value, it might take some time before the CTARGET is displayed.
    // To prevent this nuiseance we include the CTARGET when we are in AUTO
    if (puCH->s.cv.uB[0] == BGN_AP_MODE1_AUTO) {
        c = BGN_CNR_A6_AP_CTARGET;
        puCH = (union CH*)&(payload[r]);
        puCH->s.CH = c;
        puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
        puCH->s.cv.uW[0] = htons(pCtrl->ACP.CourseTarget.value.nLong);
        r = r + bgnet_channel_length(pCtrl, puCH);
    }

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
    if (pCtrl->ACP.Joystick.value.nByte == BGN_JOYSTICK_OFF) {
        puCH->s.cv.uB[1] = pCtrl->ACP.MODE.value.nByte;  // COMP/NAV/VANE
    } else {
        puCH->s.cv.uB[1] = BGN_AP_MODE2_JOYSTICK;  // JOYSTCK
    }
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_0B_RUDDER;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(-7);
    r = r + bgnet_channel_length(pCtrl, puCH);

    // when in STBY, the display shows the HEADING (compass reading)
    c = BGN_CNR_49_HEADING;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(221);
    r = r + bgnet_channel_length(pCtrl, puCH);

    // when in AUTO, the diplay shows the COURSE TARGET (set of the autopilot)
    c = BGN_CNR_A6_AP_CTARGET;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(pCtrl->ACP.CourseTarget.value.nLong);
    r = r + bgnet_channel_length(pCtrl, puCH);

    // SPEED source selector (0-n)
    c = BGN_CNR_46_AP_SPEED;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uB[0] = 0;
    puCH->s.cv.uB[1] = pCtrl->ACP.SpeedSelect.value.nByte;
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_41_BOATSPEEDKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(125);  // displayed as 1.2
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_EB_SOGKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(325);  // displayed as 3.2
    r = r + bgnet_channel_length(pCtrl, puCH);

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
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_0B_RUDDER;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(-8+(t&0x01F));
    r = r + bgnet_channel_length(pCtrl, puCH);

    // when in STBY, the display shows the HEADING (compass reading)
    c = BGN_CNR_49_HEADING;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(222);
    r = r + bgnet_channel_length(pCtrl, puCH);

    // when in AUTO, the diplay shows the COURSE TARGET (set of the autopilot)
    c = BGN_CNR_A6_AP_CTARGET;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(pCtrl->ACP.CourseTarget.value.nLong);
    r = r + bgnet_channel_length(pCtrl, puCH);
 
    c = BGN_CNR_51_AWINDANGLE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(100);
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_59_TWINDANGLE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(102);
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_EE_XTE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(-57);
    r = r + bgnet_channel_length(pCtrl, puCH);
 
    // optional channels
    switch(pCtrl->ACP.STATE.value.nByte) {
        case BGN_AP_MODE1_AUTO:
            break;
        case BGN_AP_MODE1_STBY:
            break;
    }
    switch(pCtrl->ACP.MODE.value.nByte) {
        case BGN_AP_MODE2_COMP:
            break;
        case BGN_AP_MODE2_NAV:
            c = BGN_CNR_A7_AP_WPB;
            puCH = (union CH*)&(payload[r]);
            puCH->s.CH = c;
            puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
            puCH->s.cv.sW[0] = htons(123);
            r = r + bgnet_channel_length(pCtrl, puCH);

            c = BGN_CNR_E8_WPD;
            puCH = (union CH*)&(payload[r]);
            puCH->s.CH = c;
            puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
            puCH->s.cv.sW[0] = htons(333);
            r = r + bgnet_channel_length(pCtrl, puCH);

            break;
        case BGN_AP_MODE2_VANE:
            // AWA signal shall be provided by WIND
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
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_41_BOATSPEEDKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(125);
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_49_HEADING;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(77);
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_51_AWINDANGLE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(-95);
    r = r + bgnet_channel_length(pCtrl, puCH);

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
    return(bgnet_channel_length(pCtrl, pCH));
}

int makeRESPresponse(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r = 0,c;
    union CH* puCH;

    c = BGN_CNR_A6_AP_CTARGET; // RESP selector
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uB[0] = 0;
    puCH->s.cv.uB[1] = pCtrl->ACP.Response.value.nByte;
    r = r + bgnet_channel_length(pCtrl, puCH);

    return (r);
}
int makeWAresponse(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r = 0,c;
    union CH* puCH;

    c = BGN_CNR_76_AP_WA_ALARM; // WA limit
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    if (pCtrl->ACP.WA.value.nLong == 0) {
        puCH->s.cv.uW[0] = htons(4); // default value
    } else {
        puCH->s.cv.uW[0] = htons(pCtrl->ACP.WA.value.nLong);
    }
    r = r + bgnet_channel_length(pCtrl, puCH);

    return (r);
}
int makeOCresponse(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r = 0,c;
    union CH* puCH;

    c = BGN_CNR_AF_AP_OC_ALARM; // OC limit
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    if (pCtrl->ACP.OC.value.nLong == 0) {
        puCH->s.cv.uW[0] = htons(15); // default value
    } else {
        puCH->s.cv.uW[0] = htons(pCtrl->ACP.OC.value.nLong);
    }
    r = r + bgnet_channel_length(pCtrl, puCH);

    return (r);
}
int makeSPEEDresponse(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r = 0,c;
    union CH* puCH;

    c = BGN_CNR_46_AP_SPEED; // SPEED source selector
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uB[0] = 0;
    puCH->s.cv.uB[1] = pCtrl->ACP.SpeedSelect.value.nByte;
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_41_BOATSPEEDKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(125);
    r = r + bgnet_channel_length(pCtrl, puCH);

    c = BGN_CNR_EB_SOGKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(325);
    r = r + bgnet_channel_length(pCtrl, puCH);

    return (r);
}

#ifdef KOEN
// update timestamp and increment counter (with wrap-around)
void CounterUpDATA(BGN_CHANNEL *pCH, time_t t) {
    pCH->x01.CounterRx.tTimestamp = t;
    // update counter (with wrap-arouny, BYTE2ASCII_SPACED);d)
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
size_t bgnet_process(BGNET_CTRL *pCtrl, unsigned char* buffer, size_t size) {
    int i;
    size_t r;
    time_t t;
    time(&t); // save the time

    // erase output buffer (to prevent compiler warnings about unused arguments)
    memset(buffer, 0, size);
    r = 0;

    // check if initialisation is required
    bgnet_initialize(pCtrl);

    // device received new data
     if ((pCtrl->iNewCommand == BGN_NEW_RESET)) {
        // nothing to do
     } else {
        log_debug("bgnet_process: Resetting CMD");
        for (i=0; i<BGN_COMMAND_MAX; i++) {            if (pCtrl->CMD[i].iNewCommand == BGN_NEW_RESET) {
                // nothing to do
            } else {
                switch(i) {
                    default:
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
        log_debug("bgnet_process: Resetting DATA");
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
        log_debug("bgnet_process: Resetting DEV");
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
        len_msg = bgnet_encode_frame(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);

        // put it for transmission
        memcpy(&buffer[r], &tmp_buffer, len_msg); // put output message in write buffer
        r = r + len_msg; // increment byte counter in TX buffer

        // update statistics
        //memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].ucFrameTx, &tmp_buffer, len_msg);
        CounterTxUpCMD(&pCtrl->CMD[BGNnET_FRAME_CMD_DATA], t);

        len_pl = makeQUADresponse2(pCtrl, payload);
        len_msg = bgnet_encode_frame(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
        
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

// returns the length of a frame/subframe
size_t bgnet_frame_length(unsigned char* pFrameBuffer) {
    size_t r;
    if (pFrameBuffer[BGNET_FRAME_HEAD_SIZE] == 0) {
        r = BGNET_FRAME_HEAD;
    } else {
        r = BGNET_FRAME_HEAD + pFrameBuffer[BGNET_FRAME_HEAD_SIZE] + BGNET_FRAME_TAIL_CS;
    }
    return(r);
}
// returns the length of a channel block; it all depends on the FMT specifier (in the FMT Definition Table).
size_t bgnet_channel_length(BGNET_CTRL* pCtrl, union CH* pCH) {
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
        y = bgnet_frame_length(ucFrame);
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
            switch (puCH->s.CH) {
                case BGN_CNR_46_AP_SPEED:
                case BGN_CNR_B5_AP_STATUS:
                    sprintf(pStr,"0x%02X 0x%02X", puCH->s.cv.uB[0], puCH->s.cv.uB[1]);
                    iValid = 2;
                    break;
                default:
                    iRawValue = decode_channel_numeric(puCH);
                    iValid = 1;
                    break;
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
        y = bgnet_channel_length(pCtrl, pCH);

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
                case BGN_CNR_CF_TRIPNM:
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

            // when Import/Export is enabled, do what has to be done
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
        memcpy(pCtrl->CHN[ucCHN].x06.ucRAW, buffer, bgnet_frame_length(buffer));
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
        len = bgnet_frame_length(buffer);
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
size_t bgnet_decode_frame(BGNET_CTRL* pCtrl, unsigned char *buffer, size_t len) {
    FRAME_H* pFR;   // pointer to the RECEIVE frame
    union CH *pCHR; // pointer to the channel structure we RECEIVE (eighter a CMD-REQ or a CMD-DAT command)

    unsigned char cs; // checksum calculated
    unsigned char *pBuf, *pEnd; // pointer to start and end of buffer
    int RxSize, x, z, n;
    int iIsValid, iValue;
    time_t t;
    size_t y;
    // inside this routine we work with our local buffers to store generated frames
    // this is only done in the same thread, so we do not have to do lock/unlock stuff
    unsigned char payload[1024], TxBuf[1024];
    int len_pl, len_msg, TxSize;
    char asciibuf[80];

    pBuf = buffer; // gets incremented as we go
    pEnd = buffer+len; // end of buffer
    z = 0; // subframe counter

    // process (possible multiple) subframes in buffer
    while (pBuf < pEnd) {
        pFR = (FRAME_H*)pBuf;
        // update the time
        time(&t);
        // initialize some variables
        TxSize = 0;
        len_pl = 0;
        len_msg = 0;
        iIsValid = 1; // tricky: if we hit a 'default' in a switch we reset this, so statistics is not updated

        byte2ascii(asciibuf, sizeof(asciibuf), pBuf, bgnet_frame_length(pBuf), BYTE2ASCII_SPACED);
        log_info("bgnet_decode_frame: Command 0x%02X: %s", pFR->CMD, asciibuf);

        cs = bgnet_checksum(pBuf, BGNET_FRAME_CR_HEAD, 0, BGNET_FRAME_HEAD-1); // calculate the checksum over the header (without the CR)
        if (pFR->CSH == cs) {
            // check if this subframe is adressed TO us and not FROM us
            if (pFR->FROM == BGNET_ADDRESS_ME) {
                RxSize = pEnd - pBuf; // dump the remainder of the frame; it's a readback of my own transmission
            } else {
                // The HEADER is OK, process it. Note that some CMD's do not have a body and in such case, we should skip the body & CS part.
                RxSize = bgnet_frame_length(pBuf); // expected framesize = head (including CR), body, and frame CR

                // When we receive someting from the network, we register it, regardless whether it has a CS error,
                // or if we will discard it.
                // The reason is, that if we have a 'active' node on the BGNET, we must avoid a situation where we allow
                // the import (from NMEA2000 or SignalK) if we have a active transmitter.
                // It could well be that our received frame is in error (and that we incorretly assert that the node is active),
                // but ignoring it and allowing external sources overwrite internal sources would a;sp create havoc.
                pCtrl->DEV[pFR->FROM].source = BGN_SOURCE_BGNET;
                pCtrl->iNewDevice = BGN_NEW_SET; // signal this device
                pCtrl->DEV[pFR->FROM].iNewDevice = BGN_NEW_SET;
                memcpy(pCtrl->DEV[pFR->FROM].ucFrameRx, pBuf, RxSize);
                CounterUp(&pCtrl->DEV[pFR->FROM].CounterRx, t);

                x = bgnet_deviceindex(pCtrl, pFR->FROM); // lookup by device address/type, so we can print the device name
                if (pFR->PL == 0) {  // No payload (no frame CS)
                    RxSize = BGNET_FRAME_HEAD;
                    switch(pFR->CMD) {
                        case BGNET_FRAME_CMD_PILOT_HELLO:
                            // when we receive this message, we know that there is atleast 1 active PILOT on the network.
                            // We set the counter; timer thread counts it down and if this message is no longer received
                            // we (the ACP) know that there is no longer user-input possible because no PILOT is connected.
                            log_info("  Command %02X (header only) KEEPALIVE by %02X %s", pFR->CMD, pFR->FROM, pCtrl->pDEVDT[x].sModelName);

                            // set the counter (count down done by timer thread)
                            pthread_mutex_lock(&g_data_mutex);
                            pCtrl->ACP.PILOT.value.nLong = BGN_PILOT_SET;
                            pCtrl->ACP.PILOT.source = BGN_SOURCE_BGNET;
                            CounterUp(&pCtrl->ACP.PILOT.Counter, t);
                            pthread_mutex_unlock(&g_data_mutex);

                            len_pl = makeACPresponseMINI(pCtrl, payload);
                            len_msg = bgnet_encode_frame(&TxBuf[TxSize], pFR->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
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
                            // when we receive this message one (or more) devices went though the startup process (ie. power was
                            // switch ON while we (the ACP) were still running.
                            // At this moment i see no reason to do something special at this occasion. Realise that most networks
                            // contain multiple units. You might expect a burst of these messages. Some units (like WIND) take some
                            // more time then others. Also units that are normally passive (not broadcasting channels, like DATA)
                            // will send this message.
                            log_info("  Command %02X (header only) POWERUP by %02X %s", pFR->CMD, pFR->FROM, pCtrl->pDEVDT[x].sModelName);
                            // At first i thought that resetting our own structure was a good idea. It is not.
                            // At startup, a lot of initial messages are being send (command 0x02, 0x06) and if we reset our structure
                            // we erase all that we received. Better not resetting.
                            //bgnet_initialize_reset(pCtrl);
                            break;
                        default:
                            iIsValid = 0; // do not update device/command statistics
                            log_info("  Command %02X (header only) IGNORED (from %02X %s)", pFR->CMD, pFR->FROM, pCtrl->pDEVDT[x].sModelName);
                            break;
                    }
                    if (iIsValid > 0) {
                        // update CMD structure
                        pCtrl->iNewCommand = BGN_NEW_SET; // signal possible CMD change
                        pCtrl->CMD[pFR->CMD].iNewCommand = BGN_NEW_SET; // signal this command
                        memcpy(pCtrl->CMD[pFR->CMD].ucFrameRx, pBuf, RxSize);
                        CounterUp(&pCtrl->CMD[pFR->CMD].CounterRx, t);
                    }
                } else {
                    cs = bgnet_checksum(pFR->payload, BGNET_FRAME_CR_FRAME, 0, pFR->PL);
                    if (pFR->payload[pFR->PL] == cs) {
                        switch(pFR->CMD) {
                            case BGNET_FRAME_CMD_DATA:
                                // process DATA command (broadcast)
                                n = decode_command_DATA(pCtrl, pBuf);
                                log_info("  Command %02X (with payload) DATA, %d channels", pFR->CMD, n);
                                break;
                            case BGNET_FRAME_CMD_LABEL:
                                // process LABEL command (only at startup, only by DATA)
                                decode_command_LABEL(pCtrl, pBuf);
                                log_info("  Command %02X (with payload) LABEL", pFR->CMD);
                                break;
                            case BGNET_FRAME_CMD_SUBSCRIBE:
                                // process SUBSCRIBE command (only at startup, only by DATA)
                                decode_command_SUBSCRIBE(pCtrl, pBuf);
                                log_info("  Command %02X (with payload) SUBSCRIBE", pFR->CMD);
                                break;
                            case BGNET_FRAME_CMD_BACKLIGHT:
                                // process LIGHTS command (when you press the LIGHTS button)
                                // paylod: 1 byte (intensity 0, 3-1)
                                pCtrl->ACP.BackLight.value.nByte = pFR->payload[0]; // first byte is the LIGHTS state
                                pCtrl->ACP.BackLight.source = BGN_SOURCE_BGNET;
                                log_info("  Command %02X (with payload) BACKLIGHT (BY: %02X, %s, LEVEL: %02X)",
                                                    pFR->CMD,
                                                    pFR->FROM,
                                                    pCtrl->pDEVDT[x].sModelName,
                                                    pCtrl->ACP.BackLight.value.nByte);
                                break;
                            default:
                                // Here we group all the commands that require a ACK response
                                switch(pFR->CMD) {
                                    case BGNET_FRAME_CMD_QUAD_RESET:

                                    // PILOT/SETUP actions
                                    case BGNET_FRAME_CMD_PILOT_SPDREQ: // CMD-DAT is carried by PILOT_AM
                                    case BGNET_FRAME_CMD_PILOT_RESPREQ:
                                    case BGNET_FRAME_CMD_PILOT_RESPDAT:
                                    case BGNET_FRAME_CMD_PILOT_WAREQ:
                                    case BGNET_FRAME_CMD_PILOT_WADAT:
                                    case BGNET_FRAME_CMD_PILOT_OCREQ:
                                    case BGNET_FRAME_CMD_PILOT_OCDAT:
                                    case BGNET_FRAME_CMD_PILOT_DMPREQ:
                                    case BGNET_FRAME_CMD_PILOT_DMPDAT:
                                    // PILOT/SETUP/FR CMD-REQ is carries by SPDREQ, CMD-DAT is carried by PILOT_MODE

                                    // PILOT KEY actions
                                    case BGNET_FRAME_CMD_PILOT_AM:
                                    case BGNET_FRAME_CMD_PILOT_MODE:
                                    case BGNET_FRAME_CMD_PILOT_STEP:
                                    case BGNET_FRAME_CMD_PILOT_ADJUST:

                                        // Step 1 - understand what we receive here:
                                        //
                                        // It might differ from one command to the other, so we split the commands again.
                                        // CMD-REQ: 2 bytes payload: ACKSEQ, CHNID (1 byte) -> PILOT wants to receive a channel with data from ACP 
                                        // CMD-DAT: 5 bytes payload: ACKSEQ, CHN (4 bytes) -> PILOT <> ACP channel with data to handle

                                        // Step 2 - prepare the ACK message
                                        // This was the place where i generated the CMD-ACK response command. But appearently, the CMD-DAT message must be
                                        // send before the CMD-ACK, otherwise the SETUP SELECT does not pickup the value we send.
                                        // So i moved the CMD-ACK stuff to the end
                                        // Note: When the PILOT expects a CMD-DAT frame before the CMD-ACK, but the CMD-DAT is not present, the PILOT assumes
                                        // 'some value'. This is not the default value. Appearently the default value is in the firmware of the ACP and if the
                                        // CMD-DAT is not received from the ACP, PILOT falls back to 'some value'.

                                        // Step 3 - setup pointer to received channel. Realise that when this is a CMD-DAT command, we have a 4-byte
                                        // channel, but if this is a CMD-REQ we only have the ChannelID (and the last 3 bytes can be anything)
                                        pCHR = (union CH *)(&pFR->payload[1]);

                                        // Step 4 - handle the data from the received command.
                                        // CMD-REQ: 2 bytes payload -> send a requested CMD-DAT response + CMD-ACK response
                                        // CMD-DAT: 5 bytes payload -> handle the received channel + send a CMD-ACK response
                                        // Note: when we must send data, do NOT use the DATA command (0x01), but use the corresponding
                                        // CMD-DAT command; i found out the hard way.
                                        // Note: i send the CMD-DAT and CMD-ACK message TO the node that we received the CMD-REQ command FROM. It might be the
                                        // case that 1 (or both) must be send to ALL (0xFF) in case there are multiple PILOTS on the network. As i have only 1
                                        // PILOT unit i could not establish one or the other and choose to do it like this.
                                        switch(pFR->CMD) {
                                            case BGNET_FRAME_CMD_PILOT_AM:
                                                // the AUTO or OFF key was pressed. We receive 5 bytes payload.
                                                // 0=ACKSEQ, 1=0xB5/AP_STATUS, 2=FMT_01, 3=0, 4= new state (1=AUTO , 0=STBY)
                                                switch(pCHR->s.CH) {
                                                    case BGN_CNR_B5_AP_STATUS:
                                                        // update the channel value (store it in the X00 section)
                                                        pCtrl->CHN[pCHR->s.CH].iNewChannel = BGN_NEW_SET;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucTO = pFR->TO;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucFROM = pFR->FROM;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucCMD = pFR->CMD;
                                                        y = bgnet_channel_length(pCtrl, pCHR);
                                                        memcpy(pCtrl->CHN[pCHR->s.CH].x00.ucRAW, pCHR->ucRAW, y);
                                                        decode_channel_string(pCtrl->CHN[pCHR->s.CH].x00.cValue, pCHR);
                                                        CounterUp(&pCtrl->CHN[pCHR->s.CH].x00.CounterRx, t);
                                                        if (pCtrl->CHN[BGN_CNR_49_HEADING].x01.CounterRx.iCounter == 0) {
                                                            // HEADING was never touched, do not go to AUTO
                                                        } else {
                                                            // i'm not going to make things more complicated at this point. But realise that when
                                                            // we have no accurate HEADING data, i do not know where to steer the boad when going to AUTO
                                                        }
                                                        // get the new STATE; initialize the CTARGET with current HEADING if we go into AUTO
                                                        if (pCtrl->ACP.STATE.value.nByte == BGN_AP_MODE1_STBY) { // current STATE is STBY
                                                            if (pCHR->s.cv.uB[1] == BGN_AP_MODE1_AUTO) { // new STATE is AUTO
                                                                // it might be the case we do not have a recent HEADING channel received prior.
                                                                // in such case we initialize to 0 (good luck!)
                                                                // should we actually refuse to go into AUTO?
                                                                pCtrl->ACP.CourseTarget.value.nLong = ntohs(((union CH*)(pCtrl->CHN[BGN_CNR_49_HEADING].x01.ucRAW))->s.cv.uW[0]);
                                                                pCtrl->ACP.CourseTarget.source = BGN_SOURCE_ACP;
                                                                CounterUp(&pCtrl->ACP.CourseTarget.Counter, t);
                                                            }
                                                        }
                                                        // set new STATE (AUTO or STBY)
                                                        pCtrl->ACP.STATE.value.nByte = pCHR->s.cv.uB[1];
                                                        pCtrl->ACP.STATE.source = BGN_SOURCE_ACP;
                                                        CounterUp(&pCtrl->ACP.STATE.Counter, t);
                                                        // make DATA command (to ALL, so also other PILOT units, if present, receive it)
                                                        len_pl = makeACPresponseSTATUS(pCtrl, payload);
                                                        len_msg = bgnet_encode_frame(&TxBuf[TxSize], BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                                                        memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].ucFrameTx, &TxBuf[TxSize], len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].CounterTx, t);
                                                        memcpy(&pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                                                        CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                        TxSize =TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_FRAME_CMD_PILOT_MODE:
                                                // the MODE key was pressed. We receive 5 bytes payload:
                                                // 0=ACKSEQ, 1=0xA6/AP_CTARGET, 2=FMT_01, 3=0, 4= new state (1=COMP, 3=NAV, 4=VANE)
                                                // the SPEED selector is made (SPEED key, but also PILOT/SETUP/SPD selection)
                                                // 0=ACKSEQ, 1=0x46/AP_SPEED, 2=FMT_01, 3=0, 4= new SPEED mode (0=TRA, 1=MAN, 2=SOG)
                                                // Note: MANUAL speed selection can be set at 3 bands (SLW, MED, FST) via the PILOT/SPEED key (select
                                                // SETUP/SPD/MAN first)
                                                switch(pCHR->s.CH) {
                                                    // MODE SELECT (COMP/NAV/VANE)
                                                    case BGN_CNR_A6_AP_CTARGET:
                                                        // update the channel value
                                                        pCtrl->CHN[pCHR->s.CH].iNewChannel = BGN_NEW_SET;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucTO = pFR->TO;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucFROM = pFR->FROM;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucCMD = pFR->CMD;
                                                        y = bgnet_channel_length(pCtrl, pCHR);
                                                        memcpy(pCtrl->CHN[pCHR->s.CH].x00.ucRAW, pCHR->ucRAW, y);
                                                        decode_channel_string(pCtrl->CHN[pCHR->s.CH].x00.cValue, pCHR);
                                                        CounterUp(&pCtrl->CHN[pCHR->s.CH].x00.CounterRx, t);

                                                        // get the new MODE; do not set mode if we are in JOYSTCK
                                                        if (pCtrl->ACP.Joystick.value.nByte == BGN_JOYSTICK_OFF) {
                                                            pCtrl->ACP.MODE.value.nByte = pCHR->s.cv.uB[1];
                                                            pCtrl->ACP.MODE.source = BGN_SOURCE_ACP;
                                                            CounterUp(&pCtrl->ACP.MODE.Counter, t);
                                                        }

                                                        // make DATA command (to ALL, so also other PILOT units, if present, receive it)
                                                        len_pl = makeACPresponseSTATUS(pCtrl, payload);
                                                        len_msg = bgnet_encode_frame(&TxBuf[TxSize], BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                                                        memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].ucFrameTx, &TxBuf[TxSize], len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].CounterTx, t);
                                                        memcpy(&pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                                                        CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                    // SETUP > SPEED SELECT (TRA/MAN/SOG) > ENTER
                                                    case BGN_CNR_46_AP_SPEED:
                                                        // update the channel value
                                                        pCtrl->CHN[pCHR->s.CH].iNewChannel = BGN_NEW_SET;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucTO = pFR->TO;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucFROM = pFR->FROM;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucCMD = pFR->CMD;
                                                        y = bgnet_channel_length(pCtrl, pCHR);
                                                        memcpy(pCtrl->CHN[pCHR->s.CH].x00.ucRAW, pCHR->ucRAW, y);
                                                        decode_channel_string(pCtrl->CHN[pCHR->s.CH].x00.cValue, pCHR);
                                                        CounterUp(&pCtrl->CHN[pCHR->s.CH].x00.CounterRx, t);

                                                        // get the new SPEED SELECT from the channel
                                                        pCtrl->ACP.SpeedSelect.value.nByte = pCHR->s.cv.uB[1];
                                                        pCtrl->ACP.SpeedSelect.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->ACP.SpeedSelect.Counter, t);
                                                        break;
                                                    // SETUP > FR SELECT (ON/OFF) > ENTER
                                                    case BGN_CNR_07_AP_FR:
                                                        // update the channel value
                                                        pCtrl->CHN[pCHR->s.CH].iNewChannel = BGN_NEW_SET;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucTO = pFR->TO;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucFROM = pFR->FROM;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucCMD = pFR->CMD;
                                                        y = bgnet_channel_length(pCtrl, pCHR);
                                                        memcpy(pCtrl->CHN[pCHR->s.CH].x00.ucRAW, pCHR->ucRAW, y);
                                                        decode_channel_string(pCtrl->CHN[pCHR->s.CH].x00.cValue, pCHR);
                                                        CounterUp(&pCtrl->CHN[pCHR->s.CH].x00.CounterRx, t);

                                                        // get the new FA SELECT from the channel
                                                        pCtrl->ACP.SETUP_FR.value.nByte = pCHR->s.cv.uB[1];
                                                        pCtrl->ACP.SETUP_FR.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->ACP.SETUP_FR.Counter, t);
                                                        break;
                                                }
                                                break;
                                            case BGNET_FRAME_CMD_PILOT_SPDREQ:
                                                // the SPEED key was pressed. We receive 2 bytes payload.
                                                // 0=ACKSEQ, 1=0x46
                                                switch(pCHR->s.CH) {
                                                    case BGN_CNR_46_AP_SPEED:
                                                        // create command with current SPEED SELECT
                                                        payload[0] = ACPAckSeqGet(pCtrl, pFR, t);
                                                        payload[1] = BGN_CNR_46_AP_SPEED;
                                                        payload[2] = BGN_FMT_01;
                                                        payload[3] = 0;
                                                        payload[4] = pCtrl->ACP.SpeedSelect.value.nByte;
                                                        len_pl = 5;
                                                        len_msg = bgnet_encode_frame(&TxBuf[TxSize], pFR->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_PILOT_MODE, payload, len_pl);

                                                        memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_PILOT_MODE].ucFrameTx, &TxBuf[TxSize], len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_PILOT_MODE].CounterTx, t);
                                                        memcpy(&pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                                                        CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                    case BGN_CNR_07_AP_FR:
                                                        // create command with current FR SELECT
                                                        payload[0] = ACPAckSeqGet(pCtrl, pFR, t);
                                                        payload[1] = BGN_CNR_07_AP_FR;
                                                        payload[2] = BGN_FMT_01;
                                                        payload[3] = 0;
                                                        payload[4] = pCtrl->ACP.SETUP_FR.value.nByte;
                                                        len_pl = 5;
                                                        len_msg = bgnet_encode_frame(&TxBuf[TxSize], pFR->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_PILOT_MODE, payload, len_pl);

                                                        memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_PILOT_MODE].ucFrameTx, &TxBuf[TxSize], len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_PILOT_MODE].CounterTx, t);
                                                        memcpy(&pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                                                        CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_FRAME_CMD_PILOT_RESPREQ:
                                                // in SETUP, the RESPONSE parameter is configured, 2 bytes payload.
                                                // 0=ACKSEQ, 1=0xA6
                                                switch(pCHR->s.CH) {
                                                    case BGN_CNR_A6_AP_CTARGET:
                                                        // create command with current RESP SELECT
                                                        //payload[0] = ACPAckSeqGet(pCtrl, pF, t);
                                                        payload[0] = pFR->payload[0];
                                                        payload[1] = BGN_CNR_A6_AP_CTARGET;
                                                        payload[2] = BGN_FMT_01;
                                                        payload[3] = 0;
                                                        payload[4] = pCtrl->ACP.Response.value.nByte;
                                                        len_pl = 5;
                                                        len_msg = bgnet_encode_frame(&TxBuf[TxSize], pFR->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_PILOT_RESPDAT, payload, len_pl);

                                                        memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_PILOT_RESPDAT].ucFrameTx, &TxBuf[TxSize], len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_PILOT_RESPDAT].CounterTx, t);
                                                        memcpy(&pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                                                        CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_FRAME_CMD_PILOT_RESPDAT:
                                                // in SETUP, the RESPONSE parameter is configured, 5 bytes payload.
                                                // 0=ACKSEQ, 1=0xA6/AP_CTARGET, 2=FMT_01, 3=0, 4= new RESP selector
                                                switch(pCHR->s.CH) {
                                                    case BGN_CNR_A6_AP_CTARGET:
                                                        // update the channel value
                                                        pCtrl->CHN[pCHR->s.CH].iNewChannel = BGN_NEW_SET;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucTO = pFR->TO;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucFROM = pFR->FROM;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucCMD = pFR->CMD;
                                                        y = bgnet_channel_length(pCtrl, pCHR);
                                                        memcpy(pCtrl->CHN[pCHR->s.CH].x00.ucRAW, pCHR->ucRAW, y);
                                                        decode_channel_string(pCtrl->CHN[pCHR->s.CH].x00.cValue, pCHR);
                                                        CounterUp(&pCtrl->CHN[pCHR->s.CH].x00.CounterRx, t);

                                                        pCtrl->ACP.Response.value.nByte = pCHR->s.cv.uB[1];
                                                        pCtrl->ACP.Response.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->ACP.Response.Counter, t);
                                                        break;
                                                }
                                                break;
                                            case BGNET_FRAME_CMD_PILOT_WAREQ:
                                                // in PILOT/SETUP, the WATCH ALARM is configured, 2 bytes payload
                                                // 0=ACKSEQ, 1=0x76
                                                switch(pCHR->s.CH) {
                                                    case BGN_CNR_76_AP_WA_ALARM:
                                                        // create command with current WA SELECT
                                                        payload[0] = ACPAckSeqGet(pCtrl, pFR, t);
                                                        payload[1] = BGN_CNR_76_AP_WA_ALARM;
                                                        payload[2] = BGN_FMT_18;
                                                        uint16_t uwTemp = htons(pCtrl->ACP.WA.value.nLong);
                                                        payload[3] = (unsigned char)(uwTemp & 0x00FF);
                                                        payload[4] = (unsigned char)((uwTemp & 0xFF00) >> 8);
                                                        len_pl = 5;
                                                        len_msg = bgnet_encode_frame(&TxBuf[TxSize], pFR->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_PILOT_WADAT, payload, len_pl);
                                                        
                                                        memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_PILOT_WADAT].ucFrameTx, &TxBuf[TxSize], len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_PILOT_WADAT].CounterTx, t);
                                                        memcpy(&pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                                                        CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_FRAME_CMD_PILOT_WADAT:
                                                // in PILOT/SETUP, when WATCH ALARM setting is entered, 5 bytes payload
                                                // 0=ACKSEQ, 1=0x76, 2=FMT_01, 3/4=WA limit in minutes (1-360)
                                                switch(pCHR->s.CH) {
                                                    case BGN_CNR_76_AP_WA_ALARM:
                                                        // update the channel value 
                                                        pCtrl->CHN[pCHR->s.CH].iNewChannel = BGN_NEW_SET;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucTO = pFR->TO;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucFROM = pFR->FROM;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucCMD = pFR->CMD;
                                                        y = bgnet_channel_length(pCtrl, pCHR);
                                                        memcpy(pCtrl->CHN[pCHR->s.CH].x00.ucRAW, pCHR->ucRAW, y);
                                                        decode_channel_string(pCtrl->CHN[pCHR->s.CH].x00.cValue, pCHR);
                                                        CounterUp(&pCtrl->CHN[pCHR->s.CH].x00.CounterRx, t);

                                                        pCtrl->ACP.WA.value.nLong = ntohs(pCHR->s.cv.uW[0]);
                                                        pCtrl->ACP.WA.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->ACP.WA.Counter, t);
                                                        break;
                                                }
                                                break;
                                            case BGNET_FRAME_CMD_PILOT_OCREQ:
                                                // in PILOT/SETUP, the OFF-COURSE ALARM is configured, 2 bytes payload.
                                                // 0=ACKSEQ, 1=0xAF
                                                switch(pCHR->s.CH) {
                                                    case BGN_CNR_AF_AP_OC_ALARM:
                                                        // create command with current OC SELECT
                                                        payload[0] = ACPAckSeqGet(pCtrl, pFR, t);
                                                        payload[1] = BGN_CNR_AF_AP_OC_ALARM;
                                                        payload[2] = BGN_FMT_18;
                                                        uint16_t uwTemp = htons(pCtrl->ACP.OC.value.nLong);
                                                        payload[3] = (unsigned char)(uwTemp & 0x00FF);
                                                        payload[4] = (unsigned char)((uwTemp & 0xFF00) >> 8);
                                                        len_pl = 5;
                                                        len_msg = bgnet_encode_frame(&TxBuf[TxSize], pFR->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_PILOT_OCDAT, payload, len_pl);
                                                        
                                                        memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_PILOT_OCDAT].ucFrameTx, &TxBuf[TxSize], len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_PILOT_OCDAT].CounterTx, t);
                                                        memcpy(&pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                                                        CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_FRAME_CMD_PILOT_OCDAT:
                                                // in PILOT/SETUP, when OFF-COURSE ALARM setting is entered, 5 bytes payload
                                                // 0=ACKSEQ, 1=0x76, 2=FMT_01, 3/4=OC limit in degrees (1-99)
                                                switch(pCHR->s.CH) {
                                                    case BGN_CNR_AF_AP_OC_ALARM:
                                                        // update the channel value 
                                                        pCtrl->CHN[pCHR->s.CH].iNewChannel = BGN_NEW_SET;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucTO = pFR->TO;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucFROM = pFR->FROM;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucCMD = pFR->CMD;
                                                        y = bgnet_channel_length(pCtrl, pCHR);
                                                        memcpy(pCtrl->CHN[pCHR->s.CH].x00.ucRAW, pCHR->ucRAW, y);
                                                        decode_channel_string(pCtrl->CHN[pCHR->s.CH].x00.cValue, pCHR);
                                                        CounterUp(&pCtrl->CHN[pCHR->s.CH].x00.CounterRx, t);

                                                        pCtrl->ACP.OC.value.nLong = ntohs(pCHR->s.cv.uW[0]);
                                                        pCtrl->ACP.OC.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->ACP.OC.Counter, t);
                                                        break;
                                                }
                                                break;
                                            case BGNET_FRAME_CMD_PILOT_ADJUST:
                                                // 1 of the 4 +/- keys was pressed, 5 bytes payload: 0-ACKSEQ, 1=0xA6/CHID, 2=0x01/FMT, 3/4=signed 16bits inc/decr value
                                                switch(pCHR->s.CH) {
                                                    case BGN_CNR_A6_AP_CTARGET:
                                                        // inc/dec displayed value (CTARGET) when in AUTO
                                                        if (pCtrl->ACP.STATE.value.nByte == BGN_AP_MODE1_AUTO) {
                                                            iValue = decode_channel_numeric(pCHR);
                                                            pCtrl->ACP.CourseTarget.value.nLong = pCtrl->ACP.CourseTarget.value.nLong + iValue;
                                                            pCtrl->ACP.CourseTarget.source = BGN_SOURCE_ACP;
                                                            CounterUp(&pCtrl->ACP.CourseTarget.Counter, t);
                                                        }
                                                        len_pl = makeACPresponseCTARGET(pCtrl, payload);
                                                        len_msg = bgnet_encode_frame(&TxBuf[TxSize], BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                                                        memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].ucFrameTx, &TxBuf[TxSize], len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].CounterTx, t);
                                                        memcpy(&pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                                                        CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                        TxSize =TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_FRAME_CMD_PILOT_DMPREQ:
                                                // in PILOT/SETUP, the COMPASS DMP is configured, 2 bytes payload
                                                // 0=ACKSEQ, 1=0x49
                                                switch(pCHR->s.CH) {
                                                    case BGN_CNR_49_HEADING:
                                                        // create command with current DMP setting
                                                        payload[0] = ACPAckSeqGet(pCtrl, pFR, t);
                                                        payload[1] = BGN_CNR_49_HEADING;
                                                        payload[2] = BGN_FMT_01;
                                                        payload[3] = 0;
                                                        payload[4] = pCtrl->ACP.SETUP_DMP.value.nByte; // (0-99)                                                        len_pl = 5;
                                                        len_pl = 5;
                                                        len_msg = bgnet_encode_frame(&TxBuf[TxSize], pFR->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_PILOT_DMPDAT, payload, len_pl);

                                                        memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_PILOT_DMPDAT].ucFrameTx, &TxBuf[TxSize], len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_PILOT_DMPDAT].CounterTx, t);
                                                        memcpy(&pCtrl->DEV[BGNET_ADDRESS_ME].ucFrameTx, &TxBuf[TxSize], len_msg);
                                                        CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_FRAME_CMD_PILOT_DMPDAT:
                                                // in PILOT/SETUP, when COMPASS DMP setting is entered, 5 bytes payload
                                                // 0=ACKSEQ, 1=0x49, 2=FMT_01, 3=0, 4=DMP in seconds (1-99, 4=default)
                                                switch(pCHR->s.CH) {
                                                    case BGN_CNR_49_HEADING:
                                                        // update the channel value 
                                                        pCtrl->CHN[pCHR->s.CH].iNewChannel = BGN_NEW_SET;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucTO = pFR->TO;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucFROM = pFR->FROM;
                                                        pCtrl->CHN[pCHR->s.CH].x00.ucCMD = pFR->CMD;
                                                        y = bgnet_channel_length(pCtrl, pCHR);
                                                        memcpy(pCtrl->CHN[pCHR->s.CH].x00.ucRAW, pCHR->ucRAW, y);
                                                        decode_channel_string(pCtrl->CHN[pCHR->s.CH].x00.cValue, pCHR);
                                                        CounterUp(&pCtrl->CHN[pCHR->s.CH].x00.CounterRx, t);

                                                        pCtrl->ACP.SETUP_DMP.value.nByte = (pCHR->s.cv.uB[1]);
                                                        pCtrl->ACP.SETUP_DMP.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->ACP.SETUP_DMP.Counter, t);
                                                        break;
                                                }
                                                break;
                                        }

                                        // Step 2 - prepare the ACK message
                                        payload[0] = pFR->payload[0]; // get 1st payload byte and return it
                                        len_pl = 1;
                                        // make ACK frame, update statistics
                                        len_msg = bgnet_encode_frame(&TxBuf[TxSize], pFR->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_ACK, payload, len_pl);
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
                                    default:
                                        // unhandled command, also no ACK required
                                        iIsValid = 0; // do not update device/command statistics
                                        x = bgnet_deviceindex(pCtrl, *(pBuf+BGNET_FRAME_HEAD_FROM)); // lookup by device address/type, so we can print the device name
                                        log_info("  Command %02X (with body) IGNORED (from %02X %s)", pFR->CMD, pFR->FROM, pCtrl->pDEVDT[x].sModelName);
                                        break;
                                }
                        }
                        // we only want to register our incoming command if we know what it is.
                        if (iIsValid > 0) {
                            // update CMD structure
                            pCtrl->iNewCommand = BGN_NEW_SET; // signal possible CMD change
                            pCtrl->CMD[pFR->CMD].iNewCommand = BGN_NEW_SET; // signal this command
                            memcpy(pCtrl->CMD[pFR->CMD].ucFrameRx, pBuf, RxSize);
                            CounterUp(&pCtrl->CMD[pFR->CMD].CounterRx, t);
                        }   
                    } else {
                        // payload CS is bad, only skip this sub-frame, the length is known
                        log_error("    Frame Checksum ERR, framesize: %d (received: 0x%02X, calculated: 0x%02X)", RxSize, pFR->payload[pFR->PL], cs);
                        pCtrl->DEV[pFR->FROM].iCSError_Frame++;
                    }
                }
            }
        } else {
            // header CS is bad, we can not process this subframe, we can not establish a subframe length unambigiously, so we dump the whole buffer
            log_error("  Header Checksum ERR, dumping RX frame (received: 0x%02X, calculated: 0x%02X)", pFR->CSH, cs);
            pCtrl->DEV[pFR->FROM].iCSError_Header++;
            break;
        }
        pBuf = pBuf + RxSize;
        z = z + 1;
    }
    return(z);
}

#endif
