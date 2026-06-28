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

        pCtrl->ACP.STATE.value.nByte = BGN_AP_MODE1_AUTO;
        pCtrl->ACP.STATE.source = BGN_SOURCE_NONE;
        pCtrl->ACP.STATE.timestamp = t;

        pCtrl->ACP.MODE.value.nByte = BGN_AP_MODE2_COMP;
        pCtrl->ACP.MODE.source = BGN_SOURCE_NONE;
        pCtrl->ACP.MODE.timestamp = t;

        pCtrl->ACP.Joystick.value.nByte = 0; // 0=OFF, 1=ON, overrides control by PILOT
        pCtrl->ACP.Joystick.source = BGN_SOURCE_NONE;
        pCtrl->ACP.Joystick.timestamp = t;

        pCtrl->ACP.BackLight.value.nByte = 0; // backlight (0=OFF, 3-1=ON)
        pCtrl->ACP.BackLight.source = BGN_SOURCE_NONE;
        pCtrl->ACP.BackLight.timestamp = t;

        pCtrl->ACP.Memory.value.nLong = 123; // Course To Stear saved when going to STBY
        pCtrl->ACP.Memory.source = BGN_SOURCE_NONE;
        pCtrl->ACP.Memory.timestamp = t;

        pCtrl->ACP.StoredLogNM.value.nLong = 12345*10+6; // channel CD/STORELOGNM (on the real system this is stored in non-volitile memory)
        pCtrl->ACP.StoredLogNM.source = BGN_SOURCE_NONE;
        pCtrl->ACP.StoredLogNM.timestamp = t;

        pCtrl->ACP.TripLogNM.value.nLong = 1234*10+5; // channel CD/STORELOGNM (on the real system this is stored in non-volitile memory)
        pCtrl->ACP.TripLogNM.source = BGN_SOURCE_NONE;
        pCtrl->ACP.TripLogNM.timestamp = t;

        // stage 1 - initialize device listVA
        pCtrl->iNewDevice = 0; // no new data available
        for (i=0; i<BGNET_DEVICE_MAX; i++) {
            // error counters Checksum
            pCtrl->DEV[i].iCSError_Header = 0;
            pCtrl->DEV[i].iCSError_Frame = 0;
            // packet counters & timestamps
            pCtrl->DEV[i].iCounterRx = 0;
            pCtrl->DEV[i].iCounterTx = 0;
            pCtrl->DEV[i].tTimestampRx = t;
            pCtrl->DEV[i].tCounterResetRx = t;
            pCtrl->DEV[i].tTimestampTx = t;
            pCtrl->DEV[i].tCounterResetTx = t;
            memset(pCtrl->DEV[i].ucFrameRx, 0, BGN_FRAME_SIZE);
            memset(pCtrl->DEV[i].ucFrameTx, 0, BGN_FRAME_SIZE);
        }

        // stage 2 - initialize command list
        for (i=0; i<BGNET_COMMAND_MAX; i++) {
            pCtrl->CMD[i].iNewCommand = 0;
            pCtrl->CMD[i].iCounterRx = 0;
            pCtrl->CMD[i].tTimestampRx = t;
            pCtrl->CMD[i].tCounterResetRx = t;
            pCtrl->CMD[i].iCounterTx = 0;
            pCtrl->CMD[i].tTimestampTx = t;
            pCtrl->CMD[i].tCounterResetTx = t;
            memset(pCtrl->CMD[i].ucFrameRx, 0, BGN_FRAME_SIZE);
            memset(pCtrl->CMD[i].ucFrameTx, 0, BGN_FRAME_SIZE);
        }

        // stage 3 - initialize channel list
        for (i=0; i<BGN_CHANNEL_MAX; i++) {
            pCtrl->DATA[i].ucFROM = 0;
            memset(pCtrl->DATA[i].ucRAW, 0, sizeof(pCtrl->DATA[i].ucRAW));
            strcpy(pCtrl->DATA[i].cValue, "");
            pCtrl->DATA[i].iCounter = 0;
            pCtrl->DATA[i].iNewChannel = 0;
            pCtrl->DATA[i].tTimestamp = t;
            pCtrl->DATA[i].tCounterReset = t;
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

    c = BGN_CNR_81_TRIPLOGNM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(333); 
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_CF_DR_DISTNM;
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
    // this message is send from PILOT to ACP when the AUTO or OFF button is pressed
int makeAUTOSTBYack(BGNET_CTRL *pCtrl, unsigned char* payload, int iState, int iMode) {
    int r = 5;
    payload[0] = 0x00;
    payload[1] = 0xB5;
    payload[2] = 0x01; // request AUTO/STBY
    payload[3] = 0x00; //iMode; // mode request
    payload[4] = iState; // unknown
    return(r);
}
// this response is only send whenever a button on the PILOT was pressed
int makeACPresponseFAST(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r = 0,c;
    union CH* puCH;

    c = BGN_CNR_53_TARGET_TWA;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(-9);
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_A6_AP_CTARGET;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(353);
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_A7_AP_WPB;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(355);
    r = r + ChannelByteLength(pCtrl, puCH);

    c = BGN_CNR_B5_AP_MODE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uB[0] = pCtrl->ACP.STATE.value.nByte; // AUTO/STBY
    puCH->s.cv.uB[1] = pCtrl->ACP.MODE.value.nByte;  // COMP/NAV/VANE
    r = r + ChannelByteLength(pCtrl, puCH);

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
// replay message on TRIP RESET by QUAD
int makeQUADresponseRESET(BGNET_CTRL *pCtrl, unsigned char* payload) {
    payload[0] = 0;
    payload[1] = BGN_CNR_CF_DR_DISTNM;
    return(2);
}

// use this macro to convert a floating point value (double) to integer with round-off
#define DTOIROUND(d) ((int)(d >= 0.0 ? (d + 0.5) : (d - 0.5)))

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

int makeAUTOresponse(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int len;
    time_t t;
    time(&t);
    len = 0;
    payload[0] = BGN_CNR_B5_AP_MODE;
    payload[1] = pCtrl->pCHNDT[BGN_CNR_B5_AP_MODE].ucFMT;
    payload[2] = pCtrl->ACP.STATE.value.nByte; // AUTO/STBY
    if (pCtrl->ACP.Joystick.value.nByte == 0) {
        payload[3] = pCtrl->ACP.MODE.value.nByte;
    } else {
        payload[3] = BGN_AP_MODE2_JOYSTICK;
    }
    payload[4] = BGN_CNR_0B_RUDDER;
    payload[5] = pCtrl->pCHNDT[BGN_CNR_0B_RUDDER].ucFMT;
    payload[6] = 0;
    payload[7] = 0x0A;  // 0A = 10
    payload[8] = BGN_CNR_49_HEADING;
    payload[9] = pCtrl->pCHNDT[BGN_CNR_49_HEADING].ucFMT;
    payload[10] = 0;
    payload[11] = 0x4D; // 0x4D = 77
    payload[12] = BGN_CNR_8D_BATTVOLT;

    payload[13] = pCtrl->pCHNDT[BGN_CNR_8D_BATTVOLT].ucFMT;
    payload[14] = 0x00;
    payload[15] = 0x61; // 61 = 97 = 9.7
    payload[15] = (t & 0x0F); 
    len = 16;
    return (len);
}
int makeAUTOresponse2(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int len;
    time_t t;
    time(&t);
    len = 0;
    payload[0] = BGN_CNR_B5_AP_MODE;
    payload[1] = pCtrl->pCHNDT[BGN_CNR_B5_AP_MODE].ucFMT;
    payload[2] = pCtrl->ACP.STATE.value.nByte; // AUTO/STBY
    if (pCtrl->ACP.Joystick.value.nByte == 0) {
        payload[3] = pCtrl->ACP.MODE.value.nByte;
    } else {
        payload[3] = BGN_AP_MODE2_JOYSTICK;
    }
    payload[4] = BGN_CNR_0B_RUDDER;
    payload[5] = pCtrl->pCHNDT[BGN_CNR_0B_RUDDER].ucFMT;
    payload[6] = 0;
    payload[7] = 0x0A;  // 0A = 10
    payload[8] = BGN_CNR_49_HEADING;
    payload[9] = pCtrl->pCHNDT[BGN_CNR_49_HEADING].ucFMT;
    payload[10] = 0;
    payload[11] = 0x4D; // 0x4D = 77
    payload[12] = BGN_CNR_8D_BATTVOLT;

    payload[13] = pCtrl->pCHNDT[BGN_CNR_8D_BATTVOLT].ucFMT;
    payload[14] = 0x00;
    payload[15] = 0x61; // 61 = 97 = 9.7
    payload[15] = (t & 0x0F); 
    len = 16;
    return (len);
}
int makeSPEEDresponse(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int len;
    len = 0;
    union CH* puCH;

    puCH = (union CH*)&(payload[0]);
    puCH->s.CH = BGN_CNR_46_AP_SPEEDKT;
    puCH->s.FMT = 0x08;
    puCH->s.cv.uB[0] = 1;
    puCH->s.cv.uB[0] = 2;

    puCH = (union CH*)&(payload[4]);
    puCH->s.CH = BGN_CNR_41_BOATSPEEDKT;
    puCH->s.FMT = 0x08;
    puCH->s.cv.uB[0] = 1;
    puCH->s.cv.uB[0] = 2;

    puCH = (union CH*)&(payload[8]);
    puCH->s.CH = BGN_CNR_EB_SOGKT;
    puCH->s.FMT = 0x08;
    puCH->s.cv.uB[0] = 1;
    puCH->s.cv.uB[0] = 2;

    len = 12;
    return (len);
}

// update timestamp and increment counter (with wrap-around)
void CounterUpDATA(BGN_CHANNEL *pCH, time_t t) {
    pCH->iNewChannel = 1; // signal this channel as new/changed
    pCH->tTimestamp = t;
    // update counter (with wrap-around)
    if (pCH->iCounter < SHRT_MAX) {
        pCH->iCounter++;
    } else {
        pCH->iCounter = 1;
        pCH->tCounterReset = t;
    }
}
void CounterRxUpCMD(BGN_COMMAND *pCMD, time_t t) {
    pCMD->tTimestampRx = t;
    // update counter (with wrap-around)
    if (pCMD->iCounterRx < SHRT_MAX) {
        pCMD->iCounterRx++;
    } else {
        pCMD->iCounterRx = 1;
        pCMD->tCounterResetRx = t;
    }
}
void CounterTxUpCMD(BGN_COMMAND *pCMD, time_t t) {
    pCMD->tTimestampTx = t;
    // update counter (with wrap-around)
    if (pCMD->iCounterTx < SHRT_MAX) {
        pCMD->iCounterTx++;
    } else {
        pCMD->iCounterTx = 1;
        pCMD->tCounterResetTx = t;
    }
}
void CounterRxUpDEV(BGN_DEVICE *pDEV, time_t t) {
    pDEV->tTimestampRx = t;
    // update counter (with wrap-around)
    if (pDEV->iCounterRx < SHRT_MAX) {
        pDEV->iCounterRx++;
    } else {
        pDEV->iCounterRx = 1;
        pDEV->tCounterResetRx = t;
    }
}
void CounterTxUpDEV(BGN_DEVICE *pDEV, time_t t) {
    pDEV->tTimestampTx = t;
    // update counter (with wrap-around)
    if (pDEV->iCounterTx < SHRT_MAX) {
        pDEV->iCounterTx++;
    } else {
        pDEV->iCounterTx = 1;
        pDEV->tCounterResetTx = t;
    }
}

// This is the "device processor". Data received was placed into the control block. Here we scan the
// control block to see if we have to generate a reply message.
// We attempt to emulate the real thing.
// Realise, while this routines executes, other threads are locked out. Do not sleep or do slow/blocking actions.
// What we put in the output buffer is a complete frame, possibly with sub-frames. ready to send.
size_t bgn_net_process(BGNET_CTRL *pCtrl, unsigned char* buffer, size_t size) {
    unsigned char tmp_buffer[512]; // might hold op to 1 frame (max 200 bytes)
    unsigned char payload[256]; // usually a frame is usually less than 200 bytes
    char asciibuffer[256];
    int i;
    size_t len_pl,len_msg;
    time_t t;
    size_t r = 0;

    // device received new data
    if ((pCtrl->iNewDevice < 1)) {
        // nothing to do
    } else {

        time(&t); // save the time

        // Stage 1 - check for new commands
        for (i=0; i<BGNET_COMMAND_MAX; i++) {
            if (pCtrl->CMD[i].iNewCommand < 1) {
                // nothing to do
            } else {
                log_info("bgn_net_process: Command 0x%02X processing", i);
                switch(i) {
                    case BGNET_FRAME_CMD_KEEPALIVE:
                        // we received a KEEPALIVE signal (only by PILOT). generate a ACP response message. The PILOT needs both the FAST and the
                        // scheduled message to come alive
                        len_pl = makeACPresponseFAST(pCtrl, payload);
                        len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                        memcpy(&buffer[r],&tmp_buffer,len_msg); // put output message in write buffer
                        
                        r = r + len_msg; // increment byte counter in TX buffer
                        //len_pl = makeACPresponse(pCtrl, payload);
                        //len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                        //memcpy(&buffer[r],&tmp_buffer,len_msg); // put output message in write buffer
                        //r = r + len_msg; // increment byte counter in TX buffer
                        break;
                    case BGNET_FRAME_CMD_POWERUP:
                        // we received a powerup from the network. This happens when the network did a POWERUP while we are running
                        // to synchronize what we are doing, set the init request. The actual reset is done the next "device" run.
                        bgn_net_initialize_reset(pCtrl);
                        // no reply message required
                        break;
                    case BGNET_FRAME_CMD_LATLON:
                    case BGNET_FRAME_CMD_DATA:
                        // these 2 commands send channel data; data was placed into the DATA structure by the "frame decoder".
                        // no response required
                        break;
                    case BGNET_FRAME_CMD_QUAD_RESET:
                    case BGNET_FRAME_CMD_QUAD_RESET_R:
                        // replay is immediately send when received; nothing to do here            
                        break;
                    case BGNET_FRAME_CMD_PILOT_AM:
                        //len_pl = makeAUTOresponse(pCtrl, payload);
                        //len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                        //memcpy(&buffer[r], &tmp_buffer, len_msg); // put output message in write buffer
                        //r = r + len_msg; // increment byte counter in TX buffer
                        break;
                    case BGNET_FRAME_CMD_PILOT_MODE:
                        //len_pl = makeAUTOresponse(pCtrl, payload);
                        //len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                        //memcpy(&buffer[r], &tmp_buffer, len_msg); // put output message in write buffer
                        //r = r + len_msg; // increment byte counter in TX buffer
                        break;
                    case BGNET_FRAME_CMD_PILOT_SPEED:
                        //len_pl = makeSPEEDresponse(pCtrl, payload);
                        //len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                        //memcpy(&buffer[r], &tmp_buffer, len_msg); // put output message in write buffer
                        //r = r + len_msg; // increment byte counter in TX buffer
                        break;
                    case BGNET_FRAME_CMD_PILOT_STEP:
                        //len_pl = makeAUTOresponse(pCtrl, payload);
                        //len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                        //memcpy(&buffer[r], &tmp_buffer, len_msg); // put output message in write buffer
                        //r = r + len_msg; // increment byte counter in TX buffer
                        break;
                }
                pCtrl->CMD[i].iNewCommand = 0; // reset new data flag for this command
            }
        }

        // stage 2 - check for new channel data
        for (i=0; i<BGN_CHANNEL_MAX; i++) {
            if (pCtrl->DATA[i].iNewChannel < 1) {
                // nothing to do
            } else {
                // process new channel data (to be added later)
                pCtrl->DATA[i].iNewChannel = 0; // reset newdata flag
            }
        }

//#define BGN_ENABLE_QUAD_TX
        // stage 3 - do some scheduled messages
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
        // stage 4 - error incase too much data
        if ((len_msg * 10) > (size * 9)) {
            log_error("Device/Tx buffer dangerously full (>90%)");
        }


        // stage 5 - check if we are allowed to send output
        // check if we are in RW mode; if not, destroy our output. I'd rather do it here that making the logic complexer
        // by integrating the readonly/readwrite mode into the logic
        if (g_serial_mode == BGN_SERIAL_MODE_RW) {
            // leave output data
        } else {
            r = 0; // destroy what we have in the buffer
        }

        // / stage 6 - reset new data flag
        pCtrl->iNewDevice = 0;
    }
    
    // if we produced some response, show it here
    if (r > 0) {
        byte2ascii(asciibuffer, sizeof(asciibuffer), buffer, r, BYTE2ASCII_SPACED);
        log_info("bgn_net_process: output: %s (%ld bytes)", asciibuffer, r);
    }
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
char* decode_channel(char* pStr, union CH* puCH) {
    int iRawValue;
    int iValid; // 0= invalid DataType, set to "???", 1= Numerical, convert iValue byte sequence, 2= ASCII, already set (text, timer, hex)
    unsigned int d;
    const double fDiv[4]    = { 1.0,   10.0,  100.0, 1000.0};
    const char cPrint[4][6] = {"%d", "%.1f", "%.2f", "%.3f"};
    switch(puCH->s.FMT & BGN_FMT_MASK_DT) {
        case BGN_FMT_01: // signed 16bit register (-999 to +999)
            iRawValue = (int16_t)ntohs(puCH->s.cv.sW[0]);
            iValid = 1;
            break;
        case BGN_FMT_02: // unsigned 32bit register (0xCD, storedlog)
            iRawValue = ntohl(puCH->s.cv.uL[0]);
            iValid = 1;
            break;
        case BGN_FMT_03: // 3 bytes timer value (0x75, timer)
                         // note: QUAD displays H:MM:SS maximum (left digit is not displayed
                         //       DATA displays H:MM only (seconds are displayed in the topline)
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
            if (puCH->s.CH == BGN_CNR_B5_AP_MODE) {
                sprintf(pStr,"0x%02X 0x%02X", puCH->s.cv.uB[0], puCH->s.cv.uB[1]);
                iValid = 2;
            } else {
                iRawValue = ntohs(puCH->s.cv.uW[0]);
                iValid = 1;
            }
            break;
        default:
            pStr[0] = '?';
            pStr[1] = '?';
            pStr[2] = '?';
            pStr[3] = '\0';
            iValid = 0;
    }
    // iValid: 0= invalid DataType, set to "???", 1= Numerical, convert iValue byte sequence, 2= ASCII, already set
    if (iValid == 1) {
        // get divisor value (0-3) from FMT byte
        d = (puCH->s.FMT & BGN_FMT_MASK_DIVPAD_DIV)>>6;
        switch (d) {
            case 1:  // scaled value is printed with a decimals
            case 2:
            case 3:
                // convert to a real value, apply divisor (1.0-1000.0)
                sprintf(pStr, cPrint[d], ((double)iRawValue)/fDiv[d]);
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
        pCtrl->DATA[pCH->s.CH].ucFROM = *(buffer + BGNET_FRAME_HEAD_FROM); // originator of the frame (that included this channel)
        // decode channel, get displayed value
        decode_channel(pCtrl->DATA[pCH->s.CH].cValue, pCH); // decode the value for this channel
        // number of bytes in this channel (used to copy channel bytestream, but also to advance to the next channel in this frame)
        y = ChannelByteLength(pCtrl, pCH);
        memcpy(pCtrl->DATA[pCH->s.CH].ucRAW, pCH, y); // copy y bytes from channel number

        CounterUpDATA(&pCtrl->DATA[pCH->s.CH], t);

        // sometimes we have to do something when some data comes in
        switch(pCH->s.CH) {
            case BGN_CNR_B5_AP_MODE:
                pCtrl->ACP.STATE.value.nByte = (pCH->s.cv.uB[0] & BGN_AP_MODE1_MASK_STATE);  // get AUTO/STBY state into ACP
                pCtrl->ACP.STATE.timestamp = t;
                pCtrl->ACP.STATE.source = BGN_SOURCE_BGNIO;
                pCtrl->ACP.MODE.value.nByte  = (pCH->s.cv.uB[1] & BGN_AP_MODE2_MASK_MODE);   // get COMP/NAV/VANE mode into ACP
                pCtrl->ACP.MODE.timestamp = t;
                pCtrl->ACP.MODE.source = BGN_SOURCE_BGNIO;
        }

        log_debug("Channel #%d, ID=0x%02X, FMT=0x%02X, label=%s, value=%s", z
                                                                        , pCH->s.CH
                                                                        , pCH->s.FMT
                                                                        , pCtrl->pCHNDT[pCH->s.CH].cLabel
                                                                        , pCtrl->DATA[pCH->s.CH].cValue
                                                                    );

        pBuf = pBuf + y; // advance to next channel in this frame
        z++; // count channels in this frame
    }
    return(z); // return number of channels
}
// routine to process values in the body. A body might contain multiple values
void decode_command_SUBSCRIBE(unsigned char *buffer) {
    // this type of frame shall have a payload of 1 byte
    if (buffer[BGNET_FRAME_HEAD_SIZE] == 1) {
        log_debug("    Ch=%02X, Subscribed by %02X on %02X", buffer[BGNET_FRAME_HEAD+0], buffer[BGNET_FRAME_HEAD_FROM], buffer[BGNET_FRAME_HEAD_TO]);
    }
    return;
}
// routine to advertise values that can be subscribed
void decode_command_LABEL(unsigned char *buffer) {
    unsigned char string[255];
    size_t len;
    // this type of frame has a variable payload: Ch#, 3 bytes data, nbytes ascii
    if ((buffer[BGNET_FRAME_HEAD_SIZE] > 4) & (buffer[BGNET_FRAME_HEAD_SIZE] < 12)) {
        len=buffer[BGNET_FRAME_HEAD_SIZE]-4;
        log_debug("  1st=%02X, len= %ld", buffer[BGNET_FRAME_HEAD+4], len);
        memcpy(string, &buffer[BGNET_FRAME_HEAD+4], len);
        string[len] = 0x00;
        log_debug("    Ch=%02X, advertised by %02X to %02X, value= %s", buffer[BGNET_FRAME_HEAD+0], buffer[BGNET_FRAME_HEAD_FROM], buffer[BGNET_FRAME_HEAD_TO], string);
    }
    return;
}

// returns >0 when this message is for me to process
int IsMessageForMe(unsigned char *pBuf) {
    int r = 0;
    if (pBuf[BGNET_FRAME_HEAD_FROM] == BGNET_ADDRESS_ME) {
        // do not process this message (also to ALL)
    } else {
        // adressed to ALL, ME or MY-TYPE
        if ((pBuf[BGNET_FRAME_HEAD_TO] == BGNET_FRAME_TO_ALL) | (pBuf[BGNET_FRAME_HEAD_TO] == BGNET_ADDRESS_ME) | ((pBuf[BGNET_FRAME_HEAD_TO] & BGNET_TYPE_MASK) == (BGNET_ADDRESS_ME & BGNET_TYPE_MASK))) {
            // yes, it's for me
            r++;
        }
    }
    return(r);
}

//enable this to get a debug trace
//#define FRAME_DEBUG

// decode the received buffer, possibly multiple subframes, return number of sub-frames
size_t decode_frame(BGNET_CTRL* pCtrl, unsigned char *buffer, size_t len) {
    unsigned char cs; // checksum calculated
    unsigned char *pBuf,*pEnd; // pointer to start and end of buffer
    char asciibuffer[BGN_FRAME_SIZE];
    int xSize,x,z,n;
    int iIsForMe, iIsValid;
    unsigned char ucCMD, ucFROM;
    time_t t;
    unsigned char payload[1024], tmp_buffer[1024];
    int pl_len, msg_len, msg_offset;

    pBuf = buffer; // gets incremented as we go
    pEnd = buffer+len; // end of buffer
    z = 0; // subframe counter

    // process multiple subframes in our buffer
    while (pBuf < pEnd) {
        // the header is fixed size
        ucFROM = pBuf[BGNET_FRAME_HEAD_FROM];
        cs = calculate_checksum(pBuf, BGNET_FRAME_CR_HEAD, BGNET_FRAME_HEAD_TO, BGNET_FRAME_HEAD-1); // calculate the checksum over the header (without the CR)
#ifdef FRAME_DEBUG
        log_debug("  Header Checksum: %02X (seed),%02X (received),%02X (calculated)", BGNET_FRAME_CR_HEAD, *(pBuf+BGNET_FRAME_HEAD_CS), cs);
#endif
        if (*(pBuf+BGNET_FRAME_HEAD_CS) == cs) {
#ifdef FRAME_DEBUG
            log_debug("                   TO: %02X", *(pBuf+BGNET_FRAME_HEAD_TO));
            log_debug("                   FROM: %02X", *(pBuf+BGNET_FRAME_HEAD_FROM));
#endif

            // check if this subframe is adressed TO us and not FROM us
            iIsForMe = IsMessageForMe(pBuf);
            if (iIsForMe < 1) {
                //log_info("                   IGNORE (it's from myself)");
                xSize = pEnd - pBuf; // dump the remainder of the frame; it's a readback of my own transmission

                // update DEVICE structure (statistics). We do count it as an incoming frame, but we do no process it
                CounterRxUpDEV(&pCtrl->DEV[ucFROM], t);
            } else {
                // The HEADER is OK, process it. Note that some CMD's do not have a body and in such case, we should skip the body & CS part.
                // update DEVICE structure (statistics)
                CounterRxUpDEV(&pCtrl->DEV[ucFROM], t);

                // get the device type, device index and timestamp (we might need them)
                x = bgn_FindDeviceIndex(pCtrl, *(pBuf+BGNET_FRAME_HEAD_FROM)); // lookup by device address/type, so we can print the device name
                time(&t); // get timestamp

                if (*(pBuf+BGNET_FRAME_HEAD_SIZE) == 0) {
                    // 0 payload length, so no frame CS
                    xSize = BGNET_FRAME_HEAD;
                    byte2ascii(asciibuffer, sizeof(asciibuffer), pBuf, xSize, BYTE2ASCII_SPACED);
#ifdef FRAME_DEBUG
                    log_debug("  Header Checksum OK, CMD=%02X, number of data bytes: %d, framesize: %d, data: %s",*(pBuf+BGNET_FRAME_HEAD_CMD),*(pBuf+BGNET_FRAME_HEAD_SIZE), xSize, asciibuffer);
#endif
                    // these are the commands without payload (no channel data)
                    ucCMD = *(pBuf+BGNET_FRAME_HEAD_CMD);
                    iIsValid = 0;
                    switch(ucCMD) {
                        case BGNET_FRAME_CMD_KEEPALIVE:
                            iIsValid = 1;
                            log_info("  Command %02X (header only) KEEPALIVE by %02X %s", ucCMD, ucFROM, pCtrl->pDEVDT[x].sModelName);
                            payload[0]=0x00;
                            payload[1]=BGN_CNR_B5_AP_MODE;
                            payload[2]=0x00;
                            payload[3]=0x00;
                            payload[4]=0x03;
                            payload[5]=0x07;
                            pl_len = 6;
                            msg_len = bgn_net_makemessage(tmp_buffer, 0x50, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_PILOT_AM, payload, pl_len);                                

                            // put it for transmission
                            pthread_mutex_lock(&g_data_mutex);
                            memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &tmp_buffer, msg_len); // put output message in write buffer
                            g_serial_data_lenTx = g_serial_data_lenTx + msg_len; // increment byte counter in TX buffer
                            pthread_mutex_unlock(&g_data_mutex);

                            break;
                        case BGNET_FRAME_CMD_POWERUP:
                            iIsValid = 1;
                            log_info("  Command %02X (header only) POWERUP by %02X %s", ucCMD, ucFROM, pCtrl->pDEVDT[x].sModelName);
                            break;
                        default:
                            log_info("  Command %02X (header only) IGNORED from %02X %s)", ucCMD, ucFROM, pCtrl->pDEVDT[x].sModelName);
                            break;
                    }
                    if (iIsValid > 0) {
                        // update DEV & CMD structure
                        pCtrl->iNewDevice = 1; // signal this device
                        memcpy(pCtrl->DEV[ucFROM].ucFrameRx, pBuf, xSize);
                        pCtrl->CMD[ucCMD].iNewCommand = 1; // signal this command
                        memcpy(pCtrl->CMD[ucCMD].ucFrameRx, pBuf, xSize);
                        CounterRxUpCMD(&pCtrl->CMD[ucCMD], t);
                    }
                } else {
                    // payload present, check frame-CR
                    xSize = BGNET_FRAME_HEAD + *(pBuf+BGNET_FRAME_HEAD_SIZE) + BGNET_FRAME_TAIL_CS; // expected framesize = head (including CR), body, and frame CR
#ifdef FRAME_DEBUG
                    log_debug("  Header Checksum OK, CMD=%02X, number of data bytes: %d, framesize: %d",*(pBuf+BGNET_FRAME_HEAD_CMD),*(pBuf+BGNET_FRAME_HEAD_SIZE), xSize);
#endif
                    cs = calculate_checksum(pBuf, BGNET_FRAME_CR_FRAME, BGNET_FRAME_HEAD_TO, xSize-1); // calculate the checksum over the whole frame (including the header with CR, but without the frame CR)
#ifdef FRAME_DEBUG
                    log_debug("  Frame Checksum: %02X (seed),%02X (received),%02X (calculated)",BGNET_FRAME_CR_FRAME,*(pBuf+xSize-1),cs);
#endif
                    if (*(pBuf+xSize-1) == cs) {
                        // CS is good
                        byte2ascii(asciibuffer, sizeof(asciibuffer), pBuf, xSize, BYTE2ASCII_SPACED);
#ifdef FRAME_DEBUG
                        log_debug("  Frame Checksum OK, framesize: %d, data: %s", xSize, asciibuffer);
#endif
                        // commands with payload (otherwise we would have no CR)
                        ucCMD = *(pBuf + BGNET_FRAME_HEAD_CMD);
                        iIsValid = 0;
                        switch(ucCMD) {
                            case BGNET_FRAME_CMD_DATA:
                            case BGNET_FRAME_CMD_LATLON:
                                iIsValid = 1;
                                // process DATA/LATLON message
                                n = decode_command_DATA(pCtrl, pBuf);
                                log_info("  Command %02X (with payload) DATA, %d channels", ucCMD, n);
                                break;
                            case BGNET_FRAME_CMD_LABEL:
                                iIsValid = 1;
                                // process message (only at startup)
                                decode_command_LABEL(pBuf);
                                log_info("  Command %02X (with payload) LABEL", ucCMD);
                                break;
                            case BGNET_FRAME_CMD_SUBSCRIBE:
                                iIsValid = 1;
                                // process message (only at startup)
                                decode_command_SUBSCRIBE(pBuf);
                                log_info("  Command %02X (with payload) SUBSCRIBE", ucCMD);
                                break;
                            case BGNET_FRAME_CMD_BACKLIGHT:
                                iIsValid = 1;
                                pCtrl->ACP.BackLight.value.nByte = *(pBuf+BGNET_FRAME_HEAD+0); // first byte is the LIGHTS state
                                pCtrl->ACP.BackLight.timestamp = t;
                                pCtrl->ACP.BackLight.source = BGN_SOURCE_BGNIO;
                                log_info("  Command %02X (with payload) BACKLIGHT (BY: %02X, %s, LEVEL: %02X)",
                                                    *(pBuf+BGNET_FRAME_HEAD_CMD),
                                                    ucFROM,
                                                    pCtrl->pDEVDT[x].sModelName,
                                                    pCtrl->ACP.BackLight.value.nByte);
                                // if this was the PB on the QUAD, do something extra
                                switch(*(pBuf+BGNET_FRAME_HEAD_FROM)) { 
                                    case BGNET_TYPE_NETQUAD:
                                        break;
                                    case BGNET_TYPE_NETWIND:
                                        break;
                                }
                                break;
                            case BGNET_FRAME_CMD_QUAD_RESET:
                            case BGNET_FRAME_CMD_QUAD_RESET_R:
                            //makeQUADresponseRESET
                                //iIsValid = 1;
                                //pCtrl->ACP.uiTripLogNM = 0; // reset log trip counter            
                                //len_pl = makeQUADresponseRESET(pCtrl, payload);
                                //len_msg = bgn_net_makemessage(tmp_buffer, 0x30, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_QUAD_RESET1, payload, len_pl);
                                //memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &tmp_buffer, len_msg); // put output message in write buffer
                                //g_serial_data_lenTx = g_serial_data_lenTx + len_msg; // increment byte counter in TX buffer
                                break;
                            case BGNET_FRAME_CMD_PILOT_AM:

                                // set AUTO/STBY state. pressing the AUTO while in AUTO, no message is send by PILOT.
                                // The buttons AUTO and OFF generate the same message and effectively the buttons are a toggle
                                pCtrl->ACP.STATE.value.nByte = (1 - pCtrl->ACP.STATE.value.nByte);
                                pCtrl->ACP.STATE.timestamp = t;
                                pCtrl->ACP.STATE.source = BGN_SOURCE_BGNIO;

                                iIsValid = 1;
                                log_info("  Command %02X AUTO/STBY BUTTON PRESSED (BY: %02X, %s, state= %d)", *(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName, *(pBuf+BGNET_FRAME_HEAD+4));

                                // create ACK response
                                payload[0] = *(pBuf + BGNET_FRAME_HEAD + 0); // get 1st payload byte and return it
                                pl_len = 1;
                                msg_offset = 0;
                                // make ACK frame, update statistics
                                msg_len = bgn_net_makemessage(&tmp_buffer[msg_offset], *(pBuf + BGNET_FRAME_HEAD_FROM), BGNET_ADDRESS_ME, BGNET_FRAME_CMD_ACK, payload, pl_len);
                                memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_ACK].ucFrameTx, &tmp_buffer[msg_offset], msg_len);
                                CounterTxUpCMD(&pCtrl->CMD[BGNET_FRAME_CMD_ACK], t);

                                msg_offset = msg_offset + msg_len;
                                // make DATA command
                                pl_len = makeACPresponseFAST(pCtrl, payload);
                                // make DATA frame, update statistics
                                msg_len = bgn_net_makemessage(&tmp_buffer[msg_offset], BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, pl_len);
                                memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].ucFrameTx, &tmp_buffer[msg_offset], msg_len);
                                CounterTxUpCMD(&pCtrl->CMD[BGNET_FRAME_CMD_DATA], t);

                                msg_offset = msg_offset+ msg_len;
                                // put it for transmission
                                pthread_mutex_lock(&g_data_mutex);
                                memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &tmp_buffer, msg_offset); // put output message in write buffer
                                g_serial_data_lenTx = g_serial_data_lenTx + msg_offset; // increment byte counter in TX buffer
                                pthread_mutex_unlock(&g_data_mutex);

                                break;
                            case BGNET_FRAME_CMD_PILOT_MODE:

                                // set AUTO/STBY state. pressing the AUTO while in AUTO, no message is send by PILOT.
                                // The buttons AUTO and OFF generate the same message and effectively the buttons are a toggle
                                if (pCtrl->ACP.Joystick.value.nByte == 0) {
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
                                    pCtrl->ACP.MODE.timestamp = t;
                                    pCtrl->ACP.MODE.timestamp = BGN_SOURCE_BGNIO;
                                }

                                iIsValid = 1;
                                log_info("  Command %02X MODE BUTTON PRESSED (BY: %02X, %s, state= %d)", *(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName, *(pBuf+BGNET_FRAME_HEAD+4));

                                // create ACK response
                                payload[0] = *(pBuf + BGNET_FRAME_HEAD + 0); // get 1st payload byte and return it
                                pl_len = 1;
                                msg_offset = 0;
                                // make ACK frame, update statistics
                                msg_len = bgn_net_makemessage(&tmp_buffer[msg_offset], *(pBuf + BGNET_FRAME_HEAD_FROM), BGNET_ADDRESS_ME, BGNET_FRAME_CMD_ACK, payload, pl_len);
                                memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_ACK].ucFrameTx, &tmp_buffer[msg_offset], msg_len);
                                CounterTxUpCMD(&pCtrl->CMD[BGNET_FRAME_CMD_ACK], t);

                                msg_offset = msg_offset + msg_len;
                                // make DATA command
                                pl_len = makeACPresponseFAST(pCtrl, payload);
                                // make DATA frame, update statistics
                                msg_len = bgn_net_makemessage(&tmp_buffer[msg_offset], BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, pl_len);
                                memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_DATA].ucFrameTx, &tmp_buffer[msg_offset], msg_len);
                                CounterTxUpCMD(&pCtrl->CMD[BGNET_FRAME_CMD_DATA], t);

                                msg_offset = msg_offset+ msg_len;
                                // put it for transmission
                                pthread_mutex_lock(&g_data_mutex);
                                memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &tmp_buffer, msg_offset); // put output message in write buffer
                                g_serial_data_lenTx = g_serial_data_lenTx + msg_offset; // increment byte counter in TX buffer
                                pthread_mutex_unlock(&g_data_mutex);

                                break;
                            case BGNET_FRAME_CMD_PILOT_SPEED:
                                iIsValid = 1;
                                log_info("  Command %02X SPEED BUTTON PRESSED (BY: %02X, %s, state= %d)", *(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName, *(pBuf+BGNET_FRAME_HEAD+4));
                                //pCtrl->ACP.ucSTATE = *(pBuf+BGNET_FRAME_HEAD+4); // AUTO/STBY state
                                break;
                            case BGNET_FRAME_CMD_PILOT_STEP:
                                iIsValid = 1;
                                log_info("  Command %02X 1-of-4 STEP BUTTONS PRESSED (BY: %02X, %s, state= %d)", *(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName, *(pBuf+BGNET_FRAME_HEAD+4));
                                //pCtrl->ACP.ucMODE = *(pBuf+BGNET_FRAME_HEAD+4); // COMP/VANE/NAV mode
                                break;
                            default:
                                log_info("  Command %02X (with body) IGNORED (from %02X %s)", *(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName);
                                break;
                        }
                        if (iIsValid > 0) {
                            // update DEV & CMD structure
                            pCtrl->iNewDevice = 1; // signal this device
                            memcpy(pCtrl->DEV[ucFROM].ucFrameRx, pBuf, xSize);
                            pCtrl->CMD[ucCMD].iNewCommand = 1; // signal this command
                            memcpy(pCtrl->CMD[ucCMD].ucFrameRx, pBuf, xSize);
                            CounterRxUpCMD(&pCtrl->CMD[ucCMD], t);
                        }   
                    } else {
                        // CS is bad (only skip this sub-frame, the length is known)
                        log_info("    Frame Checksum BAD, framesize: %d", xSize);
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
        pBuf=pBuf+xSize;
        z=z+1;
    }
#ifdef FRAME_DEBUG
    log_debug("Rx frame processed, %d subframes", z);
#endif
    return(z);
}

#endif
