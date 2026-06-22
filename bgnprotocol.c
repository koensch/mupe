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
        if ((pCtrl->pDEVDT[i].iDeviceType & BGNET_DEVICETYPE_MASK) == (ucDevAddr & BGNET_DEVICETYPE_MASK)) {
            r = i; // first match
            break;
        }
    }
//    log_info("bgn_FindDeviceIndex: done Adr= 0x%X, index=%d, type=0x%0X", ucDevAddr, r, pCtrl->pDev[i].iDeviceType);
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
        pCtrl->ACP.ucJOYSTICK = 0; // joystick takes prefecence over the PILOT
        pCtrl->ACP.ucSTATE = 0;
        pCtrl->ACP.ucMODE = 0;
        pCtrl->ACP.uiMemory = 0x0123; // value 0x0123 = 291
        pCtrl->ACP.ucLight = 0; // backlight (0=OFF, 1-3=ON)
        pCtrl->ACP.ucSTATE = BGN_AP_MODE1_AUTO; // set STBY/AUTO
        pCtrl->ACP.ucMODE |= BGN_AP_MODE2_COMP; // NAV mode
        // log distances
        pCtrl->ACP.uiStoredLogNM = 12345; // channel CD
        pCtrl->ACP.uiTripLogNM = 123; // channel CF

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
        }

        // stage 2 - initialize command list
        for (i=0; i<BGNET_COMMAND_MAX; i++) {
            pCtrl->CMD[i].iNewCommand = 0;
            pCtrl->CMD[i].iCounterRx = 0;
            pCtrl->CMD[i].iCounterTx = 0;
            pCtrl->CMD[i].tTimestampRx = t;
            pCtrl->CMD[i].tCounterResetRx = t;
            pCtrl->CMD[i].tTimestampTx = t;
            pCtrl->CMD[i].tCounterResetTx = t;
        }

        // stage 3 - initialize channel list
        for (i=0; i<BGN_CHANNEL_MAX; i++) {
            pCtrl->DATA[i].ucFROM = 0;
            pCtrl->DATA[i].iNewChannel = 0;
            pCtrl->DATA[i].iCounter = 0;
            pCtrl->DATA[i].tTimestamp = t;
            pCtrl->DATA[i].tCounterReset = t;
            strcpy(pCtrl->DATA[i].cValue, "");
            memset(pCtrl->DATA[i].ucRAW, 0, sizeof(pCtrl->DATA[i].ucRAW));
        }
        log_debug("bgn_net_initialize: Done");
    }
};


size_t bgn_net_makemessage(unsigned char *pBuf, int iTo, int iFrom, int iCmd, unsigned char *payload, int payloadlength) {
    int i;
    size_t xSize;
    pBuf[BGNET_FRAME_HEAD_TO] = iTo;
    pBuf[BGNET_FRAME_HEAD_FROM] = iFrom;
    pBuf[BGNET_FRAME_HEAD_SIZE] = 0; // assume command without payload
    pBuf[BGNET_FRAME_HEAD_CMD] = iCmd;
    switch (iCmd) {
        case BGNET_FRAME_CMD_POWERUP:
            // this message takes no payload
            if (payloadlength == 0) {
                pBuf[BGNET_FRAME_HEAD_CS] = calculate_checksum(pBuf,BGNET_FRAME_CR_HEAD,0,BGNET_FRAME_HEAD-1);
                xSize = BGNET_FRAME_HEAD;
            } else {
                xSize = 0;
            }
            break;
        case BGNET_FRAME_CMD_BACKLIGHT:
            if (payloadlength == 1) {
                pBuf[BGNET_FRAME_HEAD_SIZE] = 1; // 1-byte payload
                pBuf[BGNET_FRAME_HEAD_CS] = calculate_checksum(pBuf,BGNET_FRAME_CR_HEAD,0,BGNET_FRAME_HEAD-1);
                pBuf[BGNET_FRAME_HEAD+0] = payload[0]; // 1-byte payload
                xSize = BGNET_FRAME_HEAD+pBuf[BGNET_FRAME_HEAD_SIZE]+BGNET_FRAME_TAIL_CS;
                pBuf[BGNET_FRAME_HEAD+1] = calculate_checksum(pBuf,BGNET_FRAME_CR_FRAME,0,xSize-1);
            } else {
                xSize = 0;
            }
            break;
        case BGNET_FRAME_CMD_DATA:
            if (payloadlength == 0) {
                xSize = 0; // always must have some payload
            } else {
                pBuf[BGNET_FRAME_HEAD_SIZE] = payloadlength;
                pBuf[BGNET_FRAME_HEAD_CS] = calculate_checksum(pBuf,BGNET_FRAME_CR_HEAD,0,BGNET_FRAME_HEAD-1);
                for (i=0; i<payloadlength; i++) {
                    pBuf[BGNET_FRAME_HEAD+i] = payload[i]; // copy whole payload
                }
                xSize = BGNET_FRAME_HEAD+pBuf[BGNET_FRAME_HEAD_SIZE]+BGNET_FRAME_TAIL_CS;
                pBuf[BGNET_FRAME_HEAD+i] = calculate_checksum(pBuf,BGNET_FRAME_CR_FRAME,0,xSize-1);
            }
            break;
        case BGNET_FRAME_CMD_QUAD_RESET1:            
        case BGNET_FRAME_CMD_QUAD_RESET3:            
        case BGNET_FRAME_CMD_PILOT_AM:            
        case BGNET_FRAME_CMD_PILOT_MODE:            
        case BGNET_FRAME_CMD_PILOT_SPEED:            
        case BGNET_FRAME_CMD_PILOT_STEP:
            if (payloadlength == 0) {
                xSize = 0; // always must have some payload
            } else {
                pBuf[BGNET_FRAME_HEAD_SIZE] = payloadlength;
                pBuf[BGNET_FRAME_HEAD_CS] = calculate_checksum(pBuf,BGNET_FRAME_CR_HEAD,0,BGNET_FRAME_HEAD-1);
                for (i=0; i<payloadlength; i++) {
                    pBuf[BGNET_FRAME_HEAD+i] = payload[i]; // copy whole payload
                }
                xSize = BGNET_FRAME_HEAD+pBuf[BGNET_FRAME_HEAD_SIZE]+BGNET_FRAME_TAIL_CS;
                pBuf[BGNET_FRAME_HEAD+i] = calculate_checksum(pBuf,BGNET_FRAME_CR_FRAME,0,xSize-1);
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

    c = BGN_CNR_41_BOATSPEEDKTS;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.uW[0] = htons(125);
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    c = BGN_CNR_42_BOATSPEEDRAW;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.uW[0] = htons(120);
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    // the next 3 values are 'rotated' by the QUAD: CD every other time, 64/65 every 4'th time
    // but that is too complex at this moment. Later i will build a rotating scheme.

    c = BGN_CNR_CD_STORLOGNM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.uL[0] = htonl(112233);

    c = BGN_CNR_64_SPEEDAVGKTS;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.uW[0] = htons(111); 
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    c = BGN_CNR_65_SPEEDAVGRAW;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.uW[0] = htons(222); 
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    // the next values are always present 

    c = BGN_CNR_CF_TRIPLOGNM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.uW[0] = htons(333); 
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    c = BGN_CNR_81_DR_DIST;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.uW[0] = htons(55); 
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    c = BGN_CNR_D3_DR_COURSE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.sW[0] = htons(-20); 
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    c = BGN_CNR_20_SEATEMPQ;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.sW[0] = htons(12); 
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    c = BGN_CNR_75_TIMER;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.uB[0] = 1; // only 1 digit is shown 
    puCH->s.v.uB[1] = 59;
    puCH->s.v.uB[2] = 59;
    r = r + pCtrl->pCHNDT[c].iChannelLen;

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
    puCH->s.v.uC[0] = '\0'; 
    puCH->s.v.uC[0] = ' '; 
    puCH->s.v.uC[0] = 'F'; 
    puCH->s.v.uC[0] = ' ';
    r = r + pCtrl->pCHNDT[c].iChannelLen;
    
    // the next 3 values are 'rotated' by the QUAD: FE every other time, FC/FD every 4'th time
    // but that is too complex at this moment. Later i will build a rotating scheme.

    c = BGN_CNR_FC_SHALLOWALARM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.uC[0] = '\0'; 
    puCH->s.v.uC[0] = 'O'; 
    puCH->s.v.uC[0] = 'F'; 
    puCH->s.v.uC[0] = 'F';
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    c = BGN_CNR_FD_DEPTHALARM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.sW[0] = htons(-5); 
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    c = BGN_CNR_FE_ANCHORALARM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.uC[0] = '\0'; 
    puCH->s.v.uC[0] = 'O'; 
    puCH->s.v.uC[0] = 'F'; 
    puCH->s.v.uC[0] = 'F';
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    // the next values are always present 

    c = BGN_CNR_FF_DEPTH_CAL;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.sW[0] = htons(4); 
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    c = BGN_CNR_C4_DEPTHM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.sW[0] = htons(-6); 
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    c = BGN_CNR_C1_DEPTHM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.sW[0] = htons(2); 
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    return(r);
}
// this message is send from PILOT to ACP when the AUTO or OFF button is pressed
int makeAUTOSTBYack(BGNET_CTRL *pCtrl, unsigned char* payload, int iState, int iMode) {
    int r = 5;
    payload[0] = 0xFF;
    payload[1] = 0xB5;
    payload[2] = iState; // request AUTO/STBY
    payload[3] = iMode; // mode request
    payload[4] = 0xFF; // unknown
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
    puCH->s.v.sW[0] = htons(-9);
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    c = BGN_CNR_A6_AP_CTARGET;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.uW[0] = htons(353);
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    c = BGN_CNR_A7_AP_WPB;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.uW[0] = htons(355);
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    c = BGN_CNR_B5_AP_MODE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.uB[0] = pCtrl->ACP.ucSTATE; // AUTO/STBY
    puCH->s.v.uB[1] = pCtrl->ACP.ucMODE;  // COMP/NAV/VANE
    r = r + pCtrl->pCHNDT[c].iChannelLen;

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
    puCH->s.v.sW[0] = htons(-8+(t&0x01F));
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    c = BGN_CNR_41_BOATSPEEDKTS;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.uW[0] = htons(118);
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    c = BGN_CNR_49_HEADING;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.uW[0] = htons(77);
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    c = BGN_CNR_51_AWINDANGLE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.v.sW[0] = htons(-95);
    r = r + pCtrl->pCHNDT[c].iChannelLen;

    return (r);
}
// replay message on TRIP RESET by QUAD
int makeQUADresponseRESET(BGNET_CTRL *pCtrl, unsigned char* payload) {
    payload[0] = 1;
    payload[1] = BGN_CNR_CF_TRIPLOGNM;
    return(2);
}

// use this macro to convert a floating point value (double) to integer with round-off
#define DTOIR(d) ((int)(d >= 0.0 ? (d + 0.5) : (d - 0.5)))

// establish the divisor (we do not support values > 999)
// -1 = clamp to 999 with divisor 0 (=1), no decimals
//  0 = 100-999, 3 digits with divisor 0 (=1), no decimals
//  1 = 10.0-99.9, 2 digits + 1 decimal with divisor 1 (=10)
//  2 = 1.00-9.99, 1 digit + 2 decimals with divisor 2 (=100)
//  3 = 0.00-0.99, 1 digit + 2 decimals with divisor 3 (=1000)
// note: i is expected to be a integer. To correctly scale fractions we use
//       the i*10 (and all the factors scaled up also.

#define MKDIV(i)      (i*10<10000?(i*10<1000?(i*10<100?(i*10<1?(3):(2)):(1)):(0)):(-1))

int MakeChannelValue(BGNET_CTRL *pCtrl, union CH* pCH ,unsigned char ucCH, union V* puV) {
    int iDiv;
    int iDivisor[4] = {1,10,100,1000};

    pCH->s.CH = ucCH;
    pCH->s.FMT = (pCtrl->pCHNDT[ucCH].ucFMT & BGN_FMT_MASK_DT); // default DataType from Definition Table
    switch (pCH->s.FMT & BGN_FMT_MASK_DT) {
        case BGN_FMT_01: // signed 16bit register
            // autoscaling not supported
            pCH->s.v.sW[0] = (int16_t)htons(puV->sW[0]);
            pCH->s.FMT = pCH->s.FMT & BGN_FMT_MASK_DT; // mask-out DIV and PAD
            break;
        case BGN_FMT_08: // unsigned 16bit register 
            if ((pCH->s.FMT & ~BGN_FMT_MASK_DT) == 0) {
                // autoscaling disabled
                pCH->s.v.uW[0] = htons(puV->uW[0]);
            } else {
                // autoscaling enabled
                iDiv = MKDIV(puV->uW[0]);
                if ((iDiv<0)|(iDiv>3)) {
                    // overflow/underflow
                    pCH->s.v.uW[0] = htons(999);
                    pCH->s.FMT = pCH->s.FMT & BGN_FMT_MASK_DT; // mask-out DIV and PAD
                } else {
                    pCH->s.v.uW[0] = htons(puV->uW[0]*iDivisor[iDiv]);
                    pCH->s.FMT = pCH->s.FMT | ((iDiv << 6) & BGN_FMT_MASK_DIVPAD_DIV);
                }
            }   
            break;
        case BGN_FMT_02: // unsigned 32bit register (0xCD, storedlog, value *10)
            pCH->s.v.uL[0] = puV->uL[0];
            pCH->s.FMT = BGN_FMT_42; // divisor 1 (*10)
            break;
        case BGN_FMT_03: // 3 bytes timer value (0x75, timer)
            pCH->s.v.uB[0] = puV->uB[0];
            pCH->s.v.uB[1] = puV->uB[1];
            pCH->s.v.uB[2] = puV->uB[2];
            pCH->s.FMT = BGN_FMT_03; // no divisor, no padding
            break;
        case BGN_FMT_04: // 3 bytes ASCII
            pCH->s.v.uB[0] = 0;
            pCH->s.v.uB[1] = puV->uB[0];
            pCH->s.v.uB[2] = puV->uB[1];
            pCH->s.v.uB[3] = puV->uB[2];
            pCH->s.FMT = BGN_FMT_04; // no divisor, no padding
            break;
        default: // unsupported FMT, make it something
            pCH->s.v.uB[0] = 0;
            pCH->s.v.uB[1] = 'E';
            pCH->s.v.uB[2] = 'R';
            pCH->s.v.uB[3] = 'R';
            pCH->s.FMT = BGN_FMT_04; // no divisor, no padding
            break;
    }
    return(ChannelByteLength(pCtrl, pCH->s.FMT)); // return the length of the byte sequence
}

int makeAUTOresponse(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int len;
    time_t t;
    time(&t);
    len = 0;
    payload[0] = BGN_CNR_B5_AP_MODE;
    payload[1] = pCtrl->pCHNDT[BGN_CNR_B5_AP_MODE].ucFMT;
    payload[2] = pCtrl->ACP.ucSTATE; // AUTO/STBY
    if (pCtrl->ACP.ucJOYSTICK == 0) {
        payload[3] = pCtrl->ACP.ucMODE;
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
    payload[2] = pCtrl->ACP.ucSTATE; // AUTO/STBY
    if (pCtrl->ACP.ucJOYSTICK == 0) {
        payload[3] = pCtrl->ACP.ucMODE;
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
    puCH->s.CH = BGN_CNR_46_AP_SPEEDKTS;
    puCH->s.FMT = 0x08;
    puCH->s.v.uB[0] = 1;
    puCH->s.v.uB[0] = 2;

    puCH = (union CH*)&(payload[4]);
    puCH->s.CH = BGN_CNR_41_BOATSPEEDKTS;
    puCH->s.FMT = 0x08;
    puCH->s.v.uB[0] = 1;
    puCH->s.v.uB[0] = 2;

    puCH = (union CH*)&(payload[8]);
    puCH->s.CH = BGN_CNR_EB_SOGKT;
    puCH->s.FMT = 0x08;
    puCH->s.v.uB[0] = 1;
    puCH->s.v.uB[0] = 2;

    len = 12;
    return (len);
}

// This is the "device processor". Data received was placed into the control block. Here we scan the
// control block to see if we have to generate a reply message.
// We attempt to emulate the real thing.
// Realise, while this routines executes, other threads are locked out. Do not sleep or do slow/blocking actions.
// What we put in the output buffer is a complete frame, possibly with sub-frames. ready to send.
size_t bgn_net_process(BGNET_CTRL *pCtrl, unsigned char* buffer) {
    unsigned char tmp_buffer[256],payload[256];
    char asciibuffer[256];
    int i,len_pl,len_msg;

    size_t r = 0;

    // device received new data
    if ((pCtrl->iNewDevice < 1)) {
        // nothing to do
    } else {
        // Stage 1 - check for new commands
        for (i=0; i<BGNET_COMMAND_MAX; i++) {
            if (pCtrl->CMD[i].iNewCommand < 1) {
                // no n.ucSTATEew commands
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
                    case BGNET_FRAME_CMD_QUAD_RESET1:
                    case BGNET_FRAME_CMD_QUAD_RESET3:
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
                // process new channel data
                pCtrl->DATA[i].iNewChannel = 0; // reset newdata flag
            }
        }

        // stage 3 - do some scheduled messages
        //len_pl = makeQUADresponse1(pCtrl, payload);
        //len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
        //memcpy(&buffer[r], &tmp_buffer, len_msg); // put output message in write buffer
        //r = r + len_msg; // increment byte counter in TX buffer
        //len_pl = makeQUADresponse2(pCtrl, payload);
        //len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
        //memcpy(&buffer[r], &tmp_buffer, len_msg); // put output message in write buffer
        //r = r + len_msg; // increment byte counter in TX buffer

        // stage 4 - check if we are allowed to send output
        // check if we are in RW mode; if not, destroy our output. I'd rather do it here that making the logic complexer
        // by integrating the readonly/readwrite mode into the logic
        if (g_serial_mode == BGN_SERIAL_MODE_RW) {
            // leave output data
        } else {
            r = 0; // destroy what we have in the buffer
        }

        // / stage 4 - reset new data flag
        pCtrl->iNewDevice = 0;
    }
    
    // if we produced some response, show it here
    if (r > 0) {
        byte2ascii(asciibuffer, sizeof(asciibuffer), buffer, r, BYTE2ASCII_SPACED);
        log_info("bgn_net_process: output: %s (%ld bytes)", asciibuffer, r);
    }
    return(r);
};

// the FMT byte in the value header has some packed information
void fmt_decode(unsigned int buf, unsigned int *divisor, unsigned int *pad, unsigned int *fmt) {
    *divisor=((buf & BGN_FMT_MASK_DIVPAD_DIV) >> 6);
    *pad=((buf & BGN_FMT_MASK_DIVPAD_PAD) >> 4);
    *fmt=(buf & BGN_FMT_MASK_DT);
}
void fmt_encode(unsigned int *buf, unsigned int div, unsigned int pad, unsigned int fmt) {
    *buf = ((div << 6) & BGN_FMT_MASK_DIVPAD_DIV) | ((pad << 4) & BGN_FMT_MASK_DIVPAD_PAD) | (fmt & BGN_FMT_MASK_DT);
} 

// returns the length of a channel block; it all depends on the FMT specifier.
int ChannelByteLength(BGNET_CTRL* pCtrl, unsigned char ucFMT) {
    int r;
    r = pCtrl->pFMTDT[(ucFMT & BGN_FMT_MASK_DT)].iLen;
    if (r > 0) {
        // this is OK
    } else {
        // is the value is unknown, i just make a guess. 4 is better than zero or -1
        r = 4; // just a guess
    }
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

// copy command frame into a ascii buffer (for display purposes)
char* CommandToAscii(char* asciibuf, unsigned char* ucFrame) {
    int n, y;
    char cWork[BGN_CMD_ASCII];
    strcpy(cWork, "");
    if (ucFrame[BGNET_FRAME_HEAD_CMD] == 0) {
        // invalid command, buffer must be empty
        strcpy(asciibuf,"---");
    } else {
        strcpy(asciibuf, "");
        y = CommandByteLength(ucFrame);
        if (y*3 < BGN_CMD_ASCII) {
            // it fits, just copy it all
            byte2ascii(asciibuf, BGN_CMD_ASCII, ucFrame, y, BYTE2ASCII_SPACED);
        } else {
            // we use a 7-character separation string: " /.../ " to avoid excessive long output
            // we also copy the first 10 (5 header with CS + 5 first data) and the tail (5 last data + CS = 6 bytes)
            // 6 bytes tail = 18 ascii characters
            n = (BGN_CMD_ASCII - strlen(BGN_CMD_ASCII_SEP))/3; // we have ascii space for 'n' databytes
            byte2ascii(asciibuf, BGN_CMD_ASCII, ucFrame, n-BGN_CMD_DATA_TAIL, BYTE2ASCII_SPACED);
            strcat(asciibuf, BGN_CMD_ASCII_SEP); // append separator
            byte2ascii(cWork, BGN_CMD_ASCII, &ucFrame[y-BGN_CMD_DATA_TAIL], BGN_CMD_DATA_TAIL, BYTE2ASCII_SPACED);
            strcat(asciibuf, cWork);
        }
    }
    // return a null terminated ascii string
    return(asciibuf);
}


// returns the length of the channel value bytes and generates the text representation of the channel value
char* decode_value(char* pStr, union CH* puCH) {
    int iValue, iValid;
    double fValue;
    unsigned int d, p, f;
    double fDiv[4] = {1.0,10.0,100.0,1000.0};
    // switch according to DataType
    switch(puCH->s.FMT & BGN_FMT_MASK_DT) {
        case BGN_FMT_01: // signed 16bit register (-128 to +128)
            iValue = (int16_t)ntohs(puCH->s.v.sW[0]);
            iValid = 1;
            break;
        case BGN_FMT_02: // unsigned 32bit register (0xCD, storedlog)
            iValue = ntohl(puCH->s.v.uL[0]);
            iValid = 1;
            break;
        case BGN_FMT_03: // 3 bytes timer value (0x75, timer)
            sprintf(pStr,"%02d:%02d:%02d", puCH->s.v.uB[0], puCH->s.v.uB[1], puCH->s.v.uB[2]);
            iValid = 2;
            break;
        case BGN_FMT_04: // 3 bytes ASCII
            pStr[0] = puCH->s.v.uB[1];
            pStr[1] = puCH->s.v.uB[2];
            pStr[2] = puCH->s.v.uB[3];
            pStr[3] = '\0';
            iValid = 2;
            break;
        case BGN_FMT_08: // unsigned 16bit register
            iValue = ntohs(puCH->s.v.uW[0]);
            iValid = 1;
            break;
        default:
            pStr[0] = '?';
            pStr[1] = '?';
            pStr[2] = '?';
            pStr[3] = '\0';
            iValid = 0;
    }
    // iValid: 0 = not recognised, 1= numeric (scaled with divisor), 2=bytes/ascii/time
    if (iValid == 1) {
        fmt_decode(puCH->s.FMT, &d, &p, &f);
        fValue = ((double)iValue)/fDiv[d];
        switch (d) {
            case 1:
                sprintf(pStr,"%.1f", fValue);
                break;
            case 2:
                sprintf(pStr,"%.2f", fValue);
                break;
            case 3:
                sprintf(pStr,"%.3f", fValue);
                break;
            default:
                sprintf(pStr,"%d", iValue);
                break;
        }
    }
    return(pStr);
}

// update timestamp and increment counter (with wrap-around)
void CounterUpDATA(BGN_CHANNEL *pCH, time_t t) {
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

// routine to process values in the body. A body might contain multiple values
int process_CMD_DATA(BGNET_CTRL* pCtrl, unsigned char *buffer) {
    int y,z;
    unsigned char *pBuf,*pEnd;
    unsigned char ucCH, ucFMT;
    time_t t;

    pBuf = buffer+BGNET_FRAME_HEAD;
    pEnd = buffer+BGNET_FRAME_HEAD + buffer[BGNET_FRAME_HEAD_SIZE];
    z = 0;
    time(&t);
    // scan through frame/subframe
    while (pBuf < pEnd) {
        ucCH = *(pBuf + BGN_VALUE_CNR);
        ucFMT = *(pBuf + BGN_VALUE_FMT);
        // update CHANNEL structure
        pCtrl->DATA[ucCH].ucFROM = *(buffer + BGNET_FRAME_HEAD_FROM); // originator of the frame is sender of all channels within
        pCtrl->DATA[ucCH].iNewChannel = 1; // signal this channel as new/changed
        y = ChannelByteLength(pCtrl, ucFMT);
        decode_value(pCtrl->DATA[ucCH].cValue, (union CH*)(pBuf + BGN_VALUE_CNR)); // decode the value for this channel
        memcpy(pCtrl->DATA[ucCH].ucRAW, pBuf + BGN_VALUE_CNR, y); // copy y bytes from channel number

        CounterUpDATA(&pCtrl->DATA[ucCH], t);

        pBuf = pBuf + y;
        z++;
    }
    return(z); // return number of processed values
}
// routine to process values in the body. A body might contain multiple values
void process_CMD_SUBSCRIBE(unsigned char *buffer) {
    // this type of frame shall have a payload of 1 byte
    if (buffer[BGNET_FRAME_HEAD_SIZE] == 1) {
        log_debug("    Ch=%02X, Subscribed by %02X on %02X", buffer[BGNET_FRAME_HEAD+0], buffer[BGNET_FRAME_HEAD_FROM], buffer[BGNET_FRAME_HEAD_TO]);
    }
    return;
}
// routine to advertise values that can be subscribed
void process_CMD_VALUE(unsigned char *buffer) {
    unsigned char string[255];
    size_t len;
    // this type of frame has a variable payload: Ch#, 3 bytes data, nbytes ascii
    if (buffer[BGNET_FRAME_HEAD_SIZE] > 4) {
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
        if ((pBuf[BGNET_FRAME_HEAD_TO] == BGNET_FRAME_TO_ALL) | (pBuf[BGNET_FRAME_HEAD_TO] == BGNET_ADDRESS_ME) | ((pBuf[BGNET_FRAME_HEAD_TO] & BGNET_DEVICETYPE_MASK) == (BGNET_ADDRESS_ME & BGNET_DEVICETYPE_MASK))) {
            // yes, it's for me
            r++;
        }
    }
    return(r);
}

//enable this to get a debug trace
//#define FRAME_DEBUG

// decode the received buffer, possibly multiple subframes, return number of sub-frames
size_t frame_decoder(BGNET_CTRL* pCtrl, unsigned char *buffer, size_t len) {
    unsigned char cs; // checksum calculated
    unsigned char *pBuf,*pEnd; // pointer to start and end of buffer
    char asciibuffer[BGN_ASCIIFRAME_SIZE];
    int xSize,x,z,n;
    int iIsForMe, iIsValid;
    unsigned char ucCMD, ucFROM;
    time_t t;
    unsigned char payload[1024], tmp_buffer[1024];
    int len_pl, len_msg;
                                //pCtrl->ACP.ucMODE = *(pBuf+BGNET_FRAME_HEAD+4); // COMP/VANE/NAV mode

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

                // update DEVICE structure (statistics)
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
                            log_info("  Command %02X (header only) KEEPALIVE by %02X %s",*(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName);
                            break;
                        case BGNET_FRAME_CMD_POWERUP:
                            iIsValid = 1;
                            log_info("  Command %02X (header only) POWERUP by %02X %s",*(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName);
                            break;
                        default:
                            log_info("  Command %02X (header only) IGNORED from %02X %s)",*(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName);
                            break;
                    }
                    if (iIsValid > 0) {
                        // update CMD structure
                        pCtrl->iNewDevice = 1; // signal this device
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
                        ucCMD = *(pBuf+BGNET_FRAME_HEAD_CMD);
                        iIsValid = 0;
                        switch(ucCMD) {
                            case BGNET_FRAME_CMD_DATA:
                            case BGNET_FRAME_CMD_LATLON:
                                iIsValid = 1;
                                // process DATA/LATLON message
                                n = process_CMD_DATA(pCtrl, pBuf);
                                log_info("  Command %02X (with body), %d channels",*(pBuf+BGNET_FRAME_HEAD_CMD), n);
                                break;
                            case BGNET_FRAME_CMD_LABEL:
                                iIsValid = 1;
                                // process message (only at startup)
                                process_CMD_VALUE(pBuf);
                                log_info("  Command %02X (with body) LABEL",*(pBuf+BGNET_FRAME_HEAD_CMD));
                                break;
                            case BGNET_FRAME_CMD_SUBSCRIBE:
                                iIsValid = 1;
                                // process message (only at startup)
                                process_CMD_SUBSCRIBE(pBuf);
                                log_info("  Command %02X (with body) SUBSCRIBE",*(pBuf+BGNET_FRAME_HEAD_CMD));
                                break;
                            case BGNET_FRAME_CMD_BACKLIGHT:
                                iIsValid = 1;
                                log_info("  Command %02X BACKLIGHT BUTTON PRESSED (BY: %02X, %s, LEVEL: %02X)", *(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName, *(pBuf+BGNET_FRAME_HEAD+0), *(pBuf+BGNET_FRAME_HEAD+4));
                                pCtrl->ACP.ucLight = *(pBuf+BGNET_FRAME_HEAD+0); // first byte is the LIGHTS state
                                // if this was the PB on the QUAD, do something extra
                                if (*(pBuf+BGNET_FRAME_HEAD_FROM) == 0x30) {
                                    // toggle AUTO/STBY state
                                    pCtrl->ACP.ucSTATE = 1 - pCtrl->ACP.ucSTATE;
                                }
                                break;
                            case BGNET_FRAME_CMD_QUAD_RESET1:
                            case BGNET_FRAME_CMD_QUAD_RESET3:
                            //makeQUADresponseRESET
                                //iIsValid = 1;
                                //pCtrl->ACP.uiTripLogNM = 0; // reset log trip counter            
                                //len_pl = makeQUADresponseRESET(pCtrl, payload);
                                //len_msg = bgn_net_makemessage(tmp_buffer, 0x30, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_QUAD_RESET1, payload, len_pl);
                                //memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &tmp_buffer, len_msg); // put output message in write buffer
                                //g_serial_data_lenTx = g_serial_data_lenTx + len_msg; // increment byte counter in TX buffer
                                break;
                            case BGNET_FRAME_CMD_PILOT_AM:
                                iIsValid = 1;
                                log_info("  Command %02X AUTO/STBY BUTTON PRESSED (BY: %02X, %s, state= %d)", *(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName, *(pBuf+BGNET_FRAME_HEAD+4));
                                pCtrl->ACP.ucSTATE = *(pBuf+BGNET_FRAME_HEAD+4); // AUTO/STBY state
                                //len_pl = makeACPresponseFAST(pCtrl, payload);
                                //len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_PILOT_AM, payload, len_pl);
                                //memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &tmp_buffer, len_msg); // put output message in write buffer
                                //g_serial_data_lenTx = g_serial_data_lenTx + len_msg; // increment byte counter in TX buffer

                                // create ACK response
                                len_pl = makeAUTOSTBYack(pCtrl, payload, BGN_AP_MODE1_AUTO, BGN_AP_MODE2_NAV);
                                len_msg = bgn_net_makemessage(tmp_buffer, 0x50, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_PILOT_AM, payload, len_pl);                                

                                // put it for transmission
                                memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &tmp_buffer, len_msg); // put output message in write buffer
                                g_serial_data_lenTx = g_serial_data_lenTx + len_msg; // increment byte counter in TX buffer

                                // update statistics
                                memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_PILOT_AM].ucFrameTx, &tmp_buffer, len_msg);
                                CounterTxUpCMD(&pCtrl->CMD[BGNET_FRAME_CMD_PILOT_AM], t);
                                break;
                            case BGNET_FRAME_CMD_PILOT_MODE:
                                iIsValid = 1;
                                log_info("  Command %02X MODE BUTTON PRESSED (BY: %02X, %s, state= %d)", *(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName, *(pBuf+BGNET_FRAME_HEAD+4));
                                pCtrl->ACP.ucMODE = *(pBuf+BGNET_FRAME_HEAD+4); // COMP/VANE/NAV mode
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
                            // update CMD structure
                            pCtrl->iNewDevice = 1; // signal this device
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
            // header CS invalid, we can not process this subframe, we can not establish a subframe length unambigiously, so we dump the receive buffer
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
