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
            pCtrl->DEV[i].iCounterRx = 0;
            pCtrl->DEV[i].iCounterTx = 0;
            pCtrl->DEV[i].tTimestamp = t;
            pCtrl->DEV[i].tCounterReset = t;
        }

        // stage 2 - initialize command list
        for (i=0; i<BGNET_COMMAND_MAX; i++) {
            pCtrl->CMD[i].iNewCommand = 0;
            pCtrl->CMD[i].iCounter = 0;
            pCtrl->CMD[i].tTimestamp = t;
            pCtrl->CMD[i].tCounterReset = t;
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
                        len_pl = makeACPresponse(pCtrl, payload);
                        len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                        memcpy(&buffer[r],&tmp_buffer,len_msg); // put output message in write buffer
                        r = r + len_msg; // increment byte counter in TX buffer
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
                        len_pl = makeAUTOresponse(pCtrl, payload);
                        len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                        memcpy(&buffer[r], &tmp_buffer, len_msg); // put output message in write buffer
                        r = r + len_msg; // increment byte counter in TX buffer
                        break;
                    case BGNET_FRAME_CMD_PILOT_MODE:
                        len_pl = makeAUTOresponse(pCtrl, payload);
                        len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                        memcpy(&buffer[r], &tmp_buffer, len_msg); // put output message in write buffer
                        r = r + len_msg; // increment byte counter in TX buffer
                        break;
                    case BGNET_FRAME_CMD_PILOT_SPEED:
                        len_pl = makeSPEEDresponse(pCtrl, payload);
                        len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                        memcpy(&buffer[r], &tmp_buffer, len_msg); // put output message in write buffer
                        r = r + len_msg; // increment byte counter in TX buffer
                        break;
                    case BGNET_FRAME_CMD_PILOT_STEP:
                        len_pl = makeAUTOresponse(pCtrl, payload);
                        len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                        memcpy(&buffer[r], &tmp_buffer, len_msg); // put output message in write buffer
                        r = r + len_msg; // increment byte counter in TX buffer
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
        len_pl = makeQUADresponse1(pCtrl, payload);
        len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
        memcpy(&buffer[r], &tmp_buffer, len_msg); // put output message in write buffer
        r = r + len_msg; // increment byte counter in TX buffer
        len_pl = makeQUADresponse2(pCtrl, payload);
        len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
        memcpy(&buffer[r], &tmp_buffer, len_msg); // put output message in write buffer
        r = r + len_msg; // increment byte counter in TX buffer

        // stage 4 - check if we are allowed to send output
        // check if we are in RW mode; if not, destroy our output. I'd rather do it here that making the logic complexer
        // by integrating the readonly/readwrite mode into the logic
        if (g_serial_mode == BGN_SERIAL_MODE_RW) {
            // leave output data
        } else {
            r = 0;
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

// value length for DATA element (command 1 & 3)
int value_length_DATA(BGNET_CTRL* pCtrl, unsigned char CNR, unsigned char FMT) {
    int l;
    l = pCtrl->pFMTDT[(FMT & BGN_FMT_MASK_DT)].iLen;
    if (l > 0) {
        // should be OK
    } else {
        log_error("Channel 0x%02X, FMT= 0x%02X Undefined: %d", CNR, FMT, l);
        l = 4; // most are 4 bytes (it's just a wild guess, -1 or 0 is even worse)
    }
    return(l);
}

// the FMT byte in the value header has some packed information
void fmt_decode(unsigned int buf, unsigned int *divisor, unsigned int *size, unsigned int *fmt) {
    *divisor=(buf>>6&0x03);
    *size=(buf>>4&0x03);
    *fmt=(buf&0x0F);
}
void fmt_encode(unsigned int *buf, unsigned int div, unsigned int size, unsigned int fmt) {
    *buf = ((div&0x03)<<6)|((size&0x03)<<4)|(fmt&0x0F);
}
// return the format specifier as a string
char *fmt_string(unsigned char buf) {
    static char *fmtid[]={"xxxx."," xxx.","  xx.","   x.","xxx.x"," xx.x","  x.x","   .x"," xx.xx","  x.xx","  .xx","  .0x","x.xxx"," x.xx","  .xx","  .00x"};
    return(fmtid[buf>>4&0x0F]);
}
// we do not wat to han        #define BGN_CNR_BA_UNKNOWN          0xBAdle our value as float, but as a integer with a divisor
//char *strDecimals(unsigned char* pStr, int iV, int iS, int iD) {
//    //convert value (2 bytes, -32766/+32767) to ascii string for display purposes
//    sprintf(pStr,"%06d", iV);
//    return(pStr);
//}


// copy substring in cSource at pos to cTarget for len characters
char* substring(char* pTarget, char* pSource, int pos, int len) {
    memcpy(pTarget, pSource+pos, len);
    *(pTarget+len) = '\0';
    return(pTarget);
}

// insert a.ucSTATE 'divisor' (".") in the string. Realize that we could have a case where "0" should
// be represented as "0.000"
// our tactic:
// - we use "00000" and append our value string
// - we use a table/index to insert a '.' on the right place
// - we use a table/index to pick the starting point in the string
// - we use a table/index to pick the number of characters from the string 
char* insertDivisor(char* strC, char* string, unsigned char ucFMT) {
    // bit 7,6 = divisor (0=1, 1=10, 2=100, 3=1000)
    // bit 5,4 = size/characters (0=1, 1=2, 2=3, 3=4)
    // we do not want to do arithemetic because of rounding errors, so we do "charachter shifting"

    // we shift it right and get an index 0-F we can use to lookup the maniplation
    // this is how it is supposed to look for each of the possible values/format:
    //      "x.","xx.","xxx.","xxxx.","0.x","x.x","xx.x","xxx.x","0.0x","0.xx","x.xx","xx.xx","0.00x","0.0xx","0.xxx","x.xxx"};
    // - we append our value-string "1", "12", "123" or "1234" to "0.0000", resulting in:
    //          "0.0000x",     "0.0000xx",     "0.0000xxx"   or   "0.0000xxxx" (depending on the string length, 1-4 positions)
    // - we append "0000", so we get someting like:
    //          "0.0000x0000", "0.0000xx0000", "0.0000xxx0000" or "00000x.xxx0000"
    // we move the decimal point into position (shift-right, 0=do not shift)
    //                    div=0       l+0        l+0         l+0            l+0
    //                    div=1       l+3        l+3         l+3            l+3
    //                    div=2       l+2        l+2         l+2            l+2
    //                    div=3       l+1        l+1         l+1            l+1
    // we take the substring (start/length):
    //                    div=0       7/l+0      7/l+0       7/l+0          7/l+0        
    //                    div=1       5/l+2      6/l+1       6/l+1          6/l+1
    //                    div=2       4/l+4      5/l+4       6/l+1          6/l+1
    //                    div=3       3/l+4      4/l+3       5/l+2          6/l+1                   
    //                           
    // realise that the string passed (containing the value) does not have the same
    // length as the 'size' specification in the FMT byte.

    unsigned char shift[16] = {0,0,0,0, 3,3,3,3, 2,2,2,2, 1,1,1,1};
    unsigned char start[16] = {7,7,7,7, 5,6,6,6, 4,5,6,6, 3,4,5,6};
    unsigned char size [16] = {0,0,0,0, 2,1,1,1, 4,4,1,1, 4,3,2,1};
    int i,x;
    // mask the high nibble (divisor & padding), shift it right, range 0x00-0x0F
    int n = (ucFMT & BGN_FMT_MASK_DIVPAD)>>4;
    int l = strlen(string);
    char cWork[255], cTmp;
    strcpy(cWork, "0.0000"); // fill it with leading zero's
    strcat(cWork, string); // append our value-string
    strcat(cWork, "0000"); // append traling zero's
    x = shift[n];
    //log_debug("sValue %s (len=%d, index=%d) -> step 1 = %s, shift= %d", string, l, n, cWork, x);
    if (x > 0) {
        // move dec, size_timal point to the right, only when non-zero
        // decimal point is at 1, shift it right
        for (i=0; i<x+l; i++) {
            cTmp = cWork[i+1]; // save it
            cWork[i+1] = cWork[i+2]; // move it right
            cWork[i+2] = cTmp; // put back saved character
        }
    }
    substring(strC, cWork, start[n], l+size[n]);
    //log_debug("right-shift %s, substring %s, n= %d, start= %d, size= %d", cWork, strC, n, start[n], size[n]);
    return(strC);
}

// convert the channel data to ASCII for display purposes
char *decode_value(char* pStr, unsigned char* pBuf) {
    //unsigned int d, s, f;
    char sWork[256];
    union ValueUnion {
        uint16_t ui16;
        int32_t i32;
        int i;
        unsigned char c[8];
    } VU;
    // mask the low nibble (conversion factor, bits 3-0)
    switch (pBuf[BGN_VALUE_FMT] & BGN_FMT_MASK_DT) {
        case 0x00://used for CMD=3/ASCII (LATLON position information) 
            sprintf(pStr, "untested");
            break;
        case 0x05: //time 0-99, untested
        case 0x06: //raw LCD segments
        case 0x07:
            sprintf(pStr, "untested");
            break;
        case 0x0A: //signed integer (32bits, 4 bytes), untested
            VU.i = (pBuf[5]|(pBuf[4]<<8)|(pBuf[3]<<16)|(pBuf[2]<<24));
            if (VU.i > INT_MAX) {
                VU.i = VU.i-UINT_MAX-1;
            }
            sprintf(pStr, "%d", VU.i);
            break;
        case 0x02: //4 bytes, data 3x numerical
            VU.i = pBuf[5]+(pBuf[4]*0xFF)+(pBuf[3]*0xFFFF)+(pBuf[2]*0xFFFFFF);
            sprintf(pStr,"%d",VU.i);
            break;
        case 0x03: //timer value (5 bytes), tested
            sprintf(pStr,"%02d:%02d:%02d", pBuf[2], pBuf[3], pBuf[4]); // includes the null-terminator    
            break;
        case 0x04: //1 spare byte (any value) + 3 ASCII characters, tested
            pStr[0] = pBuf[3];
            pStr[1] = pBuf[4];
            pStr[2] = pBuf[5];
            pStr[3] = 0x00; // string terminator
            break;
        case 0x01: //signed integer (-9999/+9999), used for rudderangle (-45/+45), windangle (-180/+180)
        case 0x08: //unsigned integer (0-511), used for speed (0-nn.nn), course (0-360)
            VU.i = (pBuf[3]|(pBuf[2]<<8));
            if (VU.i > SHRT_MAX) {
                VU.i = VU.i-USHRT_MAX-1;
            }
            sprintf(pStr, "%d", VU.i);
            break;
        default: // 09, 0B, 0C, 0D and 0F are undefined
            sprintf(pStr, "??????");
            break;
    }
    //switch (d) {
    //    case 0: // no decimals, leave like it is
    //        break;
    //    default:
            insertDivisor(sWork, pStr, pBuf[BGN_VALUE_FMT]);
            strcpy(pStr, sWork); // return it
    //        break;
    //}
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
void CounterUpCMD(BGN_COMMAND *pCMD, time_t t) {
    pCMD->tTimestamp = t;
    // update counter (with wrap-around)
    if (pCMD->iCounter < SHRT_MAX) {
        pCMD->iCounter++;
    } else {
        pCMD->iCounter = 1;
        pCMD->tCounterReset = t;
    }
}
void CounterRxUpDEV(BGN_DEVICE *pDEV, time_t t) {
    pDEV->tTimestamp = t;
    // update counter (with wrap-around)
    if (pDEV->iCounterRx < SHRT_MAX) {
        pDEV->iCounterRx++;
    } else {
        pDEV->iCounterRx = 1;
        pDEV->tCounterReset = t;
    }
}
void CounterTxUpDEV(BGN_DEVICE *pDEV, time_t t) {
    pDEV->tTimestamp = t;
    // update counter (with wrap-around)
    if (pDEV->iCounterTx < SHRT_MAX) {
        pDEV->iCounterTx++;
    } else {
        pDEV->iCounterTx = 1;
        pDEV->tCounterReset = t;
    }
}

// routine to process values in the body. A body might contain multiple values
int process_CMD_DATA(BGNET_CTRL* pCtrl, unsigned char *buffer) {
    int y,z;
    //unsigned int d,s,f;
    unsigned char *pBuf,*pEnd;
    unsigned char ucCH, ucFMT;
    time_t t;

    pBuf = buffer+BGNET_FRAME_HEAD;
    pEnd = buffer+BGNET_FRAME_HEAD + buffer[BGNET_FRAME_HEAD_SIZE];
    z=0;
    time(&t);
    // scan through frame/subframe
    while (pBuf < pEnd) {
        ucCH = *(pBuf+BGN_VALUE_CNR);
        ucFMT = *(pBuf+BGN_VALUE_FMT);
        y = value_length_DATA(pCtrl, ucCH, ucFMT); // includes protection against unknown types
        //fmt_decode(ucFMT, &d, &s, &f);

        // update CHANNEL structure
        pCtrl->DATA[ucCH].ucFROM = *(buffer + BGNET_FRAME_HEAD_FROM); // originator of the frame is sender of all channels within
        pCtrl->DATA[ucCH].iNewChannel = 1; // signal this channel as new/changed
        decode_value(pCtrl->DATA[ucCH].cValue, pBuf + BGN_VALUE_CNR); // decode the value for this channel
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
                ucFROM = pBuf[BGNET_FRAME_HEAD_FROM];
                CounterTxUpDEV(&pCtrl->DEV[ucFROM], t);

            } else {
                // The HEADER is OK, process it. Note that some CMD's do not have a body and in such case, we should skip the body & CS part.

                // update DEVICE structure (statistics)
                ucFROM = pBuf[BGNET_FRAME_HEAD_FROM];
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
                        strcpy(pCtrl->CMD[ucCMD].cFrame, asciibuffer);
                        memcpy(pCtrl->CMD[ucCMD].ucFrame, pBuf, xSize);

                        CounterUpCMD(&pCtrl->CMD[ucCMD], t);
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
                                break;
                            case BGNET_FRAME_CMD_QUAD_RESET1:
                            case BGNET_FRAME_CMD_QUAD_RESET3:
                            //makeQUADresponseRESET
                                iIsValid = 1;
                                pCtrl->ACP.uiTripLogNM = 0; // reset log trip counter            
                                len_pl = makeQUADresponseRESET(pCtrl, payload);
                                len_msg = bgn_net_makemessage(tmp_buffer, 0x30, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_QUAD_RESET1, payload, len_pl);
                                memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &tmp_buffer, len_msg); // put output message in write buffer
                                g_serial_data_lenTx = g_serial_data_lenTx + len_msg; // increment byte counter in TX buffer
                                break;
                            case BGNET_FRAME_CMD_PILOT_AM:
                                iIsValid = 1;
                                log_info("  Command %02X AUTO/STBY BUTTON PRESSED (BY: %02X, %s, state= %d)", *(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDEVDT[x].sModelName, *(pBuf+BGNET_FRAME_HEAD+4));
                                pCtrl->ACP.ucSTATE = *(pBuf+BGNET_FRAME_HEAD+4); // AUTO/STBY state
                                len_pl = makeACPresponseFAST(pCtrl, payload);
                                len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, 0xD3, payload, len_pl);
                                memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &tmp_buffer, len_msg); // put output message in write buffer
                                g_serial_data_lenTx = g_serial_data_lenTx + len_msg; // increment byte counter in TX buffer
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
                            strcpy(pCtrl->CMD[ucCMD].cFrame, asciibuffer);
                            memcpy(pCtrl->CMD[ucCMD].ucFrame, pBuf, xSize);

                            CounterUpCMD(&pCtrl->CMD[ucCMD], t);
                        }   
                    } else {
                        // CS is bad (only skip this sub-frame, the length is known)
                        log_info("    Frame Checksum BAD, framesize: %d", xSize);
                    }
                }
            }
        } else {
            // header CS invalid, we can not process this subframe, we can not establish a subframe length unambigiously, so we dump the receive buffer
            break;
            log_error("  Header Checksum ERR, dumping RX frame");
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
