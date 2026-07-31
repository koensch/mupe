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

#define BGNET_COMMANDTYPE_REQ   1
#define BGNET_COMMANDTYPE_DTA   2
#define BGNET_COMMANDTYPE_ACK   3
#define BGNET_COMMANDTYPE_NORM  0
unsigned char bgnet_command_type(FRAME_H* pF) {
    unsigned char r;
    switch(pF->CMD) {
        case BGNET_FRAME_CMD_CD_PILOT_DMPREQ:
        case BGNET_FRAME_CMD_CF_PILOT_MANREQ:
        case BGNET_FRAME_CMD_D0_PILOT_SPDREQ:
        case BGNET_FRAME_CMD_D1_PILOT_RESPREQ:
        case BGNET_FRAME_CMD_D2_PILOT_MANREQ:
        case BGNET_FRAME_CMD_D3_PILOT_AM:
        case BGNET_FRAME_CMD_D4_PILOT_MODE:
        case BGNET_FRAME_CMD_DE_PILOT_WAREQ:
        case BGNET_FRAME_CMD_DF_PILOT_OCREQ:
        //case BGNET_FRAME_CMD_EA_QUAD_UNKNOWN:
            r = BGNET_COMMANDTYPE_REQ;
            break;
        case BGNET_FRAME_CMD_20_PILOT_OCDAT:
        case BGNET_FRAME_CMD_21_PILOT_WADAT:
        case BGNET_FRAME_CMD_B1_PILOT_DELTADAT:
        case BGNET_FRAME_CMD_D5_PILOT_RESPDAT:
            r = BGNET_COMMANDTYPE_DTA;
            break;
        case BGNET_FRAME_CMD_ACK:
            r = BGNET_COMMANDTYPE_ACK;
            break;
        default:
            r = BGNET_COMMANDTYPE_NORM;
            break;
    }
    return(r);
}

#define BGNET_FRAME_ACKSEQ  0
// when the ACKSEQ is a multple of 13 (0x0D) we see weird things:
// the 0A phenomenon: the next CMD-REQ/CMD-DAT/CMD-ACK handshake after 0x0C, we expect the ACKSEQ to be 0x0D, but we get 0x0A, causing a CS error. But if we
//                    correct the CS-SEED, the PILOT does not accept our CMD-DAT reply. If we replace/patch the 0x0A with 0x0D in the received CMD-REQ, the
//                    receiving frame's CS is OK and when we send a CMD-DAT with the ACKSEQ with the 0x0D, every thing seems fine.
// the 13 phenomenon: We still observer a CS error when the ACKSEQ we receive is a multiple of 13 (0x0D). It was obverved with 0x27 (3x13=39=0x27) and with
//                    0xEE (12x13=238=0xEE).
//                    My guess is that the "0A phenomenon" is an instance of the "13 phenomenon" in disquise
// If this behavour is intended (to confuse reverse-engineering), a bug or a work-around for some other phenomenon, i don't know.
// I have deliberated to skip the CS checking entirely for every CMD-REQ/CMD-DAT/CMD-ACK sequence that follows a multiple of 12, but decided not to do so. This
// allows me to investigate CS errors and come up with a 100% solution
unsigned char bgnet_weird_seed(BGNET_CTRL *pCtrl, FRAME_H* pF) {
    unsigned char r;
    r = BGNET_FRAME_CR_FRAME;
    if ((pF->FROM == BGNET_TYPE_NETPILOT) & (bgnet_command_type(pF) == BGNET_COMMANDTYPE_REQ)) {
        if (pCtrl->ACP.weird.ACKSEQ > pF->payload[BGNET_FRAME_ACKSEQ]) {
            // out of sequence, this is weird
            log_debug("ACKSEQ out-of-sequence: 0x%02X (received) vs 0x%02X (stored)", pF->payload[BGNET_FRAME_ACKSEQ], pCtrl->ACP.weird.ACKSEQ);
            // keep the default SEED, but patch the ACKSEQ to the correct value
            switch (pF->payload[BGNET_FRAME_ACKSEQ]) {
                case 0x0A:
                    pF->payload[BGNET_FRAME_ACKSEQ] = 0x0D;  // correct ACKSEQ, makes calculated CS correct against the received CS
                    break;
            }
        } else {
            if (pF->payload[BGNET_FRAME_ACKSEQ] == 0xFF) {
                // roll-over when at 0xFF
                pCtrl->ACP.weird.ACKSEQ = 0;
            } else {
                pCtrl->ACP.weird.ACKSEQprev = pCtrl->ACP.weird.ACKSEQ;
                pCtrl->ACP.weird.ACKSEQ = pF->payload[BGNET_FRAME_ACKSEQ];
            }
        }
    }
    return(r);
}

// calculate the checksum over a bytes sequence.
// Used generate a CR for sending messages, and checking the received CR
unsigned char bgnet_checksum(unsigned char *pBuf, unsigned char init, int start, int max) {
    int i;
    unsigned char c = -init;
    for (i=start; i<(start+max); i++) {
        c -= pBuf[i];
    }
    return (c);
}
unsigned char bgnet_checksum_head_calc(FRAME_H* pF) {
    return(bgnet_checksum((unsigned char*)pF, BGNET_FRAME_CR_HEAD, 0, BGNET_FRAME_HEAD-1));
}
unsigned char bgnet_checksum_frame_calc(BGNET_CTRL* pCtrl, FRAME_H* pF) {
    return(bgnet_checksum(pF->payload, pCtrl->ACP.weird.FRAME_SEED, 0, pF->PL));
}
#define BGNET_CHECKSUM_OK   0x01
unsigned char bgnet_checksum_head_check(FRAME_H* pF) {
    unsigned char r, cs;
    cs = bgnet_checksum_head_calc(pF);
    if (pF->CSH == cs) {
        r = BGNET_CHECKSUM_OK;
    } else {
        r = ~BGNET_CHECKSUM_OK;
        log_error("  Header Checksum ERR, framesize: %d (received: 0x%02X, calculated: 0x%02X)", bgnet_sizeof_frame(pF), pF->payload[pF->PL], cs);
    }
    return(r);
}
unsigned char bgnet_checksum_frame_check(BGNET_CTRL* pCtrl, FRAME_H* pF) {
    unsigned char r, cs;
    pCtrl->ACP.weird.FRAME_SEED = bgnet_weird_seed(pCtrl, pF); // we are doing something weird
    cs = bgnet_checksum_frame_calc(pCtrl, pF);
    if (pF->payload[pF->PL] == cs) {
        r = BGNET_CHECKSUM_OK;
    } else {
        r = ~BGNET_CHECKSUM_OK;
        log_error("    Frame Checksum ERR, framesize: %d (received: 0x%02X, calculated: 0x%02X, seed= 0x%02X, previous=0x%02X)", bgnet_sizeof_frame(pF), pF->payload[pF->PL], cs, pCtrl->ACP.weird.FRAME_SEED, pCtrl->ACP.weird.ACKSEQprev);
    }
    return(r);
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

// initialize a counter
void CounterInit(BGNET_COUNT *Counter, time_t t) {
    Counter->iCounter = 0; // 0 means 'never touched'
    Counter->tCounterReset = t;
    Counter->tTimestamp = t;
}
// increment a counter
void CounterUp(BGNET_COUNT *Counter, time_t t) {
    if (Counter->iCounter < SHRT_MAX) {
        Counter->iCounter++;
    } else {
        Counter->iCounter = 1; // start at 1 because 0 means 'never touched'
        Counter->tCounterReset = t;
    }
    Counter->tTimestamp = t;
}
// initialize a NMEA value
void NMEAinit(NMEA2K_VALUE* pNMEA, time_t t) {
    pNMEA->source = BGN_SOURCE_NONE;
    pNMEA->value.nLong = 0;
    pNMEA->value.dValue = 0.0;
    CounterInit(&pNMEA->Counter, t);
    return;
}
// used by log (dump buffer contents)
void log_debug_buffer(unsigned char *pBuf, size_t y) {
    char asciibuf[BGN_FRAME_SIZE];
    byte2ascii(asciibuf, sizeof(asciibuf), pBuf, y, BYTE2ASCII_SPACED);
    log_debug("log_debug_buffer: %d bytes -> %s", y, asciibuf);
}
// used by log (dump a frame)
void log_debug_frame(FRAME_H* pF) {
    char asciibuf[BGN_FRAME_SIZE];
    byte2ascii(asciibuf, sizeof(asciibuf), (unsigned char*)pF, bgnet_sizeof_frame(pF), BYTE2ASCII_SPACED);
    log_debug("log_debug_frame: Command 0x%02X -> %s", pF->CMD, asciibuf);
}

// a reset is usually done:
// - at startup of this driver
// - when we observe "POWERUP" message(s) from the NETWORK (of any of the units)
void bgnet_initialize_reset(BGNET_CTRL *pCtrl) {
    pCtrl->iInitialized = BGNET_CTRL_INIT_RESET; // reset initialized state
}
// initialize our own internal structures
void bgnet_initialize(BGNET_CTRL *pCtrl) {
    int i;
    time_t t;

    if (pCtrl->iInitialized != BGNET_CTRL_INIT_RESET){
        // controlblock is already initialized. This means that this initialisation only happens if the 'initialized state' was reset
        // prior this function.
    } else {
        // note: we do NOT iniliaize the pointers to the data structures. These are set upon start and never change

        pCtrl->ACP.weird.ACKSEQ = 0x00;
        pCtrl->ACP.weird.FRAME_SEED = BGNET_FRAME_CR_FRAME;

        // stage 1: set state 'initialized'
        time(&t); // save timestamp to we use same timestamp everywhere
        pCtrl->iInitialized = BGNET_CTRL_INIT; // mark as initialized
        pCtrl->iDeviceCounter = BGNET_CTRL_DEVCNTR_INIT;
        pCtrl->iDeviceRun = 0; //reset run counter (0-1000)

        // stage 2 - ACP initialize data structure
        pCtrl->ACP.iNewACP = 0;
        pCtrl->ACP.tTimestamp = t;

        // PILOT watchdog (set by PILOT_HELLO, count-down by timer)
        pCtrl->ACP.PILOTWD.value.nLong = BGNET_PILOTWD_SET;
        pCtrl->ACP.PILOTWD.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.PILOTWD.Counter, t);

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
        pCtrl->ACP.SETUP_SPD.value.nByte = 1; //0=Transmitter, 1=Manual, 2=SOG
        pCtrl->ACP.SETUP_SPD.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.SETUP_SPD.Counter, t);

        pCtrl->ACP.SETUP_RESP.value.nByte = 2;  //0=Normal, 1=Downwind, 2=Eco
        pCtrl->ACP.SETUP_RESP.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.SETUP_RESP.Counter, t);

        pCtrl->ACP.SETUP_WA_MIN.value.nLong = 14; // default value is 4 minutes, range 1-360 (so it must be a 16 bits register)
        pCtrl->ACP.SETUP_WA_MIN.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.SETUP_WA_MIN.Counter, t);

        pCtrl->ACP.SETUP_WA_STAT.value.nLong = 0; // default is OFF
        pCtrl->ACP.SETUP_WA_STAT.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.SETUP_WA_STAT.Counter, t);

        pCtrl->ACP.SETUP_OC_DEG.value.nLong = 25; // default value is 15degrees; range 1-99
        pCtrl->ACP.SETUP_OC_DEG.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.SETUP_OC_DEG.Counter, t);

        pCtrl->ACP.SETUP_OC_STAT.value.nLong = 0; // default is OFF
        pCtrl->ACP.SETUP_OC_STAT.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.SETUP_OC_STAT.Counter, t);

        pCtrl->ACP.SETUP_JSM.value.nLong = 1; // Joystick Steering Mode (Normal, Proportional)
        pCtrl->ACP.SETUP_JSM.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.SETUP_JSM.Counter, t);

        pCtrl->ACP.SETUP_DMP.value.nLong = 44; // Compass Damping, 1-99 sec, 4= default
        pCtrl->ACP.SETUP_DMP.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.SETUP_DMP.Counter, t);

        pCtrl->ACP.SETUP_FR.value.nLong = 1; // Fault Reporting (On/Off)
        pCtrl->ACP.SETUP_FR.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.SETUP_FR.Counter, t);

        // manual speed parameters
        pCtrl->ACP.MANSPEED_SEL.value.nLong = 0;
        pCtrl->ACP.MANSPEED_SEL.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.MANSPEED_SEL.Counter, t);

        pCtrl->ACP.MANSPEED_FST.value.nLong = 310;      // value is displayed as xx.x, but the decimal is always 0 and only whole number can be set
        pCtrl->ACP.MANSPEED_FST.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.MANSPEED_FST.Counter, t);

        pCtrl->ACP.MANSPEED_MED.value.nLong = 230;      // value is displayed as xx.x, but the decimal is always 0 and only whole number can be set
        pCtrl->ACP.MANSPEED_MED.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.MANSPEED_MED.Counter, t);

        pCtrl->ACP.MANSPEED_SLW.value.nLong = 120;      // value is displayed as xx.x, but the decimal is always 0 and only whole number can be set
        pCtrl->ACP.MANSPEED_SLW.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.MANSPEED_SLW.Counter, t);

        // other parameters
        pCtrl->ACP.BackLight.value.nByte = 0; // backlight (0=OFF, 3-1=ON)
        pCtrl->ACP.BackLight.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.BackLight.Counter, t);

        pCtrl->ACP.CTS.value.nLong = 123; // Course To Steer (calculated by the Autopilot, range 0-359)
        pCtrl->ACP.CTS.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.CTS.Counter, t);

        pCtrl->ACP.Heading.value.nLong = 321; // Heading from compass (0-359)
        pCtrl->ACP.Heading.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.Heading.Counter, t);

        pCtrl->ACP.Rudder.value.nLong = -2; // Rudder position (-45-+45)
        pCtrl->ACP.Rudder.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.Rudder.Counter, t);

        // we do NOT use a floating point value; otherwise we get all kind or rounding errors. Just keep it as handled within the
        // B&G system: as an integer*10 (so we get 1 decimal)
        pCtrl->ACP.StoredLogNM.value.nLong = 12345*BGNET_DIVISOR_STORLOG+6; // channel CD/STORELOGNM (on the real system this is stored in non-volitile memory by QUAD/SPEED, no reset)
        pCtrl->ACP.StoredLogNM.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.StoredLogNM.Counter, t);

        // it appears as if the triplog has a multiplier of 64 (and not 10)
        pCtrl->ACP.TripLogNM.value.nLong = 1234*BGNET_DIVISOR_TRIPLOG+5; // channel CF/TRIPLOGNM (on the real system this is stored in non-volitile memory by QUAD/SPEED, resetable)
        pCtrl->ACP.TripLogNM.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.TripLogNM.Counter, t);

        pCtrl->ACP.DRDNM.value.nLong = 123; // channel 81/DRDNM (on the real system this is stored in non-volitile memory by QUAD/SPEED, resetable)
        pCtrl->ACP.DRDNM.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.DRDNM.Counter, t);

        pCtrl->ACP.Speed.value.nLong = 125; // used to initialize SpeedMAX and SpeedAVG
        pCtrl->ACP.Speed.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.Speed.Counter, t);

        pCtrl->ACP.SpeedMAX.value.nLong = 222; // reset with DRD
        pCtrl->ACP.SpeedMAX.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.SpeedMAX.Counter, t);

        pCtrl->ACP.SpeedAVG.value.nLong = 111; // reset with DRD
        pCtrl->ACP.SpeedAVG.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.SpeedAVG.Counter, t);

        pCtrl->ACP.QUAD_DMP.value.nLong = 7;
        pCtrl->ACP.QUAD_DMP.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.QUAD_DMP.Counter, t);

        pCtrl->ACP.QUAD_CAL.value.nLong = 1;
        pCtrl->ACP.QUAD_CAL.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.QUAD_CAL.Counter, t);

        pCtrl->ACP.QUAD_CALLOG.value.nLong = 2;
        pCtrl->ACP.QUAD_CALLOG.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.QUAD_CALLOG.Counter, t);

        pCtrl->ACP.QUAD_CALAUTO.value.nLong = 3;
        pCtrl->ACP.QUAD_CALAUTO.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->ACP.QUAD_CALAUTO.Counter, t);

        // stage 3 - initialize device structure
        pCtrl->iNewDevice = BGN_NEW_RESET;
        for (i=0; i<BGNET_DEVICE_MAX; i++) {
            pCtrl->DEV[i].iNewDevice = BGN_NEW_RESET;
            // error counters Checksum
            pCtrl->DEV[i].iCSError_Header = 0;
            pCtrl->DEV[i].iCSError_Frame = 0;
            CounterInit(&pCtrl->DEV[i].CounterRx, t);
            memset(pCtrl->DEV[i].Rx.ucFrame, 0, BGN_FRAME_SIZE);
            pCtrl->DEV[i].Rx.FH.CMD = BGNET_COMMAND_INIT;
            CounterInit(&pCtrl->DEV[i].CounterTx, t);
            memset(pCtrl->DEV[i].Tx.ucFrame, 0, BGN_FRAME_SIZE);
            pCtrl->DEV[i].Tx.FH.CMD = BGNET_COMMAND_INIT;
            pCtrl->DEV[i].source = BGN_SOURCE_NONE;
        }

        // stage 4 - initialize command structure
        pCtrl->iNewCommand = BGN_NEW_RESET; // no new data available
        for (i=0; i<BGNET_COMMAND_MAX; i++) {
            pCtrl->CMD[i].iNewCommand = BGN_NEW_RESET;
            CounterInit(&pCtrl->CMD[i].CounterRx, t);
            memset(pCtrl->CMD[i].Rx.ucFrame, 0, BGN_FRAME_SIZE);
            pCtrl->CMD[i].Rx.FH.CMD = BGNET_COMMAND_INIT;
            CounterInit(&pCtrl->CMD[i].CounterTx, t);
            memset(pCtrl->CMD[i].Tx.ucFrame, 0, BGN_FRAME_SIZE);
            pCtrl->CMD[i].Tx.FH.CMD = BGNET_COMMAND_INIT;
        }

        // stage 5 - initialize channel structure
        pCtrl->iNewChannel = BGN_NEW_RESET; // no new data available
        for (i=0; i<BGNET_CHANNEL_MAX; i++) {
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
        strcpy(pBuf, "undefined command");  // must be shorter than BGNET_CMD_LABEL/20
    }
    return(pBuf);
};

// get the channel label from the Channel Definition Table
char* strChannelLabel(BGNET_CTRL *pCtrl, char *pBuf, unsigned char ucCHN) {
    if (pCtrl->pCHNDT[ucCHN].ucChannelIndex == ucCHN) {
        strcpy(pBuf, pCtrl->pCHNDT[ucCHN].cLabel);
    } else {
        strcpy(pBuf, "undefined channel");  // must be shorter than BGNET_CHN_LABEL/20
    }
    return(pBuf);
};

// encode a frame (with or without payload)
size_t bgnet_encode_frame(BGNET_CTRL* pCtrl, FRAME_H* pFH, int iTo, int iFrom, int iCmd, unsigned char *payload, int payloadlength) {
    pFH->TO = iTo;
    pFH->FROM = iFrom;
    pFH->PL = payloadlength;
    pFH->CMD = iCmd;
    pFH->CSH = bgnet_checksum_head_calc(pFH);
    if (payloadlength == 0) {
        // nothing to do
    } else {
        memcpy(pFH->payload, payload, payloadlength);
        pFH->payload[payloadlength] = bgnet_checksum_frame_calc(pCtrl, pFH);
    }
    return(bgnet_sizeof_frame(pFH));
}

// this one usually sends 8 channels
int makeQUADresponse2(BGNET_CTRL* pCtrl, unsigned char* payload) {
    int r=0, c;
    union CH* puCH;

    c = BGN_CNR_41_BOATSPEEDKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(pCtrl->ACP.Speed.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_42_BOATSPEEDRAW;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(225);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    // the next 3 values are 'rotated' by the QUAD: CD every other time, 64/65 every 4'th time
    // but that is too complex at this moment. Later i will build a rotating scheme.

    c = BGN_CNR_CD_STORLOGNM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uL[0] = htonl(pCtrl->ACP.StoredLogNM.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_64_SPEEDAVGKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(pCtrl->ACP.SpeedAVG.value.nLong); 
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_65_SPEEDMAXKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(pCtrl->ACP.SpeedMAX.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    // the next values are always present 

    c = BGN_CNR_81_DRDNM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(pCtrl->ACP.DRDNM.value.nLong); 
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_CF_TRIPNM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(pCtrl->ACP.TripLogNM.value.nLong); 
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_D3_DR_COURSE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(-20); 
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_20_SEATEMPQ;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(12); 
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_75_TIMER;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uB[0] = 23; // HH
    puCH->s.cv.uB[1] = 59; // MM
    puCH->s.cv.uB[2] = 59; // SS
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

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
    puCH->s.cv.uC[1] = ' '; 
    puCH->s.cv.uC[2] = 'F'; 
    puCH->s.cv.uC[3] = ' ';
    r = r + bgnet_sizeof_channel(pCtrl, puCH);
    
    // the next 3 values are 'rotated' by the QUAD: FE every other time, FC/FD every 4'th time
    // but that is too complex at this moment. Later i will build a rotating scheme.

    c = BGN_CNR_FC_SHALLOWALARM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uC[0] = '\0'; 
    puCH->s.cv.uC[1] = 'O'; 
    puCH->s.cv.uC[2] = 'F'; 
    puCH->s.cv.uC[3] = 'F';
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_FD_DEPTHALARM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(5); 
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_FE_ANCHORALARM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uC[0] = '\0'; 
    puCH->s.cv.uC[1] = 'O'; 
    puCH->s.cv.uC[2] = 'F'; 
    puCH->s.cv.uC[3] = 'F';
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    // the next values are always present 

    c = BGN_CNR_FF_DEPTH_CAL;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(4); 
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_C4_DEPTHM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(6); 
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_C1_DEPTHM;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(2); 
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    return(r);
}

int makeACPresponseCTARGET(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r = 0,c;
    union CH* puCH;

    c = BGN_CNR_A6_AP_CTARGET;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(pCtrl->ACP.CTS.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    return(r);
}
int makeACPresponseSTATUSONLY(BGNET_CTRL *pCtrl, unsigned char* payload) {
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
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    // CTARGET is displayed by PILOT in large digits. If we switch to AUTO, and we omit
    // the CTARGET value, it might take some time before the CTARGET is displayed.
    // To prevent this nuiseance we include the CTARGET when we are in AUTO
    if (puCH->s.cv.uB[0] == BGN_AP_MODE1_AUTO) {
        c = BGN_CNR_A6_AP_CTARGET;
        puCH = (union CH*)&(payload[r]);
        puCH->s.CH = c;
        puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
        puCH->s.cv.uW[0] = htons(pCtrl->ACP.CTS.value.nLong);
        r = r + bgnet_sizeof_channel(pCtrl, puCH);
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
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_0B_RUDDER;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(pCtrl->ACP.Rudder.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    // when in STBY, the display shows the HEADING (compass reading)
    c = BGN_CNR_49_HEADING;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(pCtrl->ACP.Heading.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    // when in AUTO, the display shows the COURSE TARGET (setpoint for the autopilot)
    c = BGN_CNR_A6_AP_CTARGET;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(pCtrl->ACP.CTS.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

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

    c = BGN_CNR_B5_AP_STATUS;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uB[0] = pCtrl->ACP.STATE.value.nByte; // AUTO/STBY
    puCH->s.cv.uB[1] = pCtrl->ACP.MODE.value.nByte;  // COMP/NAV/VANE
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_0B_RUDDER;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(pCtrl->ACP.Rudder.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    // when in STBY, the display shows the HEADING (compass reading)
    c = BGN_CNR_49_HEADING;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(222);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    // when in AUTO, the diplay shows the COURSE TARGET (set of the autopilot)
    c = BGN_CNR_A6_AP_CTARGET;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(pCtrl->ACP.CTS.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);
 
    c = BGN_CNR_51_AWINDANGLE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(100);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_59_TWINDANGLE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(102);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_EE_XTE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(-57);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);
 
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
            r = r + bgnet_sizeof_channel(pCtrl, puCH);

            c = BGN_CNR_E8_WPD;
            puCH = (union CH*)&(payload[r]);
            puCH->s.CH = c;
            puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
            puCH->s.cv.sW[0] = htons(333);
            r = r + bgnet_sizeof_channel(pCtrl, puCH);

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

    c = BGN_CNR_0B_RUDDER;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(pCtrl->ACP.Rudder.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_41_BOATSPEEDKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(pCtrl->ACP.Speed.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_49_HEADING;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(77);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_51_AWINDANGLE;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(-95);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

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
    return(bgnet_sizeof_channel(pCtrl, pCH));
}

int makeRESPresponse(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r = 0,c;
    union CH* puCH;

    c = BGN_CNR_A6_AP_CTARGET; // RESP selector
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uB[0] = 0;
    puCH->s.cv.uB[1] = pCtrl->ACP.SETUP_RESP.value.nByte;
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    return (r);
}
int makeWAresponse(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r = 0,c;
    union CH* puCH;
    c = BGN_CNR_76_AP_WA_ALARM; // WA limit (minutes) and ON/OFF status
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    if (pCtrl->ACP.SETUP_WA_STAT.value.nLong == 0) {
        puCH->s.FMT = BGN_FMT_08 ;
    } else {
        puCH->s.FMT = BGN_FMT_18 ;
    }
    puCH->s.cv.uW[0] = htons(pCtrl->ACP.SETUP_WA_MIN.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);
    return (r);
}
int makeOCresponse(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r = 0,c;
    union CH* puCH;
    c = BGN_CNR_AF_AP_OC_ALARM; // OC limit (degress) amd ON/OFF status
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    if (pCtrl->ACP.SETUP_OC_STAT.value.nLong == 0) {
        puCH->s.FMT = BGN_FMT_08 ;
    } else {
        puCH->s.FMT = BGN_FMT_18 ;
    }
    puCH->s.cv.uW[0] = htons(pCtrl->ACP.SETUP_OC_DEG.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);
    return (r);
}
int makeSPEEDresponse(BGNET_CTRL *pCtrl, unsigned char* payload) {
    int r = 0,c;
    union CH* puCH;

    c = BGN_CNR_41_BOATSPEEDKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(pCtrl->ACP.Speed.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    c = BGN_CNR_EB_SOGKT;
    puCH = (union CH*)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(325);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    return (r);
}

// This is the "device processor".
// We scan through the data stored by the frame decoder.
// If we want to generate some output for transmission, put the frame in the transmission buffer
// (at this moment we see no need to do such thing)
size_t bgnet_process(BGNET_CTRL *pCtrl, unsigned char* buffer, size_t size) {
    int i;
    size_t r;
    time_t t;
    time(&t); // save the time

    //log_debug("bgnet_process: Device Processor run");

    // erase output buffer (to prevent compiler warnings about unused arguments)
    memset(buffer, 0, size);
    r = 0;

    // check if initialisation is required
    bgnet_initialize(pCtrl);

    // device received new data
    if ((pCtrl->iNewCommand == BGN_NEW_RESET)) {
        // nothing to do
    } else {
        //log_debug("bgnet_process: Resetting CMD");
        for (i=0; i<BGNET_COMMAND_MAX; i++) {            if (pCtrl->CMD[i].iNewCommand == BGN_NEW_RESET) {
                // nothing to do
            } else {
                switch(i) {
                    default:
                        break;
                }
                pCtrl->CMD[i].iNewCommand = BGN_NEW_RESET; // reset new data flag for this command
                //log_debug("bgnet_process: Resetting COMMAND 0x%02X", i);
            }
        }
        pCtrl->iNewCommand = BGN_NEW_RESET;
    }

    if (pCtrl->iNewChannel == BGN_NEW_RESET) {
        // nothing to do
    } else {
        //log_debug("bgnet_process: Resetting CHN");
        for (i=0; i<BGNET_CHANNEL_MAX; i++) {
            if (pCtrl->CHN[i].iNewChannel == BGN_NEW_RESET) {
                // nothing to do
            } else {
                // process new channel data (to be added later)
                pCtrl->CHN[i].iNewChannel = BGN_NEW_RESET; // reset newdata flag
                //log_debug("bgnet_process: Resetting CHANNEL 0x%02X", i);
            }
        }
        pCtrl->iNewChannel = BGN_NEW_RESET; // reset
    }

    if ((pCtrl->iNewDevice == BGN_NEW_RESET)) {
        // nothing to do
    } else {
        //log_debug("bgnet_process: Resetting DEV");
        for (i=0; i<BGNET_DEVICE_MAX; i++) {
            if (pCtrl->DEV[i].iNewDevice == BGN_NEW_RESET) {
                // nothing to do
            } else {
                // process new channel data (to be added later)
                pCtrl->DEV[i].iNewDevice = BGN_NEW_RESET; // reset newdata flag
                //log_debug("bgnet_process: Resetting DEVICE 0x%02X", i);
            }
        }
        pCtrl->iNewDevice = BGN_NEW_RESET; // reset
    }
    
#define BGNET_ENABLE_QUAD_TX
#ifdef BGNET_ENABLE_QUAD_TX
    size_t len_msg;
    int len_pl;
    unsigned char payload[1024];
    FRAME_H FTx, *pFTx;
    pFTx = &FTx;

    len_pl = makeQUADresponse1(pCtrl, payload);
    len_msg = bgnet_encode_frame(pCtrl, pFTx, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_01_DATA, payload, len_pl);
    register_DATA_frame(pCtrl, pFTx);

    //log_debug_frame(pFTx);

    // put it for transmission
    memcpy(&buffer[r], pFTx, len_msg); // put output message in write buffer
    r = r + len_msg; // increment byte counter in TX buffer

    // update statistics
    memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_01_DATA].Tx.ucFrame, pFTx, len_msg);
    CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_01_DATA].CounterTx, t);

    len_pl = makeQUADresponse2(pCtrl, payload);
    len_msg = bgnet_encode_frame(pCtrl, pFTx, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_01_DATA, payload, len_pl);
    register_DATA_frame(pCtrl, pFTx);
        
    // put it for transmission
    memcpy(&buffer[r], pFTx, len_msg); // put output message in write buffer
    r = r + len_msg; // increment byte counter in TX buffer

    // update statistics
    memcpy(&pCtrl->CMD[BGNET_FRAME_CMD_01_DATA].Tx.ucFrame, pFTx, len_msg);
    CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_01_DATA].CounterTx, t);
#endif

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
size_t bgnet_sizeof_frame(FRAME_H* pF) {
    size_t r;
    if (pF->PL == 0) {
        r = BGNET_FRAME_HEAD;
    } else {
        r = BGNET_FRAME_HEAD + pF->PL + BGNET_FRAME_TAIL_CS;
    }
    return(r);
}

// returns the length of a channel block; it all depends on the FMT specifier (in the FMT Definition Table).
size_t bgnet_sizeof_channel(BGNET_CTRL* pCtrl, union CH* pCH) {
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
            // the value is unknown, i just make a guess. 4 is better than zero or -1
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
char* CommandToAscii(char* asciibuf, size_t size, FRAME_H* pF) {
    int n;
    size_t y;
    char cTail[BGN_CMD_ASCII];
    if (pF->CMD == BGNET_COMMAND_INIT) {
        // seems an empty/uninitialized buffer; we can not safely use sizeof_frame()
        strcpy(asciibuf,"---");
    } else {
        y = bgnet_sizeof_frame(pF);
        if (y*BYTE2ASCII_SPACED < size) {
            // it fits, just copy it all
            byte2ascii(asciibuf, size, (unsigned char*)pF, y, BYTE2ASCII_SPACED);
        } else {
            // we use a 7-character separation string: " /.../ " to avoid excessive long output
            // we also copy the first 10 (5 header with CS + 5 first data) and the tail (5 last data + CS = 6 bytes)
            // 6 bytes tail = 18 ascii characters
            n = (sizeof(cTail) - strlen(BGN_CMD_ASCII_SEP))/BYTE2ASCII_SPACED; // we have ascii space for 'n' databytes
            byte2ascii(asciibuf, size, (unsigned char*)pF, n-BGN_CMD_DATA_TAIL, BYTE2ASCII_SPACED);
            strcat(asciibuf, BGN_CMD_ASCII_SEP); // append separator
            byte2ascii(cTail, sizeof(cTail), ((unsigned char*)(pF))+y-BGN_CMD_DATA_TAIL, BGN_CMD_DATA_TAIL, BYTE2ASCII_SPACED);
            strcat(asciibuf, cTail);
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
                //case BGN_CNR_CF_TRIPNM: // it seems that the TRIPLOG has not 1 10.0 divisor, but a 64.0
                //    iRawValue = decode_channel_numeric(puCH);
                //    sprintf(pStr, "%.1f", ITOD(iRawValue)/BGNET_DIVISOR_TRIPLOG);
                //    iValid = 2;
                //    break;
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

// sometimes we send a DATA frame, but that is not registered (because we not process our own commands)
// using this function we can make them into our status page. we only expect single-frame data
size_t register_DATA_frame(BGNET_CTRL *pCtrl, FRAME_H* pF) {
    unsigned char* pucP; // character pointer inside the payload
    union CH* pCH;
    size_t y, z;
    time_t t;

    // we expect only a single DATA command to have one (or more) channels
    time(&t);
    z = 0;
    if (pF->CMD == BGNET_FRAME_CMD_01_DATA ) {
        pucP = pF->payload;
        while (pucP < (pF->payload + pF->PL)) {
            pCH = (union CH*)pucP;
            y = bgnet_sizeof_channel(pCtrl, pCH);
            pCtrl->CHN[pCH->s.CH].iNewChannel = BGN_NEW_SET;
            pCtrl->CHN[pCH->s.CH].x01.ucFROM = pF->FROM;
            decode_channel_string(pCtrl->CHN[pCH->s.CH].x01.cValue, pCH);
            memcpy(pCtrl->CHN[pCH->s.CH].x01.ucRAW, pCH, y);
            CounterUp(&pCtrl->CHN[pCH->s.CH].x01.CounterRx, t);
            z++;
            pucP = pucP + y;
        }
    }
    return(z); // return number of channels
}

// routine to process values in the body. A body might contain multiple values
int decode_command_DATA(BGNET_CTRL* pCtrl, FRAME_H* pF) {
    int y,z;
    unsigned char *pBuf;
    union CH* pCH;
    time_t t;  // used to timestamp incoming channel

    pBuf = pF->payload;
    z = 0; // number of channels in this command
    time(&t);
    // scan through frame/subframe
    while (pBuf < pF->payload + pF->PL) {
        pCH = (union CH*)(pBuf + BGN_VALUE_CNR);
        pCtrl->CHN[pCH->s.CH].x01.ucFROM = pF->FROM; // originator of the frame (that included this channel)

        // we need the channel length to advance to the next in this frame
        y = bgnet_sizeof_channel(pCtrl, pCH);

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
                    pCtrl->ACP.CTS.value.nLong = ntohs(pCH->s.cv.uW[0]);
                    pCtrl->ACP.CTS.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->ACP.CTS.Counter, t);
                    break;
                case BGN_CNR_49_HEADING:
                    pCtrl->ACP.Heading.value.nLong = ntohs(pCH->s.cv.uW[0]);
                    pCtrl->ACP.Heading.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->ACP.Heading.Counter, t);
                    break;
                case BGN_CNR_0B_RUDDER:
                    pCtrl->ACP.Rudder.value.nLong = (int16_t)(ntohs(pCH->s.cv.uW[0])); // make it signed
                    pCtrl->ACP.Rudder.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->ACP.Rudder.Counter, t);
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
                                g_n2kdata.boat.rudder.value.dValue = ITOD(pCtrl->ACP.Rudder.value.nLong) * BGNET_PI/180;
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
                                g_n2kdata.boat.rudder.value.dValue = ITOD((int16_t)(ntohs(pCH->s.cv.sW[0]))) * BGNET_PI/180;
                                g_n2kdata.boat.rudder.source = BGN_SOURCE_BGNET;
                                CounterUp(&g_n2kdata.boat.rudder.Counter, t);
                            }
                            break;
                    }
                    break;
                    break;
            }
#ifdef KOEN
    char buffer[BGNET_CMD_LABEL];
            log_debug("Channel #%d, ID=0x%02X, FMT=0x%02X, label=%s, value=%s", z
                                                                        , pCH->s.CH
                                                                        , pCH->s.FMT
                                                                        , strChannelLabel(pCtrl, buffer, pCH->s.CH)
                                                                        , pCtrl->CHN[pCH->s.CH].x01.cValue
                                                                    );
#endif
                                                                }
        pBuf = pBuf + y; // advance to next channel in this frame
        z++; // count channels in this frame
    }
    return(z); // return number of channels
}
// routine to register a channel for subscription
void decode_command_SUBSCRIBE(BGNET_CTRL* pCtrl, FRAME_H* pF) {
    unsigned char ucCHN;
    time_t t;

    time(&t);
    if (pF->PL == 1) { // shall only have 1 payload byte
        ucCHN = pF->payload[0]; // first payoad byte
        pCtrl->CHN[ucCHN].x06.ucTO = pF->TO;
        pCtrl->CHN[ucCHN].x06.ucFROM = pF->FROM;
        CounterUp(&pCtrl->CHN[ucCHN].x06.CounterRx, t);
        memcpy(pCtrl->CHN[ucCHN].x06.ucRAW, pF, bgnet_sizeof_frame(pF));
        log_debug("    Ch=%02X, Subscribed by %02X on %02X", ucCHN, pF->FROM, pF->TO);
    }
    return;
}
// routine to advertise values that can be subscribed
void decode_command_LABEL(BGNET_CTRL* pCtrl, FRAME_H* pF) {
    unsigned char ucCHN;
    size_t len;
    time_t t;

    time(&t);
    // this type of frame has a variable payload: Ch#, 3 bytes data, n bytes ascii
    ucCHN = pF->payload[0];
    pCtrl->CHN[ucCHN].iNewChannel = BGN_NEW_SET;
    if ((pF->PL > 4) & (pF->PL < sizeof(pCtrl->CHN[ucCHN].x02.ucRAW))) {
        pCtrl->CHN[ucCHN].x02.CounterRx.iCounter++;
        pCtrl->CHN[ucCHN].x02.CounterRx.tTimestamp = t;
        // copy the frame
        len = bgnet_sizeof_frame(pF);
        memcpy(pCtrl->CHN[ucCHN].x02.ucRAW, pF, len);
        // copy the label
        len = (pF->PL - 4); // number of ascii characters in the payload
        strncpy(pCtrl->CHN[ucCHN].x02.cValue, (char*)&pF->payload[4], len);
        pCtrl->CHN[ucCHN].x02.cValue[len] = '\0';
    } else {
        strcpy(pCtrl->CHN[ucCHN].x02.cValue, "???");
    }
    return;
}

// decode the received buffer, possibly multiple subframes, return number of sub-frames
size_t bgnet_decode_frame(BGNET_CTRL* pCtrl, unsigned char* buffer, size_t len) {
    FRAME_H* pFRx;   // pointer to the RECEIVE frame
    FRAME_H* pFTx;   // pointer to the WRITE frame
    union CH* pCHRx; // pointer to the channel structure we RECEIVE (eighter a CMD-REQ or a CMD-DAT command)
    union CH* pCHTx; // pointer to the channel structure we WRITE

    //unsigned char cs; // checksum calculated
    unsigned char *pBuf; // pointer to start and end of buffer
    int RxSize, x, z, n;
    int iIsValid, iValue;
    time_t t;
    size_t y;
    // inside this routine we work with our local buffers to store generated frames
    // this is only done in the same thread, so we do not have to do lock/unlock stuff
    unsigned char payload[1024], TxBuf[1024];
    int len_pl, len_msg, TxSize;

    pBuf = buffer; // gets incremented as we go
    z = 0; // subframe counter

    // process (possible multiple) subframes in buffer
    while (pBuf < (buffer+len)) {
        // setup pointer to our RECEIVING frame
        pFRx = (FRAME_H*)pBuf;
        
        // initialize some variables
        time(&t);
        memset(payload, 0, sizeof(payload));
        memset(TxBuf, 0, sizeof(TxBuf));
        TxSize = 0;
        len_pl = 0;
        len_msg = 0;
        iIsValid = 1; // tricky: if we hit a 'default' in a switch we reset this, so statistics is not updated

        // log received frame (at least, i hope it's a frame)
        //log_debug_frame(pFRx);

        // sorry, but the code branch and switch/cases have become more complex than i ever envisaged. It was developped over
        // many hours of reverse-engineering. You do not know where you are heading and what approch would be a neat solution
        // some pseudo-code to give you a better understanding:
        //  if CS in header OK?
        //      if no payload?
        //          yes, handle commands without payload, no frame CS
        //      else
        //          if CS frame is OK?
        //              yes, handle commands with payload
        //              switch (command type)
        //                  case generic* commands
        //                  all others
        //                      switch (node type)
        //                          case QUAD
        //                              handle specific* QUAD commands
        //                          case PILOT
        //                              handle specific* PILOT commands
        //                      end switch
        //              end switch
        //          else
        //              discard this subframe (continue with possible 2nd subframe
        //          endif
        //      endif
        //  else
        //      discard whole buffer
        // endif
        //
        // *= specific commands have a different meaning, depending on the FROM node type; generic commands behave the same for all node types
        // The branches have become too deep for my liking, but this is what i was left with after i got a good understanding of the protocol

        // if the HEAD-CR is OK, we can safely assume the byte stream is a BGNET frame
        if (bgnet_checksum_head_check(pFRx) == BGNET_CHECKSUM_OK) {
            // The HEADER is OK, process it. Note that some CMD's do not have a body and in such case, we should skip the body & CS part.
            RxSize = bgnet_sizeof_frame(pFRx); // expected framesize
            // check if this subframe is adressed TO us and not FROM us
            if (pFRx->FROM == BGNET_ADDRESS_ME) {
                // do not process this (sub)frame
            } else {
                // When we receive someting from the network, we register it, regardless whether it has a CS error,
                // or if we will discard it.
                // The reason is, that if we have a 'active' node on the BGNET, we must avoid a situation where we allow
                // the import (from NMEA2000 or SignalK) if we have a active transmitter.
                // It could well be that our received frame is in error (and that we incorretly assert that the node is active),
                // but ignoring it and allowing external sources overwrite internal sources could create havoc.
                pCtrl->DEV[pFRx->FROM].source = BGN_SOURCE_BGNET;
                pCtrl->iNewDevice = BGN_NEW_SET; // signal this device
                pCtrl->DEV[pFRx->FROM].iNewDevice = BGN_NEW_SET;
                memcpy(pCtrl->DEV[pFRx->FROM].Rx.ucFrame, pFRx, RxSize);
                CounterUp(&pCtrl->DEV[pFRx->FROM].CounterRx, t);

                x = bgnet_deviceindex(pCtrl, pFRx->FROM); // lookup by device address/type, so we can print the device name
                if (pFRx->PL == 0) {  // No payload (no frame CS)
                    switch(pFRx->CMD) {
                        case BGNET_FRAME_CMD_0E_PILOT_HELLO:
                            // when we receive this message, we know that there is atleast 1 active PILOT on the network.
                            // We set the counter; timer thread counts it down and if this message is no longer received
                            // we (the ACP) know that there is no longer user-input possible because no PILOT is connected.
                            log_info("  Command %02X (header only) KEEPALIVE by %02X %s", pFRx->CMD, pFRx->FROM, pCtrl->pDEVDT[x].sModelName);

                            // set the counter (count down done by timer thread)
                            log_info("PILOTWD reset from %d to %d", pCtrl->ACP.PILOTWD.value.nLong, BGNET_PILOTWD_SET);
                            pthread_mutex_lock(&g_data_mutex);
                            pCtrl->ACP.PILOTWD.value.nLong = BGNET_PILOTWD_SET;
                            pCtrl->ACP.PILOTWD.source = BGN_SOURCE_BGNET;
                            CounterUp(&pCtrl->ACP.PILOTWD.Counter, t);
                            pthread_mutex_unlock(&g_data_mutex);

                            // this reply gets the PILOT out of the "No PILOT" state
                            pFTx = (FRAME_H*)&TxBuf[TxSize];
                            len_pl = makeACPresponseMINI(pCtrl, payload);
                            len_msg = bgnet_encode_frame(pCtrl, pFTx, pFRx->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_01_DATA, payload, len_pl);
                            register_DATA_frame(pCtrl, pFTx);

                            memcpy(pCtrl->CMD[BGNET_FRAME_CMD_01_DATA].Tx.ucFrame, pFTx, len_msg);
                            CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_01_DATA].CounterTx, t);
                            memcpy(pCtrl->DEV[BGNET_ADDRESS_ME].Tx.ucFrame, pFTx, len_msg);
                            CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                            TxSize = TxSize + len_msg;

                            // put it for transmission
                            pthread_mutex_lock(&g_data_mutex);
                            memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &TxBuf, TxSize); // put output message in write buffer
                            g_serial_data_lenTx = g_serial_data_lenTx + TxSize; // increment byte counter in TX buffer
                            pthread_mutex_unlock(&g_data_mutex);

                            break;
                        case BGNET_FRAME_CMD_0A_POWERUP:
                            // when we receive this message one (or more) devices went though the startup process (ie. power was
                            // switch ON while we (the ACP) were still running.
                            // At this moment i see no reason to do something special at this occasion. Realise that most networks
                            // contain multiple units. You might expect a burst of these messages. Some units (like WIND) take some
                            // more time then others. Also units that are normally passive (not broadcasting channels, like DATA)
                            // will also send this message.
                            log_info("  Command %02X (header only) POWERUP by %02X %s", pFRx->CMD, pFRx->FROM, pCtrl->pDEVDT[x].sModelName);
                            // At first i thought that resetting our own structure was a good idea. It is not.
                            // At startup, a lot of initial messages are being send (command 0x02, 0x06) and if we reset our structure
                            // we erase all of that in the process. Better not resetting.
                            //bgnet_initialize_reset(pCtrl);
                            break;
                        default:
                            iIsValid = 0; // do not update device/command statistics
                            log_info("  Command %02X (header only) IGNORED (from %02X %s)", pFRx->CMD, pFRx->FROM, pCtrl->pDEVDT[x].sModelName);
                            break;
                    }
                    if (iIsValid > 0) {
                        // update CMD structure
                        pCtrl->iNewCommand = BGN_NEW_SET; // signal possible CMD change
                        pCtrl->CMD[pFRx->CMD].iNewCommand = BGN_NEW_SET; // signal this command
                        memcpy(pCtrl->CMD[pFRx->CMD].Rx.ucFrame, pFRx, RxSize);
                        CounterUp(&pCtrl->CMD[pFRx->CMD].CounterRx, t);
                    }
                } else {
                    // frame has some payload; check CR over the payload
                    if (bgnet_checksum_frame_check(pCtrl, pFRx) == BGNET_CHECKSUM_OK) {
                        switch(pFRx->CMD) {
                            case BGNET_FRAME_CMD_01_DATA:
                                // process DATA command (broadcast)
                                n = decode_command_DATA(pCtrl, pFRx);
                                log_info("  Command %02X (with payload) DATA, %d channels", pFRx->CMD, n);
                                break;
                            case BGNET_FRAME_CMD_02_LABEL:
                                // process LABEL command (only at startup, only by DATA)
                                decode_command_LABEL(pCtrl, pFRx);
                                log_info("  Command %02X (with payload) LABEL", pFRx->CMD);
                                break;
                            case BGNET_FRAME_CMD_06_SUBSCRIBE:
                                // process SUBSCRIBE command (only at startup, only by DATA)
                                decode_command_SUBSCRIBE(pCtrl, pFRx);
                                log_info("  Command %02X (with payload) SUBSCRIBE", pFRx->CMD);
                                break;
                            case BGNET_FRAME_CMD_C9_BACKLIGHT:
                                // process LIGHTS command (when you press the LIGHTS button)
                                // paylod: 1 byte (intensity 0, 3-1)
                                pCtrl->ACP.BackLight.value.nByte = pFRx->payload[0]; // first byte is the LIGHTS state
                                pCtrl->ACP.BackLight.source = BGN_SOURCE_BGNET;
                                log_info("  Command %02X (with payload) BACKLIGHT (BY: %02X, %s, LEVEL: %02X)", pFRx->CMD, pFRx->FROM, pCtrl->pDEVDT[x].sModelName, pFRx->payload[0]);
#define BGNET_ENABLE_RESET_LOG
#ifdef BGNET_ENABLE_RESET_LOG
                                // exprimental: create a "QUAD RESET" when press <LIGHTS> on NETDATA 
                                if (pFRx->FROM == BGNET_TYPE_NETDATA) {
                                    payload[0] = 0x00;
                                    payload[1] = BGN_CNR_CF_TRIPNM;
                                    len_pl = 2;
                                    pFTx = (FRAME_H*)&TxBuf[TxSize];
                                    len_msg = bgnet_encode_frame(pCtrl, pFTx, BGNET_TYPE_NETQUAD, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_CD_PILOT_DMPREQ, payload, len_pl);
                                                        
                                    memcpy(pCtrl->CMD[BGNET_FRAME_CMD_CD_PILOT_DMPREQ].Tx.ucFrame, pFTx, len_msg);
                                    CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_CD_PILOT_DMPREQ].CounterTx, t);
                                    memcpy(pCtrl->DEV[BGNET_ADDRESS_ME].Tx.ucFrame, pFTx, len_msg);
                                    CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                    TxSize =TxSize + len_msg;
                                    log_info("Experimental - QUAD RESET COMMAND: TRIPLOG");
                                    log_debug_frame(pFTx);
                                }
                                // exprimental: initialize ACP memory when press <LIGHTS> on NETWIND
                                if (pFRx->FROM == BGNET_TYPE_NETWIND) {
                                    bgnet_initialize_reset(pCtrl);
                                }
#endif
                                break;
                            default:
                                // Some commands are part of a CMD-REQ/CDM-DAT/CMD-ACK handshake. All next commands require a CMD-ACK response
                                // (and possible some other things too). The CMD-ACK is send last
                                // holy shit.... CMD/CHN is not unique within the network. The same combination can be found on PILOT and QUAD
                                // with different behaviour. I think a first make a new GIT commit before i screw up
                                // we switch for the DEVICE TYPE (not the actual FROM address)

                                // setup the pointer to our received CMD-REQ or CMD-DAT command
                                pCHRx = (union CH *)(&pFRx->payload[1]);
                                // split behaviour by device type
                                switch(pFRx->FROM & BGNET_TYPE_MASK) {
                                    case (BGNET_TYPE_NETQUAD & BGNET_TYPE_MASK):
                                        switch(pFRx->CMD) {
                                            // QUAD/RESET actions
                                            case BGNET_FRAME_CMD_CB_QUAD_RESET:
                                            case BGNET_FRAME_CMD_EA_QUAD_UNKNOWN:
                                                switch(pFRx->CMD) {
                                                    case BGNET_FRAME_CMD_CB_QUAD_RESET:
                                                        // RESET LOG command received (from QUAD)
                                                        // Must be a TYPEr, because a 'TYPEt' QUAD handles the TRIPLOG (and RESET) internally.
                                                        switch(pCHRx->s.CH) {
                                                            case BGN_CNR_CF_TRIPNM: // RESET LOG
                                                                pCtrl->ACP.TripLogNM.value.nLong = 0;
                                                                pCtrl->ACP.TripLogNM.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->ACP.TripLogNM.Counter, t);
                                                                break;
                                                            case BGN_CNR_81_DRDNM: // RESET DRD
                                                                // QUAD RESET DND comes here, i guess to reset the DRD
                                                                pCtrl->ACP.DRDNM.value.nLong = 0;
                                                                pCtrl->ACP.DRDNM.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->ACP.DRDNM.Counter, t);
                                                                break;
                                                        }
                                                        break;
                                                    case BGNET_FRAME_CMD_EA_QUAD_UNKNOWN:
                                                        // encountered at 4th SPEED/SETUP option on QUAD
                                                        switch(pCHRx->s.CH) {
                                                            case BGN_CNR_01_UNKNOWN:
                                                                break;                                                    
                                                        }
                                                        break;
                                                }
                                                break;
                                            default:
                                                // unhandled command, also no ACK required
                                                iIsValid = 0; // do not update device/command statistics
                                                x = bgnet_deviceindex(pCtrl, *(pBuf+BGNET_FRAME_HEAD_FROM)); // lookup by device address/type, so we can print the device name
                                                log_info("  Command %02X (with body) IGNORED (from %02X %s)", pFRx->CMD, pFRx->FROM, pCtrl->pDEVDT[x].sModelName);
                                                break;
                                        }
                                        break;
                                    case (BGNET_TYPE_NETPILOT & BGNET_TYPE_MASK):
                                        switch(pFRx->CMD) {

                                            // PILOT/SETUP actions
                                            case BGNET_FRAME_CMD_D0_PILOT_SPDREQ: // SPDDAT is carried via PILOT_AM
                                            case BGNET_FRAME_CMD_D1_PILOT_RESPREQ:
                                            case BGNET_FRAME_CMD_D5_PILOT_RESPDAT:
                                            case BGNET_FRAME_CMD_DE_PILOT_WAREQ:
                                            case BGNET_FRAME_CMD_21_PILOT_WADAT:
                                            case BGNET_FRAME_CMD_DF_PILOT_OCREQ:
                                            case BGNET_FRAME_CMD_20_PILOT_OCDAT:
                                            case BGNET_FRAME_CMD_CD_PILOT_DMPREQ:
                                            case BGNET_FRAME_CMD_CE_PILOT_DMPDAT:
                                            // FRREQ is carried via SPDREQ, FRDAT is carried via PILOT_MODE
                                            case BGNET_FRAME_CMD_CF_PILOT_MANREQ: // manual speed selection
                                            case BGNET_FRAME_CMD_D6_PILOT_SLWDAT:
                                            case BGNET_FRAME_CMD_D2_PILOT_MANREQ:

                                            // PILOT KEY actions
                                            case BGNET_FRAME_CMD_D3_PILOT_AM:
                                            case BGNET_FRAME_CMD_D4_PILOT_MODE:
                                            case BGNET_FRAME_CMD_DC_PILOT_STEPDAT:
                                            case BGNET_FRAME_CMD_B1_PILOT_DELTADAT:

                                                // Step 4 - handle the data from the received command.
                                                // CMD-REQ: 2 bytes payload -> send a requested CMD-DAT response + CMD-ACK response
                                                // CMD-DAT: 5 bytes payload -> handle the received channel + send a CMD-ACK response
                                                // Note: the CMD-ACK is allways generated (at the emd of the switch statement)
                                                // Note: when we must send data, do NOT use the DATA command (0x01), but use the corresponding
                                                // CMD-DAT command; i found out the hard way.
                                               // Note: i send the CMD-DAT and CMD-ACK message to the node that we received the CMD-REQ command from. It might be the
                                                // case that 1 (or both) must be send to ALL (0xFF) in case there are multiple PILOTS on the network. As i have only 1
                                                // PILOT unit i could not establish one or the other and choose to do it like this.
                                                switch(pFRx->CMD) {
                                                    case BGNET_FRAME_CMD_D3_PILOT_AM:
                                                        // the AUTO or OFF key was pressed. We receive 5 bytes payload.
                                                        // 0=ACKSEQ, 1=0xB5/AP_STATUS, 2=FMT_01, 3=0, 4= new state (1=AUTO , 0=STBY)
                                                        switch(pCHRx->s.CH) {
                                                            case BGN_CNR_B5_AP_STATUS:
                                                                // update the channel value
                                                                pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                                y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                                memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                                decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                                CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);
                                                                if (pCtrl->CHN[BGN_CNR_49_HEADING].x01.CounterRx.iCounter == 0) {
                                                                    // HEADING was never touched, do not go to AUTO
                                                                } else {
                                                                    // i'm not going to make things more complicated at this point. But realise that when
                                                                    // we have no accurate HEADING data, i do not know where to steer the boad when going to AUTO
                                                                }
                                                                // get the new STATE; initialize the CTARGET with current HEADING if we go into AUTO
                                                                if (pCtrl->ACP.STATE.value.nByte == BGN_AP_MODE1_STBY) { // current STATE is STBY
                                                                    if (pCHRx->s.cv.uB[1] == BGN_AP_MODE1_AUTO) { // new STATE is AUTO
                                                                        // it might be the case we do not have a recent HEADING channel received prior.
                                                                        // in such case we initialize to 0 (good luck!)
                                                                        // should we actually refuse to go into AUTO?
                                                                        pCtrl->ACP.CTS.value.nLong = pCtrl->ACP.Heading.value.nLong;
                                                                        pCtrl->ACP.CTS.source = BGN_SOURCE_BGNET;
                                                                        CounterUp(&pCtrl->ACP.CTS.Counter, t);
                                                                    }
                                                                }
                                                                // set new STATE (AUTO or STBY)
                                                                pCtrl->ACP.STATE.value.nByte = pCHRx->s.cv.uB[1];
                                                                pCtrl->ACP.STATE.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->ACP.STATE.Counter, t);

                                                                // make DATA command (to ALL, so also other PILOT units, if present, receive it)
                                                                len_pl = makeACPresponseSTATUSONLY(pCtrl, payload);
                                                                pFTx = (FRAME_H*)&TxBuf[TxSize];
                                                                len_msg = bgnet_encode_frame(pCtrl, pFTx, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_01_DATA, payload, len_pl);
                                                                register_DATA_frame(pCtrl, pFTx);
                                                        
                                                                memcpy(pCtrl->CMD[BGNET_FRAME_CMD_01_DATA].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_01_DATA].CounterTx, t);
                                                                memcpy(pCtrl->DEV[BGNET_ADDRESS_ME].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                                TxSize =TxSize + len_msg;
                                                                break;
                                                            // SPEED/MAN/FST entered speed value > ENTER
                                                            case BGN_CNR_46_AP_SPEED:
                                                                // update the channel value
                                                                pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                                y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                                memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                                decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                                CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);

                                                                // get the new SPEED SELECT from the channel
                                                                pCtrl->ACP.MANSPEED_FST.value.nLong = ntohs(pCHRx->s.cv.uW[0]);
                                                                pCtrl->ACP.MANSPEED_FST.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->ACP.MANSPEED_FST.Counter, t);
                                                                break;
                                                        }
                                                        break;                                               
                                                    case BGNET_FRAME_CMD_D6_PILOT_SLWDAT:
                                                        // the MAN/SLW speed value was entered. We receive 5 bytes payload.
                                                        // 0=ACKSEQ, 1=0x46, 2=FMT_01, 3/4= new value (0-990)
                                                        switch(pCHRx->s.CH) {
                                                            // SPEED/MAN/SLW entered speed value > ENTER
                                                            case BGN_CNR_46_AP_SPEED:
                                                                // update the channel value
                                                                pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                                y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                                memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                                decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                                CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);

                                                                // get the new SPEED SELECT from the channel
                                                                pCtrl->ACP.MANSPEED_SLW.value.nLong = ntohs(pCHRx->s.cv.uW[0]);
                                                                pCtrl->ACP.MANSPEED_SLW.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->ACP.MANSPEED_SLW.Counter, t);
                                                                break;
                                                        }
                                                        break;
                                                    case BGNET_FRAME_CMD_D4_PILOT_MODE:
                                                        // the MODE key was pressed, 5 bytes payload:
                                                        // 0=ACKSEQ, 1=0xA6/AP_CTARGET, 2=FMT_01, 3=0, 4= new state (1=COMP, 3=NAV, 4=VANE)
                                                        // the SPEED selector is made (SPEED key, but also PILOT/SETUP/SPD selection)
                                                        // 0=ACKSEQ, 1=0x46/AP_SPEED, 2=FMT_01, 3=0, 4= new SPEED mode (0=TRA, 1=MAN, 2=SOG)
                                                        // Note: MANUAL speed selection can be set at 3 bands (SLW, MED, FST) via the PILOT/SPEED key
                                                        switch(pCHRx->s.CH) {
                                                            // MODE SELECT (COMP/NAV/VANE)
                                                            case BGN_CNR_A6_AP_CTARGET:
                                                                // update the channel value
                                                                pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                                y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                                memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                                decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                                CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);

                                                                // get the new MODE; do not set mode if we are in JOYSTCK
                                                                if (pCtrl->ACP.Joystick.value.nByte == BGN_JOYSTICK_OFF) {
                                                                    pCtrl->ACP.MODE.value.nByte = pCHRx->s.cv.uB[1];
                                                                    pCtrl->ACP.MODE.source = BGN_SOURCE_ACP;
                                                                    CounterUp(&pCtrl->ACP.MODE.Counter, t);
                                                                } else {
                                                                    // the JOYSTCK mode on a real system is via a physical switch wired to the ACP
                                                                    // we can not emulate this.
                                                                    // Going IN or OUT of JOYSTCK is not someting we can emulate. I keep it simple:
                                                                    // JOYSTCK mode is always OFF (unless someone comes along with a great idea)
                                                                }

                                                                // make DATA command (to ALL, so also other PILOT units, if present, receive it)
                                                                len_pl = makeACPresponseSTATUSONLY(pCtrl, payload);
                                                                pFTx = (FRAME_H*)&TxBuf[TxSize];
                                                                len_msg = bgnet_encode_frame(pCtrl, pFTx, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_01_DATA, payload, len_pl);
                                                                register_DATA_frame(pCtrl, pFTx);

                                                                memcpy(pCtrl->CMD[BGNET_FRAME_CMD_01_DATA].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_01_DATA].CounterTx, t);
                                                                memcpy(pCtrl->DEV[BGNET_ADDRESS_ME].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                                TxSize = TxSize + len_msg;
                                                                break;
                                                            // SETUP > SPEED SELECT (TRA/MAN/SOG) > ENTER
                                                            case BGN_CNR_46_AP_SPEED:
                                                                // update the channel value
                                                                pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                                y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                                memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                                decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                                CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);

                                                                // get the new SPEED SELECT from the channel
                                                                pCtrl->ACP.SETUP_SPD.value.nByte = pCHRx->s.cv.uB[1];
                                                                pCtrl->ACP.SETUP_SPD.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->ACP.SETUP_SPD.Counter, t);
                                                                break;
                                                            // SETUP > FR SELECT (ON/OFF) > ENTER
                                                            case BGN_CNR_07_AP_FR:
                                                                // update the channel value
                                                                pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                                y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                                memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                                decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                                CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);

                                                                // get the new FA SELECT from the channel
                                                                pCtrl->ACP.SETUP_FR.value.nByte = pCHRx->s.cv.uB[1];
                                                                pCtrl->ACP.SETUP_FR.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->ACP.SETUP_FR.Counter, t);
                                                                break;
                                                        }
                                                        break;
                                                    case BGNET_FRAME_CMD_D0_PILOT_SPDREQ:
                                                        // the SPEED key was pressed, 2 bytes payload.
                                                        // 0=ACKSEQ, 1=0x46 -> retunr current selected SPD source
                                                        // in SETUP, FR option is requested, 2 bytes payload
                                                        // 0=ACKSEQ, 1=0x07 -> return current select FR state (1=OFF, 0=ON)
                                                        switch(pCHRx->s.CH) {
                                                            case BGN_CNR_46_AP_SPEED:
                                                                // create command with current SETUP/SPD setting (0=TRA, 1=MAN, 2=SOG)
                                                                payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                                pCHTx = (union CH*)&payload[1];
                                                                pCHTx->s.CH = BGN_CNR_46_AP_SPEED;
                                                                pCHTx->s.FMT = BGN_FMT_01;
                                                                pCHTx->s.cv.uB[0] = 0;
                                                                pCHTx->s.cv.uB[1] = pCtrl->ACP.SETUP_SPD.value.nByte;
                                                                len_pl = 5;
                                                                pFTx = (FRAME_H*)&TxBuf[TxSize];
                                                                len_msg = bgnet_encode_frame(pCtrl, pFTx, pFRx->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_D4_PILOT_MODE, payload, len_pl);

                                                                memcpy(pCtrl->CMD[BGNET_FRAME_CMD_D4_PILOT_MODE].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_D4_PILOT_MODE].CounterTx, t);
                                                                memcpy(pCtrl->DEV[BGNET_ADDRESS_ME].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                                TxSize = TxSize + len_msg;
                                                                break;
                                                            case BGN_CNR_07_AP_FR:
                                                                // create command with current FR SELECT
                                                                payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                                payload[1] = BGN_CNR_07_AP_FR;
                                                                payload[2] = BGN_FMT_08;
                                                                payload[3] = 0;
                                                                payload[4] = pCtrl->ACP.SETUP_FR.value.nByte;
                                                                len_pl = 5;
                                                                pFTx = (FRAME_H*)&TxBuf[TxSize];
                                                                len_msg = bgnet_encode_frame(pCtrl, pFTx, pFRx->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_D4_PILOT_MODE, payload, len_pl);

                                                                memcpy(pCtrl->CMD[BGNET_FRAME_CMD_D4_PILOT_MODE].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_D4_PILOT_MODE].CounterTx, t);
                                                                memcpy(pCtrl->DEV[BGNET_ADDRESS_ME].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                                TxSize = TxSize + len_msg;
                                                                break;
                                                        }
                                                        break;
                                                    case BGNET_FRAME_CMD_CF_PILOT_MANREQ:
                                                        // in SPEED/SETUP/MAN, the FST speed value is requested, 2 bytes payload
                                                        // 0=ACKSEQ, 1=0x46
                                                        switch(pCHRx->s.CH) {
                                                            case BGN_CNR_41_BOATSPEEDKT: 
                                                                // QUAD RESET DND comes here, 2 bytes payload reset SpeedMAX/AVG
                                                                pCtrl->ACP.SpeedMAX.value.nLong = pCtrl->ACP.Speed.value.nLong;
                                                                pCtrl->ACP.SpeedMAX.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->ACP.SpeedMAX.Counter, t);
                                                                break;
                                                            case BGN_CNR_46_AP_SPEED:
                                                                // SEND SPD MAN/FST
                                                                payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                                pCHTx = (union CH*)&payload[1];
                                                                pCHTx->s.CH = BGN_CNR_46_AP_SPEED;
                                                                pCHTx->s.FMT = BGN_FMT_01;
                                                                pCHTx->s.cv.uW[0] = htons(pCtrl->ACP.MANSPEED_FST.value.nLong);
                                                                len_pl = 5;
                                                                pFTx = (FRAME_H*)&TxBuf[TxSize];
                                                                len_msg = bgnet_encode_frame(pCtrl, pFTx, pFRx->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_D3_PILOT_AM, payload, len_pl);

                                                                memcpy(pCtrl->CMD[BGNET_FRAME_CMD_D3_PILOT_AM].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_D3_PILOT_AM].CounterTx, t);
                                                                memcpy(pCtrl->DEV[BGNET_ADDRESS_ME].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                                TxSize = TxSize + len_msg;
                                                                break;
                                                        }
                                                        break;
                                                    case BGNET_FRAME_CMD_D1_PILOT_RESPREQ:
                                                        // in SETUP, the RESPONSE parameter is configured, 2 bytes payload.
                                                        // 0=ACKSEQ, 1=0xA6
                                                        switch(pCHRx->s.CH) {
                                                            case BGN_CNR_A6_AP_CTARGET:
                                                                // create command with current RESP SELECT
                                                                payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                                payload[1] = BGN_CNR_A6_AP_CTARGET;
                                                                payload[2] = BGN_FMT_01;
                                                                payload[3] = 0;
                                                                payload[4] = pCtrl->ACP.SETUP_RESP.value.nByte;
                                                                len_pl = 5;
                                                                pFTx = (FRAME_H*)&TxBuf[TxSize];
                                                                len_msg = bgnet_encode_frame(pCtrl, pFTx, pFRx->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_D5_PILOT_RESPDAT, payload, len_pl);

                                                                memcpy(pCtrl->CMD[BGNET_FRAME_CMD_D5_PILOT_RESPDAT].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_D5_PILOT_RESPDAT].CounterTx, t);
                                                                memcpy(pCtrl->DEV[BGNET_ADDRESS_ME].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                                TxSize = TxSize + len_msg;
                                                                break;
                                                            case BGN_CNR_46_AP_SPEED:
                                                                // SEND SPD MAN/MED
                                                                payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                                pCHTx = (union CH*)&payload[1];
                                                                pCHTx->s.CH = BGN_CNR_46_AP_SPEED;
                                                                pCHTx->s.FMT = BGN_FMT_01;
                                                                pCHTx->s.cv.uW[0] = htons(pCtrl->ACP.MANSPEED_MED.value.nLong);
                                                                len_pl = 5;
                                                                pFTx = (FRAME_H*)&TxBuf[TxSize];
                                                                len_msg = bgnet_encode_frame(pCtrl, pFTx, pFRx->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_D5_PILOT_RESPDAT, payload, len_pl);

                                                                memcpy(pCtrl->CMD[BGNET_FRAME_CMD_D5_PILOT_RESPDAT].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_D5_PILOT_RESPDAT].CounterTx, t);
                                                                memcpy(pCtrl->DEV[BGNET_ADDRESS_ME].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                                TxSize = TxSize + len_msg;
                                                                break;
                                                        }
                                                        break;
                                                    case BGNET_FRAME_CMD_D2_PILOT_MANREQ:
                                                        // when MAN in SPD setup via SETUP, press the SPEED key, 2 bytes payload.
                                                        // 0=ACKSEQ, 1=0x46
                                                        switch(pCHRx->s.CH) {
                                                            case BGN_CNR_46_AP_SPEED:
                                                                // SEND SPD MAN/SLW
                                                                payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                                pCHTx = (union CH*)&payload[1];
                                                                pCHTx->s.CH = BGN_CNR_46_AP_SPEED;
                                                                pCHTx->s.FMT = BGN_FMT_01;
                                                                pCHTx->s.cv.uW[0] = htons(pCtrl->ACP.MANSPEED_SLW.value.nLong);
                                                                len_pl = 5;
                                                                pFTx = (FRAME_H*)&TxBuf[TxSize];
                                                                len_msg = bgnet_encode_frame(pCtrl, pFTx, pFRx->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_D6_PILOT_SLWDAT, payload, len_pl);

                                                                memcpy(pCtrl->CMD[BGNET_FRAME_CMD_D6_PILOT_SLWDAT].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_D6_PILOT_SLWDAT].CounterTx, t);
                                                                memcpy(pCtrl->DEV[BGNET_ADDRESS_ME].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                                TxSize = TxSize + len_msg;
                                                                break;
                                                        }
                                                        break;
                                                    case BGNET_FRAME_CMD_D5_PILOT_RESPDAT:
                                                        // in SETUP, the RESPONSE parameter is configured, 5 bytes payload.
                                                        // 0=ACKSEQ, 1=0xA6/AP_CTARGET, 2=FMT_01, 3=0, 4= new RESP selector
                                                        switch(pCHRx->s.CH) {
                                                            case BGN_CNR_A6_AP_CTARGET:
                                                                // update the channel value
                                                                pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                                y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                                memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                                decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                                CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);

                                                                pCtrl->ACP.SETUP_RESP.value.nByte = pCHRx->s.cv.uB[1];
                                                                pCtrl->ACP.SETUP_RESP.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->ACP.SETUP_RESP.Counter, t);
                                                                break;
                                                            // SPEED/MAN/MED entered speed value > ENTER
                                                            case BGN_CNR_46_AP_SPEED:
                                                                // update the channel value
                                                                pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                                y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                                memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                                decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                                CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);

                                                                // get the new SPEED SELECT from the channel
                                                                pCtrl->ACP.MANSPEED_MED.value.nLong = ntohs(pCHRx->s.cv.uW[0]);
                                                                pCtrl->ACP.MANSPEED_MED.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->ACP.MANSPEED_MED.Counter, t);
                                                                break;
                                                            // broadcast by PILOT to FF, 3 bytes payload, unknown 
                                                            // FF 50 03 D5 D9 B5 EB 0F FB
                                                            // 0xEB could be a channel ID
                                                            // 0x0F could be a format specifier to switch on or off something (we have seen a similar
                                                            //      behaviour with the WA and OC alarms ON/OFF), but we have  never encountered a FMT_07 before
                                                            // Because it's a broadcast, it should be relevant for all NETWORK attached units.
                                                            // For the time being, i just log it.  
                                                            case BGN_CNR_EB_SOGKT:
                                                                // update the channel value
                                                                pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                                y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                                memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                                decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                                CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);
                                                                break;
                                                        }
                                                        break;
                                                    case BGNET_FRAME_CMD_DE_PILOT_WAREQ:
                                                        // in PILOT/SETUP, the WATCH ALARM is configured, 2 bytes payload
                                                        // 0=ACKSEQ, 1=0x76
                                                        switch(pCHRx->s.CH) {
                                                            case BGN_CNR_76_AP_WA_ALARM:
                                                                // create command with current WA SELECT
                                                                payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                                pCHTx = (union CH*)&payload[1];
                                                                pCHTx->s.CH = BGN_CNR_76_AP_WA_ALARM;
                                                                if (pCtrl->ACP.SETUP_WA_STAT.value.nLong == 0) {
                                                                    pCHTx->s.FMT = BGN_FMT_08;
                                                                } else {
                                                                    pCHTx->s.FMT = BGN_FMT_18;
                                                                }
                                                                pCHTx->s.cv.uW[0] = htons(pCtrl->ACP.SETUP_WA_MIN.value.nLong);
                                                                len_pl = 5;
                                                                pFTx = (FRAME_H*)&TxBuf[TxSize];
                                                                len_msg = bgnet_encode_frame(pCtrl, pFTx, pFRx->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_21_PILOT_WADAT, payload, len_pl);
                                                        
                                                                memcpy(pCtrl->CMD[BGNET_FRAME_CMD_21_PILOT_WADAT].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_21_PILOT_WADAT].CounterTx, t);
                                                                memcpy(pCtrl->DEV[BGNET_ADDRESS_ME].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                                TxSize = TxSize + len_msg;
                                                                break;
                                                        }
                                                        break;
                                                    case BGNET_FRAME_CMD_21_PILOT_WADAT:
                                                        // in PILOT/SETUP, when WATCH ALARM setting is entered, 5 bytes payload
                                                        // 0=ACKSEQ, 1=0x76, 2=FMT_01, 3/4=WA limit in minutes (1-360)
                                                        switch(pCHRx->s.CH) {
                                                            case BGN_CNR_76_AP_WA_ALARM:
                                                                // update the channel value last, so we can compare if anything changed
                                                                pCHTx = (union CH*)pCtrl->CHN[pCHRx->s.CH].x00.ucRAW; // we abuse the pCHTx pointer
                                                                if (pCHTx->s.FMT == pCHRx->s.FMT) {
                                                                    // no change
                                                                } else {
                                                                   if (pCHRx->s.FMT == BGN_FMT_08) {
                                                                        pCtrl->ACP.SETUP_WA_STAT.value.nLong = 0;
                                                                    } else {
                                                                        pCtrl->ACP.SETUP_WA_STAT.value.nLong = 1;
                                                                    }
                                                                    pCtrl->ACP.SETUP_WA_STAT.source = BGN_SOURCE_BGNET;
                                                                    CounterUp(&pCtrl->ACP.SETUP_WA_STAT.Counter, t);
                                                                }
                                                                if (pCHTx->s.cv.uW[0] == pCHRx->s.cv.uW[0]) {
                                                                    // no change
                                                                } else {
                                                                    pCtrl->ACP.SETUP_WA_MIN.value.nLong = ntohs(pCHRx->s.cv.uW[0]);
                                                                    pCtrl->ACP.SETUP_WA_MIN.source = BGN_SOURCE_BGNET;
                                                                    CounterUp(&pCtrl->ACP.SETUP_WA_MIN.Counter, t);
                                                                }
                                                                // update the channel value (if something changed)
                                                                if ((pCHTx->s.FMT == pCHRx->s.FMT) & (pCHTx->s.cv.uW[0] == pCHRx->s.cv.uW[0])) {
                                                                    // no change                                                        
                                                                } else {
                                                                    pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                                    pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                                    pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                                    pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                                    y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                                    memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                                    decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                                    CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);
                                                                }
                                                                break;
                                                        }
                                                        break;
                                                    case BGNET_FRAME_CMD_DF_PILOT_OCREQ:
                                                        // in PILOT/SETUP, the OFF-COURSE ALARM is configured, 2 bytes payload.
                                                        // 0=ACKSEQ, 1=0xAF
                                                        switch(pCHRx->s.CH) {
                                                            case BGN_CNR_AF_AP_OC_ALARM:
                                                                // create command with current OC SELECT
                                                                payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                                pCHTx = (union CH*)&payload[1];
                                                                pCHTx->s.CH = BGN_CNR_AF_AP_OC_ALARM;
                                                                if (pCtrl->ACP.SETUP_OC_STAT.value.nLong == 0) {
                                                                    pCHTx->s.FMT = BGN_FMT_08;
                                                                } else {
                                                                    pCHTx->s.FMT = BGN_FMT_18;
                                                                }
                                                                pCHTx->s.cv.uW[0] = htons(pCtrl->ACP.SETUP_OC_DEG.value.nLong);
                                                                len_pl = 5;
                                                                pFTx = (FRAME_H*)&TxBuf[TxSize];
                                                                len_msg = bgnet_encode_frame(pCtrl, pFTx, pFRx->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_20_PILOT_OCDAT, payload, len_pl);
                                                        
                                                                memcpy(pCtrl->CMD[BGNET_FRAME_CMD_20_PILOT_OCDAT].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_20_PILOT_OCDAT].CounterTx, t);
                                                                memcpy(pCtrl->DEV[BGNET_ADDRESS_ME].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                                TxSize = TxSize + len_msg;
                                                                break;
                                                        }
                                                        break;
                                                    case BGNET_FRAME_CMD_20_PILOT_OCDAT:
                                                        // in PILOT/SETUP, when OFF-COURSE ALARM setting is entered, 5 bytes payload
                                                        // 0=ACKSEQ, 1=0x76, 2=FMT_01, 3/4=OC limit in degrees (1-99)
                                                        switch(pCHRx->s.CH) {
                                                            case BGN_CNR_AF_AP_OC_ALARM:
                                                                // update the channel value last, so we can compare if anything changed
                                                                pCHTx = (union CH*)pCtrl->CHN[pCHRx->s.CH].x00.ucRAW; // we abuse the pCHTx pointer
                                                                if (pCHTx->s.FMT == pCHRx->s.FMT) {
                                                                    // no change
                                                                } else {
                                                                   if (pCHRx->s.FMT == BGN_FMT_08) {
                                                                        pCtrl->ACP.SETUP_OC_STAT.value.nLong = 0;
                                                                    } else {
                                                                        pCtrl->ACP.SETUP_OC_STAT.value.nLong = 1;
                                                                    }
                                                                    pCtrl->ACP.SETUP_OC_STAT.source = BGN_SOURCE_BGNET;
                                                                    CounterUp(&pCtrl->ACP.SETUP_OC_STAT.Counter, t);
                                                                }
                                                                if (pCHTx->s.cv.uW[0] == pCHRx->s.cv.uW[0]) {
                                                                    // no change
                                                                } else {
                                                                    pCtrl->ACP.SETUP_OC_DEG.value.nLong = ntohs(pCHRx->s.cv.uW[0]);
                                                                    pCtrl->ACP.SETUP_OC_DEG.source = BGN_SOURCE_BGNET;
                                                                    CounterUp(&pCtrl->ACP.SETUP_OC_DEG.Counter, t);
                                                                }
                                                                // update the channel value (if something changed)
                                                                if ((pCHTx->s.FMT == pCHRx->s.FMT) & (pCHTx->s.cv.uW[0] == pCHRx->s.cv.uW[0])) {
                                                                    // no change                                                        
                                                                } else {
                                                                    pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                                    pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                                    pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                                    pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                                    y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                                    memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                                    decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                                    CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);
                                                                }
                                                                break;
                                                        }
                                                        break;
                                                    case BGNET_FRAME_CMD_B1_PILOT_DELTADAT:
                                                        // 1 of the 4 +/- keys was pressed, 5 bytes payload: 0-ACKSEQ, 1=0xA6/CHID, 2=0x01/FMT, 3/4=signed 16bits inc/decr value
                                                        switch(pCHRx->s.CH) {
                                                            case BGN_CNR_A6_AP_CTARGET:
                                                                iValue = decode_channel_numeric(pCHRx);
                                                                pCtrl->ACP.CTS.value.nLong = pCtrl->ACP.CTS.value.nLong + iValue;
                                                                pCtrl->ACP.CTS.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->ACP.CTS.Counter, t);

                                                                // create command with current CTARGET setting
                                                                payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                                pCHTx = (union CH*)&payload[1];
                                                                pCHTx->s.CH = BGN_CNR_A6_AP_CTARGET;
                                                                pCHTx->s.FMT = BGN_FMT_01;
                                                                pCHTx->s.cv.uW[0] = htons(pCtrl->ACP.CTS.value.nLong);
                                                                len_pl = 5;
                                                                pFTx = (FRAME_H*)&TxBuf[TxSize];
                                                                len_msg = bgnet_encode_frame(pCtrl, pFTx, pFRx->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_01_DATA, payload, len_pl);
                                                                register_DATA_frame(pCtrl, pFTx);

                                                                memcpy(pCtrl->CMD[BGNET_FRAME_CMD_01_DATA].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_01_DATA].CounterTx, t);
                                                                memcpy(pCtrl->DEV[BGNET_ADDRESS_ME].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                                TxSize =TxSize + len_msg;
                                                                break;
                                                        }
                                                        break;
                                                    case BGNET_FRAME_CMD_CD_PILOT_DMPREQ:
                                                        // in PILOT/SETUP, the COMPASS DMP is configured, 2 bytes payload
                                                        // 0=ACKSEQ, 1=0x49
                                                        switch(pCHRx->s.CH) {
                                                            case BGN_CNR_41_BOATSPEEDKT: 
                                                                // QUAD RESET DND comes here, 2 bytes payload reset SpeedMAX/AVG
                                                                pCtrl->ACP.SpeedMAX.value.nLong = pCtrl->ACP.Speed.value.nLong;
                                                                pCtrl->ACP.SpeedMAX.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->ACP.SpeedMAX.Counter, t);
                                                                break;
                                                            case BGN_CNR_49_HEADING:
                                                                // create command with current DMP setting
                                                                payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                                payload[1] = BGN_CNR_49_HEADING;
                                                                payload[2] = BGN_FMT_01;
                                                                payload[3] = 0;
                                                                payload[4] = pCtrl->ACP.SETUP_DMP.value.nByte; // (0-99)                                                        len_pl = 5;
                                                                len_pl = 5;
                                                                pFTx = (FRAME_H*)&TxBuf[TxSize];
                                                                len_msg = bgnet_encode_frame(pCtrl, pFTx, pFRx->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_CE_PILOT_DMPDAT, payload, len_pl);

                                                                memcpy(pCtrl->CMD[BGNET_FRAME_CMD_CE_PILOT_DMPDAT].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_CE_PILOT_DMPDAT].CounterTx, t);
                                                                memcpy(pCtrl->DEV[BGNET_ADDRESS_ME].Tx.ucFrame, pFTx, len_msg);
                                                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                                TxSize = TxSize + len_msg;
                                                                break;
                                                        }
                                                        break;
                                                    case BGNET_FRAME_CMD_CE_PILOT_DMPDAT:
                                                        // in PILOT/SETUP, when COMPASS DMP setting is entered, 5 bytes payload
                                                        // 0=ACKSEQ, 1=0x49, 2=FMT_01, 3=0, 4=DMP in seconds (1-99, 4=default)
                                                        switch(pCHRx->s.CH) {
                                                            case BGN_CNR_49_HEADING:
                                                                // update the channel value 
                                                                pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                                pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                                y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                                memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                                decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                                CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);

                                                                pCtrl->ACP.SETUP_DMP.value.nByte = (pCHRx->s.cv.uB[1]);
                                                                pCtrl->ACP.SETUP_DMP.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->ACP.SETUP_DMP.Counter, t);
                                                                break;
                                                        }
                                                        break;
                                                }

                                                payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                len_pl = 1;
                                                pFTx = (FRAME_H*)&TxBuf[TxSize];
                                                len_msg = bgnet_encode_frame(pCtrl, pFTx, pFRx->FROM, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_ACK, payload, len_pl);
                                                memcpy(pCtrl->CMD[BGNET_FRAME_CMD_ACK].Tx.ucFrame, pFTx, len_msg);
                                                CounterUp(&pCtrl->CMD[BGNET_FRAME_CMD_ACK].CounterTx, t);
                                                memcpy(pCtrl->DEV[BGNET_ADDRESS_ME].Tx.ucFrame, pFTx, len_msg);
                                                CounterUp(&pCtrl->DEV[BGNET_ADDRESS_ME].CounterTx, t);
                                                TxSize = TxSize + len_msg;

                                                //log_debug_buffer(TxBuf, TxSize);

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
                                                log_info("  Command %02X (with body) IGNORED (from %02X %s)", pFRx->CMD, pFRx->FROM, pCtrl->pDEVDT[x].sModelName);
                                                break;
                                        }
                                        break;
                                }
                                
                        }
                        // we only want to register our incoming command if we know what it is.
                        if (iIsValid > 0) {
                            // update CMD structure
                            pCtrl->iNewCommand = BGN_NEW_SET; // signal possible CMD change
                            pCtrl->CMD[pFRx->CMD].iNewCommand = BGN_NEW_SET; // signal this command
                            memcpy(pCtrl->CMD[pFRx->CMD].Rx.ucFrame, pFRx, RxSize);
                            CounterUp(&pCtrl->CMD[pFRx->CMD].CounterRx, t);
                        }   
                    } else {
                        // payload CS is bad, only skip this sub-frame, the length is known. We will continue other subframes tha are in the
                        // the same receive buffer
                        // The error is logged by the routine checking
                        pCtrl->DEV[pFRx->FROM].iCSError_Frame++;
                        log_debug_frame(pFRx);
                    }
                }
            }
        } else {
            // header CS is bad, we can not process this subframe, we can not establish a subframe length unambigiously, so we dump the whole buffer
            // The error is logged by the routine checking
            pCtrl->DEV[pFRx->FROM].iCSError_Header++;
            log_debug_frame(pFRx);
            // this usually happens when we startup against a running network. we receive an incomplete frame.
            // Can also (sporadically) happen during normal service. Most likely the next time the message is received it's OK.
            // Note: realise because the CS is bad, also the FROM address could be in error. This causes the counter to be mis-placed (with the wrong Node)
            // and not being displayed/reported at all. Shit happens.
            // If that appears to be a problem, we could report a SUM of all the Header-CS errors and report it in the webpage (but is see no reason yet)
            break;  // exit the loop, dump whole buffer 
        }
        pBuf = pBuf + RxSize;
        z = z + 1;
    }
    return(z);
}

#endif
