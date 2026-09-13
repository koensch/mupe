#ifndef BGNPROTOSUB_H
#define BGNPROTOSUB_H

#include <stdio.h>
#include <time.h>
#include <limits.h>
#include <string.h>
#include <stdlib.h>
#include <arpa/inet.h>

#include "app_state.h"
#include "config.h"
#include "logger.h"
#include "bgnprotocol.h"

#define BGNET_COMMANDTYPE_REQ   1
#define BGNET_COMMANDTYPE_DTA   2
#define BGNET_COMMANDTYPE_ACK   3
#define BGNET_COMMANDTYPE_NORM  0
unsigned char bgnet_command_type(FRAME_H *pF) {
    unsigned char r;
    switch(pF->CMD) {
        case BGNET_CMD_CA_QUAD_TIMER:
        case BGNET_CMD_CB_QUAD_RESET:
        case BGNET_CMD_CD_PILOT_DMPREQ:
        case BGNET_CMD_CF_PILOT_MANREQ:
        case BGNET_CMD_D0_PILOT_SPDREQ:
        case BGNET_CMD_D1_PILOT_RESPREQ:
        case BGNET_CMD_D2_PILOT_MANREQ:
        case BGNET_CMD_D3_PILOT_AM:
        case BGNET_CMD_D4_PILOT_MODE:
        case BGNET_CMD_DE_PILOT_WAREQ:
        case BGNET_CMD_DD_QUAD_SHAREQ:
        case BGNET_CMD_DF_PILOT_OCREQ:
        case BGNET_CMD_EA_QUAD_ACAL_REQ:
            r = BGNET_COMMANDTYPE_REQ;
            break;
        case BGNET_CMD_20_PILOT_OCDAT:
        case BGNET_CMD_21_PILOT_WADAT:
        case BGNET_CMD_22_DEPTH_DADAT:
        case BGNET_CMD_B1_PILOT_DELTADAT:
        case BGNET_CMD_CE_PILOT_DMPDAT:
        case BGNET_CMD_D5_PILOT_RESPDAT:
        case BGNET_CMD_E9_QUAD_ACAL_DAT:
            r = BGNET_COMMANDTYPE_DTA;
            break;
        case BGNET_CMD_ACK:
            r = BGNET_COMMANDTYPE_ACK;
            break;
        default:
            r = BGNET_COMMANDTYPE_NORM;
            break;
    }
    return(r);
}

#define BGNET_FRAME_ACKSEQ  0
// There is some handshaking done between ACP and NETWORK nodes. These handshare commands have a 'Acknowledge Sequence Number' in their payload (byte 0).
// When a 'Request Command' is send, the addressed node replies with a 'Data Command' and a 'Acknowledge Command'. If the 'Data Command' and 'Ack Command'
// returns the correct sequence number, the requesting node knows that these reply belong to what request.
// Usually the sequence number is incremented after use. Numbers should be in sequence.
// But....... when the ACKSEQ is a multple of 13 (0x0D) we see weird things:
// the 0A phenomenon: the next CMD-REQ/CMD-DAT/CMD-ACK handshake after 0x0C, we expect the ACKSEQ to be 0x0D, but we get 0x0A, causing a CS error. But if we
//                    correct the CS-SEED, the PILOT does not accept our CMD-DAT reply. If we replace/patch the 0x0A with 0x0D in the received CMD-REQ, the
//                    receiving frame's CS is OK and when we send a CMD-DAT with the ACKSEQ with the 0x0D, every thing seems fine.
// the 13 phenomenon: We still observer a CS error when the ACKSEQ we receive is a multiple of 13 (0x0D). It was obverved with 0x27 (3x13=39=0x27) and with
//                    0xEE (12x13=238=0xEE).
//                    My guess is that the "0A phenomenon" is an instance of the "13 phenomenon" in disquise
// If this behavour is intended (to confuse reverse-engineering), a bug or a work-around for some other phenomenon, i don't know.
// I have considered to skip the CS checking all together for every CMD-REQ/CMD-DAT/CMD-ACK sequence that follows a multiple of 12, but decided not to do so. This
// allows me to investigate CS errors and come up with a 100% solution
//
// what this routine does, is under certain conditions, replace a ACKSEQ (that is 0x0A) with a value of 0x0D. This seems to correct the phenomenon. Don't ask
// me why this 'feature' exists........ It is what it is.
unsigned char bgnet_weird_seed(BGNET_CTRL *pCtrl, FRAME_H *pF) {
    // this phenomenon is observed with PILOT and QUAD modules
    switch (pF->FROM & BGNET_TYPE_MASK) {
        case BGNET_TYPE_NETPILOT:
        case BGNET_TYPE_NETQUAD:
            switch (bgnet_command_type(pF)) { // put this in a seperate routine to avoid too much distraction here
                // we observe this behaviour only with CMD-REQ and CMD-DTA commands
                case BGNET_COMMANDTYPE_REQ:
                case BGNET_COMMANDTYPE_DTA:
                    if (pCtrl->BGNET.weird.ACKSEQ > pF->payload[BGNET_FRAME_ACKSEQ]) {
                        // the current received ACKSEQ is LESS than the previous one. It's of sequence, this is weird
                        log_debug("ACKSEQ out-of-sequence: 0x%02X (received) vs 0x%02X (stored)", pF->payload[BGNET_FRAME_ACKSEQ], pCtrl->BGNET.weird.ACKSEQ);
                        // keep the default SEED, but patch the ACKSEQ to the correct value, this will result into a matching CS (received vs calculated)
                        switch (pF->payload[BGNET_FRAME_ACKSEQ]) {
                            case 0x0A:
                                pF->payload[BGNET_FRAME_ACKSEQ] = 0x0D;  // correct ACKSEQ, makes calculated CS correct against the received CS
                                break;
                        }
                        // NOTE: we have observed that phenomenon with ACKSEQ with a multiple of 0x0D. So far just 'patching' the first occurance of 0x0A
                        //       seems suffictient. If we need to patch multiples of 0x0D's, we'll find out. So far, so good.
                    } else {
                        // the current received ACKSEQ is MORE (or equal) to the previous one. This looks normal behaviour.
                        // because it's a unsigned number, we can not detect a roll-over (0xFF to 0x00), so we do that as we see a ACKSEQ of 0xFF
                        if (pF->payload[BGNET_FRAME_ACKSEQ] == 0xFF) {
                            // roll-over when at 0xFF, it's just a single, unsiged byte
                            pCtrl->BGNET.weird.ACKSEQ = 0x00;
                        } else {
                            // shift the current ACKSEQ to the previous one (for next cycle)
                            pCtrl->BGNET.weird.ACKSEQprev = pCtrl->BGNET.weird.ACKSEQ;
                            pCtrl->BGNET.weird.ACKSEQ = pF->payload[BGNET_FRAME_ACKSEQ];
                        }
                    }
                    // check the CS byte. it seems that when 0x0D is the calculated CS, it's replaced it with 0x0A, causing again a CS error at our receiving end
                    // so, when we receive a 0x0A checksum, we put back the 0x0D.
                    switch(pF->payload[pF->PL]) {
                        case 0x0A:
                            pF->payload[pF->PL] = 0x0D;
                            break;
                    }
                    break;
            }
            break;
    }
    return(BGNET_FRAME_CR_FRAME);
}

// select the effective speed based on the speedselector and manual FST/MED/SLW selection
// this is my own invention......
// The reason is that i expect that this behaviour is taken care of by the SPEED or QUAD node
// when in 'normal service' (ie in TRAnsmitter mode). But we do (most likely) not deal with
// a SPEEDt/QUADt (who would still have a 40 years old transmitter in it's hull????). Most likely
// we are dealing with a SPEEDr/QUADr so we need to come up with some logic to distribute the
// signals we have to the repeater displays (to mimic the SPEEDt/QUADt behaviour)
// That is why i invented this logic.
// - when we receive the SPEED/0x41 channel (from the NETWORK), we'll happy using it
// - when we do not receive a SPEED/0x41 channel (and we have to send it to the NETWORK), we use the
//   EBS for distribution.
uint16_t EffectiveBoatSpeed(BGNET_CTRL *pCtrl, NMEA2K_VALUE *pV) {
    switch(pCtrl->BGNET.PILOT.SETUP.SPD.value.nByte) {
        default:
        case BGNET_SPD_TRA:
            pV->value.dValue = pCtrl->BGNET.SPEED.STW.value.dValue + pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.value.dValue;
            break;
        case BGNET_SPD_MAN_FST:
            pV->value.dValue = ITOD(pCtrl->BGNET.PILOT.SPEED.FST.value.nLong);
            break;
        case BGNET_SPD_SOG:
            pV->value.dValue = ITOD(pCtrl->BGNET.PILOT.NMEA.VTG.SOG.value.dValue);
            break;
        case BGNET_SPD_MAN_MED:
            pV->value.dValue = ITOD(pCtrl->BGNET.PILOT.SPEED.MED.value.nLong);
            break;
        case BGNET_SPD_MAN_SLW:
            pV->value.dValue = ITOD(pCtrl->BGNET.PILOT.SPEED.SLW.value.nLong);
            break;
    }
    pV->source = BGN_SOURCE_ACP;
    CounterUp(&pV->Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
    return(pCtrl->BGNET.PILOT.SETUP.SPD.value.nByte);
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
unsigned char bgnet_checksum_head_calc(FRAME_H *pF) {
    return(bgnet_checksum((unsigned char *)pF, BGNET_FRAME_CR_HEAD, 0, BGNET_FRAME_HEAD-1));
}
unsigned char bgnet_checksum_frame_calc(FRAME_H *pF) {
    return(bgnet_checksum(pF->payload, BGNET_FRAME_CR_FRAME, 0, pF->PL));
}
#define BGNET_CHECKSUM_OK   0x01
unsigned char bgnet_checksum_head_check(FRAME_H *pF) {
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
unsigned char bgnet_checksum_frame_check(BGNET_CTRL *pCtrl, FRAME_H *pF) {
    unsigned char r, cs;
    r = ~BGNET_CHECKSUM_OK;
    bgnet_weird_seed(pCtrl, pF); // we are doing something weird
    cs = bgnet_checksum_frame_calc(pF);
    if (pF->payload[pF->PL] == cs) {
        r = BGNET_CHECKSUM_OK;
    } else {
        if ((pF->payload[pF->PL] + 3) == cs) {
            r = BGNET_CHECKSUM_OK;
        } else {
            if (pF->payload[pF->PL] == (cs + 3)) {
                r = BGNET_CHECKSUM_OK;
            } else {
                log_error("    Frame Checksum ERR, framesize: %d (received: 0x%02X, calculated: 0x%02X, previous=0x%02X)", bgnet_sizeof_frame(pF), pF->payload[pF->PL], cs, pCtrl->BGNET.weird.ACKSEQprev);
            }
        }
    }
    return(r);
}
// get the index for the devicetype we are looking for
int bgnet_deviceindex(BGNET_CTRL *pCtrl, unsigned char ucDevAddr) {
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
// safe copy WayPointID, size=12, last usable index=11
void safecopyWPID(NMEA2K_VALUE *pV, char *pString) {
    strncpy(pV->value.cString, pString, sizeof(pV->value.cString)-1);
    pV->value.cString[sizeof(pV->value.cString)-1] = BGNET_ASC_NULL;
    return;
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
        Counter->iCounter = 1; // after a reset we start at 1 because 0 means 'never touched'
        Counter->tCounterReset = t;
    }
    Counter->tTimestamp = t;
}
// initialize a NMEA value
void NMEAinit(NMEA2K_VALUE *pNMEA, time_t t) {
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
// used by log (dump a channel)
void log_debug_channel(BGNET_CTRL *pCtrl, union CH *pCH) {
    char asciibuf[BGN_FRAME_SIZE];
    byte2ascii(asciibuf, sizeof(asciibuf), (unsigned char *)pCH, bgnet_sizeof_channel(pCtrl, pCH), BYTE2ASCII_SPACED);
    log_debug("log_debug_channel: Channel 0x%02X -> %s", pCH->s.CH, asciibuf);
}
// used by log (dump a frame)
void log_debug_frame(FRAME_H *pF) {
    char asciibuf[BGN_FRAME_SIZE];
    byte2ascii(asciibuf, sizeof(asciibuf), (unsigned char *)pF, bgnet_sizeof_frame(pF), BYTE2ASCII_SPACED);
    log_debug("log_debug_frame: Command 0x%02X -> %s", pF->CMD, asciibuf);
}

// a reset is usually doSPEED.TIMERne:
// - at startup of this driver
// - when we observe "POWERUP" message(s) from the NETWORK (of any of the units)
void bgnet_initialize_reset(BGNET_CTRL *pCtrl) {
    pCtrl->iInitialized = BGNET_CTRL_INIT_RESET; // reset initialized state
}

// worker routine to convert the timer into HH:MM:SS
// timer runs to 99:59:59 maximum (= 100x3600-1 or 360000-1). Observe that TIMER channel 0x75 carries the timer value as 3
// seperate bytes and does not hit the limit of a 16bits unsigned integer (65.535)
time_t tTimerCalc(BGNET_CTRL *pCtrl, unsigned char *pHH, unsigned char *pMM, unsigned char *pSS) {
    time_t r;
    r = BGNET_MIN(pCtrl->BGNET.SERVICE.SystemTime.value.tTime - pCtrl->BGNET.SPEED.TIMER.value.tTime, 360000-1);
    // do not write if there is a NULL pointer
    if (pHH != NULL) *pHH = labs(r/3600) % 24;
    if (pMM != NULL) *pMM = labs(r/60) % 60;
    if (pSS != NULL) *pSS = labs(r) % 60;
    return(r);
}
// return the TIMER value as a formatted string
char *strTimerCalc(BGNET_CTRL *pCtrl, char *pBuf) {
    unsigned char ucHH, ucMM, ucSS;
    tTimerCalc(pCtrl, &ucHH, &ucMM, &ucSS);
    sprintf(pBuf, "%02d:%02d:%02d", ucHH, ucMM, ucSS);
    return(pBuf);
}
void ALARM_Init(BGNET_ALARM *pA, unsigned char ucCHN, char *pText0, char *pText1, time_t t) {
    pA->ucCHN = ucCHN;
    pA->ucSTATE = 0x00;
    pA->ucBEEP = 0x00;
    pA->ucRESET = 0x00;
    CounterInit(&pA->Counter, t);
    CounterInit(&pA->Flasher, t);
    strcpy(pA->AlarmText0, pText0);
    strcpy(pA->AlarmText1, pText1);
}
void ALARM_On(BGNET_ALARM *pA, time_t t) {
    pA->ucSTATE = 1;
    pA->ucBEEP = BGNET_FMT_ALARMBEEP;
    pA->ucRESET = 1;
    CounterUp(&pA->Counter, t);
    CounterInit(&pA->Flasher, t);
}
// alarm on/of with edge detection, only handle changes
void ALARM_Edge(BGNET_ALARM *pA, unsigned char ucSTATE, time_t t) {
    if (pA->ucSTATE == 0) {
        // current state is OFF
        if (ucSTATE == 0) {
            // new state is OFF => no action
        } else {
            // new state is ON => set alarm ON and set new state
            pA->ucSTATE = ucSTATE;
            ALARM_On(pA, t);
        }
    } else {
        // current state is ON
        if (ucSTATE == 0) {
            // new state is OFF => alarm is cleared => no action (wait for SILENCE command)
        } else {
            // new state is ON => slarm is not yet cleared => no action
        }
    }
}
void ALARM_Off(BGNET_ALARM *pA) {
    pA->ucSTATE = 0;
    pA->ucBEEP = 0x00;
    // do not clear ucRESET, this must be done after sending a NULL string
}
// work ALARM data into Channel structure
size_t ALARM_CHN(BGNET_CTRL *pCtrl, union CH *puCH, BGNET_ALARM *pA, time_t t) {
    size_t r = 0;
    if (pA->ucSTATE == 0) {
        if (pA->ucRESET == 0) {
            // reset command was already send, do not send anything
            r = 0;
        } else {
            pA->ucRESET = 0; // clear RESET flag
            puCH->s.CH = pA->ucCHN;
            puCH->s.FMT = BGN_FMT_07;
            strcpy((char *)puCH->s.cv.sB, ""); // send null string
            r = bgnet_sizeof_channel(pCtrl, puCH);
        }
    } else {
        puCH->s.CH = pA->ucCHN;
        puCH->s.FMT = (BGN_FMT_07 | pA->ucBEEP);
        if ((pA->Flasher.iCounter % 2) == 0) {
            strcpy((char *)puCH->s.cv.sB, pA->AlarmText0);
        } else {
            strcpy((char *)puCH->s.cv.sB, pA->AlarmText1);
        }
        CounterUp(&pA->Flasher, t);
        r = bgnet_sizeof_channel(pCtrl, puCH);
    }
    return(r);    
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

        // initialize some weird stuff, read the notes in the routine dealing with it
        pCtrl->BGNET.weird.ACKSEQ = 0x00;
        pCtrl->BGNET.weird.ACKSEQprev = 0x00;

        // stage 1: set state 'initialized'
        time(&t); // save timestamp to we use same timestamp everywhere
        pCtrl->iInitialized = BGNET_CTRL_INIT; // mark as initialized
        pCtrl->iDeviceCounter = BGNET_CTRL_DEVCNTR_INIT;
        pCtrl->iDeviceRun = 0; //reset run counter (0-1000)

        // stage 2 - ACP initialize data structure
        pCtrl->BGNET.iNewACP = 0;
        pCtrl->BGNET.tTimestamp = t; // this timestamp is set on initialisation. this indicates when the application started

        // SystemTime // this is the ACP time; the actual time of the computer this application is running on, set by timer-thread, used where we need the time
        pCtrl->BGNET.SERVICE.SystemTime.value.tTime = t;
        pCtrl->BGNET.SERVICE.SystemTime.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.SERVICE.SystemTime.Counter, t);

        // NetworkTime // this is the time received from external source (GPS, TFTP, API or whatever), this indicates what others think the time is
        pCtrl->BGNET.SERVICE.NetworkTime.value.tTime = t;
        pCtrl->BGNET.SERVICE.NetworkTime.source = BGN_SOURCE_NONE;
        CounterInit(&pCtrl->BGNET.SERVICE.NetworkTime.Counter, t);

        // PILOT watchdog (set by PILOT_HELLO, count-down by timer)
        pCtrl->BGNET.SERVICE.PILOTWD.value.nLong = BGNET_PILOTWD_SET;
        pCtrl->BGNET.SERVICE.PILOTWD.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.SERVICE.PILOTWD.Counter, t);

        // initialize AutoPilot to STBY/COMP; joystick OFF
        pCtrl->BGNET.ACP.STATE.value.nByte = (BGN_AP_MODE1_STBY | BGN_AP_MODE1_VANE);
        pCtrl->BGNET.ACP.STATE.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.ACP.STATE.Counter, t);

        pCtrl->BGNET.ACP.MODE.value.nByte = BGN_AP_MODE2_COMP;
        pCtrl->BGNET.ACP.MODE.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.ACP.MODE.Counter, t);

        pCtrl->BGNET.ACP.Joystick.value.nByte = BGN_JOYSTICK_OFF; // overrides PILOT control
        pCtrl->BGNET.ACP.Joystick.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.ACP.Joystick.Counter, t);

        pCtrl->BGNET.PILOT.ucNAV_SIM = 0;

        pCtrl->BGNET.PILOT.NAV_SEL.value.nByte = 0x00;
        pCtrl->BGNET.PILOT.NAV_SEL.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.NAV_SEL.Counter, t);

        // PILOT SETUP parameters
        pCtrl->BGNET.PILOT.SETUP.SPD.value.nByte = BGNET_SPD_TRA;
        pCtrl->BGNET.PILOT.SETUP.SPD.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.SETUP.SPD.Counter, t);

        pCtrl->BGNET.PILOT.SETUP.RESP.value.nByte = 2;  //0=Normal, 1=Downwind, 2=Eco
        pCtrl->BGNET.PILOT.SETUP.RESP.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.SETUP.RESP.Counter, t);

        // only visible when bit 4 in MODE byte is set (undocumented feature)
        pCtrl->BGNET.PILOT.SETUP.WIND_SEL.value.nByte = 0;
        pCtrl->BGNET.PILOT.SETUP.WIND_SEL.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.SETUP.WIND_SEL.Counter, t);

        pCtrl->BGNET.PILOT.SETUP.WA_MIN.value.nLong = 14; // default value is 4 minutes, range 1-360 (so it must be a 16 bits register)
        pCtrl->BGNET.PILOT.SETUP.WA_MIN.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.SETUP.WA_MIN.Counter, t);

        pCtrl->BGNET.PILOT.SETUP.WA_STAT.value.nByte = 0; // default is OFF
        pCtrl->BGNET.PILOT.SETUP.WA_STAT.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.SETUP.WA_STAT.Counter, t);

        pCtrl->BGNET.PILOT.SETUP.OC_DEG.value.nLong = 25; // default value is 15degrees; range 1-99
        pCtrl->BGNET.PILOT.SETUP.OC_DEG.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.SETUP.OC_DEG.Counter, t);

        pCtrl->BGNET.PILOT.SETUP.OC_STAT.value.nByte = 0; // default is OFF
        pCtrl->BGNET.PILOT.SETUP.OC_STAT.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.SETUP.OC_STAT.Counter, t);

        pCtrl->BGNET.PILOT.SETUP.JSM.value.nLong = 1; // Joystick Steering Mode (Normal, Proportional)
        pCtrl->BGNET.PILOT.SETUP.JSM.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.SETUP.JSM.Counter, t);

        pCtrl->BGNET.PILOT.SETUP.DMP.value.nLong = 44; // Compass Damping, 1-99 sec, 4= default
        pCtrl->BGNET.PILOT.SETUP.DMP.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.SETUP.DMP.Counter, t);

        pCtrl->BGNET.PILOT.SETUP.FR.value.nLong = 1; // Fault Reporting (On/Off)
        pCtrl->BGNET.PILOT.SETUP.FR.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.SETUP.FR.Counter, t);

        // manual speed parameters
        pCtrl->BGNET.PILOT.SPEED.SEL.value.nLong = BGNET_SPD_MAN_FST; // default FST
        pCtrl->BGNET.PILOT.SPEED.SEL.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.SPEED.SEL.Counter, t);

        pCtrl->BGNET.PILOT.SPEED.FST.value.nLong = 81;      // value is displayed as xx.x, but the decimal is always 0 and only whole number can be set
        pCtrl->BGNET.PILOT.SPEED.FST.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.SPEED.FST.Counter, t);

        pCtrl->BGNET.PILOT.SPEED.MED.value.nLong = 62;      // value is displayed as xx.x, but the decimal is always 0 and only whole numberpCtrl->BGNET.ACP.MODE.value.nByte can be set
        pCtrl->BGNET.PILOT.SPEED.MED.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.SPEED.MED.Counter, t);

        pCtrl->BGNET.PILOT.SPEED.SLW.value.nLong = 43;      // value is displayed as xx.x, but the decimal is always 0 and only whole number can be set
        pCtrl->BGNET.PILOT.SPEED.SLW.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.SPEED.SLW.Counter, t);

        // NETWORK parameters
        pCtrl->BGNET.NETWORK.BackLight.value.nByte = 0; // backlight (0=OFF, 3-1=ON)
        pCtrl->BGNET.NETWORK.BackLight.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.NETWORK.BackLight.Counter, t);

        CounterInit(&pCtrl->BGNET.NETWORK.AlarmReset, t);

        pCtrl->BGNET.ACP.TARGET_AWA.value.nLong = -97; // Target AWA, range -180/+180)
        pCtrl->BGNET.ACP.TARGET_AWA.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.ACP.TARGET_AWA.Counter, t);

        pCtrl->BGNET.ACP.CTS.value.nLong = 123; // Course To Steer (calculated by the Autopilot, range 0-359)
        pCtrl->BGNET.ACP.CTS.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.ACP.CTS.Counter, t);

        pCtrl->BGNET.ACP.Heading.value.nLong = 321; // Heading from compass (0-359)
        pCtrl->BGNET.ACP.Heading.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.ACP.Heading.Counter, t);

        pCtrl->BGNET.ACP.Rudder.value.nLong = -2; // Rudder position (-45-+45)
        pCtrl->BGNET.ACP.Rudder.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.ACP.Rudder.Counter, t);

        pCtrl->BGNET.ACP.CRS.value.nLong = 0; // Preset Course To Steer
        pCtrl->BGNET.ACP.CRS.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.ACP.CRS.Counter, t);

        pCtrl->BGNET.ACP.MEM.value.nLong = 0; // Memorized Course To Steer
        pCtrl->BGNET.ACP.MEM.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.ACP.MEM.Counter, t);

        // we do NOT use a floating point value; otherwise we get all kind or rounding errors. Just keep it as handled within the
        // B&G system: as an integer*10 (so we get 1 decimal)
        pCtrl->BGNET.SPEED.StoredLogNM.value.nLong = 12345*BGNET_DIVISOR_STORLOG; // channel CD/STORELOGNM (on the real system this is stored in non-volitile memory by QUAD/SPEED, no reset)
        pCtrl->BGNET.SPEED.StoredLogNM.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.SPEED.StoredLogNM.Counter, t);

        // it appears as if the triplog has a multiplier of 64 (and not 10)
        pCtrl->BGNET.SPEED.TripLogNM.value.dValue = 0.287*BGNET_DIVISOR_TRIPLOG; // channel CF/TRIPLOGNM (on the real system this is stored in non-volitile memory by QUAD/SPEED, resetable)
        pCtrl->BGNET.SPEED.TripLogNM.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.SPEED.TripLogNM.Counter, t);

        pCtrl->BGNET.SPEED.DRD.value.dValue = 1.13; // resetable with DND-RESET
        pCtrl->BGNET.SPEED.DRD.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.SPEED.DRD.Counter, t);

        pCtrl->BGNET.SPEED.DRC.value.nLong = 358;
        pCtrl->BGNET.SPEED.DRC.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.SPEED.DRC.Counter, t);

        pCtrl->BGNET.SPEED.Temperature.value.dValue= 19.26;
        pCtrl->BGNET.SPEED.Temperature.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.SPEED.Temperature.Counter, t);

        pCtrl->BGNET.SPEED.STW.value.dValue =4.0; // also used to initialize SpeedMAX and SpeedAVG
        pCtrl->BGNET.SPEED.STW.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.SPEED.STW.Counter, t);

        pCtrl->BGNET.SPEED.SpeedMAX.value.dValue = 2.22; // reset with DRD
        pCtrl->BGNET.SPEED.SpeedMAX.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.SPEED.SpeedMAX.Counter, t);

        pCtrl->BGNET.SPEED.SpeedAVG.value.dValue = 1.11; // reset with DRD
        pCtrl->BGNET.SPEED.SpeedAVG.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.SPEED.SpeedAVG.Counter, t);

        pCtrl->BGNET.SPEED.TIMER.value.tTime = t;            // the TIMER value is 'the current time' (SERVICE.SystemTime.value.tTime) minus this (SPEED.TIMER.value.tTime)
        pCtrl->BGNET.SPEED.TIMER.source = BGN_SOURCE_ACP;    // converted to HH, MM and SS
        CounterInit(&pCtrl->BGNET.SPEED.TIMER.Counter, t);

        pCtrl->BGNET.PILOT.NMEA.VTG.COG.value.nLong = 312;
        pCtrl->BGNET.PILOT.NMEA.VTG.COG.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.NMEA.VTG.COG.Counter, t);

        pCtrl->BGNET.PILOT.NMEA.VTG.SOG.value.dValue = 22.8;
        pCtrl->BGNET.PILOT.NMEA.VTG.SOG.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.NMEA.VTG.SOG.Counter, t);

        pCtrl->BGNET.PILOT.NMEA.VTG.VTG_SEL.value.nByte = 0;
        pCtrl->BGNET.PILOT.NMEA.VTG.VTG_SEL.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.NMEA.VTG.VTG_SEL.Counter, t);

        pCtrl->BGNET.PILOT.NMEA.RMB.XTE.value.dValue = +0.0234;
        pCtrl->BGNET.PILOT.NMEA.RMB.XTE.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.NMEA.RMB.XTE.Counter, t);

        safecopyWPID(&pCtrl->BGNET.PILOT.NMEA.RMB.WPID, "M1234567890"); // safecopy WPID (maximum 11 characters + 1 nullbyte)
        pCtrl->BGNET.PILOT.NMEA.RMB.WPID.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.NMEA.RMB.WPID.Counter, t);

        pCtrl->BGNET.PILOT.NMEA.RMB.WPD.value.dValue = 9.876;
        pCtrl->BGNET.PILOT.NMEA.RMB.WPD.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.NMEA.RMB.WPD.Counter, t);

        pCtrl->BGNET.PILOT.NMEA.RMB.WPB.value.nLong = 318;
        pCtrl->BGNET.PILOT.NMEA.RMB.WPB.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.NMEA.RMB.WPB.Counter, t);

        pCtrl->BGNET.PILOT.NMEA.RMB.RMB_SEL.value.nByte = 0;
        pCtrl->BGNET.PILOT.NMEA.RMB.RMB_SEL.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.PILOT.NMEA.RMB.RMB_SEL.Counter, t);

        // evaluate effective speed (do this AFTER SPD/TRA, MAN/SEL/FST/MED/SLW and SOG are initialized)
        pCtrl->BGNET.ACP.EBS_SEL.value.nByte = EffectiveBoatSpeed(pCtrl, &pCtrl->BGNET.ACP.EBS);
        pCtrl->BGNET.ACP.EBS_SEL.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.ACP.EBS_SEL.Counter, t);
        pCtrl->BGNET.ACP.EBS.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.ACP.EBS.Counter, t);

        // Alarms

        ALARM_Init(&pCtrl->BGNET.PILOT.ALARM.WA, BGNET_CHN_76_AP_WA_ALARM, "WATCH   ", "ALARM   ", t);
        ALARM_Init(&pCtrl->BGNET.PILOT.ALARM.OC, BGNET_CHN_AF_AP_OC_ALARM, "  OFF   ", "COURSE  ", t);

        // initialize key counters (we need them to detect double-key presses and simultaneous key presses)

        CounterInit(&pCtrl->BGNET.PILOT.KEY.AUTO, t);
        CounterInit(&pCtrl->BGNET.PILOT.KEY.OFF, t);
        CounterInit(&pCtrl->BGNET.PILOT.KEY.COURSE, t);
        CounterInit(&pCtrl->BGNET.PILOT.KEY.P10, t);
        CounterInit(&pCtrl->BGNET.PILOT.KEY.S10, t);
        CounterInit(&pCtrl->BGNET.PILOT.KEY.P1, t);
        CounterInit(&pCtrl->BGNET.PILOT.KEY.S1, t);

// ==== SPEED SETUP

        pCtrl->BGNET.SPEED.SETUP.DMP.value.nLong = 33;
        pCtrl->BGNET.SPEED.SETUP.DMP.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.SPEED.SETUP.DMP.Counter, t);

        // SETUP/#3 = adjust measured speed againts a known reference speed (from a other boat).
        //            when selecting #3, the current boatspeed is displayed. With UP/DOWN you increase/decrease the speed displayed.
        //            when you press ENTER, a calibration factor is calculated and stored
        //            The measured boatspeed is adjusted with the calibration factor and displayed as STW (Speed Through Water)
        // SETUP/#4 = adjust measured speed against a know distance and time
        //            when selecting #4, the current boatspeed is displayed. After you calculate the boatspeed you increase/decrease
        //            the displayed boa structure
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

        // stage 5 - initialize channeltspeed. When you press ENTER, a calibration factor is calculated and stored.
        // SETUP/#5 = automatic calibration is effectively the same as #4, with the difference that the SPEED/QUAD node is doing
        //            the timekeeping.
        // All 3 calibration methods store their correction factor in the same variable
        // I feel it's of little practical use to implement these calibation factor. It is set at a fixed value of 1.0
        // From my engineering point of view, i would expect 2 variables: a 'offset' and a 'muliplier'.
        
        pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.value.dValue = BGNET_SPEED_CAL_OFFSET;
        pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.Counter, t);

        pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.value.dValue = BGNET_SPEED_CAL_FACTOR;
        pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.Counter, t);

//====== DEPTH SETUP

        pCtrl->BGNET.DEPTH.SETUP.CAL.value.dValue = +0.252; // sensor depth below the waterline, depth shown is depth from the waterline
        pCtrl->BGNET.DEPTH.SETUP.CAL.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.DEPTH.SETUP.CAL.Counter, t);

        pCtrl->BGNET.DEPTH.Depth_RAW.value.dValue = 2.598;
        pCtrl->BGNET.DEPTH.Depth_RAW.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.DEPTH.Depth_RAW.Counter, t);

        pCtrl->BGNET.DEPTH.Depth_CAL.value.dValue = pCtrl->BGNET.DEPTH.Depth_RAW.value.dValue + pCtrl->BGNET.DEPTH.SETUP.CAL.value.dValue;
        pCtrl->BGNET.DEPTH.Depth_CAL.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.DEPTH.Depth_CAL.Counter, t);

        pCtrl->BGNET.DEPTH.SETUP.SA_M.value.dValue = 50.50;
        pCtrl->BGNET.DEPTH.SETUP.SA_M.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.DEPTH.SETUP.SA_M.Counter, t);

        pCtrl->BGNET.DEPTH.SETUP.SA_STAT.value.nByte = 0; // default =OFF
        pCtrl->BGNET.DEPTH.SETUP.SA_STAT.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.DEPTH.SETUP.SA_STAT.Counter, t);

        pCtrl->BGNET.DEPTH.SETUP.DA_M.value.dValue = 40.4;
        pCtrl->BGNET.DEPTH.SETUP.DA_M.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.DEPTH.SETUP.DA_M.Counter, t);

        pCtrl->BGNET.DEPTH.SETUP.DA_STAT.value.nByte = 4;
        pCtrl->BGNET.DEPTH.SETUP.DA_STAT.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.DEPTH.SETUP.DA_STAT.Counter, t);

        pCtrl->BGNET.DEPTH.SETUP.ASA_M.value.dValue = -10.10;
        pCtrl->BGNET.DEPTH.SETUP.ASA_M.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.DEPTH.SETUP.ASA_M.Counter, t);

        pCtrl->BGNET.DEPTH.SETUP.ADA_M.value.dValue = -20.20;
        pCtrl->BGNET.DEPTH.SETUP.ADA_M.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.DEPTH.SETUP.ADA_M.Counter, t);

        pCtrl->BGNET.DEPTH.SETUP.AMA_M.value.dValue = 30.3;
        pCtrl->BGNET.DEPTH.SETUP.AMA_M.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.DEPTH.SETUP.AMA_M.Counter, t);

        pCtrl->BGNET.DEPTH.SETUP.AA_STAT.value.nByte = 3;
        pCtrl->BGNET.DEPTH.SETUP.AA_STAT.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.DEPTH.SETUP.AA_STAT.Counter, t);

        CounterInit(&pCtrl->BGNET.DEPTH.SETUP.AA_TOGGLE, t);

        //DEPTH ALARMS

        ALARM_Init(&pCtrl->BGNET.DEPTH.ALARM.SHAL, BGNET_CHN_FC_SHALLOWALARM, "SHALLOW ", "ALARM   ", t);
        ALARM_Init(&pCtrl->BGNET.DEPTH.ALARM.DEEP, BGNET_CHN_FD_DEEPALARM,    "DEEP    ", "ALARM   ", t);
        ALARM_Init(&pCtrl->BGNET.DEPTH.ALARM.ANCH, BGNET_CHN_FE_ANCHORALARM,  "ANCHOR  ", "ALARM   ", t);

//====== WIND

        pCtrl->BGNET.WIND.AWS.value.dValue = 8.12;
        pCtrl->BGNET.WIND.AWS.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.WIND.AWS.Counter, t);

        pCtrl->BGNET.WIND.AWA.value.nLong = -40; // range -180 to +180 (displayed on PILOT when in VANE mode)
        pCtrl->BGNET.WIND.AWA.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.WIND.AWA.Counter, t);

        pCtrl->BGNET.WIND.TWS.value.dValue = 7.34;
        pCtrl->BGNET.WIND.TWS.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.WIND.TWS.Counter, t);

        pCtrl->BGNET.WIND.TWA.value.nLong = -45; // range -180 to +180
        pCtrl->BGNET.WIND.TWA.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.WIND.TWA.Counter, t);

        pCtrl->BGNET.WIND.VMG.value.dValue = 4.123;
        pCtrl->BGNET.WIND.VMG.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.WIND.VMG.Counter, t);

        pCtrl->BGNET.WIND.BATTERY.value.nLong = DTOIROUND(12.8 * 10); // x10
        pCtrl->BGNET.WIND.BATTERY.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.WIND.BATTERY.Counter, t);

        pCtrl->BGNET.WIND.SETUP.DS.value.nLong = 4; // factory default
        pCtrl->BGNET.WIND.SETUP.DS.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.WIND.SETUP.DS.Counter, t);

        pCtrl->BGNET.WIND.SETUP.DA.value.nLong = 1; // factory default
        pCtrl->BGNET.WIND.SETUP.DA.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.WIND.SETUP.DA.Counter, t);

        pCtrl->BGNET.WIND.SETUP.OF.value.nLong = 356; // factory default, range 0-359 (do not use negative value)
        pCtrl->BGNET.WIND.SETUP.OF.source = BGN_SOURCE_ACP;
        CounterInit(&pCtrl->BGNET.WIND.SETUP.OF.Counter, t);

        // stage 3 - initialize device structure

        pCtrl->iNewDevice = BGN_NEW_RESET;
        for (i=0; i<BGNET_DEVICE_MAX; i++) {
            pCtrl->DEV[i].iNewDevice = BGN_NEW_RESET;
            CounterInit(&pCtrl->DEV[i].PowerUp_Seen, t);
            pCtrl->DEV[i].PowerUp_InitSeen = 0;
            pCtrl->DEV[i].PowerUp_InitDone = 0;
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

            pCtrl->CHN[i].x01.ucFROM = 0x00;
            memset(pCtrl->CHN[i].x01.ucRAW, 0, sizeof(pCtrl->CHN[i].x01.ucRAW));
            strcpy(pCtrl->CHN[i].x01.cValue, "");
            CounterInit(&pCtrl->CHN[i].x01.CounterRx, t);
            pCtrl->CHN[i].x01.pBGNET = NULL;
            pCtrl->CHN[i].x01.BGNET_DATATYPE = BGNET_DATATYPE_UNDEF;

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
        // Here we will setup some pointers for generating broadcasts
        pCtrl->CHN[BGNET_CHN_0B_RUDDER].x01.pBGNET = &pCtrl->BGNET.ACP.Rudder;
        pCtrl->CHN[BGNET_CHN_0B_RUDDER].x01.BGNET_DATATYPE = BGNET_DATATYPE_LONG;

        pCtrl->CHN[BGNET_CHN_20_SEATEMPQ].x01.pBGNET = &pCtrl->BGNET.SPEED.Temperature;
        pCtrl->CHN[BGNET_CHN_20_SEATEMPQ].x01.BGNET_DATATYPE = BGNET_DATATYPE_DOUBLE;

        log_debug("bgnet_initialize: Done");
    }
};

// get the command label from the Command Definition Table
char *strCommandLabel(BGNET_CTRL *pCtrl, char *pBuf, unsigned char ucCMD) {
    if (pCtrl->pCMDDT[ucCMD].ucCommandIndex == ucCMD) {
        strcpy(pBuf, pCtrl->pCMDDT[ucCMD].cLabel);
    } else {
        strcpy(pBuf, "undefined command");  // must be shorter than BGNET_CMD_LABEL_MAX/20
    }
    return(pBuf);
};

// get the channel label from the Channel Definition Table
char *strChannelLabel(BGNET_CTRL *pCtrl, char *pBuf, unsigned char ucCHN) {
    if (pCtrl->pCHNDT[ucCHN].ucChannelIndex == ucCHN) {
        strcpy(pBuf, pCtrl->pCHNDT[ucCHN].cName);
    } else {
        strcpy(pBuf, "undefined channel");  // must be shorter than BGNET_CHN_LABEL/20
    }
    return(pBuf);
};

// encode a frame (with or without payload)
size_t bgnet_encode_frame(FRAME_H *pFH, unsigned char ucTo, unsigned char ucFrom, unsigned char ucCmd, unsigned char *payload, unsigned char ucPL) {
    pFH->TO = ucTo;
    pFH->FROM = ucFrom;
    pFH->PL = ucPL;
    pFH->CMD = ucCmd;
    pFH->CSH = bgnet_checksum_head_calc(pFH);
    if (ucPL == 0) {
        // nothing to do
    } else {
        memcpy(pFH->payload, payload, ucPL);
        pFH->payload[ucPL] = bgnet_checksum_frame_calc(pFH);
    }
    return(bgnet_sizeof_frame(pFH));
}

// decode a double value from a numerical type (FMT_01, FMT_08, FMT_0D) channel. Note that
// FMT_0D has 2x 16 bits registers (used for ANC alarm setting)
double decode_channel_double(union CH *pCH, int iX) {
    double dR=-9999.99, dD;
    int iD;
    double dDivisor[4] = {1.0, 10.0, 100.0, 1000.0};

    iD = (pCH->s.FMT & BGNET_FMT_DIVPAD_DIV)>>6;
    dD = dDivisor[iD];
    switch (iX) {
        case 0:
            switch (pCH->s.FMT & BGNET_FMT_DT) {
                case (BGN_FMT_01 & BGNET_FMT_DT):
                    dR = ((double)((int16_t)(pCH->s.cv.uW[iX])/dD)); // interpret as signed
                    break;
                case (BGN_FMT_08 & BGNET_FMT_DT):
                case (BGN_FMT_0D & BGNET_FMT_DT):
                    dR = ((double)((int16_t)ntohs(pCH->s.cv.uW[iX])/dD)); // interpret as signed
                    break;
            }
            break;
        case 1:
            switch (pCH->s.FMT & BGNET_FMT_DT) {
                case (BGN_FMT_0D & BGNET_FMT_DT):
                    dR = ((double)ntohs(pCH->s.cv.uW[iX])/dD);
                    break;
            }
            break;
    }
    return(dR);
}

// encode a double value into a numerical type (FMT 01, 08) channel
size_t encode_channel_double(BGNET_CTRL *pCtrl, union CH *pCH, unsigned char ucCH, unsigned char ucFMT, double dV, int iX) {
    uint8_t uD, uP;  // DIVisor and PADding
    uint16_t uW; // unsigned 16-bit register
    size_t sChannelSize;
    // set ChannelID
    pCH->s.CH = ucCH;
    uP = 0; // we do not use padding (yet)
    // set FMT byte. Skip auto-scaling when passed FMT is already scaled
    // note: use used to check for a zero "DIVPAD", here, but we later found out that
    // bit 4 could also be used for ALARM.
    // hance we have changed it to check for a zero "DIV" only.
    //if ((ucFMT & BGNET_FMT_DIVPAD) == 0) {
    if ((ucFMT & BGNET_FMT_DIVPAD_DIV) == 0) {
        // do auto-scale, scale it before rounding/converting to int
        if (dV < 1.0) {
            uW = DTOIROUND(dV * 1000.0);
            uD = 3;
        } else {
            if (dV < 10.0) {
                uW = DTOIROUND(dV * 100.0);
                uD = 2;
            } else {
                if (dV < 100.0) {
                    uW = DTOIROUND(dV * 10.0);
                    uD = 1;
                } else {
                    uW = DTOIROUND(dV); // in case of overflow, B&G devices show only the 3 rightmost digits
                    uD = 0;
                }
            }
        }
        // pack the FMT specifier, shift DIV and inject PAD. Put all the bits into the FMT byte
        pCH->s.FMT = (ucFMT & BGNET_FMT_DT)| ((uD<<6) & BGNET_FMT_DIVPAD_DIV) | (uP & BGNET_FMT_DIVPAD_PAD);
    } else {
        // skip autoscale (value is already scaled)
        pCH->s.FMT = ucFMT;
        uW = DTOIROUND(dV);
    }
    sChannelSize = bgnet_sizeof_channel(pCtrl, pCH);
    switch(iX) { // we only accept index 0(5 byte payload, FMT_01, FMT_08, FMT_0D) or 1 (7 byte payload, FMT_0D)
        case 0: // normally we only have 1 2-byte value
            switch(ucFMT & BGNET_FMT_DT) {
                case BGN_FMT_01: // convert unsigned to signed 16bit register
                    pCH->s.cv.sW[iX] = (int16_t)htons(uW);
                    break;
                case BGN_FMT_08: // unsigned 16bit register
                    pCH->s.cv.uW[iX] = htons(uW);
                    break;
                case BGN_FMT_0D: // unsigned 16bit register
                    pCH->s.cv.uW[iX] = htons(uW);
                    break;
            }
            break;
        case 1: // sometimes we have 2 2-byte values
            switch(ucFMT & BGNET_FMT_DT) {
                case BGN_FMT_0D: // unsigned 16bit register
                    pCH->s.cv.uW[iX] = htons(uW);
                    break;
            }
            break;
        default:
            break;
    }
    return(sChannelSize);
}

// returns a string representation of a channel value
// numeric: do not call this against non-numeric FMT's. Returns a numeric you can use in calculations
// string: use this to display values (ascii, numeric, time, whatever)
int decode_channel_numeric(union CH *puCH) {
    int iRawValue;
    switch(puCH->s.FMT & BGNET_FMT_DT) {
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
            iRawValue = 0;
            break;
    }
    return(iRawValue);
}
char *decode_channel_string(char *pStr, union CH *puCH) {
    int iRawValue;
    double dRawValue;
    int iValid; // 0= invalid DataType, set to "???"
                // 1= Numerical, convert RawValue byte sequence (with optional scaling)
                // 2= ASCII, already set (text, timer, hex)
                // 3= double, value in dRawValue, scaled
    unsigned int d;
    const double fDiv[4]    = { 1.0,   10.0,  100.0, 1000.0};
    const char cPrint[4][6] = {"%d", "%.1f", "%.2f", "%.3f"};
    switch (puCH->s.CH) {
        // some channels have no data
        case BGNET_CHN_98_NMEA_ACTIVE:
            strcpy(pStr,"{null}");
            iValid = 2;
            break;
        default:
            switch(puCH->s.FMT & BGNET_FMT_DT) {
                case BGN_FMT_00:
                    strcpy(pStr,"{null}");
                    iValid = 2;
                    break;
                case BGN_FMT_01: // signed 16bit register (-999 to +999)
                    switch (puCH->s.CH) {
                        case BGNET_CHN_AF_AP_OC_ALARM:
                            sprintf(pStr,"0x%02X", puCH->s.cv.uB[1]);
                            iValid = 2;
                            break;
                        case BGNET_CHN_EE_XTE:
                            dRawValue = decode_channel_numeric(puCH)/100.0;
                            iValid = 3;
                            break;
                        default:
                            iRawValue = decode_channel_numeric(puCH);
                            iValid = 1;
                            break;
                    }
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
                    pStr[3] = BGNET_ASC_NULL;
                    iValid = 2;
                    break;
                case BGN_FMT_07: // variable length ASCII string (null-terminated)
                    strcpy(pStr, puCH->s.cv.uC);
                    iValid = 2;
                    break;
                case BGN_FMT_08: // unsigned 16bit register
                    switch (puCH->s.CH) {
                        case BGNET_CHN_46_AP_SPEED:
                        case BGNET_CHN_B5_AP_STATUS:
                            sprintf(pStr,"0x%02X 0x%02X", puCH->s.cv.uB[0], puCH->s.cv.uB[1]);
                            iValid = 2;
                            break;
                        //case BGNET_CHN_CF_TRIPNM: // it seems that the TRIPLOG has not 1 10.0 divisor, but a 64.0
                        //    iRawValue = decode_channel_numeric(puCH);
                        //    sprintf(pStr, "%.1f", ITOD(iRawValue)/BGNET_DIVISOR_TRIPLOG);
                        //    iValid = 2;
                        //    break;
                        case BGNET_CHN_E8_WPD:
                            dRawValue = decode_channel_numeric(puCH)/100.0;
                            iValid = 3;
                            break;
                        default:
                            iRawValue = decode_channel_numeric(puCH);
                            iValid = 1;
                            break;
                    }
                    break;
                case BGN_FMT_0A: // 4 bytes ASCII (length = 6)
                    pStr[0] = puCH->s.cv.uB[0];
                    pStr[1] = puCH->s.cv.uB[1];
                    pStr[2] = puCH->s.cv.uB[2];
                    pStr[3] = puCH->s.cv.uB[3];
                    pStr[4] = BGNET_ASC_NULL;
                    iValid = 2;
                    break;
                case BGN_FMT_0C: // 3 bytes ASCII, byte 0 is unused
                    pStr[0] = puCH->s.cv.uB[0];
                    pStr[1] = puCH->s.cv.uB[1];
                    pStr[2] = puCH->s.cv.uB[2];
                    pStr[3] = BGNET_ASC_NULL;
                    iValid = 2;
                    break;
                default:        // all others
                    pStr[0] = BGNET_ASC_QUEST;
                    pStr[1] = BGNET_ASC_QUEST;
                    pStr[2] = BGNET_ASC_QUEST;
                    pStr[3] = BGNET_ASC_NULL;
                    iValid = 0;
            }
            break;
    }
    switch(iValid) {
        case 0: // invalue format, result already there
            break;
        case 1: // numerical with auto-scaling
            // get divisor value (0-3) from FMT byte
            d = (puCH->s.FMT & BGNET_FMT_DIVPAD_DIV)>>6;
            switch (d) {
                case 1:  // scaled value is printed with a decimals
                case 2:
                case 3:
                    // convert to a real value, apply divisor (1.0-1000.0)
                    sprintf(pStr, cPrint[d], ITOD(iRawValue)/fDiv[d]);
                    break;
                default: // non-scaled value is printed a integer (without a decimalpoint)
                    sprintf(pStr, cPrint[d], iRawValue);
                    break;
            }
            break;
        case 2: // ASCII output, result already there
            break;
        case 3: // fixed scaled integer, use dRawValue
            sprintf(pStr, "%.2f", dRawValue);
            break;           
    }
    return(pStr);
}

// Send WIND/SETUP parameters (usually at PowerUp)
size_t sendWINDSETUP(BGNET_CTRL *pCtrl, unsigned char ucTO, unsigned char ucFROM) {
    size_t len_msg, TxSize;
    union CH *puCH;
    FRAME_H *pFTx;
    unsigned char TxBuf[BGNET_RXTXRAW_SIZE], payload[BGNET_FRAME_PAYLOAD_MAX];

    len_msg = 0;
    TxSize = 0;

    // WIND/SETUP/#1 DS (damping windspeed)
    pFTx = (FRAME_H *)&TxBuf[TxSize];
    payload[0] = 0; // ACKSEQ
    puCH = (union CH *)&payload[1];
    puCH->s.CH = BGNET_CHN_50_AWS;
    puCH->s.FMT = BGN_FMT_08;
    puCH->s.cv.uW[0] = htons(pCtrl->BGNET.WIND.SETUP.DS.value.nLong);
    len_msg =  bgnet_encode_frame(pFTx, ucTO, ucFROM, BGNET_CMD_CE_PILOT_DMPDAT, payload, 5);
    TxSize = TxSize + len_msg;

    // register channel written
    pCtrl->CHN[puCH->s.CH].iNewChannel = BGN_NEW_SET;
    pCtrl->CHN[puCH->s.CH].x00.ucTO = pFTx->TO;
    pCtrl->CHN[puCH->s.CH].x00.ucFROM = pFTx->FROM;
    pCtrl->CHN[puCH->s.CH].x00.ucCMD = pFTx->CMD;
    memcpy(pCtrl->CHN[puCH->s.CH].x00.ucRAW, puCH->ucRAW, 4);
    decode_channel_string(pCtrl->CHN[puCH->s.CH].x00.cValue, puCH);
    CounterUp(&pCtrl->CHN[puCH->s.CH].x00.CounterRx, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);

    // register command send
    memcpy(pCtrl->CMD[BGNET_CMD_CE_PILOT_DMPDAT].Tx.ucFrame, pFTx, len_msg);
    CounterUp(&pCtrl->CMD[BGNET_CMD_CE_PILOT_DMPDAT].CounterTx, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
    memcpy(pCtrl->DEV[pFTx->TO].Tx.ucFrame, pFTx, len_msg);
    CounterUp(&pCtrl->DEV[pFTx->TO].CounterTx, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);

    // WIND/SETUP/#2 DA (damping windangle)
    pFTx = (FRAME_H *)&TxBuf[TxSize];
    payload[0] = 0; // ACKSEQ
    puCH = (union CH *)&payload[1];
    puCH->s.CH = BGNET_CHN_51_AWA;
    puCH->s.FMT = BGN_FMT_08;
    puCH->s.cv.uW[0] = htons(pCtrl->BGNET.WIND.SETUP.DA.value.nLong);
    len_msg =  bgnet_encode_frame(pFTx, ucTO, ucFROM, BGNET_CMD_CE_PILOT_DMPDAT, payload, 5);
    TxSize = TxSize + len_msg;

    // register channel written
    pCtrl->CHN[puCH->s.CH].iNewChannel = BGN_NEW_SET;
    pCtrl->CHN[puCH->s.CH].x00.ucTO = pFTx->TO;
    pCtrl->CHN[puCH->s.CH].x00.ucFROM = pFTx->FROM;
    pCtrl->CHN[puCH->s.CH].x00.ucCMD = pFTx->CMD;
    memcpy(pCtrl->CHN[puCH->s.CH].x00.ucRAW, puCH->ucRAW, 4);
    decode_channel_string(pCtrl->CHN[puCH->s.CH].x00.cValue, puCH);
    CounterUp(&pCtrl->CHN[puCH->s.CH].x00.CounterRx, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);

    // register command send
    memcpy(pCtrl->CMD[BGNET_CMD_CE_PILOT_DMPDAT].Tx.ucFrame, pFTx, len_msg);
    CounterUp(&pCtrl->CMD[BGNET_CMD_CE_PILOT_DMPDAT].CounterTx, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
    memcpy(pCtrl->DEV[pFTx->TO].Tx.ucFrame, pFTx, len_msg);
    CounterUp(&pCtrl->DEV[pFTx->TO].CounterTx, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);

    // WIND/SETUP/#3 OF (OFfset, sensor alignment correction)
    pFTx = (FRAME_H *)&TxBuf[TxSize];
    payload[0] = 0; // ACKSEQ
    puCH = (union CH *)&payload[1];
    puCH->s.CH = BGNET_CHN_51_AWA;
    puCH->s.FMT = BGN_FMT_01;
    puCH->s.cv.uW[0] = htons(pCtrl->BGNET.WIND.SETUP.OF.value.nLong);
    len_msg =  bgnet_encode_frame(pFTx, ucTO, ucFROM, BGNET_CMD_D3_PILOT_AM, payload, 5);
    TxSize = TxSize + len_msg;

    // register channel written
    pCtrl->CHN[puCH->s.CH].iNewChannel = BGN_NEW_SET;
    pCtrl->CHN[puCH->s.CH].x00.ucTO = pFTx->TO;
    pCtrl->CHN[puCH->s.CH].x00.ucFROM = pFTx->FROM;
    pCtrl->CHN[puCH->s.CH].x00.ucCMD = pFTx->CMD;
    memcpy(pCtrl->CHN[puCH->s.CH].x00.ucRAW, puCH->ucRAW, 4);
    decode_channel_string(pCtrl->CHN[puCH->s.CH].x00.cValue, puCH);
    CounterUp(&pCtrl->CHN[puCH->s.CH].x00.CounterRx, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);

    // register command send
    memcpy(pCtrl->CMD[BGNET_CMD_CE_PILOT_DMPDAT].Tx.ucFrame, pFTx, len_msg);
    CounterUp(&pCtrl->CMD[BGNET_CMD_CE_PILOT_DMPDAT].CounterTx, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
    memcpy(pCtrl->DEV[pFTx->TO].Tx.ucFrame, pFTx, len_msg);
    CounterUp(&pCtrl->DEV[pFTx->TO].CounterTx, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);

    // put it for transmission
    pthread_mutex_lock(&g_data_mutex);
    memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &TxBuf, TxSize); // put output message in write buffer
    g_serial_data_lenTx = g_serial_data_lenTx + TxSize; // increment byte counter in TX buffer
    pthread_mutex_unlock(&g_data_mutex);

    return(TxSize);
}
// Send SPEED/SETUP parameters (usually at PowerUp)
size_t sendSPEEDSETUP(BGNET_CTRL *pCtrl, unsigned char ucTO, unsigned char ucFROM) {
    size_t len_msg, TxSize;
    union CH *puCH;
    FRAME_H *pFTx;
    unsigned char TxBuf[BGNET_RXTXRAW_SIZE], payload[BGNET_FRAME_PAYLOAD_MAX];

    len_msg = 0;
    TxSize = 0;

    // SPEED/SETUP/#1 DMP (damping boatspeed in seconds)
    pFTx = (FRAME_H *)&TxBuf[TxSize];
    payload[0] = 0; // ACKSEQ
    puCH = (union CH *)&payload[1];
    puCH->s.CH = BGNET_CHN_41_STW;
    puCH->s.FMT = BGN_FMT_08;
    puCH->s.cv.uW[0] = htons(pCtrl->BGNET.SPEED.SETUP.DMP.value.nLong);
    len_msg =  bgnet_encode_frame(pFTx, ucTO, ucFROM, BGNET_CMD_CD_PILOT_DMPREQ, payload, 5);
    TxSize = TxSize + len_msg;

    // register channel written
    pCtrl->CHN[puCH->s.CH].iNewChannel = BGN_NEW_SET;
    pCtrl->CHN[puCH->s.CH].x00.ucTO = pFTx->TO;
    pCtrl->CHN[puCH->s.CH].x00.ucFROM = pFTx->FROM;
    pCtrl->CHN[puCH->s.CH].x00.ucCMD = pFTx->CMD;
    memcpy(pCtrl->CHN[puCH->s.CH].x00.ucRAW, puCH->ucRAW, 4);
    decode_channel_string(pCtrl->CHN[puCH->s.CH].x00.cValue, puCH);
    CounterUp(&pCtrl->CHN[puCH->s.CH].x00.CounterRx, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);

    // register command send
    memcpy(pCtrl->CMD[BGNET_CMD_CE_PILOT_DMPDAT].Tx.ucFrame, pFTx, len_msg);
    CounterUp(&pCtrl->CMD[BGNET_CMD_CE_PILOT_DMPDAT].CounterTx, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
    memcpy(pCtrl->DEV[pFTx->TO].Tx.ucFrame, pFTx, len_msg);
    CounterUp(&pCtrl->DEV[pFTx->TO].CounterTx, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);

    // SPEED/SETUP/#2 KNOTS/MPH (unit selection, locally only)

    // SPEED/SETUP/#3 CAL (manual calibration, 0.00-99.9 KT reference speed, same as #4)
    pFTx = (FRAME_H *)&TxBuf[TxSize];
    payload[0] = 0; // ACKSEQ
    puCH = (union CH *)&payload[1];
    // with FMT_08 we send unsigned values. A calibration value can (in theory) both negative or positive. So this must be some
    // kind of multiplier 
    encode_channel_double(pCtrl, puCH, BGNET_CHN_41_STW, BGN_FMT_08, pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.value.dValue, 0);
    len_msg =  bgnet_encode_frame(pFTx, ucTO, ucFROM, BGNET_CMD_D3_PILOT_AM, payload, 5);
    TxSize = TxSize + len_msg;

    // register channel written
    pCtrl->CHN[puCH->s.CH].iNewChannel = BGN_NEW_SET;
    pCtrl->CHN[puCH->s.CH].x00.ucTO = pFTx->TO;
    pCtrl->CHN[puCH->s.CH].x00.ucFROM = pFTx->FROM;
    pCtrl->CHN[puCH->s.CH].x00.ucCMD = pFTx->CMD;
    memcpy(pCtrl->CHN[puCH->s.CH].x00.ucRAW, puCH->ucRAW, 4);
    decode_channel_string(pCtrl->CHN[puCH->s.CH].x00.cValue, puCH);
    CounterUp(&pCtrl->CHN[puCH->s.CH].x00.CounterRx, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);

    // register command send
    memcpy(pCtrl->CMD[BGNET_CMD_CE_PILOT_DMPDAT].Tx.ucFrame, pFTx, len_msg);
    CounterUp(&pCtrl->CMD[BGNET_CMD_CE_PILOT_DMPDAT].CounterTx, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
    memcpy(pCtrl->DEV[pFTx->TO].Tx.ucFrame, pFTx, len_msg);
    CounterUp(&pCtrl->DEV[pFTx->TO].CounterTx, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);

    // SPEED/SETUP/#4 LOGCAL (manual calibration, by known distance and measured time, same as #3)

    // SPEED/SETUP/#5 AUTOCAL (Auto Calibration, just for communication purpose, not implemented)
    pFTx = (FRAME_H *)&TxBuf[TxSize];
    payload[0] = 0; // ACKSEQ
    puCH = (union CH *)&payload[1];
    encode_channel_double(pCtrl, puCH, BGNET_CHN_00_NODERESET, BGN_FMT_08, pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.value.dValue, 0);
    len_msg =  bgnet_encode_frame(pFTx, ucTO, ucFROM, BGNET_CMD_E9_QUAD_ACAL_DAT, payload, 5);
    TxSize = TxSize + len_msg;

    // register channel written
    pCtrl->CHN[puCH->s.CH].iNewChannel = BGN_NEW_SET;
    pCtrl->CHN[puCH->s.CH].x00.ucTO = pFTx->TO;
    pCtrl->CHN[puCH->s.CH].x00.ucFROM = pFTx->FROM;
    pCtrl->CHN[puCH->s.CH].x00.ucCMD = pFTx->CMD;
    memcpy(pCtrl->CHN[puCH->s.CH].x00.ucRAW, puCH->ucRAW, 4);
    decode_channel_string(pCtrl->CHN[puCH->s.CH].x00.cValue, puCH);
    CounterUp(&pCtrl->CHN[puCH->s.CH].x00.CounterRx, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);

    // register command send
    memcpy(pCtrl->CMD[BGNET_CMD_CE_PILOT_DMPDAT].Tx.ucFrame, pFTx, len_msg);
    CounterUp(&pCtrl->CMD[BGNET_CMD_CE_PILOT_DMPDAT].CounterTx, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
    memcpy(pCtrl->DEV[pFTx->TO].Tx.ucFrame, pFTx, len_msg);
    CounterUp(&pCtrl->DEV[pFTx->TO].CounterTx, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);

    // SPEED/SETUP/#6 DR (reset Dead Reckoning values, nothing to initialize)

    // put it for transmission
    pthread_mutex_lock(&g_data_mutex);
    memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &TxBuf, TxSize); // put output message in write buffer
    g_serial_data_lenTx = g_serial_data_lenTx + TxSize; // increment byte counter in TX buffer
    pthread_mutex_unlock(&g_data_mutex);

    return(TxSize);
}

// this one usually sends 8 channels
size_t makeSPEEDresponse(BGNET_CTRL *pCtrl, unsigned char *payload) {
    size_t r = 0;
    unsigned char c;
    union CH *puCH;

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_41_STW;
    r = r + encode_channel_double(pCtrl, puCH, c, pCtrl->pCHNDT[c].ucFMT, pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.value.dValue + pCtrl->BGNET.SPEED.STW.value.dValue * pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.value.dValue, 0);

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_42_STWRAW;
    r = r + encode_channel_double(pCtrl, puCH, c, pCtrl->pCHNDT[c].ucFMT, pCtrl->BGNET.SPEED.STW.value.dValue, 0);

    // the next 3 values are 'rotated' by the QUAD: CD every other time, 64/65 every 4'th time
    // but that is too complex at this moment. Later i will build a rotating scheme.

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_CD_STORLOG;
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uL[0] = htonl(pCtrl->BGNET.SPEED.StoredLogNM.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_64_SPEEDAVG;
    r = r + encode_channel_double(pCtrl, puCH, c, pCtrl->pCHNDT[c].ucFMT, pCtrl->BGNET.SPEED.SpeedAVG.value.dValue, 0);

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_65_SPEEDMAX;
    r = r + encode_channel_double(pCtrl, puCH, c, pCtrl->pCHNDT[c].ucFMT, pCtrl->BGNET.SPEED.SpeedMAX.value.dValue, 0);

    // the next values are always present 

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_81_DRD;
    r = r + encode_channel_double(pCtrl, puCH, c, pCtrl->pCHNDT[c].ucFMT, pCtrl->BGNET.SPEED.DRD.value.dValue, 0);
 
    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_CF_TRIPLOG;
    r = r + encode_channel_double(pCtrl, puCH, c, pCtrl->pCHNDT[c].ucFMT, pCtrl->BGNET.SPEED.TripLogNM.value.dValue, 0);

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_D3_DRC;
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(pCtrl->BGNET.SPEED.DRC.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_20_SEATEMPQ;
    r = r + encode_channel_double(pCtrl, puCH, c, pCtrl->pCHNDT[c].ucFMT, pCtrl->BGNET.SPEED.Temperature.value.dValue, 0);

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_75_TIMER;
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    tTimerCalc(pCtrl, &puCH->s.cv.uB[0], &puCH->s.cv.uB[1], &puCH->s.cv.uB[2]); // HH:MM:SS
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    return(r);
}
// this one usually sends 4 channels
size_t makeDEPTHresponse(BGNET_CTRL *pCtrl, unsigned char *payload) {
    size_t r = 0, x;
    unsigned char c;
    union CH *puCH;

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_31_ECHO_SNR;
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uC[0] = BGNET_ASC_NULL; 
    puCH->s.cv.uC[1] = BGNET_ASC_SPACE; 
    puCH->s.cv.uC[2] = BGNET_ASC_A; // A meas 'perfect signal'
    puCH->s.cv.uC[3] = BGNET_ASC_SPACE;
    r = r + bgnet_sizeof_channel(pCtrl, puCH);
    
    // the next 3 values are 'rotated' by the QUAD: FE every other time, FC/FD every 4'th time
    // but that is too complex at this moment. Later i will build a rotating scheme.

    puCH = (union CH *)&(payload[r]);
    x = ALARM_CHN(pCtrl, puCH, &pCtrl->BGNET.DEPTH.ALARM.SHAL, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
    if (x == 0) {
        c = BGNET_CHN_FC_SHALLOWALARM;
        puCH->s.CH = c;
        if (pCtrl->BGNET.DEPTH.SETUP.SA_STAT.value.nByte == 0) {
            puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
            puCH->s.cv.uC[0] = BGNET_ASC_NULL; 
            puCH->s.cv.uC[1] = BGNET_ASC_O; 
            puCH->s.cv.uC[2] = BGNET_ASC_F; 
            puCH->s.cv.uC[3] = BGNET_ASC_F;
            r = r + bgnet_sizeof_channel(pCtrl, puCH);
        } else {
            r = r + encode_channel_double(pCtrl, puCH, c, BGN_FMT_01, pCtrl->BGNET.DEPTH.SETUP.SA_M.value.dValue, 0);
        }
    } else {
        // alarm data was worked into channel
        r = r + x;
    }

    puCH = (union CH *)&(payload[r]);
    x = ALARM_CHN(pCtrl, puCH, &pCtrl->BGNET.DEPTH.ALARM.DEEP, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
    if (x ==0) {
        c = BGNET_CHN_FD_DEEPALARM;
        puCH->s.CH = c;
        if (pCtrl->BGNET.DEPTH.SETUP.DA_STAT.value.nByte == 0) {
            puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
            puCH->s.cv.uC[0] = BGNET_ASC_NULL; 
            puCH->s.cv.uC[1] = BGNET_ASC_O; 
            puCH->s.cv.uC[2] = BGNET_ASC_F; 
            puCH->s.cv.uC[3] = BGNET_ASC_F;
            r = r + bgnet_sizeof_channel(pCtrl, puCH);
        } else {
            r = r + encode_channel_double(pCtrl, puCH, c, BGN_FMT_01, pCtrl->BGNET.DEPTH.SETUP.DA_M.value.dValue, 0);
        }
    } else {
        // alarm data was worked into channel
        r = r + x;
    }

    puCH = (union CH *)&(payload[r]);
    x = ALARM_CHN(pCtrl, puCH, &pCtrl->BGNET.DEPTH.ALARM.ANCH, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
    if (x == 0) {
        c = BGNET_CHN_FE_ANCHORALARM;
        puCH->s.CH = c;
        if (pCtrl->BGNET.DEPTH.SETUP.AA_STAT.value.nByte == 0) {
            puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
            puCH->s.cv.uC[0] = BGNET_ASC_NULL; // OFF
            puCH->s.cv.uC[1] = BGNET_ASC_O; 
            puCH->s.cv.uC[2] = BGNET_ASC_F; 
            puCH->s.cv.uC[3] = BGNET_ASC_F;
            r = r + bgnet_sizeof_channel(pCtrl, puCH);
        } else {
            // show alternating the SHALOW or DEEP settings from the alarm SETUP
            CounterUp(&pCtrl->BGNET.DEPTH.SETUP.AA_TOGGLE, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
            if ((pCtrl->BGNET.DEPTH.SETUP.AA_TOGGLE.iCounter % 2) == 0) {
                // Shallow Alarm (measured level - shallow limit), minimum depth
                r = r + encode_channel_double(pCtrl, puCH, c, BGN_FMT_01, pCtrl->BGNET.DEPTH.SETUP.AMA_M.value.dValue - pCtrl->BGNET.DEPTH.SETUP.ASA_M.value.dValue, 0);
            } else {
                // Deep Alarm (measured level + deep limit), maximum depth
                r = r + encode_channel_double(pCtrl, puCH, c, BGN_FMT_01, pCtrl->BGNET.DEPTH.SETUP.AMA_M.value.dValue + pCtrl->BGNET.DEPTH.SETUP.ADA_M.value.dValue, 0);
            }
        }
    } else {
        // alarm data was worked into channel
        r = r + x;
    }

    // the next values are always present 

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_FF_DEPTH_CAL;
    r = r + encode_channel_double(pCtrl, puCH, c, pCtrl->pCHNDT[c].ucFMT, pCtrl->BGNET.DEPTH.SETUP.CAL.value.dValue, 0);

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_C4_DEPTH;  // displayed on QUAD
    r = r + encode_channel_double(pCtrl, puCH, c, pCtrl->pCHNDT[c].ucFMT, pCtrl->BGNET.DEPTH.Depth_CAL.value.dValue, 0);

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_C1_DEPTHRAW;
    r = r + encode_channel_double(pCtrl, puCH, c, pCtrl->pCHNDT[c].ucFMT, pCtrl->BGNET.DEPTH.Depth_RAW.value.dValue, 0);

    return(r);
}

// the LONG one is broadcasting everything (including the SHORT channels)
// long:  FF 70 20 01 70 8D 48 00 78 50 88 04 05 4E 88 00 00 51 01 00 64 52 08 00 00 57 88 05 D7 59 01 00 89 7F 88 FD 59 9B
// the SHORT one is broadcasting only the channels needed by other WIND (repeater) nodes
// short: FF 70 0C 01 84 4E 88 00 00 51 01 00 96 52 08 00 00 92
// we always send the LONG frame
size_t makeWINDresponseLONG(BGNET_CTRL *pCtrl, unsigned char *payload) {
    size_t r = 0;
    unsigned char c;
    union CH *puCH;

    // AWSRAW is read from the sensor and calibrated via WIND/SETUP parameter
    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_4E_AWSRAW;
    r = r + encode_channel_double(pCtrl, puCH, c, pCtrl->pCHNDT[c].ucFMT, pCtrl->BGNET.WIND.AWS.value.dValue + 0.43, 0);

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_50_AWS;  // shown on DATA and WIND
    r = r + encode_channel_double(pCtrl, puCH, c, pCtrl->pCHNDT[c].ucFMT, pCtrl->BGNET.WIND.AWS.value.dValue, 0);

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_51_AWA; // -180/+180 degrees shown on WIND DIAL
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = (int16_t)htons(pCtrl->BGNET.WIND.AWA.value.nLong);  // signed -180/+180
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    // AWARAW is read from the sensor and calibrated via WIND/SETUP parameter
    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_52_AWARAW; // 0/360 degrees
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(pCtrl->BGNET.WIND.AWA.value.nLong + 180 - 3);  // unsigned 0/+360
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_8D_BATTERY;  // shown on WIND (small LCD, button <VOLTS>)
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(DTOIROUND(pCtrl->BGNET.WIND.BATTERY.value.nLong)); // Volts x 10
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    // TWA, TWS and VMG are calculated by WIND, provided the WIND node receives STW from SPEED or QUAD

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_57_TWS; // shown on DATA and WIND
    r = r + encode_channel_double(pCtrl, puCH, c, pCtrl->pCHNDT[c].ucFMT, pCtrl->BGNET.WIND.TWS.value.dValue, 0);

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_59_TWA;
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = (int16_t)htons(pCtrl->BGNET.WIND.TWA.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_7F_VMG;
    r = r + encode_channel_double(pCtrl, puCH, c, pCtrl->pCHNDT[c].ucFMT, pCtrl->BGNET.WIND.VMG.value.dValue, 0);

    return(r);
}

unsigned char getAP_MODE(BGNET_CTRL *pCtrl) {
    unsigned char r;
    if (pCtrl->BGNET.ACP.Joystick.value.nByte == BGN_JOYSTICK_OFF) {
        r = pCtrl->BGNET.ACP.MODE.value.nByte;  // COMP/NAV/VANE
    } else {
        r = (pCtrl->BGNET.ACP.MODE.value.nByte & ~BGN_AP_MODE2_MASK_MODE) | BGN_AP_MODE2_JOYSTICK;  // JOYSTCK
    }
    return(r);
}

size_t makeACPresponseSTATUSONLY(BGNET_CTRL *pCtrl, unsigned char *payload) {
    size_t r = 0;
    unsigned char c;
    union CH *puCH;

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_B5_AP_STATUS;
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uB[0] = pCtrl->BGNET.ACP.STATE.value.nByte; // AUTO/STBY
    puCH->s.cv.uB[1] = getAP_MODE(pCtrl);  // COMP/NAV/VANE/JOYSTICK
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    switch (pCtrl->BGNET.ACP.STATE.value.nByte & BGN_AP_MODE1_MASK_STATE) {
        // if we are in AUTO, also transmit the CTS
        case BGN_AP_MODE1_AUTO:
            switch (pCtrl->BGNET.ACP.MODE.value.nByte & BGN_AP_MODE2_MASK_MODE) {
                case BGN_AP_MODE2_VANE:
                    // VANE mode => show heading
                    puCH = (union CH *)&(payload[r]);
                    c = BGNET_CHN_A6_AP_TARGET;
                    puCH->s.CH = c;
                    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
                    puCH->s.cv.uW[0] = htons(pCtrl->BGNET.ACP.Heading.value.nLong);
                    r = r + bgnet_sizeof_channel(pCtrl, puCH);
                    break;
                default:
                    // COMPASS and NAV mode => show CTS
                    puCH = (union CH *)&(payload[r]);
                    c = BGNET_CHN_A6_AP_TARGET;
                    puCH->s.CH = c;
                    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
                    puCH->s.cv.uW[0] = htons(pCtrl->BGNET.ACP.CTS.value.nLong);
                    r = r + bgnet_sizeof_channel(pCtrl, puCH);
                    break;
            }

            puCH = (union CH *)&(payload[r]);
            c = BGNET_CHN_53_TARGET_AWA;
            puCH->s.CH = c;
            puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
            puCH->s.cv.sW[0] = (int16_t)htons(pCtrl->BGNET.ACP.TARGET_AWA.value.nLong); // range -180/+180
            r = r + bgnet_sizeof_channel(pCtrl, puCH);

            break;
    }

    return(r);
}

// this response is meant to give the PILOT node sufficient information
// to display. Keep this small with frequent update 
size_t makeACPresponseMINI(BGNET_CTRL *pCtrl, unsigned char *payload) {
    size_t r = 0;
    unsigned char c;
    union CH *puCH;

    r = makeACPresponseSTATUSONLY(pCtrl, payload);

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_0B_RUDDER;
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(pCtrl->BGNET.ACP.Rudder.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    // when in STBY, the display shows the HEADING (compass reading)
    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_49_HEADING;
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uW[0] = htons(pCtrl->BGNET.ACP.Heading.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    return(r);
}

// handle WA and OC alarm 
size_t makeACPresponseALARM(BGNET_CTRL *pCtrl, unsigned char *payload) {
    size_t r = 0;
    union CH *puCH;

    puCH = (union CH *)&(payload[r]);
    r = r + ALARM_CHN(pCtrl, puCH, &pCtrl->BGNET.PILOT.ALARM.WA, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);

    puCH = (union CH *)&(payload[r]);
    r = r + ALARM_CHN(pCtrl, puCH, &pCtrl->BGNET.PILOT.ALARM.OC, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);

    return(r);
}

size_t makePILOTresponseNAV(BGNET_CTRL *pCtrl, unsigned char *payload) {
    size_t r = 0;
    unsigned char c;
    union CH *puCH;

    // Appearently, PILOT transmits 0xFB/WPID when received navigation input via NMEA and also includes the WPID.
    // But.... when we transmit navigation data (XTE, WPD, WPD and WPID) over the network, PILOT displays XTE, WPD and WPB
    // but NOT WPID.
    // It looks like PILOT detects a 'next leg' by receiving a different WPID. It appears that PILOT only accepts the WPID
    // from the NMEA input, but ignores it when receiving via BGNET.
    // I have been doing some experiments by simulating a '2nd PILOT', making the pilot to split NAV into NAV1 and NAV2, but
    // sofar i have not been able to make the PILOT accept navigation data via the BGNET.
    // 
    // Parculiar observation:
    // When we transmit the 0xFB/WPID channel, the specifications tells us that it displays the first 4 characters of the WPID
    // received from the NMEA/RMB sentence. NMEA has no field limit, but has a maximum of 82 characters for the whole sentence
    // However... when the 0xFB/WPID channel contains more that 2 displayable characters, QUAD starts to beep. Funny, but if we use
    // 2 characters + 2 spaces QUAD seems OK. Also 4 characters and the 3rd one being a space also seems OK.
    // For the timebeing i will only use the first 2 characters to avoid QUAD issues.
    //
    // i will put in some more efforts to emulate a 2nd PILOT on the network and hopefully make PILOT accept waypoint information
    // via the NETWORK.

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_FB_WPID;
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.FMT = BGN_FMT_0A;
    switch (strlen(pCtrl->BGNET.PILOT.NMEA.RMB.WPID.value.cString)) {
        case 0:
            puCH->s.cv.uB[0] = BGNET_ASC_QUEST;
            puCH->s.cv.uB[1] = BGNET_ASC_QUEST;
            puCH->s.cv.uB[2] = BGNET_ASC_SPACE;
            puCH->s.cv.uB[3] = BGNET_ASC_SPACE;
            break;
        case 1:
            puCH->s.cv.uB[0] = pCtrl->BGNET.PILOT.NMEA.RMB.WPID.value.cString[0];
            puCH->s.cv.uB[1] = BGNET_ASC_SPACE;
            puCH->s.cv.uB[2] = BGNET_ASC_SPACE;
            puCH->s.cv.uB[3] = BGNET_ASC_SPACE;
            break;
        case 2:
            puCH->s.cv.uB[0] = pCtrl->BGNET.PILOT.NMEA.RMB.WPID.value.cString[0];
            puCH->s.cv.uB[1] = pCtrl->BGNET.PILOT.NMEA.RMB.WPID.value.cString[1];
            puCH->s.cv.uB[2] = BGNET_ASC_SPACE;
            puCH->s.cv.uB[3] = BGNET_ASC_SPACE;
            break;
        case 3:
            puCH->s.cv.uB[0] = pCtrl->BGNET.PILOT.NMEA.RMB.WPID.value.cString[0];
            puCH->s.cv.uB[1] = pCtrl->BGNET.PILOT.NMEA.RMB.WPID.value.cString[1];
            puCH->s.cv.uB[2] = BGNET_ASC_SPACE;
            puCH->s.cv.uB[3] = BGNET_ASC_SPACE;
            break;
        default:
            // warning: if there are 'non-standard characters in this channel, QUAD is going to beep
            puCH->s.cv.uB[0] = pCtrl->BGNET.PILOT.NMEA.RMB.WPID.value.cString[0];
            puCH->s.cv.uB[1] = pCtrl->BGNET.PILOT.NMEA.RMB.WPID.value.cString[1];
            puCH->s.cv.uB[2] = BGNET_ASC_SPACE;
            puCH->s.cv.uB[3] = BGNET_ASC_SPACE;
            break;
    }
    r = r + bgnet_sizeof_channel(pCtrl, puCH);    

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_DE_WPB;
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(pCtrl->BGNET.PILOT.NMEA.RMB.WPB.value.nLong);
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_E8_WPD;
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.sW[0] = htons(DTOIROUND(pCtrl->BGNET.PILOT.NMEA.RMB.WPD.value.dValue * 100.0));
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    puCH = (union CH *)&(payload[r]);
    c = BGNET_CHN_EE_XTE;
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    // always shows 2 digits <1.23 or >1.23. any value > 9.99 is shown as 9.99
    // (if you are soo far off-track, it does not matter if it's 9.99, 15.9 or 2.3 NM)
    // do not use encode_channel_double(() with autoscaling does not work here
    // negative is displayed as "<0.12" (XTE to port, drifted to the left, steer right), positive value 
    // is displayed as ">0.12" (XTE to starboard, drifted to the right, steer left)
    puCH->s.cv.sW[0] = htons(DTOIROUND(pCtrl->BGNET.PILOT.NMEA.RMB.XTE.value.dValue * 100.0));
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    return(r);
}

// timer channel (FMT_03, channel 0x75 only)
union CH *encode_value_timer(union CH *pCH, unsigned char ucCH, unsigned char ucFMT, uint8_t HH, uint8_t MM, uint8_t SS) {
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
union CH *encode_value_ascii(union CH *pCH, unsigned char ucCH, unsigned char ucFMT, char *pcText) {
    pCH->s.CH = ucCH;
    pCH->s.FMT = ucFMT;
    switch(ucFMT) {
        case BGN_FMT_04:
            pCH->s.cv.uB[0] = BGNET_ASC_NULL;
            pCH->s.cv.uB[1] = pcText[0];
            pCH->s.cv.uB[2] = pcText[1];
            pCH->s.cv.uB[3] = pcText[2];
            break;
    }
    return(pCH);
}

size_t makeRESPresponse(BGNET_CTRL *pCtrl, unsigned char *payload) {
    size_t r = 0;
    unsigned char c;
    union CH *puCH;

    c = BGNET_CHN_A6_AP_TARGET; // RESP selector
    puCH = (union CH *)&(payload[r]);
    puCH->s.CH = c;
    puCH->s.FMT = pCtrl->pCHNDT[c].ucFMT;
    puCH->s.cv.uB[0] = 0;
    puCH->s.cv.uB[1] = pCtrl->BGNET.PILOT.SETUP.RESP.value.nByte;
    r = r + bgnet_sizeof_channel(pCtrl, puCH);

    return (r);
}

// This is the "device processor", called from the timer thread
// We scan through the data stored by the frame decoder.
// If we want to generate some output for transmission, put the frame in the transmission buffer
// (at this moment we see no need to do such thing)
size_t bgnet_process(BGNET_CTRL *pCtrl, unsigned char *buffer, size_t size) {
    int i;
    size_t r;
    uint8_t ebs_sel;
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

    // update effective boatspeed (used for navigation purposes)
    ebs_sel = EffectiveBoatSpeed(pCtrl, &pCtrl->BGNET.ACP.EBS);
    //log_debug("EBS_SEL= %d (%d), EBS= %f", ebs_sel, pCtrl->BGNET.ACP.EBS_SEL.value.nByte, pCtrl->BGNET.ACP.EBS.value.dValue);
    if (ebs_sel == pCtrl->BGNET.ACP.EBS_SEL.value.nByte) {
        // no change, no update
    } else {
        pCtrl->BGNET.ACP.EBS_SEL.value.nByte = ebs_sel;
        pCtrl->BGNET.ACP.EBS_SEL.source = BGN_SOURCE_ACP;
        CounterUp(&pCtrl->BGNET.ACP.EBS_SEL.Counter, t);
    }

    // here we will output our DATA commands, simulating the WIND and QUAD nodes.
    // note: using the routine 'register_DATA_frame' these DATA commands will appear as 'Received', altough
    // they are effectively 'Transmitted'. I do not see the need to change that. 
    if (g_serial_mode == BGN_SERIAL_MODE_RW) {
#define BGNET_ENABLE_QUAD_TX
#ifdef BGNET_ENABLE_QUAD_TX

        size_t len_msg;
        size_t len_pl;
        unsigned char payload[BGNET_FRAME_PAYLOAD_MAX];
        FRAME_H FTx, *pFTx;
        pFTx = &FTx;

        len_pl = makeDEPTHresponse(pCtrl, payload);
        len_msg = bgnet_encode_frame(pFTx, BGNET_TYPE_ALL, BGNET_TYPE_NETQUAD, BGNET_CMD_01_DATA, payload, len_pl);
        register_DATA_frame(pCtrl, pFTx);
        // put it for transmission
        memcpy(&buffer[r], pFTx, len_msg); // put output message in write buffer
        r = r + len_msg; // increment byte counter in TX buffer

        // update statistics
        memcpy(&pCtrl->CMD[BGNET_CMD_01_DATA].Tx.ucFrame, pFTx, len_msg);
        CounterUp(&pCtrl->CMD[BGNET_CMD_01_DATA].CounterTx, t);
        memcpy(pCtrl->DEV[pFTx->FROM].Tx.ucFrame, pFTx, len_msg);
        CounterUp(&pCtrl->DEV[pFTx->FROM].CounterTx, t);

        len_pl = makeSPEEDresponse(pCtrl, payload);
        len_msg = bgnet_encode_frame(pFTx,BGNET_TYPE_ALL, BGNET_TYPE_NETQUAD, BGNET_CMD_01_DATA, payload, len_pl);
        register_DATA_frame(pCtrl, pFTx);        
        // put it for transmission
        memcpy(&buffer[r], pFTx, len_msg); // put output message in write buffer
        r = r + len_msg; // increment byte counter in TX buffer

        // update statistics
        memcpy(&pCtrl->CMD[BGNET_CMD_01_DATA].Tx.ucFrame, pFTx, len_msg);
        CounterUp(&pCtrl->CMD[BGNET_CMD_01_DATA].CounterTx, t);
        memcpy(pCtrl->DEV[pFTx->FROM].Tx.ucFrame, pFTx, len_msg);
        CounterUp(&pCtrl->DEV[pFTx->FROM].CounterTx, t);
#endif

        len_pl = makeACPresponseALARM(pCtrl, payload);
        len_msg = bgnet_encode_frame(pFTx,BGNET_TYPE_ALL, BGNET_TYPE_NETACP, BGNET_CMD_01_DATA, payload, len_pl);
        register_DATA_frame(pCtrl, pFTx);        
        // put it for transmission
        memcpy(&buffer[r], pFTx, len_msg); // put output message in write buffer
        r = r + len_msg; // increment byte counter in TX buffer

        // update statistics
        memcpy(&pCtrl->CMD[BGNET_CMD_01_DATA].Tx.ucFrame, pFTx, len_msg);
        CounterUp(&pCtrl->CMD[BGNET_CMD_01_DATA].CounterTx, t);
        memcpy(pCtrl->DEV[pFTx->FROM].Tx.ucFrame, pFTx, len_msg);
        CounterUp(&pCtrl->DEV[pFTx->FROM].CounterTx, t);

#define BGNET_ENABLE_WIND_TX
#ifdef BGNET_ENABLE_WIND_TX

        len_pl = makeWINDresponseLONG(pCtrl, payload);
        len_msg = bgnet_encode_frame(pFTx,BGNET_TYPE_ALL, BGNET_TYPE_NETWIND, BGNET_CMD_01_DATA, payload, len_pl);
        register_DATA_frame(pCtrl, pFTx);        
        // put it for transmission
        memcpy(&buffer[r], pFTx, len_msg); // put output message in write buffer
        r = r + len_msg; // increment byte counter in TX buffer

        // update statistics
        memcpy(&pCtrl->CMD[BGNET_CMD_01_DATA].Tx.ucFrame, pFTx, len_msg);
        CounterUp(&pCtrl->CMD[BGNET_CMD_01_DATA].CounterTx, t);
        memcpy(pCtrl->DEV[pFTx->FROM].Tx.ucFrame, pFTx, len_msg);
        CounterUp(&pCtrl->DEV[pFTx->FROM].CounterTx, t);
#endif
        // we want to be able to enable/disable navigation data in our reverse engineering efforts
        switch (pCtrl->BGNET.PILOT.ucNAV_SIM & BGNET_TYPE_MASK) {
            case (BGNET_TYPE_NETPILOT & BGNET_TYPE_MASK):
                payload[0] = BGNET_CHN_98_NMEA_ACTIVE;
                payload[1] = BGN_FMT_00; // is this gets bigger than 0x0F, QUAD starts to beep
                len_pl = 2;
                len_msg = bgnet_encode_frame(pFTx, BGNET_TYPE_ALL, pCtrl->BGNET.PILOT.ucNAV_SIM, BGNET_CMD_01_DATA, payload, len_pl);
                register_DATA_frame(pCtrl, pFTx);
                // put it for transmission
                memcpy(&buffer[r], pFTx, len_msg); // put output message in write buffer
                r = r + len_msg; // increment byte counter in TX buffer

                // update statistics
                memcpy(&pCtrl->CMD[BGNET_CMD_01_DATA].Tx.ucFrame, pFTx, len_msg);
                CounterUp(&pCtrl->CMD[BGNET_CMD_01_DATA].CounterTx, t);
                memcpy(pCtrl->DEV[pFTx->FROM].Tx.ucFrame, pFTx, len_msg);
                CounterUp(&pCtrl->DEV[pFTx->FROM].CounterTx, t);

                len_pl= makePILOTresponseNAV(pCtrl, payload);
                len_msg = bgnet_encode_frame(pFTx, BGNET_TYPE_ALL, pCtrl->BGNET.PILOT.ucNAV_SIM, BGNET_CMD_01_DATA, payload, len_pl);
                register_DATA_frame(pCtrl, pFTx);
                // put it for transmission
                memcpy(&buffer[r], pFTx, len_msg); // put output message in write buffer
                r = r + len_msg; // increment byte counter in TX buffer

                // update statistics
                memcpy(&pCtrl->CMD[BGNET_CMD_01_DATA].Tx.ucFrame, pFTx, len_msg);
                CounterUp(&pCtrl->CMD[BGNET_CMD_01_DATA].CounterTx, t);
                memcpy(pCtrl->DEV[pFTx->FROM].Tx.ucFrame, pFTx, len_msg);
                CounterUp(&pCtrl->DEV[pFTx->FROM].CounterTx, t);
                break;
        }
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
    // if you want to send data, return the number of bytes in the output buffer
    return(r);
};

// returns the length of a frame/subframe
size_t bgnet_sizeof_frame(FRAME_H *pF) {
    size_t r;
    if (pF->PL == 0) {
        r = BGNET_FRAME_HEAD;
    } else {
        r = BGNET_FRAME_HEAD + pF->PL + BGNET_FRAME_TAIL_CS;
    }
    return(r);
}

// returns the length of a channel block; it all depends on the FMT specifier (in the received channel)
size_t bgnet_sizeof_channel(BGNET_CTRL *pCtrl, union CH *pCH) {
    int r,i,n;
    unsigned char ucFMT;
    char asciibuf[32];
    // we have some odd channels that do not follow the FMT-sizing rule
    switch (pCH->s.CH) {
        case BGNET_CHN_98_NMEA_ACTIVE:
            r = 2;
            break;
        default:
            // get FMT byte from target
            ucFMT = pCH->s.FMT;
            // make FMT into DataType, get length from Definition Table
            r = pCtrl->pFMTDT[(ucFMT & BGNET_FMT_DT)].iLen;
            if (r > 0) {
                // fixed length, use it as-is 
            } else {
                if (r == 0) {
                    // variable length, look for the null byte
                    n = -1;
                    for (i=0;i<12;i++) { 
                        if (pCH->s.cv.uC[i] == BGNET_ASC_NULL) {
                            n = i;
                            break;
                        }
                    }
                    // n can be anything between 0-11 when a null-terminator was found
                    if (n < 0) {
                        // null-terminator was not found
                        r = 4; // just a guess
                        log_error("FMT specifier 0x%02X, invalid length %d, no terminator found", ucFMT & BGNET_FMT_DT, r);
                    } else {
                        r = 2 + n + 1; // 2 bytes header, n characters, 1 null-terminator
                    }
                } else {
                    // the value is unknown, i just make a guess. 4 is better than zero or -1
                    r = 4; // just a guess
                    byte2ascii(asciibuf, sizeof(asciibuf), (unsigned char *)pCH, r, BYTE2ASCII_SPACED);
                    log_error("Channel %s, invalid FMT specifier", asciibuf);
                }
            }
            break;
    }
    // note: this return is used to advance though a frame/subframe. if this value is not positive,
    // we will never complete the parsing (and create chaos).
    return(r);
}

// copy command frame into a ascii buffer (for display purposes). It's used for logging purposes, 
// but also to paint frames into the webpages
// it converts to a string like "0F00AC", "0F 00 AC" or "0x0F 0x00 0x0AC". Realise that the produced
// string my be much longer that the original byte sequence.
char *CommandToAscii(char *asciibuf, size_t size, FRAME_H *pF) {
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
            byte2ascii(asciibuf, size, (unsigned char *)pF, y, BYTE2ASCII_SPACED);
        } else {
            // we use a 7-character separation string: " /.../ " to avoid excessive long output
            // we also copy the first 10 (5 header with CS + 5 first data) and the tail (5 last data + CS = 6 bytes)
            // 6 bytes tail = 18 ascii characters
            n = (sizeof(cTail) - strlen(BGN_CMD_ASCII_SEP))/BYTE2ASCII_SPACED; // we have ascii space for 'n' databytes
            byte2ascii(asciibuf, size, (unsigned char *)pF, n-BGN_CMD_DATA_TAIL, BYTE2ASCII_SPACED);
            strcat(asciibuf, BGN_CMD_ASCII_SEP); // append separator
            byte2ascii(cTail, sizeof(cTail), ((unsigned char *)(pF))+y-BGN_CMD_DATA_TAIL, BGN_CMD_DATA_TAIL, BYTE2ASCII_SPACED);
            strcat(asciibuf, cTail);
        }
    }
    return(asciibuf);
}

// sometimes we send a DATA frame, but that is not registered (because we not process our own commands)
// using this function we can make them into our status page. we only expect single-frame data
size_t register_DATA_frame(BGNET_CTRL *pCtrl, FRAME_H *pF) {
    unsigned char *pucP; // character pointer inside the payload
    union CH *pCH;
    size_t y, z;
    time_t t;

    // we expect only a single DATA command to have one (or more) channels
    time(&t);
    z = 0;
    if (pF->CMD == BGNET_CMD_01_DATA ) {
        pucP = pF->payload;
        while (pucP < (pF->payload + pF->PL)) {
            pCH = (union CH *)pucP;
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
int decode_command_DATA(BGNET_CTRL *pCtrl, FRAME_H *pF) {
    int y,z;
    unsigned char *pBuf;
    union CH *pCH;
    time_t t;  // used to timestamp incoming channel

    pBuf = pF->payload;
    z = 0; // number of channels in this command
    time(&t);

    // scan through frame/subframe
    while (pBuf < (pF->payload + pF->PL)) {
        pCH = (union CH *)(pBuf + BGNET_CHN_CID);
        pCtrl->CHN[pCH->s.CH].x01.ucFROM = pF->FROM; // originator of the frame (that included this channel)

        // we need the channel length to advance to the next in this frame
        y = bgnet_sizeof_channel(pCtrl, pCH);

        // check if most recent channel (stored previous time) and the new channel (in the received frame) are equal
        if (memcpy(((union CH *)pCtrl->CHN[pCH->s.CH].x01.ucRAW), pCH, y) == 0) {
            // no change, do no process this channel
        } else {

            // decode channel, get displayed value (this is what we use in the webserver)
            decode_channel_string(pCtrl->CHN[pCH->s.CH].x01.cValue, pCH);
            // copy channel byte sequence (2 purposes: to display via the webserver and to compare new/present data)
            memcpy(pCtrl->CHN[pCH->s.CH].x01.ucRAW, pCH, y);

            pCtrl->CHN[pCH->s.CH].iNewChannel = BGN_NEW_SET; // signal this channel as new/changed
            CounterUp(&pCtrl->CHN[pCH->s.CH].x01.CounterRx, t);

            // sometimes we have to do something when some data comes in
            switch(pCH->s.CH) {
                case BGNET_CHN_0B_RUDDER:
                    pCtrl->BGNET.ACP.Rudder.value.nLong = (int16_t)(ntohs(pCH->s.cv.uW[0])); // make it signed
                    pCtrl->BGNET.ACP.Rudder.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->BGNET.ACP.Rudder.Counter, t);
                    break;
                case BGNET_CHN_49_HEADING:
                    pCtrl->BGNET.ACP.Heading.value.nLong = ntohs(pCH->s.cv.uW[0]);
                    pCtrl->BGNET.ACP.Heading.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->BGNET.ACP.Heading.Counter, t);
                    break;
                case  BGNET_CHN_7F_VMG:
                    pCtrl->BGNET.WIND.VMG.value.dValue = decode_channel_double(pCH, 0);
                    pCtrl->BGNET.WIND.VMG.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->BGNET.WIND.VMG.Counter, t);
                    break;
                case BGNET_CHN_98_NMEA_ACTIVE:
                    if (pCtrl->BGNET.PILOT.NMEA.RMB.RMB_SEL.value.nByte == 0) {
                        pCtrl->BGNET.PILOT.NMEA.RMB.RMB_SEL.value.nByte = 1;
                        pCtrl->BGNET.PILOT.NMEA.RMB.RMB_SEL.source = BGN_SOURCE_BGNET;
                        CounterUp(&pCtrl->BGNET.PILOT.NMEA.RMB.RMB_SEL.Counter, t);                                                                
                    } else {
                        // is already ON
                    }
                    break;
                case BGNET_CHN_A6_AP_TARGET:
                    pCtrl->BGNET.ACP.CTS.value.nLong = ntohs(pCH->s.cv.uW[0]);
                    pCtrl->BGNET.ACP.CTS.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->BGNET.ACP.CTS.Counter, t);
                    break;
                case BGNET_CHN_C1_DEPTHRAW:
                    // we back-calculate the RAW DEPTH and ignore this channel
                    break;
                case BGNET_CHN_C4_DEPTH:
                    // on the NETWORK DEPTH is calibrated depth
                    pCtrl->BGNET.DEPTH.Depth_CAL.value.dValue = decode_channel_double(pCH, 0);
                    pCtrl->BGNET.DEPTH.Depth_CAL.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->BGNET.DEPTH.Depth_CAL.Counter, t);
                    // knowing the DEPTH CALIBRATION we can calulate the RAW DEPTH
                    pCtrl->BGNET.DEPTH.Depth_RAW.value.dValue = pCtrl->BGNET.DEPTH.Depth_CAL.value.dValue - pCtrl->BGNET.DEPTH.SETUP.CAL.value.dValue;
                    pCtrl->BGNET.DEPTH.Depth_RAW.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->BGNET.DEPTH.Depth_RAW.Counter, t);
                    break;
                case BGNET_CHN_B5_AP_STATUS:
                    // leave the fixed-bits inplace, only alter the selected STATE
                    pCtrl->BGNET.ACP.STATE.value.nByte = (pCtrl->BGNET.ACP.STATE.value.nByte & ~BGN_AP_MODE1_MASK_STATE) | (pCH->s.cv.uB[0] & BGN_AP_MODE1_MASK_STATE);  // get AUTO/STBY state into ACP
                    pCtrl->BGNET.ACP.STATE.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->BGNET.ACP.STATE.Counter, t);
                    pCtrl->BGNET.ACP.MODE.value.nByte  = (pCtrl->BGNET.ACP.MODE.value.nByte & ~BGN_AP_MODE2_MASK_MODE) | (pCH->s.cv.uB[1] & BGN_AP_MODE2_MASK_MODE);   // get COMP/NAV/VANE mode into ACP
                    pCtrl->BGNET.ACP.MODE.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->BGNET.ACP.MODE.Counter, t);
                    break;
                case BGNET_CHN_CD_STORLOG:
                    pCtrl->BGNET.SPEED.StoredLogNM.value.nLong = ntohl(pCH->s.cv.uL[0]);
                    pCtrl->BGNET.SPEED.StoredLogNM.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->BGNET.SPEED.StoredLogNM.Counter, t);
                    break;
                case BGNET_CHN_CF_TRIPLOG:
                    pCtrl->BGNET.SPEED.TripLogNM.value.dValue = decode_channel_double(pCH, 0) * BGNET_DIVISOR_TRIPLOG;
                    //pCtrl->BGNET.SPEED.TripLogNM.value.nLong = ntohs(pCH->s.cv.uW[0]);
                    pCtrl->BGNET.SPEED.TripLogNM.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->BGNET.SPEED.TripLogNM.Counter, t);
                    break;
                case BGNET_CHN_EA_COGMAG :
                    pCtrl->BGNET.PILOT.NMEA.VTG.COG.value.nLong = ntohs(pCH->s.cv.uW[0]);
                    pCtrl->BGNET.PILOT.NMEA.VTG.COG.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->BGNET.PILOT.NMEA.VTG.COG.Counter, t);
                    break;
                case BGNET_CHN_EB_SOG:
                    pCtrl->BGNET.PILOT.NMEA.VTG.SOG.value.dValue = decode_channel_double(pCH, 0);
                    pCtrl->BGNET.PILOT.NMEA.VTG.SOG.source = BGN_SOURCE_BGNET;
                    CounterUp(&pCtrl->BGNET.PILOT.NMEA.VTG.SOG.Counter, t);
                    if (pCtrl->BGNET.PILOT.NMEA.VTG.VTG_SEL.value.nByte == 0) {
                        pCtrl->BGNET.PILOT.NMEA.VTG.VTG_SEL.value.nByte = 1;
                        pCtrl->BGNET.PILOT.NMEA.VTG.VTG_SEL.source = BGN_SOURCE_BGNET;
                        CounterUp(&pCtrl->BGNET.PILOT.NMEA.VTG.VTG_SEL.Counter, t);                                                                
                    } else {
                        // is already ON
                    }
                    break;
            }
            if ((t-pCtrl->BGNET.PILOT.NMEA.RMB.RMB_SEL.Counter.tTimestamp) < 60) {
                // channel data is recently received
            } else {
                pCtrl->BGNET.PILOT.NMEA.RMB.RMB_SEL.value.nByte = 0;
                pCtrl->BGNET.PILOT.NMEA.RMB.RMB_SEL.source = BGN_SOURCE_BGNET;
                CounterUp(&pCtrl->BGNET.PILOT.NMEA.RMB.RMB_SEL.Counter, t);                                                                
            }


            // when Import/Export is enabled, do what has to be done
            switch(pCtrl->pCHNDT[pCH->s.CH].iImpExpEnable) {
                case BGNET_CHN_IMPEXP_NONE:
                    // value not designed to be import/exportable
                    break;
                case BGNET_CHN_IMPEXP_NMEA:
                    // because we are here processing channels from the BGN interface, we are going to export it
                    // (importing does not make sense because we already have it)
                    // if (later) NMEA has this data incoming, it will overwrite it (and we will not write it again)
                    // we do not need to lock/unlock it, as NMEA already does that and will cancel us out anyway
                    switch(pCH->s.CH) {
                        case BGNET_CHN_0B_RUDDER:
                            if ((g_n2kdata.boat.rudder.source == BGN_SOURCE_NMEA) | (g_n2kdata.boat.rudder.source == BGN_SOURCE_SIGNALK)) {
                                // is was already written by NMEA/SIGNALK interface; do not overwrite
                            } else {
                                // BGN sends rudder position in angle/degrees (signed, -45/+45). NMEA stores rudder
                                // position as radian. Multiply by PI/180 to get radians
                                g_n2kdata.boat.rudder.value.dValue = ITOD(pCtrl->BGNET.ACP.Rudder.value.nLong) * BGNET_PI/180;
                                g_n2kdata.boat.rudder.source = BGN_SOURCE_BGNET;
                                CounterUp(&g_n2kdata.boat.rudder.Counter, t);
                            }
                            break;
                    }
                    break;
                case BGNET_CHN_IMPEXP_SIGNALK:
                    // because we are here processing channels from the BGN interface, we are going to export it
                    // (importing does not make sense because we already have it)
                    // if (later) SIGNAL has this data incoming, it will overwrite it (and we will not write it again)
                    // we do not need to lock/unlock it, as SIGNAL already does that and will cancel us out anyway
                    switch(pCH->s.CH) {
                        case BGNET_CHN_0B_RUDDER:
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
        }
        pBuf = pBuf + y; // advance to next channel in this frame
        z++; // count channels in this frame
    }
    return(z); // return number of channels
}
// routine to advertise values that can be subscribed
void decode_command_LABEL(BGNET_CTRL *pCtrl, FRAME_H *pF) {
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
        strncpy(pCtrl->CHN[ucCHN].x02.cValue, (char *)&pF->payload[4], len);
        pCtrl->CHN[ucCHN].x02.cValue[len] = BGNET_ASC_NULL;
    } else {
        strcpy(pCtrl->CHN[ucCHN].x02.cValue, "???");
    }
    return;
}
// routine to register a channel for subscription
void decode_command_SUBSCRIBE(BGNET_CTRL *pCtrl, FRAME_H *pFRx) {
    unsigned char ucCHN;
    time_t t;
    int iIsDefault = 0, len_msg = 0, len_pl = 0;
    FRAME_H FTx, *pFTx = &FTx;
    unsigned char payload[BGNET_FRAME_PAYLOAD_MAX];

    time(&t);
    // we check the size because we have limited space in the CHN table to store the raw frame
    // if (for whatever reason) at some point we receive a longer frame than expected, i do not
    // what the risk overwriting other parts of our CHN table 
    if (pFRx->PL == 1) { // expect only 1 byte payload
        ucCHN = pFRx->payload[0]; // first payoad byte is the ChannelID
        pCtrl->CHN[ucCHN].x06.ucTO = pFRx->TO;
        pCtrl->CHN[ucCHN].x06.ucFROM = pFRx->FROM;
        CounterUp(&pCtrl->CHN[ucCHN].x06.CounterRx, t);
        memcpy(pCtrl->CHN[ucCHN].x06.ucRAW, pFRx, bgnet_sizeof_frame(pFRx));
        log_debug("    Ch=%02X, Subscribed by %02X on %02X", ucCHN, pFRx->FROM, pFRx->TO);
        // we will do only generate a reply if we are if RW mode
        if (g_serial_mode == BGN_SERIAL_MODE_RW) {
            // we only handle a subscription request from a NETDATA type node
            iIsDefault = 0;
            switch(pFRx->FROM & BGNET_TYPE_MASK) {
                case (BGNET_TYPE_NETDATA & BGNET_TYPE_MASK):
                    // byte 0 of the payload it the ChannelID the NETDATA node want
                    // we have grouped them together by the same FMT byte (at least what i think is a FMT byte)
                    payload[0] = ucCHN;
                    // bytes 1-3 vary by CHN
                    switch(ucCHN) {
                        // this seems channels that have 'text' related data
                        case BGNET_CHN_31_ECHO_SNR:
                            payload[1] = 0x00;
                            payload[2] = 0x01;
                            payload[3] = 0x00;
                            break;
                        case BGNET_CHN_51_AWA:
                        case BGNET_CHN_59_TWA:
                            payload[1] = 0x00;
                            payload[2] = 0x01;
                            payload[3] = 0x00;
                            break;
                        // this seems channels that have "text" 'distance' related data
                        case BGNET_CHN_20_SEATEMPQ:
                        case BGNET_CHN_42_STWRAW:
                        case BGNET_CHN_4A_DEPTHM:
                        case BGNET_CHN_50_AWS:
                        case BGNET_CHN_57_TWS:
                        case BGNET_CHN_7F_VMG:
                        case BGNET_CHN_8D_BATTERY:
                        case BGNET_CHN_C4_DEPTH:
                        case BGNET_CHN_CD_STORLOG:
                        case BGNET_CHN_CF_TRIPLOG:
                        case BGNET_CHN_FC_SHALLOWALARM:
                        case BGNET_CHN_FD_DEEPALARM:
                        case BGNET_CHN_FE_ANCHORALARM:
                            payload[1] = 0x40;
                            payload[2] = 0x01;
                            payload[3] = 0x00;
                            break;
                        // this seems channels that have "text" 'speed' related data
                        case BGNET_CHN_41_STW:
                        case BGNET_CHN_64_SPEEDAVG:
                        case BGNET_CHN_65_SPEEDMAX:
                        case BGNET_CHN_75_TIMER:
                        case BGNET_CHN_81_DRD:
                        case BGNET_CHN_D3_DRC:
                            payload[1] = 0x80;
                            payload[2] = 0x01;
                            payload[3] = 0x00;
                            break;
                        default:
                            iIsDefault = 1;
                            break;
                    }
                    // is we have not prepared our payload, skip it
                    if (iIsDefault == 1) {
                        // skip it
                    } else {
                        // bytes 4-N is the label (ascii characters)
                        strcpy((char *)&(payload[4]), pCtrl->pCHNDT[ucCHN].cLabel);
                        // payload length is 4 + text length
                        len_pl = 4 + strlen(pCtrl->pCHNDT[ucCHN].cLabel);
                        // assemble the frame
                        len_msg = bgnet_encode_frame(pFTx, BGNET_TYPE_ALL, BGNET_TYPE_NETACP, BGNET_CMD_02_LABEL, payload, len_pl);
                        // we do the same with our FTx as what we would do when we would have receved it
                        decode_command_LABEL(pCtrl, pFTx);
                        // register command send
                        memcpy(pCtrl->CMD[BGNET_CMD_02_LABEL].Tx.ucFrame, pFTx, len_msg);
                        CounterUp(&pCtrl->CMD[BGNET_CMD_02_LABEL].CounterTx, t);
                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                        // put it for transmission
                        pthread_mutex_lock(&g_data_mutex);
                        memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], pFTx, len_msg); // put output message in write buffer
                        g_serial_data_lenTx = g_serial_data_lenTx + len_msg; // increment byte counter in TX buffer
                        pthread_mutex_unlock(&g_data_mutex);
                    }
                    break;
            }
        } else {
            // if we are RO or passive we will leave it to the QUADt/SPEEDt/DEPTHt/WINDt
            // nodes to respond to the SUBSCRIPTION requests from the NETDATA
        }
    }
    return;
}

// decode the received buffer, possibly multiple subframes, return number of sub-frames
// NOTE: realise this routine is executed by the timer-thread. We do no see a particular reason the run it 'locked'
size_t bgnet_decode_frame(BGNET_CTRL *pCtrl, unsigned char *buffer, size_t len) {
    FRAME_H *pFRx;   // pointer to the RECEIVE frame
    FRAME_H *pFTx;   // pointer to the WRITE frame
    union CH *pCHRx; // pointer to the channel structure we RECEIVE (eighter a CMD-REQ or a CMD-DAT command)
    union CH *pCHTx; // pointer to the channel structure we WRITE

    //unsigned char cs; // checksum calculated
    unsigned char *pBuf; // pointer to start and end of buffer
    int RxSize, x, z, c;
    int iValue, bSend_CMDACK;
    time_t t;
    size_t y;
    // inside this routine we work with our local buffers to store generated frames as much as possible.
    unsigned char payload[BGNET_FRAME_PAYLOAD_MAX], TxBuf[1024];
    int len_pl, len_msg, TxSize;
    double dTemp;

    pBuf = buffer; // gets incremented as we go
    z = 0; // subframe counter

    // process (possible multiple) subframes in buffer
    while (pBuf < (buffer+len)) {
        // setup pointer to our RECEIVING frame
        pFRx = (FRAME_H *)pBuf;
        
        // initialize some variables
        time(&t);
        memset(payload, 0, sizeof(payload));
        memset(TxBuf, 0, sizeof(TxBuf));
        TxSize = 0;
        len_pl = 0;
        len_msg = 0;

        switch (g_serial_mode) {
            case BGN_SERIAL_MODE_RW:
                // in RW mode, do not log any DATA command (except what comes from PILOT devices)
                switch (pFRx->CMD) {
                    case BGNET_CMD_01_DATA:
                        switch (pFRx->FROM & BGNET_TYPE_MASK) {
                            case (BGNET_TYPE_NETPILOT & BGNET_TYPE_MASK):
                                log_debug_frame(pFRx);
                                break;
                        }
                        break;
                    default:
                        log_debug_frame(pFRx);
                        break;
                }
                break;
            default:
                // do not log some DATA/0x01 commands
                switch (pFRx->CMD) {
                    case BGNET_CMD_01_DATA:
                        switch (pFRx->FROM & BGNET_TYPE_MASK) {
                            case BGNET_TYPE_NETACP:
                            case BGNET_TYPE_NETQUAD:
                                // do not log
                                break;
                            case BGNET_TYPE_NETPILOT:
                            default:
                                log_debug_frame(pFRx);
                                break;
                        }
                        break;
                    default:
                        log_debug_frame(pFRx);
                        break;                    
                }
                break;
        }

        // sorry, but the code branch and switch/cases have become more complex than i ever envisaged. It was developped over
        // many hours of reverse-engineering. You do not know where you are heading and what approch would be a neat solution
        // some pseudo-code to give you a better understanding:
        //  if header CS OK?
        //      if no payload?
        //          yes, handle commands without payload (no frame, no frame CS)
        //      else
        //          if CS frame is OK?
        //              yes, handle commands with payload
        //              switch (command type)
        //                  case generic* commands
        //                  all others
        //                      switch (node type)
        //                          case WIND
        //                              handle specific* WIND commands
        //                          case QUAD
        //                              handle specific* QUAD commands
        //                          case PILOT
        //                              handle specific* PILOT commands
        //                      end switch
        //              end switch
        //          else
        //              discard this subframe (continue with possible 2nd subframe)
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
            if (pFRx->FROM == BGNET_TYPE_NETACP) {
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
                        case BGNET_CMD_0E_PILOT_HELLO:
                            // when we receive this message, we know that there is atleast 1 active PILOT on the network.
                            // We set the counter; timer thread counts it down and if this message is no longer received
                            // we (the ACP) know that there is no longer user-input possible because no PILOT is connected.
                            //log_info("  Command %02X (header only) KEEPALIVE by %02X %s", pFRx->CMD, pFRx->FROM, pCtrl->pDEVDT[x].sModelName);

                            // set the counter (count down done by timer thread)
                            //log_info("PILOTWD reset from %d to %d", pCtrl->BGNET.SERVICE.PILOTWD.value.nLong, BGNET_PILOTWD_SET);
                            pthread_mutex_lock(&g_data_mutex);
                            pCtrl->BGNET.SERVICE.PILOTWD.value.nLong = BGNET_PILOTWD_SET;
                            pCtrl->BGNET.SERVICE.PILOTWD.source = BGN_SOURCE_BGNET;
                            CounterUp(&pCtrl->BGNET.SERVICE.PILOTWD.Counter, t);
                            pthread_mutex_unlock(&g_data_mutex);

                            if (g_serial_mode == BGN_SERIAL_MODE_RO) {
                                // nothing to do
                            } else {
                                // this reply gets the PILOT out of the "No PILOT" state
                                pFTx = (FRAME_H *)&TxBuf[TxSize];
                                len_pl = makeACPresponseMINI(pCtrl, payload);
                                len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_01_DATA, payload, len_pl);
                                register_DATA_frame(pCtrl, pFTx);

                                memcpy(pCtrl->CMD[BGNET_CMD_01_DATA].Tx.ucFrame, pFTx, len_msg);
                                CounterUp(&pCtrl->CMD[BGNET_CMD_01_DATA].CounterTx, t);
                                memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                TxSize = TxSize + len_msg;

                                // put it for transmission
                                pthread_mutex_lock(&g_data_mutex);
                                memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &TxBuf, TxSize); // put output message in write buffer
                                g_serial_data_lenTx = g_serial_data_lenTx + TxSize; // increment byte counter in TX buffer
                                pthread_mutex_unlock(&g_data_mutex);
                            }
                            break;
                        case BGNET_CMD_0A_POWERUP:
                            CounterUp(&pCtrl->DEV[pFRx->FROM].PowerUp_Seen, t);
                            pCtrl->DEV[pFRx->FROM].PowerUp_InitSeen = 1;  // set 'PowerUp seen'
                            pCtrl->DEV[pFRx->FROM].PowerUp_InitDone = 0;  // reset 'Init done' (force init)
                            // when we receive this message one (or more) devices went though the startup process (ie. power was
                            // switch ON while we (the ACP) were still running.
                            // At this moment i see no reason to do something special at this occasion. Realise that most networks
                            // contain multiple units. You might expect a burst of these messages. Some units (like WIND) take some
                            // more time then others. Also units that are normally passive (not broadcasting channels, like DATA)
                            // will also send this message.
                            //log_info("  Command %02X (header only) POWERUP by %02X %s", pFRx->CMD, pFRx->FROM, pCtrl->pDEVDT[x].sModelName);
                            // At first i thought that resetting our own structure was a good idea. It is not.
                            // At startup, a lot of initial messages are being send (command 0x02, 0x06) and if we reset our structure
                            // we erase all of that in the process. Better not resetting.
                            //bgnet_initialize_reset(pCtrl);

                            // if this device was enabled for use
                            if (pCtrl->pDEVDT[bgnet_deviceindex(pCtrl, pFRx->FROM)].iEnable == 0) {
                                // sorry, never had this type of device in my hands and could not guarentee predictable outcome. forget it for the time being
                            } else {
                                // if powerup is complete, but initialisation not done
                                if (pCtrl->DEV[pFRx->FROM].PowerUp_InitSeen == 0) {
                                    // not a PowerUp CMD received, we do not know for sure the node is up-and-running
                                    // note: This situation may occur when the NETWORK is running and we (re)start our
                                    //       application/service. That is not a problem, because the nodes might have
                                    //       been initialized the previous time.
                                    //       if some changes were made to our application that require the nodes to
                                    //       reinitialize, the solution is to power-reset the NETWORK.
                                } else {
                                    if (pCtrl->DEV[pFRx->FROM].PowerUp_InitDone == 0) {
                                        // node needs initialisation by device type
                                        // note: the initialisation we do here is sending the settings from our variables to
                                        //       the nodes. Every devicetype has different set of settings.
                                        //       This is how we ensure the default settings from our application are set into
                                        //       the nodes memory.
                                        //       Realise that after 30 years the battery-backup of the nodes are not able to
                                        //       maintain the memory powered. They will fall-back to their internal defaults.
                                        //       So we write our settings out when we see a PowerUp command.
                                        switch (pCtrl->pDEVDT[bgnet_deviceindex(pCtrl, pFRx->FROM)].iDeviceType & BGNET_TYPE_MASK) {
                                            case (BGNET_TYPE_NETWIND & BGNET_TYPE_MASK):
                                                // initialize WIND
                                                log_debug("Initializing WIND");
                                                sendWINDSETUP(pCtrl, pFRx->FROM, BGNET_TYPE_NETACP);
                                                pCtrl->DEV[pFRx->FROM].PowerUp_InitDone = 1;
                                                break;
                                            case (BGNET_TYPE_NETQUAD & BGNET_TYPE_MASK):
                                                // initialize DEPTH & SPEED
                                                sendSPEEDSETUP(pCtrl, pFRx->FROM, BGNET_TYPE_NETACP);
                                                pCtrl->DEV[pFRx->FROM].PowerUp_InitDone = 1;
                                                break;
                                            case (BGNET_TYPE_NETPILOT & BGNET_TYPE_MASK):
                                                // initialize PILOT
                                                log_debug("Initializing PILOT");
                                                //pCtrl->DEV[pFRx->FROM].PowerUp_InitDone = 1;
                                                break;
                                            default:
                                                // other nodes (like DATA) do not require initialisation
                                                pCtrl->DEV[pFRx->FROM].PowerUp_InitDone = 1;
                                                break;
                                        }
                                    } else {
                                        // node is already initialized, nothing to do
                                    }
                                }
                            } 
                            break;
                        default:
                            log_info("  Command %02X (header only) IGNORED (from %02X %s)", pFRx->CMD, pFRx->FROM, pCtrl->pDEVDT[x].sModelName);
                            break;
                    }
                    // register all received CMD's
                    pCtrl->iNewCommand = BGN_NEW_SET; // signal possible CMD change
                    pCtrl->CMD[pFRx->CMD].iNewCommand = BGN_NEW_SET; // signal this command
                    memcpy(pCtrl->CMD[pFRx->CMD].Rx.ucFrame, pFRx, RxSize);
                    CounterUp(&pCtrl->CMD[pFRx->CMD].CounterRx, t);
                } else {
                    // frame has some payload; check CR over the payload
                    if (bgnet_checksum_frame_check(pCtrl, pFRx) == BGNET_CHECKSUM_OK) {
                        switch(pFRx->CMD) {
                            case BGNET_CMD_01_DATA:
                                // process DATA command (broadcast), but this might be more difficult than you think......
                                // When we have 'active' nodes sending data, yes we most process incoming DATA commands. But who has 30 years old through-hull sensors
                                // in their boat or wind sensors on their masts? i don't! But assuming some puts all their nodes into 'demo' mode might have some fun.
                                // you will probably run this service in 'readonly' or 'passive' mode.
                                // When we are in 'readwrite', all DATA commands are send by our goodselves. i do not want to put our data into a endless loop, so we skip
                                // the all DATA commands, except for PILOT, because these can send NAV data receive via the NMEA port
                                switch (g_serial_mode) {
                                    case BGN_SERIAL_MODE_RW:
                                        switch (pFRx->FROM & BGNET_TYPE_MASK) {
                                            case (BGNET_TYPE_NETPILOT & BGNET_TYPE_MASK):
                                                decode_command_DATA(pCtrl, pFRx);
                                                break;
                                        }
                                        break;
                                    default:
                                        decode_command_DATA(pCtrl, pFRx);
                                        break;
                                }  
                                break;
                            case BGNET_CMD_02_LABEL:
                                decode_command_LABEL(pCtrl, pFRx);
                                break;
                            case BGNET_CMD_06_SUBSCRIBE:
                                decode_command_SUBSCRIBE(pCtrl, pFRx);
                                break;
                            case BGNET_CMD_C9_BACKLIGHT:
                                pCtrl->BGNET.NETWORK.BackLight.value.nByte = pFRx->payload[0]; // first byte is the LIGHTS state
                                pCtrl->BGNET.NETWORK.BackLight.source = BGN_SOURCE_BGNET;
                                CounterUp(&pCtrl->BGNET.NETWORK.BackLight.Counter, t);

//#define BGNET_ENABLE_RESET_LOG
#ifdef BGNET_ENABLE_RESET_LOG
                                // exprimental: send a command when press <LIGHTS> on NETDATA 
                                if (pFRx->FROM == BGNET_TYPE_NETDATA) {
                                    unsigned char ucCMD, ucTO, ucFROM;
                                    // send QUAD RESET LOG to QUAD
                                    //payload[BGNET_FRAME_ACKSEQ] = 0x00;
                                    //payload[1] = BGNET_CHN_CF_TRIPNM;
                                    //len_pl = 2;
                                    //ucCMD = BGNET_CMD_CD_PILOT_DMPREQ;
                                    //ucTO = BGNET_TYPE_NETQUAD;
                                    //ucFROM = BGNET_TYPE_NETACP;

                                    // send QUAD RESET DRDNM to QUAD
                                    //payload[BGNET_FRAME_ACKSEQ] = 0x00;
                                    //payload[1] = BGNET_CHN_81_DRDNM;
                                    //len_pl = 2;
                                    //ucCMD = BGNET_CMD_CB_QUAD_RESET;
                                    //ucTO = BGNET_TYPE_NETQUAD;
                                    //ucFROM = BGNET_TYPE_NETACP;

                                    // QUAD/SPEED/SETUP/#1
                                    payload[BGNET_FRAME_ACKSEQ] = 0x00;
                                    payload[1] = BGNET_CHN_41_STW;
                                    len_pl = 2;
                                    ucCMD = BGNET_CMD_CD_PILOT_DMPREQ;
                                    ucTO = BGNET_TYPE_NETQUAD;
                                    ucFROM = BGNET_TYPE_NETACP;

                                    pFTx = (FRAME_H *)&TxBuf[TxSize];
                                    len_msg = bgnet_encode_frame(pFTx, ucTO, ucFROM, ucCMD, payload, len_pl);
                                                        
                                    memcpy(pCtrl->CMD[BGNET_CMD_CD_PILOT_DMPREQ].Tx.ucFrame, pFTx, len_msg);
                                    CounterUp(&pCtrl->CMD[BGNET_CMD_CD_PILOT_DMPREQ].CounterTx, t);
                                    memcpy(pCtrl->DEV[BGNET_TYPE_NETQUAD].Tx.ucFrame, pFTx, len_msg);
                                    CounterUp(&pCtrl->DEV[BGNET_TYPE_NETQUAD].CounterTx, t);
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
                            case BGNET_CMD_E7_ALARM_SILENCE:
                                // an ALARM SILENCE was received. Note that a ALARM SILENCE can be send from every node (that has alarm capabilities). Like the others
                                // CMD-DAT/REQ we must send a CMD-ACK, but we do not split by node type, so we handle the ALARM SILENCE here.
                                // usually we receive: 30 D4 02 E7 13 41 C4 A5 (where C4 is the alarm channel)
                                // we reply with: D4 30 01 FD FE 41 69 (returning the ACKSEQ number)
                                // keep track of the SILENCE commands processed
                                CounterUp(&pCtrl->BGNET.NETWORK.AlarmReset, t);
                                // setup the pointer to our received CMD-REQ or CMD-DAT command
                                pCHRx = (union CH *)(&pFRx->payload[1]);
                                //log_debug("ALARM SILENCE from 0x%02X for alarm channel 0x%02X", pFRx->FROM, pCHRx->s.CH);
                                // note: we only clear the beeping bit; we do (and can not) take the alarm cause out
                                switch (pCHRx->s.CH) {
                                    case BGNET_CHN_76_AP_WA_ALARM:
                                        pCtrl->BGNET.PILOT.ALARM.WA.ucBEEP = 0;
                                        break;
                                    case BGNET_CHN_AF_AP_OC_ALARM:
                                        pCtrl->BGNET.PILOT.ALARM.OC.ucBEEP = 0;
                                        break;
                                    case BGNET_CHN_FC_SHALLOWALARM:
                                        pCtrl->BGNET.DEPTH.ALARM.SHAL.ucBEEP = 0;
                                        break;
                                    case BGNET_CHN_FD_DEEPALARM:
                                        pCtrl->BGNET.DEPTH.ALARM.DEEP.ucBEEP = 0;
                                        break;
                                    case BGNET_CHN_FE_ANCHORALARM:
                                        pCtrl->BGNET.DEPTH.ALARM.ANCH.ucBEEP = 0;
                                        break;
                                }

                                if (pFRx->TO == BGNET_TYPE_ALL) {
                                    // never send a reply to a broadcast
                                } else {
                                    // make the CMD-ACK for the ALARMSILENCE command we received
                                    payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                    len_pl = 1;
                                    pFTx = (FRAME_H *)&TxBuf[TxSize];
                                    // send reply where it came from (FROM) with the originator who that node think we are (TO), except if that was a broadcast
                                    len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, pFRx->TO, BGNET_CMD_ACK, payload, len_pl);

                                    memcpy(pCtrl->CMD[BGNET_CMD_ACK].Tx.ucFrame, pFTx, len_msg);
                                    CounterUp(&pCtrl->CMD[BGNET_CMD_ACK].CounterTx, t);
                                    memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                    CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                    TxSize = TxSize + len_msg;

                                    // experiment koen
#ifdef EXPERIMENT
                                    // broadcast a SILENCE command to all
                                    payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                    payload[1] = pFRx->payload[1];
                                    len_pl = 2;
                                    pFTx = (FRAME_H *)&TxBuf[TxSize];
                                    // send reply where it came from (FROM) with the originator who that node think we are (TO), except if that was a broadcast
                                    len_msg = bgnet_encode_frame(pFTx, BGNET_TYPE_ALL, BGNET_TYPE_NETPILOT, BGNET_CMD_E7_ALARM_SILENCE, payload, len_pl);

                                    memcpy(pCtrl->CMD[BGNET_CMD_E7_ALARM_SILENCE].Tx.ucFrame, pFTx, len_msg);
                                    CounterUp(&pCtrl->CMD[BGNET_CMD_E7_ALARM_SILENCE].CounterTx, t);
                                    memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                    CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                    TxSize = TxSize + len_msg;
#endif

                                    // put it for transmission
                                    pthread_mutex_lock(&g_data_mutex);
                                    memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &TxBuf, TxSize); // put output message in write buffer
                                    g_serial_data_lenTx = g_serial_data_lenTx + TxSize; // increment byte counter in TX buffer
                                    pthread_mutex_unlock(&g_data_mutex);
                                }

                                break;
                            case BGNET_CMD_ACK:
                                // sometimes we receive a ACK to a CMD we have send. We ignore it.
                                break;
                            default:
                                // Some commands are part of a CMD-REQ/CDM-DAT/CMD-ACK handshake. All next commands require a CMD-ACK response
                                // (and possible some other things too). The CMD-ACK is send last
                                // holy shit.... CMD/CHN is not unique within the network. The same combination can be found on PILOT and QUAD
                                // with different behaviour. I think a first make a new GIT commit before i screw up
                                // we switch for the DEVICE TYPE (not the actual FROM address)

                                // setup the pointer to our received CMD-REQ or CMD-DAT command
                                pCHRx = (union CH *)(&pFRx->payload[1]);
                                // only send a CMDACK when a preceeding CMDDAT or CMDREQ was handled, assume it's OK right now
                                bSend_CMDACK = 1;
                                // split behaviour by device type
                                switch(pFRx->FROM & BGNET_TYPE_MASK) {
                                    // WIND = CMD-REQ/CMD-DAT/CMD-ACK handshake 
                                    case (BGNET_TYPE_NETWIND & BGNET_TYPE_MASK):
                                        switch(pFRx->CMD) {
                                            // WIND/SETUP actions
                                            case BGNET_CMD_CE_PILOT_DMPDAT:
                                                switch(pCHRx->s.CH) {
                                                    // WIND/SETUP/DS (#1 = Damping Speed)
                                                    case BGNET_CHN_50_AWS:
                                                        pCtrl->BGNET.WIND.SETUP.DS.value.nLong = ntohs(pCHRx->s.cv.uW[0]);
                                                        pCtrl->BGNET.WIND.SETUP.DS.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.WIND.SETUP.DS.Counter, t);
                                                        break;
                                                    // WIND/SETUP/DA (#2 = Damping Angle)
                                                    case BGNET_CHN_51_AWA:
                                                        pCtrl->BGNET.WIND.SETUP.DA.value.nLong = ntohs(pCHRx->s.cv.uW[0]);
                                                        pCtrl->BGNET.WIND.SETUP.DA.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.WIND.SETUP.DA.Counter, t);
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_D3_PILOT_AM:
                                                switch(pCHRx->s.CH) {
                                                    // WIND/SETUP/OF (#3 = Angle Offset)
                                                    case BGNET_CHN_51_AWA:
                                                        pCtrl->BGNET.WIND.SETUP.OF.value.nLong = ntohs(pCHRx->s.cv.uW[0]);
                                                        pCtrl->BGNET.WIND.SETUP.OF.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.WIND.SETUP.OF.Counter, t);
                                                        break;
                                                }
                                                break;
                                            default:
                                                // unhandled, do not send CMDACK
                                                bSend_CMDACK = 0;
                                        }
                                        break;
                                    // QUAD = CMD-REQ/CMD-DAT/CMD-ACK handshake 
                                    case (BGNET_TYPE_NETQUAD & BGNET_TYPE_MASK):
                                        // when in 'Transmitter' and/or 'Demo' mode, the QUAD/SPEED/SETUP does not do handshaking, it manages it's setup internally
                                        // when in 'Repeater' mode, the QUAD/SPEED/SETUP does handshaking with the QUAD/t But we have no
                                        // 'Transmitter'. We found out the CMD/CHN combinations by registering the command send by the QUAD/r and then
                                        // replicating these to the QUAD/t.
                                        switch(pFRx->CMD) {
                                            // QUAD/RESET actions
                                            case BGNET_CMD_20_PILOT_OCDAT: // 3 bytes payload?!
                                                switch(pCHRx->s.CH) {
                                                    // DEPTH/SETUP/ANCHOR ALARM
                                                    case BGNET_CHN_C4_DEPTH:
                                                        // switch by DataType
                                                        switch (pCHRx->s.FMT & BGNET_FMT_DT)  {
                                                            // STATUS On/OFF
                                                            case BGN_FMT_0F:
                                                                // set alarm ON or OFF, payload byte 3: 0x1F=ON, 0x0F=OFF
                                                                if (pCHRx->s.FMT == BGNET_FMT_ON) {
                                                                    // when activating the Anchor Alarm, also set the current depth (use calibrated depth)
                                                                    pCtrl->BGNET.DEPTH.SETUP.AMA_M.value.dValue = pCtrl->BGNET.DEPTH.Depth_CAL.value.dValue;
                                                                    pCtrl->BGNET.DEPTH.SETUP.AMA_M.source = BGN_SOURCE_BGNET;
                                                                    CounterUp(&pCtrl->BGNET.DEPTH.SETUP.AMA_M.Counter, t);
                                                                    // enable the anchor alarm
                                                                    pCtrl->BGNET.DEPTH.SETUP.AA_STAT.value.nByte = 1;
                                                                } else {
                                                                    // disable the anchor alarm
                                                                    pCtrl->BGNET.DEPTH.SETUP.AA_STAT.value.nByte = 0;
                                                                }
                                                                pCtrl->BGNET.DEPTH.SETUP.AA_STAT.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->BGNET.DEPTH.SETUP.AA_STAT.Counter, t);
                                                                break;
                                                            // alarm level in M
                                                            case BGN_FMT_01:
                                                                break;
                                                            case BGN_FMT_0D:
                                                                // the Anchor Alarm sends 7 bytes payload with 2 values (shallow and deep)
                                                                pCtrl->BGNET.DEPTH.SETUP.ASA_M.value.dValue = decode_channel_double(pCHRx, 0);
                                                                pCtrl->BGNET.DEPTH.SETUP.ASA_M.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->BGNET.DEPTH.SETUP.ASA_M.Counter, t);
                                                                pCtrl->BGNET.DEPTH.SETUP.ADA_M.value.dValue = decode_channel_double(pCHRx, 1);
                                                                pCtrl->BGNET.DEPTH.SETUP.ADA_M.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->BGNET.DEPTH.SETUP.ADA_M.Counter, t);
                                                                break;
                                                        }
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_21_PILOT_WADAT: // 3 bytes payload?!
                                                switch(pCHRx->s.CH) {
                                                    // DEPTH/SETUP/SHALLOW ALARM
                                                    case BGNET_CHN_C4_DEPTH:
                                                        // switch by DataType
                                                        switch (pCHRx->s.FMT & BGNET_FMT_DT)  {
                                                            // STATUS On/OFF
                                                            case BGN_FMT_0F:
                                                                // set alarm ON or OFF, payload byte 3: 0x1F=ON, 0x0F=OFF
                                                                switch (pCHRx->s.FMT) {
                                                                    case BGNET_FMT_ON:
                                                                        pCtrl->BGNET.DEPTH.SETUP.SA_STAT.value.nByte = 1;
                                                                        break;
                                                                    case BGNET_FMT_OFF:
                                                                        pCtrl->BGNET.DEPTH.SETUP.SA_STAT.value.nByte = 0;
                                                                        break;
                                                                }
                                                                pCtrl->BGNET.DEPTH.SETUP.SA_STAT.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->BGNET.DEPTH.SETUP.SA_STAT.Counter, t);
                                                                break;
                                                            // alarm level in M
                                                            case BGN_FMT_01:
                                                                pCtrl->BGNET.DEPTH.SETUP.SA_M.value.dValue = decode_channel_double(pCHRx, 0);
                                                                pCtrl->BGNET.DEPTH.SETUP.SA_M.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->BGNET.DEPTH.SETUP.SA_M.Counter, t);
                                                                break;
                                                        }
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_22_DEPTH_DADAT: // 3 bytes payload
                                                switch(pCHRx->s.CH) {
                                                    // DEPTH/SETUP/DEEP ALARM
                                                    case BGNET_CHN_C4_DEPTH:
                                                        // switch by DataType
                                                        switch (pCHRx->s.FMT & BGNET_FMT_DT)  {
                                                            // STATUS On/OFF
                                                            case BGN_FMT_0F:
                                                                // set alarm ON or OFF, payload byte 3: 0x1F=ON, 0x0F=OFF
                                                                switch (pCHRx->s.FMT) {
                                                                    case BGNET_FMT_ON:
                                                                        pCtrl->BGNET.DEPTH.SETUP.DA_STAT.value.nByte = 1;
                                                                        break;
                                                                    case BGNET_FMT_OFF:
                                                                        pCtrl->BGNET.DEPTH.SETUP.DA_STAT.value.nByte = 0;
                                                                        break;
                                                                }
                                                                pCtrl->BGNET.DEPTH.SETUP.DA_STAT.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->BGNET.DEPTH.SETUP.DA_STAT.Counter, t);
                                                                break;
                                                            // alarm level in M
                                                            case BGN_FMT_01:
                                                                pCtrl->BGNET.DEPTH.SETUP.DA_M.value.dValue = decode_channel_double(pCHRx, 0);
                                                                pCtrl->BGNET.DEPTH.SETUP.DA_M.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->BGNET.DEPTH.SETUP.DA_M.Counter, t);
                                                                break;
                                                        }
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_CA_QUAD_TIMER:
                                                // 2 bytes payload: 0=ACKSEQ + 1=minutes (no CHNID)
                                                switch(pFRx->payload[1]) {
                                                    case 0x0A: // 10 minute count-down timer
                                                        pCtrl->BGNET.SPEED.TIMER.value.tTime = pCtrl->BGNET.SERVICE.SystemTime.value.tTime + 10*60;
                                                        pCtrl->BGNET.SPEED.TIMER.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.SPEED.TIMER.Counter, t);
                                                        break;
                                                    case 0x05:
                                                        pCtrl->BGNET.SPEED.TIMER.value.tTime = pCtrl->BGNET.SERVICE.SystemTime.value.tTime + 5*60;
                                                        pCtrl->BGNET.SPEED.TIMER.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.SPEED.TIMER.Counter, t);
                                                        break;
                                                    case 0x00:
                                                        pCtrl->BGNET.SPEED.TIMER.value.tTime = pCtrl->BGNET.SERVICE.SystemTime.value.tTime + 0*60;
                                                        pCtrl->BGNET.SPEED.TIMER.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.SPEED.TIMER.Counter, t);
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_CB_QUAD_RESET:
                                                switch(pCHRx->s.CH) {
                                                    case BGNET_CHN_CF_TRIPLOG:
                                                        // SPEED/RESET LOG, 2 bytes payload
                                                        // a QUAD/r only sends a CMD 0xCB/0x41 (to the QUAD/t)
                                                        // a QUAD/t or QUAD/d sends nothing.
                                                        pCtrl->BGNET.SPEED.TripLogNM.value.dValue = 0.0;
                                                        pCtrl->BGNET.SPEED.TripLogNM.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.SPEED.TripLogNM.Counter, t);
                                                        break;
                                                    case BGNET_CHN_81_DRD:
                                                        // SPEED/DRD RESET, 2 bytes payload
                                                        // a QUAD/r only sends a CMD_CB/CHN_81 (to the QUAD/t)
                                                        // a QUAD/t or QUAD/d sends nothing.
                                                        pCtrl->BGNET.SPEED.DRD.value.dValue = 0.0;
                                                        pCtrl->BGNET.SPEED.DRD.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.SPEED.DRD.Counter, t);
                                                        pCtrl->BGNET.SPEED.SpeedMAX.value.dValue = pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.value.dValue + pCtrl->BGNET.SPEED.STW.value.dValue * pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.value.dValue;
                                                        pCtrl->BGNET.SPEED.SpeedMAX.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.SPEED.SpeedMAX.Counter, t);
                                                        pCtrl->BGNET.SPEED.SpeedAVG.value.dValue = pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.value.dValue + pCtrl->BGNET.SPEED.STW.value.dValue * pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.value.dValue;
                                                        pCtrl->BGNET.SPEED.SpeedAVG.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.SPEED.SpeedAVG.Counter, t);
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_CD_PILOT_DMPREQ:
                                                switch(pCHRx->s.CH) {
                                                    // SPEED/SETUP/#1 = DMP
                                                    case BGNET_CHN_41_STW:
                                                        // create command with current SPEED/SETUP/DMP value
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        pCHTx = (union CH *)&payload[1];
                                                        pCHTx->s.CH = BGNET_CHN_41_STW;
                                                        pCHTx->s.FMT = BGN_FMT_08;
                                                        pCHTx->s.cv.uW[0] = htons(pCtrl->BGNET.SPEED.SETUP.DMP.value.nLong);
                                                        len_pl = 5;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_CE_PILOT_DMPDAT, payload, len_pl);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_CE_PILOT_DMPDAT].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_CE_PILOT_DMPDAT].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_CE_PILOT_DMPDAT:
                                                switch(pCHRx->s.CH) {
                                                    // SPEED/SETUP/#1 = accept new setting
                                                    case BGNET_CHN_41_STW:
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
                                                        pCtrl->BGNET.SPEED.SETUP.DMP.value.nLong = ntohs(pCHRx->s.cv.uW[0]);
                                                        pCtrl->BGNET.SPEED.SETUP.DMP.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.SPEED.SETUP.DMP.Counter, t);
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_CF_PILOT_MANREQ:
                                                switch(pCHRx->s.CH) {
                                                    case BGNET_CHN_41_STW:
                                                        // create command with current SPEED/SETUP/#3, with MANCAL value (by known speed)
                                                        // create command with current SPEED/SETUP/#4, with MANCAL value (by known distance/time)
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        pCHTx = (union CH *)&payload[1];
                                                        c = BGNET_CHN_41_STW;
                                                        encode_channel_double(pCtrl, pCHTx, c, BGN_FMT_08, pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.value.dValue, 0);
                                                        len_pl = 5;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_D3_PILOT_AM, payload, len_pl);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_D3_PILOT_AM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_D3_PILOT_AM].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                    case BGNET_CHN_C4_DEPTH:
                                                        // create CMD-DAT with current DEPTH CAL value
                                                        // 0= No Calibration
                                                        // <0= offset to keel
                                                        // >0= offset to watersurface
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        pCHTx = (union CH *)&payload[1];
                                                        c = BGNET_CHN_C4_DEPTH;
                                                        encode_channel_double(pCtrl, pCHTx, c, BGN_FMT_01, pCtrl->BGNET.DEPTH.SETUP.CAL.value.dValue, 0);
                                                        len_pl = 5;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_D3_PILOT_AM, payload, len_pl);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_D3_PILOT_AM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_D3_PILOT_AM].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_D3_PILOT_AM:
                                                switch(pCHRx->s.CH) {
                                                    // SPEED/SETUP/#3 = accept new CAL setting
                                                    case BGNET_CHN_41_STW:
                                                        // update the channel value
                                                        pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                        y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                        memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                        decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                        CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);
                                                        // get the new SPEED CALibration offset and factor we receive the corrected speed from the channel. This is the 'corrected' speed after observation
                                                        // STW (and also the corrected speed) is always a positive number
                                                        dTemp = decode_channel_double(pCHRx, 0);
                                                        // with the comparison we keep a small bandwidth to avoid counter increments without actual change
                                                        // i would et to see some kind of 2nd order correction factor.
                                                        // for simplicity i just reset the OFFSET and FACTOR to the standard values
                                                        if (dTemp > (pCtrl->BGNET.SPEED.STW.value.dValue + 0.01)) {
                                                            // corrected speed GT measured speed; increment CAL value
                                                            pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.value.dValue = BGNET_SPEED_CAL_OFFSET;
                                                            pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.source = BGN_SOURCE_BGNET;
                                                            CounterUp(&pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.Counter, t);
                                                            pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.value.dValue = BGNET_SPEED_CAL_FACTOR;
                                                            pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.source = BGN_SOURCE_BGNET;
                                                            CounterUp(&pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.Counter, t);
                                                        } else if ((dTemp < (pCtrl->BGNET.SPEED.STW.value.dValue - 0.01))) {
                                                            // corrected speed LT measured speed; reduce CAL value (note: this could be negative!)
                                                            pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.value.dValue = BGNET_SPEED_CAL_OFFSET;
                                                            pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.source = BGN_SOURCE_BGNET;
                                                            CounterUp(&pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.Counter, t);
                                                            pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.value.dValue = BGNET_SPEED_CAL_FACTOR;
                                                            pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.source = BGN_SOURCE_BGNET;
                                                            CounterUp(&pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.Counter, t);
                                                        } else {
                                                            // none of the above
                                                        }
                                                        break;
                                                    case BGNET_CHN_C4_DEPTH:
                                                        // update the channel value
                                                        pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                        y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                        memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                        decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                        CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);
                                                        // get the new DEPTH CAL from the channel
                                                        pCtrl->BGNET.DEPTH.SETUP.CAL.value.dValue = decode_channel_double(pCHRx, 0);
                                                        pCtrl->BGNET.DEPTH.SETUP.CAL.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.DEPTH.SETUP.CAL.Counter, t);
                                                        break;
                                                    }
                                                break;
                                            case BGNET_CMD_DD_QUAD_SHAREQ:
                                                switch(pCHRx->s.CH) {
                                                    // DEPTH/SETUP/DEEPALM
                                                    case BGNET_CHN_C4_DEPTH:
                                                        // create CMD-DAT with current DEEP ALARM STATUS value
                                                        // 0= Off (0x0F)
                                                        // #0 = On (0x1F)
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        pCHTx = (union CH *)&payload[1];
                                                        pCHTx->s.CH = BGNET_CHN_C4_DEPTH;
                                                        switch (pCtrl->BGNET.DEPTH.SETUP.DA_STAT.value.nByte) {
                                                            case 0:
                                                                pCHTx->s.FMT = BGNET_FMT_OFF;
                                                                break;
                                                            default:
                                                                pCHTx->s.FMT = BGNET_FMT_ON;
                                                                break;
                                                        }
                                                        len_pl = 3;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_22_DEPTH_DADAT, payload, len_pl);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_22_DEPTH_DADAT].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_22_DEPTH_DADAT].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        // send alarm level
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        pCHTx = (union CH *)&payload[1];
                                                        c = BGNET_CHN_C4_DEPTH;
                                                        encode_channel_double(pCtrl, pCHTx, c, BGN_FMT_01, pCtrl->BGNET.DEPTH.SETUP.DA_M.value.dValue, 0);
                                                        len_pl = 5;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_22_DEPTH_DADAT, payload, len_pl);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_22_DEPTH_DADAT].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_22_DEPTH_DADAT].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_DE_PILOT_WAREQ:
                                                switch(pCHRx->s.CH) {
                                                    // DEPTH/SETUP/SHA
                                                    case BGNET_CHN_C4_DEPTH:
                                                        // create CMD-DAT with current SHALLOW ALARM STATUS value
                                                        // 0= Off (0x0F)
                                                        // #0 = On (0x1F)
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        pCHTx = (union CH *)&payload[1];
                                                        pCHTx->s.CH = BGNET_CHN_C4_DEPTH;
                                                        switch (pCtrl->BGNET.DEPTH.SETUP.SA_STAT.value.nByte) {
                                                            case 0:
                                                                pCHTx->s.FMT = BGNET_FMT_OFF;
                                                                break;
                                                            default:
                                                                pCHTx->s.FMT = BGNET_FMT_ON;
                                                                break;
                                                        }
                                                        len_pl = 3;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_21_PILOT_WADAT, payload, len_pl);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_21_PILOT_WADAT].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_21_PILOT_WADAT].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        // send alarm level
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        pCHTx = (union CH *)&payload[1];
                                                        c = BGNET_CHN_C4_DEPTH;
                                                        encode_channel_double(pCtrl, pCHTx, c, BGN_FMT_01, pCtrl->BGNET.DEPTH.SETUP.SA_M.value.dValue, 0);
                                                        len_pl = 5;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_21_PILOT_WADAT, payload, len_pl);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_21_PILOT_WADAT].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_21_PILOT_WADAT].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_DF_PILOT_OCREQ:
                                                switch(pCHRx->s.CH) {
                                                    // DEPTH/SETUP/ANCA
                                                    case BGNET_CHN_C4_DEPTH:
                                                        // create CMD-DAT with current ANCHOR ALARM STATUS value
                                                        // 0= Off (0x0F)
                                                        // #0 = On (0x1F)
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        pCHTx = (union CH *)&payload[1];
                                                        pCHTx->s.CH = BGNET_CHN_C4_DEPTH;
                                                        switch (pCtrl->BGNET.DEPTH.SETUP.AA_STAT.value.nByte) {
                                                            case 0:
                                                                pCHTx->s.FMT = BGNET_FMT_OFF;
                                                                break;
                                                            default:
                                                                pCHTx->s.FMT = BGNET_FMT_ON;
                                                                break;
                                                        }
                                                        len_pl = 3;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_20_PILOT_OCDAT, payload, len_pl);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_20_PILOT_OCDAT].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_20_PILOT_OCDAT].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        // send alarm level (7 bytes payload)
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        pCHTx = (union CH *)&payload[1];
                                                        c = BGNET_CHN_C4_DEPTH;
                                                        encode_channel_double(pCtrl, pCHTx, c, BGN_FMT_0D, pCtrl->BGNET.DEPTH.SETUP.ASA_M.value.dValue, 0);
                                                        encode_channel_double(pCtrl, pCHTx, c, BGN_FMT_0D, pCtrl->BGNET.DEPTH.SETUP.ADA_M.value.dValue, 1);
                                                        len_pl = 7;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_20_PILOT_OCDAT, payload, len_pl);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_20_PILOT_OCDAT].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_20_PILOT_OCDAT].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_E9_QUAD_ACAL_DAT:
                                                switch(pCHRx->s.CH) {
                                                    // SPEED/SETUP/#5 = accept new AUTOCAL setting
                                                    // Usually, it is the QUADt that starts transmitting this channel while going through the step 1, 2 and 3
                                                    // of the autocal procedure.
                                                    // incase we are dealing with no QUADt, but with a QUADr we (the ACP) have to emulate this.
                                                    // As we have no practical use of the calibration process we will not implement this feature
                                                    // the perculiar thing is, when we receive this channel from a QUADt, the channel is "00 88 00 00", if the
                                                    // AUTOCAL is started from a QUADr we receive "00 88 04 00".
                                                    case BGNET_CHN_00_NODERESET:
                                                        // update the channel value
                                                        pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                        y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                        memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                        decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                        CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);
                                                        // get the new SPEED CALibration offset and factor we receive the corrected speed from the channel. This is the 'corrected' speed after observation
                                                        // STW (and also the corrected speed) is always a positive number
                                                        dTemp = decode_channel_double(pCHRx, 0);
                                                        // with the comparison we keep a small bandwidth to avoid counter increments without actual change
                                                        // i would et to see some kind of 2nd order correction factor.
                                                        // for simplicity i just reset the OFFSET and FACTOR to the standard values
                                                        if (dTemp > (pCtrl->BGNET.SPEED.STW.value.dValue + 0.01)) {
                                                            // corrected speed GT measured speed; increment CAL value
                                                            pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.value.dValue = BGNET_SPEED_CAL_OFFSET;
                                                            pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.source = BGN_SOURCE_BGNET;
                                                            CounterUp(&pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.Counter, t);
                                                            pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.value.dValue = BGNET_SPEED_CAL_FACTOR;
                                                            pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.source = BGN_SOURCE_BGNET;
                                                            CounterUp(&pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.Counter, t);
                                                        } else if ((dTemp < (pCtrl->BGNET.SPEED.STW.value.dValue - 0.01))) {
                                                            // corrected speed LT measured speed; reduce CAL value (note: this could be negative!)
                                                            pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.value.dValue = BGNET_SPEED_CAL_OFFSET;
                                                            pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.source = BGN_SOURCE_BGNET;
                                                            CounterUp(&pCtrl->BGNET.SPEED.SETUP.CAL_OFFSET.Counter, t);
                                                            pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.value.dValue = BGNET_SPEED_CAL_FACTOR;
                                                            pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.source = BGN_SOURCE_BGNET;
                                                            CounterUp(&pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.Counter, t);
                                                        } else {
                                                            // none of the above
                                                        }
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_EA_QUAD_ACAL_REQ:
                                                // note the CMD/CHN combination for CAL and LOGCAL are the same. It is also the same correction albeit the
                                                // first derived from a "reference speed", the other derived from a "known distance and time". They are the same
                                                switch(pCHRx->s.CH) {
                                                    // SPEED/SETUP/#1 = DMP
                                                    case BGNET_CHN_01_UNKNOWN:
                                                        // SPEED/SETUP/#5 = AUTOCAL
                                                        // we receive a CHN that we do not recognise. Also i do no know what to send back.
                                                        // the QUAD/SPEED node expects some numer. i do no know how to send it.
                                                        // at least we return a CMD-ACK
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        pCHTx = (union CH *)&payload[1];
                                                        c = BGNET_CHN_00_NODERESET;
                                                        pCHTx->s.cv.uW[0] = encode_channel_double(pCtrl, pCHTx, c, pCtrl->pCHNDT[c].ucFMT, pCtrl->BGNET.SPEED.SETUP.CAL_FACTOR.value.dValue, 0);
                                                        len_pl = 5;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_E9_QUAD_ACAL_DAT, payload, len_pl);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_E9_QUAD_ACAL_DAT].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_E9_QUAD_ACAL_DAT].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            default:
                                                // do not send CMDACK
                                                bSend_CMDACK = 0;

                                                x = bgnet_deviceindex(pCtrl, pFRx->FROM); // lookup by device address/type, so we can print the device name
                                                log_info("  Command %02X (with body) IGNORED (from %02XThis message is send  %s)", pFRx->CMD, pFRx->FROM, pCtrl->pDEVDT[x].sModelName);
                                                break;
                                        }
                                        break;
                                    // PILOT = CMD-REQ/CMD-DAT/CMD-ACK handshake 
                                    case (BGNET_TYPE_NETPILOT & BGNET_TYPE_MASK):
                                        switch(pFRx->CMD) {
                                            // PILOT/SETUP actions
                                            case BGNET_CMD_20_PILOT_OCDAT:
                                                // in PILOT/SETUP, when OFF-COURSE ALARM setting is entered, 5 bytes payload
                                                // 0=ACKSEQ, 1=0x76, 2=FMT_01, 3/4=OC limit in degrees (1-99)
                                                switch(pCHRx->s.CH) {
                                                    case BGNET_CHN_AF_AP_OC_ALARM:
                                                        // update the channel value last, so we can compare if anything changed
                                                        pCHTx = (union CH *)pCtrl->CHN[pCHRx->s.CH].x00.ucRAW; // we abuse the pCHTx pointer
                                                        if (pCHTx->s.FMT == pCHRx->s.FMT) {
                                                            // no change
                                                        } else {
                                                           if (pCHRx->s.FMT == BGN_FMT_08) {
                                                                pCtrl->BGNET.PILOT.SETUP.OC_STAT.value.nByte = 0;
                                                            } else {
                                                                pCtrl->BGNET.PILOT.SETUP.OC_STAT.value.nByte = 1;
                                                            }
                                                            pCtrl->BGNET.PILOT.SETUP.OC_STAT.source = BGN_SOURCE_BGNET;
                                                            CounterUp(&pCtrl->BGNET.PILOT.SETUP.OC_STAT.Counter, t);
                                                        }
                                                        if (pCHTx->s.cv.uW[0] == pCHRx->s.cv.uW[0]) {
                                                            // no change
                                                        } else {
                                                            pCtrl->BGNET.PILOT.SETUP.OC_DEG.value.nLong = ntohs(pCHRx->s.cv.uW[0]);
                                                            pCtrl->BGNET.PILOT.SETUP.OC_DEG.source = BGN_SOURCE_BGNET;
                                                            CounterUp(&pCtrl->BGNET.PILOT.SETUP.OC_DEG.Counter, t);
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
                                            case BGNET_CMD_21_PILOT_WADAT:
                                                // in PILOT/SETUP, when WATCH ALARM setting is entered, 5 bytes payload
                                                // 0=ACKSEQ, 1=0x76, 2=FMT_01, 3/4=WA limit in minutes (1-360)
                                                switch(pCHRx->s.CH) {
                                                    case BGNET_CHN_76_AP_WA_ALARM:
                                                        // update the channel value last, so we can compare if anything changed
                                                        pCHTx = (union CH *)pCtrl->CHN[pCHRx->s.CH].x00.ucRAW; // we abuse the pCHTx pointer
                                                        if (pCHTx->s.FMT == pCHRx->s.FMT) {
                                                            // no change
                                                        } else {
                                                           if (pCHRx->s.FMT == BGN_FMT_08) {
                                                                pCtrl->BGNET.PILOT.SETUP.WA_STAT.value.nByte = 0;
                                                            } else {
                                                                pCtrl->BGNET.PILOT.SETUP.WA_STAT.value.nByte = 1;
                                                            }
                                                            pCtrl->BGNET.PILOT.SETUP.WA_STAT.source = BGN_SOURCE_BGNET;
                                                            CounterUp(&pCtrl->BGNET.PILOT.SETUP.WA_STAT.Counter, t);
                                                        }
                                                        if (pCHTx->s.cv.uW[0] == pCHRx->s.cv.uW[0]) {
                                                            // no change
                                                        } else {
                                                            pCtrl->BGNET.PILOT.SETUP.WA_MIN.value.nLong = ntohs(pCHRx->s.cv.uW[0]);
                                                            pCtrl->BGNET.PILOT.SETUP.WA_MIN.source = BGN_SOURCE_BGNET;
                                                            CounterUp(&pCtrl->BGNET.PILOT.SETUP.WA_MIN.Counter, t);
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
                                            case BGNET_CMD_B1_PILOT_DELTADAT:
                                                // 1 of the 4 +/- keys was pressed, 5 bytes payload: 0-ACKSEQ, 1=0xA6/CHID, 2=0x01/FMT, 3/4=signed 16bits inc/decr value
                                                switch(pCHRx->s.CH) {
                                                    case BGNET_CHN_A6_AP_TARGET:
                                                        iValue = decode_channel_numeric(pCHRx);
                                                        switch (iValue) {
                                                            case -10:
                                                                CounterUp(&pCtrl->BGNET.PILOT.KEY.P10, t);
                                                                break;
                                                            case -1:
                                                                CounterUp(&pCtrl->BGNET.PILOT.KEY.P1, t);
                                                                break;
                                                            case +10:
                                                                CounterUp(&pCtrl->BGNET.PILOT.KEY.S10, t);
                                                                break;
                                                            case +1:
                                                                CounterUp(&pCtrl->BGNET.PILOT.KEY.S1, t);
                                                                break;
                                                        }
                                                        pCtrl->BGNET.ACP.CTS.value.nLong = BGNET_INCREMENTWRAP(pCtrl->BGNET.ACP.CTS.value.nLong, iValue, 0, +360);
                                                        pCtrl->BGNET.ACP.CTS.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.ACP.CTS.Counter, t);
                                                        // i would expect to create a CMD-DAT message with the new CTS value
                                                        // but i do not know what command to use to send it (or if that exists at all)
                                                        // Now i rely on the 'regular' broadcast of channel data. You might observe that
                                                        // there is some delay in the update. It works, but i find it annoying
                                                        // Upside is that in case there are multiple PILOT, the DATA/0x01 (broadcast), also updates others
                                                        pCHTx = (union CH *)&payload[0];
                                                        pCHTx->s.CH = BGNET_CHN_A6_AP_TARGET;
                                                        pCHTx->s.FMT = BGN_FMT_08;
                                                        pCHTx->s.cv.uW[0] = htons(pCtrl->BGNET.ACP.CTS.value.nLong);
                                                        len_pl = 4;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, BGNET_TYPE_ALL, BGNET_TYPE_NETACP, BGNET_CMD_01_DATA, payload, len_pl);
                                                
                                                        memcpy(pCtrl->CMD[BGNET_CMD_01_DATA].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_01_DATA].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_CB_QUAD_RESET:
                                                switch (pCHRx->s.CH) {
                                                    case BGNET_CHN_A6_AP_TARGET:
                                                        // happens when PILOT wants the stored CTS from the ACP. I do not know
                                                        // what command to use, so i do not do anything. At least is makes the PILOT
                                                        // to receive the CMDACK, preventing an error beep
                                                        break;
                                                    case BGNET_CHN_A7_AP_WPB:
                                                        // happens when PILOT wants 'some unknown value' from the ACP. I do not know
                                                        // what command to use, so i do not do anything. At least is makes the PILOT
                                                        // to receive the CMDACK, preventing an error beep
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_CD_PILOT_DMPREQ:
                                                // in PILOT/SETUP, the COMPASS DMP is configured, 2 bytes payload
                                                // 0=ACKSEQ, 1=0x49
                                                switch(pCHRx->s.CH) {
                                                    case BGNET_CHN_49_HEADING:
                                                        // create command with current DMP setting
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        payload[1] = BGNET_CHN_49_HEADING;
                                                        payload[2] = BGN_FMT_01;
                                                        payload[3] = 0;
                                                        payload[4] = pCtrl->BGNET.PILOT.SETUP.DMP.value.nByte; // (0-99)                                                        len_pl = 5;
                                                        len_pl = 5;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_CE_PILOT_DMPDAT, payload, len_pl);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_CE_PILOT_DMPDAT].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_CE_PILOT_DMPDAT].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_CE_PILOT_DMPDAT:
                                                // in PILOT/SETUP, when COMPASS DMP setting is entered, 5 bytes payload
                                                // 0=ACKSEQ, 1=0x49, 2=FMT_01, 3=0, 4=DMP in seconds (1-99, 4=default)
                                                switch(pCHRx->s.CH) {
                                                    case BGNET_CHN_49_HEADING:
                                                        // update the channel value 
                                                        pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                        y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                        memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                        decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                        CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);
                                                        pCtrl->BGNET.PILOT.SETUP.DMP.value.nByte = (pCHRx->s.cv.uB[1]);
                                                        pCtrl->BGNET.PILOT.SETUP.DMP.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.PILOT.SETUP.DMP.Counter, t);
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_CF_PILOT_MANREQ:
                                                // in SPEED/SETUP/MAN, the FST speed value is requested, 2 bytes payload
                                                // 0=ACKSEQ, 1=0x46
                                                switch(pCHRx->s.CH) {
                                                    case BGNET_CHN_46_AP_SPEED:
                                                        // SEND SPD MAN/FST
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        pCHTx = (union CH *)&payload[1];
                                                        pCHTx->s.CH = BGNET_CHN_46_AP_SPEED;
                                                        pCHTx->s.FMT = BGN_FMT_01;
                                                        pCHTx->s.cv.uW[0] = htons(pCtrl->BGNET.PILOT.SPEED.FST.value.nLong);
                                                        len_pl = 5;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_D3_PILOT_AM, payload, len_pl);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_D3_PILOT_AM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_D3_PILOT_AM].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_D0_PILOT_SPDREQ:
                                                // the SPEED key was pressed, 2 bytes payload.
                                                // 0=ACKSEQ, 1=0x46 -> return current selected SPD source (TRA/MAN/SOG)
                                                // in SETUP, FR option is requested, 2 bytes payload
                                                // 0=ACKSEQ, 1=0x07 -> return current select FR state (1=OFF, 0=ON)
                                                switch(pCHRx->s.CH) {
                                                    case BGNET_CHN_46_AP_SPEED:
                                                        // create command with current SETUP/SPD setting (0=TRA, 1=MAN, 2=SOG)
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        pCHTx = (union CH *)&payload[1];
                                                        pCHTx->s.CH = BGNET_CHN_46_AP_SPEED;
                                                        pCHTx->s.FMT = BGN_FMT_01;
                                                        pCHTx->s.cv.uB[0] = 0;
                                                        pCHTx->s.cv.uB[1] = pCtrl->BGNET.PILOT.SETUP.SPD.value.nByte;
                                                        len_pl = 5;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_D4_PILOT_MODE, payload, len_pl);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_D4_PILOT_MODE].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_D4_PILOT_MODE].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                    case BGNET_CHN_07_AP_FR:
                                                        // create command with current FR SELECT
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        payload[1] = BGNET_CHN_07_AP_FR;
                                                        payload[2] = BGN_FMT_08;
                                                        payload[3] = 0;
                                                        payload[4] = pCtrl->BGNET.PILOT.SETUP.FR.value.nByte;
                                                        len_pl = 5;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_D4_PILOT_MODE, payload, len_pl);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_D4_PILOT_MODE].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_D4_PILOT_MODE].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_D1_PILOT_RESPREQ:
                                                // in SETUP, the RESPONSE parameter is configured, 2 bytes payload.
                                                // 0=ACKSEQ, 1=0xA6
                                                switch(pCHRx->s.CH) {
                                                    case BGNET_CHN_A6_AP_TARGET:
                                                        // create command with current RESP SELECT
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        payload[1] = BGNET_CHN_A6_AP_TARGET;
                                                        payload[2] = BGN_FMT_01;
                                                        payload[3] = 0;
                                                        payload[4] = pCtrl->BGNET.PILOT.SETUP.RESP.value.nByte;
                                                        len_pl = 5;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_D5_PILOT_RESPDAT, payload, len_pl);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_D5_PILOT_RESPDAT].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_D5_PILOT_RESPDAT].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                    case BGNET_CHN_46_AP_SPEED:
                                                        // SEND SPD MAN/MED
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        pCHTx = (union CH *)&payload[1];
                                                        pCHTx->s.CH = BGNET_CHN_46_AP_SPEED;
                                                        pCHTx->s.FMT = BGN_FMT_01;
                                                        pCHTx->s.cv.uW[0] = htons(pCtrl->BGNET.PILOT.SPEED.MED.value.nLong);
                                                        len_pl = 5;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_D5_PILOT_RESPDAT, payload, len_pl);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_D5_PILOT_RESPDAT].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_D5_PILOT_RESPDAT].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_D2_PILOT_MANREQ:
                                                // when MAN in SPD setup via SETUP, press the SPEED key, 2 bytes payload.
                                                // 0=ACKSEQ, 1=0x46
                                                switch(pCHRx->s.CH) {
                                                    case BGNET_CHN_46_AP_SPEED:
                                                        // SEND SPD MAN/SLW
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        pCHTx = (union CH *)&payload[1];
                                                        pCHTx->s.CH = BGNET_CHN_46_AP_SPEED;
                                                        pCHTx->s.FMT = BGN_FMT_01;
                                                        pCHTx->s.cv.uW[0] = htons(pCtrl->BGNET.PILOT.SPEED.SLW.value.nLong);
                                                        len_pl = 5;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_D6_PILOT_SLWDAT, payload, len_pl);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_D6_PILOT_SLWDAT].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_D6_PILOT_SLWDAT].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                    case BGNET_CHN_AF_AP_OC_ALARM:
                                                        // SEND WIND_SEL
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        pCHTx = (union CH *)&payload[1];
                                                        pCHTx->s.CH = BGNET_CHN_AF_AP_OC_ALARM;
                                                        pCHTx->s.FMT = BGN_FMT_01;
                                                        pCHTx->s.cv.uB[0] = 0;
                                                        pCHTx->s.cv.uB[1] = pCtrl->BGNET.PILOT.SETUP.WIND_SEL.value.nByte;
                                                        len_pl = 5;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_D6_PILOT_SLWDAT, payload, len_pl);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_D6_PILOT_SLWDAT].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_D6_PILOT_SLWDAT].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_D3_PILOT_AM:
                                                // the AUTO or OFF key was pressed. We receive 5 bytes payload.
                                                // 0=ACKSEQ, 1=0xB5/AP_STATUS, 2=FMT_01, 3=0, 4= new state (1=AUTO , 0=STBY)
                                                switch(pCHRx->s.CH) {
                                                    // SPEED/MAN/FST entered speed value > ENTER
                                                    case BGNET_CHN_46_AP_SPEED:
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
                                                        pCtrl->BGNET.PILOT.SPEED.FST.value.nLong = ntohs(pCHRx->s.cv.uW[0]);
                                                        pCtrl->BGNET.PILOT.SPEED.FST.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.PILOT.SPEED.FST.Counter, t);
                                                        // set the SPD/MAN/SEL to FST
                                                        pCtrl->BGNET.PILOT.SPEED.SEL.value.nByte = BGNET_SPD_MAN_FST;
                                                        pCtrl->BGNET.PILOT.SPEED.SEL.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.PILOT.SPEED.SEL.Counter, t);                                                                
                                                        break;
                                                    // SPEED/MAN/FST entered speed value > ENTER
                                                    case BGNET_CHN_53_TARGET_AWA:
                                                        // STBY to AUTO, new TARGET AWA from PILOT
                                                        pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                        y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                        memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                        decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                        CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);
                                                        // get the new TARGET AWA from the channel
                                                        pCtrl->BGNET.ACP.TARGET_AWA.value.nLong = (int16_t)ntohs(pCHRx->s.cv.uW[0]); // -180/+180
                                                        pCtrl->BGNET.ACP.TARGET_AWA.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.ACP.TARGET_AWA.Counter, t);
                                                        break;
                                                    case BGNET_CHN_A6_AP_TARGET:
                                                        // the PILOT was put into AUTO with a pre-stored course CRS
                                                        // update CRS variable (no CHN)
                                                        pCtrl->BGNET.ACP.CRS.value.nLong = ntohs(pCHRx->s.cv.uW[0]);
                                                        pCtrl->BGNET.ACP.CRS.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.ACP.CRS.Counter, t);
                                                        // update CTS
                                                        pCtrl->BGNET.ACP.CTS.value.nLong = pCtrl->BGNET.ACP.CRS.value.nLong;
                                                        pCtrl->BGNET.ACP.CTS.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.ACP.CTS.Counter, t);
                                                        // make DATA command (to ALL, so also other PILOT units, if present, receive it)
                                                        len_pl = makeACPresponseSTATUSONLY(pCtrl, payload);
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx,BGNET_TYPE_ALL, BGNET_TYPE_NETACP, BGNET_CMD_01_DATA, payload, len_pl);
                                                        register_DATA_frame(pCtrl, pFTx);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_01_DATA].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_01_DATA].CounterTx, t);
                                                        memcpy(pCtrl->DEV[BGNET_TYPE_NETACP].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[BGNET_TYPE_NETACP].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                    case BGNET_CHN_B5_AP_STATUS:
                                                        // update the channel value
                                                        pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                        y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                        memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                        decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                        CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);
                                                        if (pCtrl->CHN[BGNET_CHN_49_HEADING].x01.CounterRx.iCounter == 0) {
                                                            // HEADING was never touched, do not go to AUTO
                                                        } else {
                                                            // i'm not going to make things more complicated at this point. But realise that when
                                                            // we have no accurate HEADING data, i do not know where to steer the boat when going to AUTO
                                                        }
                                                        // realise that when you press AUTO and the AP is already in AUTO, no CMD is
                                                        // send by PILOT, but if you press OFF, it always sends the OFF CMD, also if the
                                                        // AP is already in STBY
                                                        switch (pCHRx->s.cv.uB[1] & BGN_AP_MODE1_MASK_STATE) {
                                                            case BGN_AP_MODE1_AUTO:
                                                                // AUTO key was pressed
                                                                // note: this command is not send of we are already in AUTO, consequently our current state is STBY
                                                                CounterUp(&pCtrl->BGNET.PILOT.KEY.AUTO, t);  // increment key counter
                                                                // it is safe to assume that our current state is STBY. We are dealing with a transition from STBY to AUTO
                                                                // we do something depending on our current MODE
                                                                switch (pCtrl->BGNET.ACP.MODE.value.nByte & BGN_AP_MODE2_MASK_MODE) {
                                                                    case BGN_AP_MODE2_COMP:
                                                                    case BGN_AP_MODE2_NAV:
                                                                        // initialize CTS from current heading (to avoid course change due to AP going into AUTO)
                                                                        pCtrl->BGNET.ACP.CTS.value.nLong = pCtrl->BGNET.ACP.Heading.value.nLong;
                                                                        pCtrl->BGNET.ACP.CTS.source = BGN_SOURCE_BGNET;
                                                                        CounterUp(&pCtrl->BGNET.ACP.CTS.Counter, t);
                                                                        break;
                                                                    case BGN_AP_MODE2_VANE:
                                                                        // initialize TARGET AWA from current AWA (to avoid course change due to AP going into AUTO)
                                                                        pCtrl->BGNET.ACP.TARGET_AWA.value.nLong = pCtrl->BGNET.WIND.AWA.value.nLong;
                                                                        pCtrl->BGNET.ACP.TARGET_AWA.source = BGN_SOURCE_BGNET;
                                                                        CounterUp(&pCtrl->BGNET.ACP.TARGET_AWA.Counter, t);
                                                                        break;
                                                                }
                                                                // set the AUTO state
                                                                pCtrl->BGNET.ACP.STATE.value.nByte = (pCtrl->BGNET.ACP.STATE.value.nByte & ~BGN_AP_MODE1_MASK_STATE) | (pCHRx->s.cv.uB[1] & BGN_AP_MODE1_MASK_STATE);
                                                                break;
                                                            default:
                                                                // OFF key was pressed
                                                                CounterUp(&pCtrl->BGNET.PILOT.KEY.OFF, t); 
                                                                // check joystick state. Turn it OFF when it's ON
                                                                if (pCtrl->BGNET.ACP.Joystick.value.nByte == BGN_JOYSTICK_OFF) {
                                                                    // joystick is OFF, check if we have to go from AUTO to STBY
                                                                    switch (pCtrl->BGNET.ACP.STATE.value.nByte & BGN_AP_MODE1_MASK_STATE) {
                                                                        case BGN_AP_MODE1_AUTO:
                                                                            // Autopilot is engaged, store the current CTS into MEM for later use
                                                                            pCtrl->BGNET.ACP.MEM.value.nLong = pCtrl->BGNET.ACP.CTS.value.nLong;
                                                                            pCtrl->BGNET.ACP.MEM.source = BGN_SOURCE_BGNET;
                                                                            CounterUp(&pCtrl->BGNET.ACP.MEM.Counter, t);
                                                                            // disengage autopilot, new state is OFF/STBY
                                                                            pCtrl->BGNET.ACP.STATE.value.nByte = (pCtrl->BGNET.ACP.STATE.value.nByte & ~BGN_AP_MODE1_MASK_STATE) | (pCHRx->s.cv.uB[1] & BGN_AP_MODE1_MASK_STATE);
                                                                            pCtrl->BGNET.ACP.STATE.source = BGN_SOURCE_BGNET;
                                                                            CounterUp(&pCtrl->BGNET.ACP.STATE.Counter, t);
                                                                            break;
                                                                        default:
                                                                            // Autopilot is not engaged, we are already in STBY; no action
                                                                            break;
                                                                    }
                                                                } else {
                                                                    // joystick is ON, switch it OFF
                                                                    pCtrl->BGNET.ACP.Joystick.value.nByte = BGN_JOYSTICK_OFF;
                                                                    pCtrl->BGNET.ACP.Joystick.source = BGN_SOURCE_BGNET;
                                                                    CounterUp(&pCtrl->BGNET.ACP.Joystick.Counter, t);
                                                                }
                                                                break;
                                                        }
                                                        // make DATA command (to ALL, so also other PILOT units, if present, receive it)
                                                        len_pl = makeACPresponseSTATUSONLY(pCtrl, payload);
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, BGNET_TYPE_ALL, BGNET_TYPE_NETACP, BGNET_CMD_01_DATA, payload, len_pl);
                                                        register_DATA_frame(pCtrl, pFTx);
                                                
                                                        memcpy(pCtrl->CMD[BGNET_CMD_01_DATA].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_01_DATA].CounterTx, t);
                                                        memcpy(pCtrl->DEV[BGNET_TYPE_NETACP].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[BGNET_TYPE_NETACP].CounterTx, t);
                                                        TxSize =TxSize + len_msg;
                                                        break;
                                                }
                                                break;                                               
                                            case BGNET_CMD_D4_PILOT_MODE:
                                                // the MODE key was pressed, 5 bytes payload:
                                                // 0=ACKSEQ, 1=0xA6/AP_CTARGET, 2=FMT_01, 3=0, 4= new state (1=COMP, 3=NAV, 4=VANE)
                                                // the SPEED selector is made (SPEED key, but also PILOT/SETUP/SPD selection)
                                                // 0=ACKSEQ, 1=0x46/AP_SPEED, 2=FMT_01, 3=0, 4= new SPEED mode (0=TRA, 1=MAN, 2=SOG)
                                                // Note: MANUAL speed selection can be set at 3 bands (SLW, MED, FST) via the PILOT/SPEED key
                                                switch(pCHRx->s.CH) {
                                                    // MODE SELECT (COMP/NAV/VANE)
                                                    case BGNET_CHN_A6_AP_TARGET:
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
                                                        if (pCtrl->BGNET.ACP.Joystick.value.nByte == BGN_JOYSTICK_OFF) {
                                                            pCtrl->BGNET.ACP.MODE.value.nByte = (pCtrl->BGNET.ACP.MODE.value.nByte & ~BGN_AP_MODE2_MASK_MODE) | (pCHRx->s.cv.uB[1] & BGN_AP_MODE2_MASK_MODE);
                                                            pCtrl->BGNET.ACP.MODE.source = BGN_SOURCE_ACP;
                                                            CounterUp(&pCtrl->BGNET.ACP.MODE.Counter, t);
                                                        } else {
                                                            // the JOYSTCK mode on a real system is via a physical switch wired to the ACP
                                                            // we can not emulate this.
                                                            // Going IN or OUT of JOYSTCK is not someting we can emulate. I keep it simple:
                                                            // JOYSTCK mode is always OFF (unless someone comes along with a great idea)
                                                        }
                                                        // make DATA command (to ALL, so also other PILOT units, if present, receive it)
                                                        len_pl = makeACPresponseSTATUSONLY(pCtrl, payload);
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx,BGNET_TYPE_ALL, BGNET_TYPE_NETACP, BGNET_CMD_01_DATA, payload, len_pl);
                                                        register_DATA_frame(pCtrl, pFTx);
                                                        memcpy(pCtrl->CMD[BGNET_CMD_01_DATA].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_01_DATA].CounterTx, t);
                                                        memcpy(pCtrl->DEV[BGNET_TYPE_NETACP].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[BGNET_TYPE_NETACP].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                    // SETUP > SPEED SELECT (TRA/MAN/SOG) > ENTER
                                                    case BGNET_CHN_46_AP_SPEED:
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
                                                        pCtrl->BGNET.PILOT.SETUP.SPD.value.nByte = pCHRx->s.cv.uB[1];
                                                        pCtrl->BGNET.PILOT.SETUP.SPD.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.PILOT.SETUP.SPD.Counter, t);
                                                        break;
                                                    // SETUP > FR SELECT (ON/OFF) > ENTER
                                                    case BGNET_CHN_07_AP_FR:
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
                                                        pCtrl->BGNET.PILOT.SETUP.FR.value.nByte = pCHRx->s.cv.uB[1];
                                                        pCtrl->BGNET.PILOT.SETUP.FR.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.PILOT.SETUP.FR.Counter, t);
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_D5_PILOT_RESPDAT:
                                                switch(pCHRx->s.CH) {
                                                    // SPEED/MAN/MED entered speed value > ENTER
                                                    case BGNET_CHN_46_AP_SPEED:
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
                                                        pCtrl->BGNET.PILOT.SPEED.MED.value.nLong = ntohs(pCHRx->s.cv.uW[0]);
                                                        pCtrl->BGNET.PILOT.SPEED.MED.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.PILOT.SPEED.MED.Counter, t);
                                                        // set the SPD/MAN/SEL to MED
                                                        pCtrl->BGNET.PILOT.SPEED.SEL.value.nByte = BGNET_SPD_MAN_MED;
                                                        pCtrl->BGNET.PILOT.SPEED.SEL.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.PILOT.SPEED.SEL.Counter, t);                                                                
                                                        break;
                                                    // IN AUTO/VANE mode one of the 4 up/down keys are pressed. Increment/decrement AWA
                                                    case BGNET_CHN_53_TARGET_AWA:
                                                        // update the channel value
                                                        pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                        y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                        memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                        decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                        CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);
                                                        // increment/decrement TARGET AWA
                                                        // <10/<1 = steer boat to PORT. We receive a positive value (10,1); we need to add the positive numebr
                                                        // 10>/1> = steer boat to STARBOARD. We receive a negative value (-10,-1); we need to add the negative number
                                                        // we need a different MAX/MIN limit (no wrap-around)
                                                        if (pCtrl->BGNET.ACP.TARGET_AWA.value.nLong < 0) {
                                                            pCtrl->BGNET.ACP.TARGET_AWA.value.nLong = BGNET_MAX(pCtrl->BGNET.ACP.TARGET_AWA.value.nLong + (int16_t)ntohs(pCHRx->s.cv.uW[0]), -180);
                                                        } else {
                                                            pCtrl->BGNET.ACP.TARGET_AWA.value.nLong = BGNET_MIN(pCtrl->BGNET.ACP.TARGET_AWA.value.nLong + (int16_t)ntohs(pCHRx->s.cv.uW[0]), +180);
                                                        }
                                                        pCtrl->BGNET.ACP.TARGET_AWA.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.ACP.TARGET_AWA.Counter, t);

                                                        // make DATA command (to ALL, so also other PILOT units, if present, receive it)
                                                        len_pl = makeACPresponseSTATUSONLY(pCtrl, payload);
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, BGNET_TYPE_ALL, BGNET_TYPE_NETACP, BGNET_CMD_01_DATA, payload, len_pl);
                                                        register_DATA_frame(pCtrl, pFTx);
                                                
                                                        memcpy(pCtrl->CMD[BGNET_CMD_01_DATA].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_01_DATA].CounterTx, t);
                                                        memcpy(pCtrl->DEV[BGNET_TYPE_NETACP].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[BGNET_TYPE_NETACP].CounterTx, t);
                                                        TxSize =TxSize + len_msg;
                                                                                                                       
                                                        break;
                                                    // in SETUP, the RESPONSE parameter is configured, 5 bytes payload.
                                                    // 0=ACKSEQ, 1=0xA6/AP_CTARGET, 2=FMT_01, 3=0, 4= new RESP selector
                                                    case BGNET_CHN_A6_AP_TARGET:
                                                        // update the channel value
                                                        pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                        y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                        memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                        decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                        CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);
                                                        pCtrl->BGNET.PILOT.SETUP.RESP.value.nByte = pCHRx->s.cv.uB[1];
                                                        pCtrl->BGNET.PILOT.SETUP.RESP.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.PILOT.SETUP.RESP.Counter, t);
                                                        break;
                                                    // NAV1 is selected when NAV2 is present
                                                    // PILOT sends: FF 50 05 D5 D7 FB AF 01 00 50 AF for as long as the PILOT is in NAV mode and NAV1 is selected (every 2 seconds)
                                                    // this indicates (i believe):
                                                    // When you receive a command 'enter NAV mode' (from me or from the 2nd PILOT): use the NAV1 data FROM 0x50 (in this case)
                                                    // and (if this message is not present), use the NAV data from the other source (if you have it)
                                                    // I believe that if there is a 2nd PILOT in the network, it might send: FF 51 05 D5 D8 FB AF 01 00 51 B0
                                                    case BGNET_CHN_AF_AP_OC_ALARM:
                                                        // update the channel value
                                                        pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                        y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                        memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                        decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                        CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);
                                                        // get the new NAV SELECT from the channel
                                                        pCtrl->BGNET.PILOT.NAV_SEL.value.nByte = pCHRx->s.cv.uB[1]; // this is address of the PILOT using NMEA/NAV data (when there are multiple NAV inputs)
                                                        pCtrl->BGNET.PILOT.NAV_SEL.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.PILOT.NAV_SEL.Counter, t);
                                                        break;
                                                    case BGNET_CHN_EB_SOG:
                                                        // broadcast by PILOT to FF, 3 bytes payload, unknown purpose. Is send when a MANual speed is selected
                                                        // FF 50 03 D5 D9 B5 EB 0F FB
                                                        // 0xB5 is most likely a ACKSEQ (this number changes in other instances)
                                                        // 0xEB seems a channel ID (SOG)
                                                        // 0x0F means "OFF" (0x1F means ON)
                                                        // FB   is a valid CS
                                                        // Because it's a broadcast, it should be relevant for all NETWORK attached units to switch something
                                                        // OFF that has to do with the SOG
                                                        // I think that it when a MANual speed is selected (FST/MED/SLW) the system should stop using the SOG
                                                        // (that might be received via NMEA 0183 interface on the PILOT). For the very good reason, if the skipper
                                                        // wanted to use the SOG, the MAN speed would never be selected (but the SOG instead). So i think that
                                                        // this command is used to tell other nodes to stop using SOG (that might continue to be received)  
                                                        pCtrl->CHN[pCHRx->s.CH].iNewChannel = BGN_NEW_SET;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucTO = pFRx->TO;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucFROM = pFRx->FROM;
                                                        pCtrl->CHN[pCHRx->s.CH].x00.ucCMD = pFRx->CMD;
                                                        y = bgnet_sizeof_channel(pCtrl, pCHRx);
                                                        memcpy(pCtrl->CHN[pCHRx->s.CH].x00.ucRAW, pCHRx->ucRAW, y);
                                                        decode_channel_string(pCtrl->CHN[pCHRx->s.CH].x00.cValue, pCHRx);
                                                        CounterUp(&pCtrl->CHN[pCHRx->s.CH].x00.CounterRx, t);
                                                        if (pCHRx->s.FMT == BGNET_FMT_OFF) {
                                                            if (pCtrl->BGNET.PILOT.NMEA.VTG.VTG_SEL.value.nByte == 0) {
                                                                // is already OFF
                                                            } else {
                                                                pCtrl->BGNET.PILOT.NMEA.VTG.VTG_SEL.value.nByte = 0;
                                                                pCtrl->BGNET.PILOT.NMEA.VTG.VTG_SEL.source = BGN_SOURCE_BGNET;
                                                                CounterUp(&pCtrl->BGNET.PILOT.NMEA.VTG.VTG_SEL.Counter, t);                                                                
                                                            }
                                                        } else {
                                                            // Action to switch ON is when we receive the SOG channel. This                                                                           pCtrl->BGNET.DEPTH.SETUP.DA_STAT.value.nByte = 1;
                                                        }
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_D6_PILOT_SLWDAT:
                                                // the MAN/SLW speed value was entered. We receive 5 bytes payload.
                                                // 0=ACKSEQ, 1=0x46, 2=FMT_01, 3/4= new value (0-990)
                                                switch(pCHRx->s.CH) {
                                                    // SPEED/MAN/SLW entered speed value > ENTER
                                                    case BGNET_CHN_46_AP_SPEED:
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
                                                        pCtrl->BGNET.PILOT.SPEED.SLW.value.nLong = ntohs(pCHRx->s.cv.uW[0]);
                                                        pCtrl->BGNET.PILOT.SPEED.SLW.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.PILOT.SPEED.SLW.Counter, t);
                                                        // set the SPD/MAN/SEL to SLW
                                                        pCtrl->BGNET.PILOT.SPEED.SEL.value.nByte = BGNET_SPD_MAN_SLW;
                                                        pCtrl->BGNET.PILOT.SPEED.SEL.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.PILOT.SPEED.SEL.Counter, t);                                                                
                                                        break;
                                                    // PILOT/SETUP/WIND option entered T or A > ENTER
                                                    case BGNET_CHN_AF_AP_OC_ALARM:
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
                                                        pCtrl->BGNET.PILOT.SETUP.WIND_SEL.value.nByte = pCHRx->s.cv.uB[1];
                                                        pCtrl->BGNET.PILOT.SETUP.WIND_SEL.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.PILOT.SETUP.WIND_SEL.Counter, t);
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_DC_PILOT_STEPREQ:
                                                switch(pCHRx->s.CH) {
                                                    // the <10, >10, <1, >1 key was pressed, 2 bytes payload (at the same time also B1 with +/- 10/1)
                                                    // 0=ACKSEQ, 1=0xA6
                                                    // AT THE SAME TIME also a CMD_B1/A6 command is send with the actual value (-10,+10,-1 or +1)
                                                    case BGNET_CHN_A6_AP_TARGET:
                                                        CounterUp(&pCtrl->BGNET.PILOT.KEY.COURSE, t);
                                                        // create command with current CTS value, but i do not know what command to use for REPLY
                                                        break;
                                                    // when a new set of navigation data is received via the NMEA port, the message "NXT xxx"
                                                    // is displayed
                                                    // While NXT is flashing, the AUTO key is pressed twice. We receive a 2-byte payload:
                                                    // FF 50 02 DC D3 8D DE 3F
                                                    // - ACKSEQ (do not send a CMDACK to a broadcast)
                                                    // - CHID, 0xDE/WPB
                                                    // This commands translate to "copy the value from CHN 0xDE/WPB to CHN 0xA6/CTS"
                                                    // This only works if the navigation data is received via the NMEA port on the PILOT.
                                                    // If we send WPID, WPB and WPD via the network, the pilot does not detect a new course
                                                    // (and does not start flashing NXT). Networked WPB and WPD are displayed on PILOT, but WPID
                                                    // is not.
                                                    // As long as the WPID stays the same, PILOT thinks we're still on the same leg (also if the
                                                    // positional or WPB data is changed)
                                                    // What the difference between 0xDE/WPB and 0xA7/WPB is not yet clear
                                                    case BGNET_CHN_DE_WPB:
                                                        // copy the value from channel WPB to CTS, save current CTS into MEM
                                                        pCtrl->BGNET.ACP.MEM.value.nLong = pCtrl->BGNET.ACP.CTS.value.nLong;
                                                        pCtrl->BGNET.ACP.MEM.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.ACP.MEM.Counter, t);
                                                        // set the Tx pointer to the most recent received value for this channel
                                                        pCHTx = (union CH *)&pCtrl->CHN[pCHRx->s.CH].x01.ucRAW;
                                                        // retrieve the most recent value and put it in our CTS
                                                        pCtrl->BGNET.ACP.CTS.value.nLong = ntohs(pCHTx->s.cv.uW[0]);
                                                        pCtrl->BGNET.ACP.CTS.source = BGN_SOURCE_BGNET;
                                                        CounterUp(&pCtrl->BGNET.ACP.CTS.Counter, t);
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_DE_PILOT_WAREQ:
                                                // in PILOT/SETUP, the WATCH ALARM is configured, 2 bytes payload
                                                // 0=ACKSEQ, 1=0x76
                                                switch(pCHRx->s.CH) {
                                                    case BGNET_CHN_76_AP_WA_ALARM:
                                                        // create command with current WA SELECT
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        pCHTx = (union CH *)&payload[1];
                                                        pCHTx->s.CH = BGNET_CHN_76_AP_WA_ALARM;
                                                        if (pCtrl->BGNET.PILOT.SETUP.WA_STAT.value.nByte == 0) {
                                                            pCHTx->s.FMT = BGN_FMT_08;
                                                        } else {
                                                            pCHTx->s.FMT = BGN_FMT_18;
                                                        }
                                                        pCHTx->s.cv.uW[0] = htons(pCtrl->BGNET.PILOT.SETUP.WA_MIN.value.nLong);
                                                        len_pl = 5;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_21_PILOT_WADAT, payload, len_pl);
                                                
                                                        memcpy(pCtrl->CMD[BGNET_CMD_21_PILOT_WADAT].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_21_PILOT_WADAT].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            case BGNET_CMD_DF_PILOT_OCREQ:
                                                // in PILOT/SETUP, the OFF-COURSE ALARM is configured, 2 bytes payload.
                                                // 0=ACKSEQ, 1=0xAF
                                                switch(pCHRx->s.CH) {
                                                    case BGNET_CHN_AF_AP_OC_ALARM:
                                                        // create command with current OC SELECT
                                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                                        pCHTx = (union CH *)&payload[1];
                                                        pCHTx->s.CH = BGNET_CHN_AF_AP_OC_ALARM;
                                                        if (pCtrl->BGNET.PILOT.SETUP.OC_STAT.value.nByte == 0) {
                                                            pCHTx->s.FMT = BGN_FMT_08;
                                                        } else {
                                                            pCHTx->s.FMT = BGN_FMT_18;
                                                        }
                                                        pCHTx->s.cv.uW[0] = htons(pCtrl->BGNET.PILOT.SETUP.OC_DEG.value.nLong);
                                                        len_pl = 5;
                                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_20_PILOT_OCDAT, payload, len_pl);
                                                
                                                        memcpy(pCtrl->CMD[BGNET_CMD_20_PILOT_OCDAT].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->CMD[BGNET_CMD_20_PILOT_OCDAT].CounterTx, t);
                                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                                        TxSize = TxSize + len_msg;
                                                        break;
                                                }
                                                break;
                                            default:                                                
                                                bSend_CMDACK = 0; // do not send CMDACK
                                                x = bgnet_deviceindex(pCtrl, pFRx->FROM); // lookup by device address/type, so we can print the device name
                                                log_info("  Command %02X (with body) IGNORED (from %02X %s)", pFRx->CMD, pFRx->FROM, pCtrl->pDEVDT[x].sModelName);
                                                break;
                                        }
                                        break;
                                    default:
                                        // we have no information about other (than PILOT, QUAD or WIND) devices doing a handshake with other nodes                                        
                                        bSend_CMDACK = 0; // do not send CMDACK
                                        break;
                                }
                                if (bSend_CMDACK == 0) {
                                    // do not send a CMD-ACK
                                } else {
                                    if (pFRx->TO == BGNET_TYPE_ALL) {
                                        // the CMDDAT/CMDREQ we received was a broadcast. Do not reply to a broadcast
                                    } else {
                                        // make the CMD-ACK for the CMD-REQ/CMD-DAT we received (only for non-default passes)
                                        payload[BGNET_FRAME_ACKSEQ] = pFRx->payload[BGNET_FRAME_ACKSEQ];
                                        len_pl = 1;
                                        pFTx = (FRAME_H *)&TxBuf[TxSize];
                                        len_msg = bgnet_encode_frame(pFTx, pFRx->FROM, BGNET_TYPE_NETACP, BGNET_CMD_ACK, payload, len_pl);

                                        memcpy(pCtrl->CMD[BGNET_CMD_ACK].Tx.ucFrame, pFTx, len_msg);
                                        CounterUp(&pCtrl->CMD[BGNET_CMD_ACK].CounterTx, t);
                                        memcpy(pCtrl->DEV[pFRx->FROM].Tx.ucFrame, pFTx, len_msg);
                                        CounterUp(&pCtrl->DEV[pFRx->FROM].CounterTx, t);
                                        TxSize = TxSize + len_msg;

                                        // put it for transmission
                                        pthread_mutex_lock(&g_data_mutex);
                                        memcpy(&g_last_serial_dataTx[g_serial_data_lenTx], &TxBuf, TxSize); // put output message in write buffer
                                        g_serial_data_lenTx = g_serial_data_lenTx + TxSize; // increment byte counter in TX buffer
                                        pthread_mutex_unlock(&g_data_mutex);
                                    }                                
                                }
                                break;
                        }
                        // register all received CMD's
                        pCtrl->iNewCommand = BGN_NEW_SET; // signal possible CMD change
                        pCtrl->CMD[pFRx->CMD].iNewCommand = BGN_NEW_SET; // signal this command
                        memcpy(pCtrl->CMD[pFRx->CMD].Rx.ucFrame, pFRx, RxSize);
                        CounterUp(&pCtrl->CMD[pFRx->CMD].CounterRx, t);
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
