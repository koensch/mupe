#ifndef BGNPROTOSUB_H
#define BGNPROTOSUB_H

#include <stdio.h>
#include <time.h>
#include <limits.h>
#include <string.h>

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
    for (i=1; i<BGNET_DEVICE_MAX; i++) {
        // devicetype in table first match with deviceaddress/type
        if ((pCtrl->pDev[i].iDeviceType & BGNET_DEVICETYPE_MASK) == (ucDevAddr & BGNET_DEVICETYPE_MASK)) {
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
        pCtrl->iInitialized = BGNET_CTRL_INIT_RESET; // reset initialized state
}

void bgn_net_initialize(BGNET_CTRL *pCtrl) {
    int i;
    time_t t;

    if (pCtrl->iInitialized != BGNET_CTRL_INIT_RESET){
        // controlblock is already initialized. This meas that this initialisation only happens if the 'initialized state' was reset
        // prior this function.
    } else {

        time(&t); // save timestamp to we use same timestamp everywhere
        pCtrl->iInitialized = BGNET_CTRL_INIT; // mark as initialized
        pCtrl->iDeviceCounter = BGNET_CTRL_DEVCNTR_INIT;
        pCtrl->iDeviceRun = 0; //reset run counter (0-1000)
        pCtrl->tx_cnt = 0; // reset write buffer

        // stage 1  -initialize command list
        pCtrl->iNewDevice = 0; // no new data available
        // initialize CMD array
        for (i=0; i<BGNET_COMMAND_MAX; i++) {
            pCtrl->CMD[i].iCounter = 0;
            pCtrl->CMD[i].tTimestamp = t;
        }

        // stage 2 - initialize channel list
        for (i=0; i<BGN_CHANNEL_MAX; i++) {
            pCtrl->DATA[i].ucTO = 0;
            pCtrl->DATA[i].ucFROM = 0;
            pCtrl->DATA[i].ucBYTES = 0;
            pCtrl->DATA[i].ucCHN = 0;
            pCtrl->DATA[i].ucFMT = 0;
            pCtrl->DATA[i].tTimeStamp = t;
            strcpy(pCtrl->DATA[i].cValue, "");
            memset(pCtrl->DATA[i].ucRAW, 0, sizeof(pCtrl->DATA[i].ucRAW));
        }
        log_info("bgn_net_initialize: done");
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
        default:
            xSize = 0;
            break;
    }
    log_info("bgn_net_makemessage: %ld (bytes) for cmd=%02X, payload= %d", xSize, iCmd, payloadlength);
    return(xSize);
}

// a paylod for ACP response contains 4 channels (53, A6, A7, B5) + a bonus to verify that the message is received
int makeACPresponse(unsigned char* payload) {
    int len;
    len = 0;
    payload[0]=BGN_CNR_53_TARGET_TWA;
    payload[1]=0x01;
    payload[2]=0xFF;
    payload[3]=0xF6; // 0xFFF6 = -9
    payload[4]=BGN_CNR_A6_AP_CTARGET;
    payload[5]=0x08;
    payload[6]=0x01;
    payload[7]=0x61; // 0161 = 353 degrees
    payload[8]=BGN_CNR_A7_AP_WPB;
    payload[9]=0x08;
    payload[10]=0x01;
    payload[11]=0x67; // 0167 = 359
    payload[12]=BGN_CNR_B5_AP_MODE;
    payload[13]=0x08; // must be 0x08!
    payload[14]=0x01; // 01 = 0000 0001 = AUTO/353 (CH A6)
    payload[15]=0x03; // 03 = 0000 0011 = STBY/NAV/"XTE>032"
    payload[16]=BGN_CNR_8D_BATTVOLTS;
    payload[17]=0x48;
    payload[18]=0x00;
    payload[19]=0x80; // 80 = 128 = 12.8
    len = 20;
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

    log_info("bgn_net_process: Start");
    size_t r = 0;

    // device received new data
    if ((pCtrl->iNewDevice < 1)) {
        // nothing to do
    } else {
        // Stage 1 - check for new commands
        for (i=0; i<BGNET_COMMAND_MAX; i++) {
            if (pCtrl->CMD[i].iNewCommand < 1) {
                // no new commands
            } else {
                log_info("bgn_net_process: Command 0x%0X processing", i);
                switch(i) {
                    case BGNET_FRAME_CMD_KEEPALIVE:
                        // we received a KEEPALIVE signal (only by PILOT).
                        // generate a ACP response message
                        len_pl = makeACPresponse(payload);
                        len_msg = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, BGNET_ADDRESS_ME, BGNET_FRAME_CMD_DATA, payload, len_pl);
                        memcpy(&buffer[r],&tmp_buffer,len_msg); // put output message in write buffer
                        r = r + len_msg; // increment byte counter in TX buffer
                        break;
                    case BGNET_FRAME_CMD_POWERUP:
                        // we received a powerup from the network. This happens when the network did a POWERUP while we are running
                        // to synchronize what we are doing, set the init request.
                        bgn_net_initialize_reset(pCtrl);
                        break;
                    case BGNET_FRAME_CMD_LATLON:
                    case BGNET_FRAME_CMD_DATA:
                        // these 2 commands send channel data; data was placed into the DATA structure, no response required
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
                // , size_tprocess new channel data
                pCtrl->DATA[i].iNewChannel = 0; // reset newdata flag
            }
        }
        pCtrl->iNewDevice = 0; // reset new data flag for this device
    }
    // if we produced some response, show it here
    if (r > 0) {
        byte2ascii(asciibuffer, sizeof(asciibuffer), buffer, r, BYTE2ASCII_SPACED);
        log_info("bgn_net_process: output: %s (%ld bytes)", asciibuffer, r);
    }
    log_info("bgn_net_process: Done, return= %d", r);
    return(r);
};

// value length for DATA element (command 1 & 3)
static int value_length_DATA(unsigned char cnr, unsigned char fmt) {
    int r;
    switch (cnr) {
        case BGN_CNR_00_NODERESET:
            r=1;
            break;
        case BGN_CNR_68_REQFORDATA:
            r=2;
            break;
        case BGN_CNR_01_UNKNOWN:
        case BGN_CNR_03_UNKNOWN:
        case BGN_CNR_0B_RUDDER:
        case BGN_CNR_1C_AIRTEMPF:
        case BGN_CNR_1D_AIRTEMPC:
        case BGN_CNR_1E_SEATEMPF:
        case BGN_CNR_1F_SEATEMPC:
        case BGN_CNR_32_TACKPERF:
        case BGN_CNR_33_REACHPERF:
        case BGN_CNR_35_OPTWINDANGL:
        case BGN_CNR_36_DEPTHGAIN:
        case BGN_CNR_41_BOATSPEEDKTS:
        case BGN_CNR_42_BOATSPEEDRAW:
        case BGN_CNR_44_YAWRATE:
        case BGN_CNR_49_HEADING:
        case BGN_CNR_4D_AWINDSPEEDKTS: 
        case BGN_CNR_4E_AWINDSPEEDRAW:
        case BGN_CNR_4F_AWINDSPEEDMS:
        case BGN_CNR_50_AWINDSPEEDKT:
        case BGN_CNR_51_AWINDANGLE:
        case BGN_CNR_52_AWINDANGLERAW:
        case BGN_CNR_53_TARGET_TWA:
        case BGN_CNR_56_TWINDSPEEDMS:
        case BGN_CNR_57_TWINDSPEEDKT:
        case BGN_CNR_59_TWINDANGLE:
        case BGN_CNR_64_SPEEDAVGKTS:
        case BGN_CNR_65_SPEEDAVGRAW :
        case BGN_CNR_69_COURSE:
        case BGN_CNR_76_UNKNOWN:
        case BGN_CNR_7F_VMGKT:
        case BGN_CNR_80_UNKNOWN:
        case BGN_CNR_81_DR_DIST:
        case BGN_CNR_8A_UNKNOWN:
        case BGN_CNR_8D_BATTVOLTS:
        case BGN_CNR_A6_AP_CTARGET:
        case BGN_CNR_A7_AP_WPB:
        case BGN_CNR_AF_AP_OFFCOURSE:
        case BGN_CNR_B5_AP_MODE:
        case BGN_CNR_C1_DEPTHM:
        case BGN_CNR_C2_DEPTHFT:
        case BGN_CNR_C3_DEPTHFM:
        case BGN_CNR_D3_DR_COURSE:
        case BGN_CNR_E9_COGTRUE:
        case BGN_CNR_EA_COGMAG:
        case BGN_CNR_CF_TRIPLOGNM:
        case BGN_CNR_DE_UNKNOWN:
        case BGN_CNR_EB_SOGKT:
        case BGN_CNR_FC_SHALLOWALARM:
        case BGN_CNR_FF_DEPTH_CAL:
            r=4;
            break;
        case BGN_CNR_46_APSPEEDKTS:
        case BGN_CNR_47_LONLAT:
        case BGN_CNR_75_TIMER:
        case BGN_CNR_EE_CROSSTRACKERR:
            r=5;
            break;
        case BGN_CNR_20_SEATEMPQ: // sometimes 4, sometimes 6
            // i am confused about this value: 20-04=6 bytes, 20-41 seems 4 bytes
            if (fmt == 0x04) {
                r=6;
            } else{
                r=4;
            }
            break;
        case BGN_CNR_27_HEADLIFTTREND:
        case BGN_CNR_31_ECHO_SNR:
        case BGN_CNR_34_HEELANGLE:
        case BGN_CNR_4A_HEADINGRAW:
        case BGN_CNR_82_LEEWAY:
        case BGN_CNR_86_BAROTREND:
        case BGN_CNR_87_BARORAW:
        case BGN_CNR_C4_DEPTHM:
        case BGN_CNR_CD_STORLOGNM:
        case BGN_CNR_FD_DEPTHALARM:
        case BGN_CNR_FE_ANCHORALARM:
            r=6;
            break;
        case BGN_CNR_37_DEPTHNOISE:
            r=7;
            break;
        case BGN_CNR_E8_WAYPDISTGYRO:
            r=8;
            break;
        default:
            r=-1; // undefined
            break;
    }
    return(r);
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

// insert a 'divisor' (".") in the string. Realize that we could have a case where "0" should
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
    log_debug("sValue %s (len=%d, index=%d) -> step 1 = %s, shift= %d", string, l, n, cWork, x);
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
    log_debug("right-shift %s, substring %s, n= %d, start= %d, size= %d", cWork, strC, n, start[n], size[n]);
    return(strC);
}

// convert the channel data to ASCII for display purposes
char *decode_value(char* pStr, unsigned char* pBuf) {
    //unsigned int d, s, f;
    char sWork[256];
    union ValueUnion {
        int i;
        char c[8];
    } VU;
    // mask the low nibble (conversion factor, bits 3-0)
    switch (pBuf[BGN_VALUE_FMT] & BGN_FMT_MASK_CON) {
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

// routine to process values in the body. A body might contain multiple values
int process_CMD_DATA(BGNET_CTRL* pCtrl, unsigned char *buffer) {
    int y,z;
    unsigned int d,s,f;
    unsigned char *pBuf,*pEnd;
    unsigned char ucCh;
    char sValue[256];

    pBuf = buffer+BGNET_FRAME_HEAD;
    pEnd = buffer+BGNET_FRAME_HEAD + buffer[BGNET_FRAME_HEAD_SIZE];
    z=0;
    while (pBuf < pEnd) {
        ucCh = *(pBuf+BGN_VALUE_CNR);
        y = value_length_DATA(ucCh, *(pBuf+BGN_VALUE_FMT));
        fmt_decode(*(pBuf+BGN_VALUE_FMT), &d, &s, &f);
        if(y < 0) {
            log_debug("    Ch=%02X Channel unknown, confused (size=%d),FMT=%02X (%x,%x,%x)", ucCh, y, *(pBuf+BGN_VALUE_FMT), d, s, f);
            break;
        } else {
            decode_value(sValue,pBuf+BGN_VALUE_CNR);
            log_debug("    Ch=%02X Channel OK (size=%2d),FMT=%02X, Value= %s (%02X, %02X) %s", ucCh, y, f, sValue, *(pBuf+BGN_VALUE_HEAD+0), *(pBuf+BGN_VALUE_HEAD+1), pCtrl->pChl[ucCh].sLabel);
            //
        }
        pBuf = pBuf+y;
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
        // adressed to ALL
        // adressed to ME
        // adressed to MYTYPE
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
    int iIsForMe;
    time_t t;

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
            // we do not want to exclude messages not meant for me, but i still like to see them, so i change the condition to  "(1>0)"
            // for the time being.
            // Realize that every device is 'not active' initially. Untill we get some traffic we know that we have an 'active' device
            // on the bus (a device sending data; we do not care about devices that are present, but do not send).
            if (iIsForMe < 1) {
                log_info("                   IGNORE (it's from myself)");
                pBuf = pEnd; // dump whole frame, assume everything has the same FROM (otherwise they would not be joined as subframes)
            } else {
                // The HEADER is OK, process it. Note that some CMD's do not have a body and in such case, we should skip the body & CS part.

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
                    switch(*(pBuf+BGNET_FRAME_HEAD_CMD)) {
                        case BGNET_FRAME_CMD_KEEPALIVE:
                            log_info("  Command %02X (header only) KEEPALIVE by %02X %s",*(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDev[x].sModelName);
                            pCtrl->iNewDevice = 1; // signal this device
                            pCtrl->CMD[BGNET_FRAME_CMD_KEEPALIVE].iNewCommand = 1; // signal this command
                            pCtrl->CMD[BGNET_FRAME_CMD_KEEPALIVE].iCounter++;
                            pCtrl->CMD[BGNET_FRAME_CMD_KEEPALIVE].tTimestamp = t;
                            strcpy(pCtrl->CMD[BGNET_FRAME_CMD_KEEPALIVE].cFrame, asciibuffer);
                            break;
                        case BGNET_FRAME_CMD_POWERUP:
                            log_info("  Command %02X (header only) POWERUP by %02X %s",*(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDev[x].sModelName);
                            pCtrl->iNewDevice = 1; // signal this device
                            pCtrl->CMD[BGNET_FRAME_CMD_POWERUP].iNewCommand = 1; // signal this command
                            pCtrl->CMD[BGNET_FRAME_CMD_POWERUP].iCounter++;
                            pCtrl->CMD[BGNET_FRAME_CMD_POWERUP].tTimestamp = t;
                            strcpy(pCtrl->CMD[BGNET_FRAME_CMD_KEEPALIVE].cFrame, asciibuffer);
                            break;
                        default:
                            log_info("  Command %02X (header only) IGNORED from %02X %s)",*(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDev[x].sModelName);
                            break;
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
                        // CR is good, log frame
                        byte2ascii(asciibuffer, sizeof(asciibuffer), pBuf, xSize, BYTE2ASCII_SPACED);
#ifdef FRAME_DEBUG
                        log_debug("  Frame Checksum OK, framesize: %d, data: %s", xSize, asciibuffer);
#endif
                        // commands with payload (otherwise we would have no CR)
                        switch(*(pBuf+BGNET_FRAME_HEAD_CMD)) {
                            case BGNET_FRAME_CMD_DATA:
                            case BGNET_FRAME_CMD_LATLON:
                                // process DATA/LATLON message
                                n = process_CMD_DATA(pCtrl, pBuf);
                                log_info("  Command %02X (with body), %d channels",*(pBuf+BGNET_FRAME_HEAD_CMD), n);
                                break;
                            case BGNET_FRAME_CMD_LABEL:
                                // process message (only at startup)
                                process_CMD_VALUE(pBuf);
                                log_info("  Command %02X (with body) LABEL",*(pBuf+BGNET_FRAME_HEAD_CMD));
                                break;
                            case BGNET_FRAME_CMD_SUBSCRIBE:
                                // process message (only at startup)
                                process_CMD_SUBSCRIBE(pBuf);
                                log_info("  Command %02X (with body) SUBSCRIBE",*(pBuf+BGNET_FRAME_HEAD_CMD));
                                break;
                            case BGNET_FRAME_CMD_BACKLIGHT:
                                log_info("  Command %02X BACKLIGHT BUTTON PRESSED (BY: %02X, %s, LEVEL: %02X)",*(pBuf+BGNET_FRAME_HEAD_CMD),*(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDev[x].sModelName, *(pBuf+BGNET_FRAME_HEAD+0));
                                pCtrl->iNewDevice = 1; // signal this device
                                pCtrl->CMD[BGNET_FRAME_CMD_BACKLIGHT].iNewCommand = 1; // signal this command
                                pCtrl->CMD[BGNET_FRAME_CMD_BACKLIGHT].iCounter++;
                                pCtrl->CMD[BGNET_FRAME_CMD_BACKLIGHT].tTimestamp = t;
                                strcpy(pCtrl->CMD[BGNET_FRAME_CMD_BACKLIGHT].cFrame, asciibuffer);
                                break;
                            default:
                                log_info("  Command %02X (with body) IGNORED (from %02X %s)",*(pBuf+BGNET_FRAME_HEAD_CMD), *(pBuf+BGNET_FRAME_HEAD_FROM), pCtrl->pDev[x].sModelName);

                                break;
                        }
                    } else {
                        log_info("    Frame Checksum BAD, framesize: %d", xSize);
                        // Bad CS, we just skip processing the body portion (header was OK and we could determine the frame length)
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

// Example: build a frame to transmit (placeholder)
// Return vale: 0-nothing to send, >0 data to send
//size_t build_tx_frame(BGNET_CTRL* pCtrl, unsigned char *pBuf, size_t maxlen) {
//    size_t r;
//    // when we have an output message from the "devices processor", copy it to the "writer process" and reset the output buffer
//    r=0; // assume nothing to write
//    if (pCtrl->tx_cnt > 0) {
//        if (pCtrl->tx_cnt < maxlen) {
//            memcpy(pBuf, pCtrl->tx_buf, pCtrl->tx_cnt); // copy output message
//            r = pCtrl->tx_cnt; // save byte count to return
//        } else {
//            log_error("write buffer overflow (len=%d, max=%d), no output", pCtrl->tx_cnt, maxlen);
//        }
//    }
//    pCtrl->tx_cnt = 0; // reset message byte count
//    return(r); // return bytes in output message
//}

#endif
