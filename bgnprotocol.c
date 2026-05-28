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
int bgn_FindDeviceIndex(BGNET_CTRL* pCtrl, unsigned char cDevType) {
    int i,r;
    r=-1; // no match is not found
    for (i=0; i<BGNET_DEVICES; i++) {
        if (pCtrl->pDev[i].iDeviceType == cDevType) {
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
        pCtrl->iInitialized = BNG_DEVCTRL_INIT_RESET; // reset initialized state
}

void bgn_net_initialize(BGNET_CTRL *pCtrl) {
    int i;
    if (pCtrl->iInitialized != BNG_DEVCTRL_INIT_RESET){
        // controlblock is already initialized. This meas that this initialisation only happens if the 'initialized state' was reset
        // prior this function.
    } else {
        pCtrl->iInitialized = BNG_DEVCTRL_INIT; // mark as initialized
        pCtrl->iNumberOfDevices = BGNET_DEVICES; // number of devices in our controlblock
        pCtrl->tx_cnt = 0; // reset write buffer
        //loop through controlblock; reset every device (if enabled)
        for (i=0; i<pCtrl->iNumberOfDevices; i++) {
            // only if the device is enabled and has a valid devicetype
            if ((pCtrl->pDev[i].iEnable > 0) & (pCtrl->pDev[i].iDeviceType > BGNET_FRAME_FROM_UNKNOWN)) {
                pCtrl->pDev[i].iActive = 0; // make it inactive
                // we assign a device address based on devicetype. There might be multiple devices of the same type on the NETWORK and
                // create duplicate addresses.
                // For some devices, it's OK to have duplicate addresses, some types (like PILOT) will do some deviceaddress negotiations.
                pCtrl->pDev[i].iDeviceAddress = pCtrl->pDev[i].iDeviceType;
            }
        }
        log_info("bgn_net_initialize: done %d (devices)\n",i);
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
    log_info("bgn_net_makemessage: %ld (bytes) for cmd=%02X, payload= %d (bytes), length= %ld (bytes)\n",xSize,iCmd,payloadlength,xSize);
    return(xSize);
}

// This is the "device processor". This code determines the behaviour of the NETWORK devices that we emulate
// in this driver ("Virtual Devices").
// The process that receives data from the serial input might (for some receved messages) set some state, wheas
// the "device processor" might create an output message.
// This process is called after every received message completed. The output (in the form of an output message) is written to the 
// output buffer.
void bgn_net_process(BGNET_CTRL *pCtrl) {
    unsigned char tmp_buffer[256],payload[256];
    char asciibuffer[256];
    size_t i, len;
    log_info("Device Processing Start");
    pCtrl->tx_cnt=0; // reset byte count in TX buffer
    // loop every item in the device structre
    for (i=0;i<pCtrl->iNumberOfDevices;i++) {
        if ((pCtrl->pDev[i].iEnable==0)) {
            // device not enabled, skip it.
        } else {
            // device enabled, process it
            if ((pCtrl->pDev[i].iActive>0)) {
                // device was initialized; process it
                // First we do some common things

                // Secondly we do some device specific things
                switch(pCtrl->pDev[i].iDeviceType) {
                    case BGNET_FRAME_FROM_NETPILOT:
                        if (pCtrl->BGNET_devdata[i].bgn_NETPILOT.iSendTest>0) {
                            payload[0]=BGN_CNR_49_HEADING;
                            payload[1]=0x08;
                            payload[2]=0;
                            payload[3]=60;
                            payload[4]=BGN_CNR_0B_RUDDER;
                            payload[5]=0x01;
                            payload[6]=0;
                            payload[7]=7;
                            payload[8]=BGN_CNR_51_AWINDANGLE;
                            payload[9]=0x01;
                            payload[10]=0;
                            payload[11]=57;
                            payload[12]=BGN_CNR_41_BOATSPEEDKTS;
                            payload[13]=0x08;
                            payload[14]=0;
                            payload[15]=153;
                            //FE(ANCHORALARM),FC(SHALLOWALARM),C4(DEPTHM),FD(DEPTHALARM),41(SPEEDKT),65(MAXSPEEDKT),
                                         //                     CD(LOGNM),64(AVGSPEEDKT),CF(TRIPNM),81(DRDISTANCENM),D3(DRCOURSEM)
                            len=bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, pCtrl->pDev[i].iDeviceAddress, BGNET_FRAME_CMD_DATA, payload, 16);
                            if (len<BGNET_FRAME_HEAD) {
                                // every message must be at least have a header; must be an error.
                            } else {
                                memcpy(&pCtrl->tx_buf[pCtrl->tx_cnt],&tmp_buffer,len); // put output message in write buffer
                                pCtrl->tx_cnt=pCtrl->tx_cnt+len; // increment byte counter in TX buffer
                            }
                            log_info("Reset testmessage request by %s",pCtrl->pDev[i].sModelName);
                            pCtrl->BGNET_devdata[i].bgn_NETPILOT.iSendTest=0;
                        }
                        if (pCtrl->BGNET_devdata[i].bgn_NETPILOT.iSendACP>0) { // simulate response from ACP
                            //ACP: payload: 8E 01 07 2C FF D8 00 00 5A
                            payload[0]=0x8E;
                            payload[1]=0x01;
                            payload[2]=0x07;
                            payload[3]=0x2C;
                            payload[4]=0xFF;
                            payload[5]=0xD8;
                            payload[6]=0x00;
                            payload[7]=0x00;
                            len = bgn_net_makemessage(tmp_buffer, 0x51, 0x00, 0x01, payload, 8);
                            if (len<BGNET_FRAME_HEAD) {
                                // every message must be at least have a header; must be an error.
                            } else {
                                memcpy(&pCtrl->tx_buf[pCtrl->tx_cnt], &tmp_buffer, len); // put output message in write buffer
                                pCtrl->tx_cnt = pCtrl->tx_cnt+len; // increment byte counter in TX buffer
                            }
                            log_info("ACP response send by %s",pCtrl->pDev[i].sModelName);
                            //QUAD: 42(DEPTHM),FC(SHALLOWALARM),4A(DEPTHM),FE(ANCHORALARM),FF(DEPTHCAL),FD(DEPTHALARM),C4(DEPTHM),41(SPEEDKT),
                            //       CD(LOGNM),31(ECHOSNR),64(AVGSPEEDKT),CF(TRIPNM),20(TEMPC),65(MAXSPEEDKT),75(TIMER),81(DRDISTANCENM),D3(DRCOURSEM)
                            //send data (depth)
                            //payload[0]=BGN_CNR_C4_DEPTHM; // displayed on QUAD/30 12 1.2M
                            //payload[1]=0x48;
                            //payload[2]=0;
                            //payload[3]=12;
                            payload[0]=BGN_CNR_1F_SEATEMPC;
                            payload[1]=0x48;
                            payload[2]=0;
                            payload[3]=21;
                            payload[4]=BGN_CNR_41_BOATSPEEDKTS;
                            payload[5]=0x48;
                            payload[6]=0;
                            payload[7]=23;
                            payload[8]=BGN_CNR_51_AWINDANGLE;
                            payload[9]=0x01;
                            payload[10]=0;
                            payload[11]=34;
                            payload[12]=BGN_CNR_65_SPEEDAVGRAW; // displayed on DATA/60
                            payload[13]=0x48;
                            payload[14]=0;
                            payload[15]=45;
                            payload[16]=BGN_CNR_64_SPEEDAVGKTS; // displayed on DATA/60
                            payload[17]=0x48;
                            payload[18]=0;
                            payload[19]=56;
                            payload[20]=BGN_CNR_59_TWINDANGLE;
                            payload[21]=0x08;
                            payload[22]=0;
                            payload[23]=123;
                            payload[24]=BGN_CNR_56_TWINDSPEEDMS;
                            payload[25]=0x08;
                            payload[26]=0;
                            payload[27]=111;
                            payload[28]=BGN_CNR_4F_AWINDSPEEDMS;
                            payload[29]=0x08;
                            payload[30]=0;
                            payload[31]=222;
                           len=bgn_net_makemessage(tmp_buffer, 0xFF, 0x32, 0x01, payload, 32);
                            if (len<BGNET_FRAME_HEAD) {
                                // every message must be at least have a header; must be an error.
                            } else {
                                memcpy(&pCtrl->tx_buf[pCtrl->tx_cnt], &tmp_buffer, len); // put output message in write buffer
                                pCtrl->tx_cnt = pCtrl->tx_cnt+len; // increment byte counter in TX buffer
                            }
                            log_info("ACP response send by %s",pCtrl->pDev[i].sModelName);
                            pCtrl->BGNET_devdata[i].bgn_NETPILOT.iSendACP = 0;
                        }
                        break;
                    case BGNET_FRAME_FROM_NETPILOT+1:


                        break;
                    default:
                        break;
                }
            } else {
                // device was not initialized yet, send POWERUP, use devicetype as FROM (we do not have allocated a own address)
                len = bgn_net_makemessage(tmp_buffer, BGNET_FRAME_TO_ALL, pCtrl->pDev[i].iDeviceAddress, BGNET_FRAME_CMD_POWERUP, payload, 0);
                if (len < BGNET_FRAME_HEAD) {
                    // every message must be at least have a header; must be an error.
                } else {
                    memcpy(&pCtrl->tx_buf[pCtrl->tx_cnt], &tmp_buffer, len); // put output message in write buffer
                    pCtrl->tx_cnt = pCtrl->tx_cnt+len; // increment byte counter in TX buffer
                }
                pCtrl->pDev[i].iActive=1; // make this device active; POWERUP message is only send once.
                log_info("built POWERUP message for %s, address %02X\n",pCtrl->pDev[i].sModelName,pCtrl->pDev[i].iDeviceType);
            }
        }
    }
    if (pCtrl->tx_cnt > 0) {
        byte2ascii(asciibuffer, sizeof(asciibuffer), pCtrl->tx_buf, pCtrl->tx_cnt, BYTE2ASCII_SPACED);
        log_info("TX buffer: %s (%ld bytes)", tmp_buffer, pCtrl->tx_cnt);
    }
    log_info("Device Processing Completed\n");
    return;
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
        case BGN_CNR_C3_DEPTHF:
        case BGN_CNR_D3_DR_COURSE:
        case BGN_CNR_E9_COGTRUE:
        case BGN_CNR_EA_COGMAG:
        case BGN_CNR_CF_TRIPLOGNM:
        case BGN_CNR_DE_UNKNOWN:
        case BGN_CNR_EB_SOG:
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
            if (fmt==0x04) {
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

// we do not wat to handle our value as float, but as a integer with a divisor
//char *strDecimals(unsigned char* pStr, int iV, int iS, int iD) {
//    //convert value (2 bytes, -32766/+32767) to ascii string for display purposes
//    sprintf(pStr,"%06d", iV);
//    return(pStr);
//}

// insert a 'divisor' (".") in the string
char* insertDivisor(char* strC, char* string, int pos) {
    int n;
    n=strlen(string)-pos;
    strncpy(strC,string,n); // copy the first portion of string1
    strC[n] = '\0'; // append NULL character
    strcat(strC,"."); // append "."
    strcat(strC,string+n); // append remainder of string1
    return(strC);
}

// convert the channel data to ASCII for display purposes
char *decode_value(char* pStr, unsigned char* pBuf) {
    unsigned int d, s, f;
    char sWork[256];
    union ValueUnion {
        int i;
        char c[8];
    } VU;
    //int fmt_sd[]={1,10,100,1000};
    fmt_decode(pBuf[BGN_VALUE_FMT],&d,&s,&f);
    //map the FMT (00-0F) to datatype (2 or 4 bytes)
    switch (f) {
        case 0x00://used for CMD=3/ASCII (lon/lat position information) 
            sprintf(pStr, "untested");
            break;
        case 0x05: //Unknown, untested
        case 0x06:
        case 0x07:
            sprintf(pStr, "untested");
            break;
        case 0x0A:
            //32bits, signed (4 bytes), untested, be carefult with endianess
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
            sprintf(pStr,"%02d:%02d:%02d", pBuf[2], pBuf[3], pBuf[4]);        
            break;
        case 0x04: //3 ASCII characters, tested
            pStr[0] = pBuf[3];
            pStr[1] = pBuf[4];
            pStr[2] = pBuf[5];
            pStr[3] = 0x00; // string terminator
            break;
        case 0x01: //16bits, signed (2 bytes), tested, could be negative, like rudderangle (-45/+45), windangle (-180/+180)
        case 0x08: //can not become negative, but still fits in a 16bits signed value, like speed (0-nn.nn),course (0-360)
            //be carefult with endianess
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
    switch (d) {
        case 0: // no decimals, leave like it is
            break;
        default:
            insertDivisor(sWork, pStr, d); // move a "." inside the value string
            strcpy(pStr, sWork); // return it
            break;
    }
    return(pStr);
}

// routine to process values in the body. A body might contain multiple values
int process_body_DATA(unsigned char *buffer) {
    int y,z;
    unsigned int d,s,f;
    unsigned char *pBuf,*pEnd;
    char sValue[256];
    //const char *strDiv[]={"1.0","10.0","100.0","1000.0"};
    //const char *strSiz[]={"0000","000","00","0"};

    pBuf = buffer+BGNET_FRAME_HEAD;
    pEnd = buffer+BGNET_FRAME_HEAD+buffer[BGNET_FRAME_HEAD_SIZE];
    z=0;
    while (pBuf < pEnd) {
        y = value_length_DATA(*(pBuf+BGN_VALUE_CNR),*(pBuf+BGN_VALUE_FMT));
        fmt_decode(*(pBuf+BGN_VALUE_FMT),&d,&s,&f);
        if(y < 0) {
            log_debug("    Ch=%02X Channel unknown, confused (size=%d),FMT=%02X (%x,%x,%x)",*(pBuf+BGN_VALUE_CNR),y,*(pBuf+BGN_VALUE_FMT),d,s,f);
            break;
        } else {
            decode_value(sValue,pBuf+BGN_VALUE_CNR);
            log_debug("    Ch=%02X Channel OK (size=%2d),FMT=%02X, Value=%s (%02X, %02X)",*(pBuf+BGN_VALUE_CNR),y,f,sValue,*(pBuf+BGN_VALUE_HEAD+0),*(pBuf+BGN_VALUE_HEAD+1));
        }
        pBuf = pBuf+y;
        z++;
    }
    return(z); // return number of processed values
}
// routine to process values in the body. A body might contain multiple values
void process_body_SUBSCRIBE(unsigned char *buffer) {
    // this type of frame shall have a payload of 1 byte
    if (buffer[BGNET_FRAME_HEAD_SIZE]==1) {
        log_debug("    Ch=%02X, Subscribed by %02X on %02X",buffer[BGNET_FRAME_HEAD+0],buffer[BGNET_FRAME_HEAD_FROM],buffer[BGNET_FRAME_HEAD_TO]);
    }
    return;
}
// routine to advertise values that can be subscribed
void process_body_VALUE(unsigned char *buffer) {
    unsigned char string[255];
    size_t len;
    // this type of frame has a variable payload: Ch#, 3 bytes data, nbytes ascii
    if (buffer[BGNET_FRAME_HEAD_SIZE]>4) {
        len=buffer[BGNET_FRAME_HEAD_SIZE]-4;
        log_debug("  1st=%02X, len= %ld",buffer[BGNET_FRAME_HEAD+4],len);
        memcpy(string, &buffer[BGNET_FRAME_HEAD+4], len);
        string[len]=0x00;
        log_debug("    Ch=%02X, advertised by %02X to %02X, value= %s",buffer[BGNET_FRAME_HEAD+0],buffer[BGNET_FRAME_HEAD_FROM],buffer[BGNET_FRAME_HEAD_TO], string);
    }
    return;
}

// returns >0 when this message is for us to process (TO us and not FROM us)
int check_message_address(BGNET_CTRL* pCtrl, unsigned char *buffer) {
    int i,iFROMus,iTOus,r;
    iFROMus=0;
    iTOus=0;
    for (i=0; i<pCtrl->iNumberOfDevices; i++) {
        if (pCtrl->pDev[i].iActive > 0) {
            if (buffer[BGNET_FRAME_HEAD_TO] == pCtrl->pDev[i].iDeviceAddress) iTOus++;
            if (buffer[BGNET_FRAME_HEAD_FROM] == pCtrl->pDev[i].iDeviceAddress) iFROMus++;
        }
    }
    if (buffer[BGNET_FRAME_HEAD_TO] == BGNET_FRAME_TO_ALL) iTOus++; // broadcast
    if (buffer[BGNET_FRAME_HEAD_TO] == BGNET_FRAME_TO_NONE) iTOus++; // KEEPALIVE command by PILOT
    r = 0;
    if ((iFROMus == 0) & (iTOus > 0)) r = 1;
    return(r);
}

unsigned int frame_decoder(BGNET_CTRL* pCtrl, unsigned char *buffer, size_t len) {
    unsigned char cs; // checksum calculated
    unsigned char *pBuf,*pEnd; // pointer to start and end of buffer
    char asciibuffer[256];
    int xSize,i,x,z,n;
    int iIsForMe;

    pBuf = buffer; // gets incremented as we go
    pEnd = buffer+len; // end of buffer
    z = 0; // subframe counter
    n = 0; // number of channels processed

    // process multiple subframes in our buffer
    while (pBuf < pEnd) {
        // the header is fixed size
        cs = calculate_checksum(pBuf,BGNET_FRAME_CR_HEAD,0,BGNET_FRAME_HEAD-1); // calculate the checksum over the head (without the CR)
        log_debug("  Header Checksum: %02X (seed),%02X (received),%02X (calculated)",BGNET_FRAME_CR_HEAD,*(pBuf+BGNET_FRAME_HEAD_CS),cs);
        if (*(pBuf+BGNET_FRAME_HEAD_CS) == cs) {
            log_debug("                   TO: %02X",*(pBuf+BGNET_FRAME_HEAD_TO));
            log_debug("                   FROM: %02X",*(pBuf+BGNET_FRAME_HEAD_FROM));
            // check if this subframe is adressed TO us and not FROM us
            iIsForMe = check_message_address(pCtrl, pBuf);
            // we do not want to exclude messages not meant for me, but i still like to see them
            if (1 < 0) {
                log_debug("                   NOT FOR US");
                pBuf = pEnd; // dump frame
            } else {
                // The HEADER is OK, process it. Note that some CMD's do not have a body and in such case, we should skip the body & CS part.
                if (*(pBuf+BGNET_FRAME_HEAD_SIZE)==0) {
                    // no payload, no frame-CS
                    xSize=BGNET_FRAME_HEAD;
                    byte2ascii(asciibuffer, sizeof(asciibuffer), pBuf, xSize, BYTE2ASCII_SPACED);
                    log_debug("  Header Checksum OK, CMD=%02X, number of data bytes: %d, framesize: %d, data: %s",*(pBuf+BGNET_FRAME_HEAD_CMD),*(pBuf+BGNET_FRAME_HEAD_SIZE), xSize, asciibuffer);
                    // these are the commands without payload
                    switch(*(pBuf+BGNET_FRAME_HEAD_CMD)) {
                        case BGNET_FRAME_CMD_KEEPALIVE:
                            log_debug("    Command %02X (header only) KEEPALIVE by %02X",*(pBuf+BGNET_FRAME_HEAD_CMD),*(pBuf+BGNET_FRAME_HEAD_FROM));
                            x = bgn_FindDeviceIndex(pCtrl, BGNET_FRAME_FROM_NETPILOT);
                            if (x < 0) {
                                log_error("Error in FindDeviceIndex");
                            } else {
                                log_debug("Set ACP message request by %02X",*(pBuf+BGNET_FRAME_HEAD_FROM));
                                pCtrl->BGNET_devdata[x].bgn_NETPILOT.iSendACP = 1;
                            }
                            break;
                        case BGNET_FRAME_CMD_POWERUP:
                            log_debug("    Command %02X (header only) POWERUP by %02X",*(pBuf+BGNET_FRAME_HEAD_CMD),*(pBuf+BGNET_FRAME_HEAD_FROM));
                            // reset initit state, one (or more) devices on the NETWORK expeciended a POWERUP situation.
                            if (iIsForMe > 0) {
                                bgn_net_initialize_reset(pCtrl);
                            }
                            break;
                        default:
                            log_debug("    Command %02X (header only) unsupported",*(pBuf+BGNET_FRAME_HEAD_CMD));
                            break;
                    }
                } else {
                    // payload present, check frame-CR
                    xSize = BGNET_FRAME_HEAD + *(pBuf+BGNET_FRAME_HEAD_SIZE)+BGNET_FRAME_TAIL_CS; // expected framesize = head (including CR), body, and frame CR
                    log_debug("  Header Checksum OK, CMD=%02X, number of data bytes: %d, framesize: %d",*(pBuf+BGNET_FRAME_HEAD_CMD),*(pBuf+BGNET_FRAME_HEAD_SIZE), xSize);
                    cs=calculate_checksum(pBuf,BGNET_FRAME_CR_FRAME,0,xSize-1); // calculate the checksum over the whole frame (without the CR)
                    log_debug("  Frame Checksum: %02X (seed),%02X (received),%02X (calculated)",BGNET_FRAME_CR_FRAME,*(pBuf+xSize-1),cs);
                    if (*(pBuf+xSize-1) == cs) {
                        // dump frame
                        byte2ascii(asciibuffer, sizeof(asciibuffer), pBuf, xSize, BYTE2ASCII_SPACED);
                        log_debug("  Frame Checksum OK, framesize: %d, data: %s", xSize, asciibuffer);
                        // these are the commands with payload
                        switch(*(pBuf+BGNET_FRAME_HEAD_CMD)) {
                            case BGNET_FRAME_CMD_DATA:
                            case BGNET_FRAME_CMD_LATLON:
                                n=process_body_DATA(pBuf);
                                //log_debug("    Command %02X (with body), %d channels",*(pBuf+BGNET_FRAME_HEAD_CMD),n);
                                break;
                            case BGNET_FRAME_CMD_VALUES:
                                process_body_VALUE(pBuf);
                                //log_debug("    Command %02X (with body) VALUES",*(pBuf+BGNET_FRAME_HEAD_CMD));
                                break;
                            case BGNET_FRAME_CMD_SUBSCRIBE:
                                process_body_SUBSCRIBE(pBuf);
                                //log_debug("    Command %02X (with body) SUBSCRIBE",*(pBuf+BGNET_FRAME_HEAD_CMD));
                                break;
                            case BGNET_FRAME_CMD_BACKLIGHT:
                                log_debug("    Command %02X BACKLIGHT BUTTON PRESSED (BY: %02X LEVEL: %02X)",*(pBuf+BGNET_FRAME_HEAD_CMD),*(pBuf+BGNET_FRAME_HEAD_FROM),*(pBuf+BGNET_FRAME_HEAD+0));
                                // we use this for testing TX messages. When you press the LIGHTS button, this will also happen.
                                if (*(pBuf+BGNET_FRAME_HEAD_FROM) == BGNET_FRAME_FROM_NETDATA) {
                                    // request our PILOT to generate a test message
                                    x = bgn_FindDeviceIndex(pCtrl, BGNET_FRAME_FROM_NETPILOT);
                                    if (x < 0) {
                                        log_error("Error in FindDeviceIndex");
                                    } else {
                                        log_debug("Set testmessage request by %02X",*(pBuf+BGNET_FRAME_HEAD_FROM));
                                        pCtrl->BGNET_devdata[x].bgn_NETPILOT.iSendTest = 1;
                                    }
                                }
                                break;
                            default:
                                log_debug("    Command %02X (with body) unsupported",*(pBuf+BGNET_FRAME_HEAD_CMD));
                                break;
                        }
                    } else {
                        log_debug("    Frame Checksum BAD, framesize: %d", xSize);
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
    return(z);
}

// Here we handle a message from the receiver. Transmissions are seperated by 100ms silence. Here we receive a buffer of such transmission.
// The sender might have packed multiple frames into 1 transmission.
void handle_received_frame(BGNET_CTRL* pCtrl, unsigned char* pBuf, size_t len) {
    int x;
    // decode received data from serial port
    x = frame_decoder(pCtrl, pBuf, len);
    log_debug("RX frame processed, %d subframes",x);
}

// Example: build a frame to transmit (placeholder)
// Return vale: 0-nothing to send, >0 data to send
size_t build_tx_frame(BGNET_CTRL* pCtrl, unsigned char *pBuf, size_t maxlen) {
    size_t r;
    // when we have an output message from the "devices processor", copy it to the "writer process" and reset the output buffer
    r=0; // assume nothing to write
    if (pCtrl->tx_cnt > 0) {
        if (pCtrl->tx_cnt < maxlen) {
            memcpy(pBuf, pCtrl->tx_buf, pCtrl->tx_cnt); // copy output message
            r = pCtrl->tx_cnt; // save byte count to return
        } else {
            log_error("write buffer overflow");
        }
    }
    pCtrl->tx_cnt = 0; // reset message byte count
    return(r); // return bytes in output message
}

}


#endif
