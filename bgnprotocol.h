#ifndef BGNPROTODEF_H
#define BGNPROTODEF_H

// notes:
// - The NETWORK series system by B&G was the pinnacle for marine instrumentation. Introduced somewhere
//   around 1985 it was the bread-and-butter for B&G until the introduction of the H1000 system in
//   2002. The network, part of this series was is the precursor of what later would be called 'fastnet'
//   as part of the H1000, H2000 and H3000 product line.
// - At present, these hardware is totally obsolete. This reverse engineering was done, just because my boat
//   (from 1982) matches the B&G product line, i got my hands on some dirt-cheap display units and (as a retired
//   engineer) i had too much time to spare. If i only could have accomplish this 30 years a go, i would be one
//   of the unknown hero's in the OpenSource community.
// - It was a great help that i could find a lot of information on public sources that was compiled by lots of 
//   people doing the groundwork for me. I'm only the guy who attemted to put it all together. Kudo's to all
//   the inventive people not only doing a deep-dive into the equipment, but also taken the time to document their
//   findings.
// - B&G NETWORK devices are of the following types:
//   - Display WITH instrument input (QUAD, DEPTH, SPEED, WIND, COMPASS, ENGINE)
//   - Display WITHOUT instrument input (DATA, TACK, QUAD-R), they use the data from other devices via the bus.
//   - Navigational aids (NAV, GPS/GPSplus, LORAN, CHART), that also have sensor input
//   - Autopilot (PILOT) connected to the ACP (Advanced Control Processor)
//   - Communication (VHF), US-only
// - some models have NMEA1083 Input/Output capabilities. We do not want to interface with the NMEA, but
//   with the system bus, because the implemented NMEA protocol is a old/outdated version that is useless with
//   modern devices. I'm going for the propriatry bus, decode/encode data stream and inject what we can. 
// - NET TACK: displays the magnified wind angle, Gets it's data from WIND and SPEED/QUAD via the bus
// - NET WIND: only one should have a windsensors, up to 3 other units can be setup as 'repeater'.
// - NET WIND: has a 'demo mode' (td)
// - NET WIND: has a 'repeater mode' (tr)
// - NET QUAD: has a 'repeater mode' (tYPEr)
// - NET QUAD-R: does have NMEA output capabilities that the QUAD module has not. You can configure a QUAD as
//   a repeater, but that does not give it the NMEA capability.
// - NET PILOT: has a 'demo mode' (d)
// - NET PILOT: has a connector at the back for NMEA0183 input. Could be a 2-pin connector (use cable 612-OA-014)
//   or a 3-pin connector (use cable 612-OA-053)
//    - 2-pin cable: pin ?/blue = NMEA In-, pin 2/red = NMEA In+ (no info)
//    - 3-pin cable: pin 1/blue = NMEA In-, pin 3/red = NMEA In+
//   A limited set of NMEA messages are supported, see user manual.
// - NET PILOT: Needs a connection with a ACP to work, unless it's in 'demo mode'
// - NET PILOT: appears to use the address range 0x50-0x5F and may be different in other situations
// - NET PILOT: when it observes a duplicate address it immediately increments its own address (0x50-0x5F) and
//   continues as normal
// - NET PILOT:
//    - normally sends 4 channels: 49 (Heading, 0/360), 0B (Rudder angle, -45/+45), 51 (AW angle, -180/+180), 41 (Speed KT, 0/99.9)
//    - when any button is pressed (except SPEED or SETUP) it transmits 4 channels, appearently to synchronize with the ACP and other
//      PILOT modules on the network: B5 (AP mode), 53 (Target TWA), A6 (AP Target Course), A7 (AP WP Bearing)
// - NET PILOT: at startup without a ACP, the message "No Pilot" is displayed and it only sends a KEEPALIVE command (0x0E) TO 0x00. We
//   have no idea what return message from ACP to PILOT makes the PILOT module go into normal service.
// - after connection a message/error is displayed; press any button to resume
// - a network shall contain at least 2 devices. A solo device starts beeping after 5-10 seconds
// - the voltage level on the IO bus seems 5V, this means you can use TTL type interface and some RS232
//   might not pickup the low voltages (RS232 is 5-12VDC). Make sure your RS232 transmitter is also set 5V
// - the NET IO bus uses TTL lever signals (5V).The data is serial (like RS232), but with inverted Tx and Rx. FDTI chips based RS232
//   interfaces allow you to configure inverted signals. Possibly others too.
// - You must invert both the Rx and the Tx signals.
// - Communications parameters: 11008 baud, 8 Databits, 2 Stopbits, ODD parity (11008 = 43*256)
// - On the bus the RX and TX are joined. The software must block RX data that was send by yourself. The
//   advantage is that you do not have to worry aboud cross-over between Tx and Rx. It does not matter if
//   you have a DTE or DCE adapter (male or female).
// - Command 01 = used to communicated DATA (values from instruments)
// - Command 02 = is used by some (QUAD, WIND) to advertise their values (with their used memnonic/units)
// - Command 03 = used to communicate DATA (lat/lon in ASCII, by NAV module), could not be tested
// - Command 06 = is used by some (DATA, possibly others) to subscribe to other devices (QUAD, PILOT, WIND)
// - Command 0A = send once by every device at POWERUP
// - Command 0E = is send by some (PILOT) as some kind of 'keep alive' signal (send every 3-4 sec)
// - realise that our dynamics are completely different than the B&G NETWORK devices; Everything is powered-on
//   at the same time and communications starts immediatly after. In our case we are 'running' (or not yet
//   running) and have no idea when the B&G NETWORK is powered on or off.
//   We deal with this situation as follows:
//   - if we see some 0A command, while already initialized, we know the B&G NETWORK experienced a POWERUP
//     while we were already running. We'll reinitialize and also send our own 'POWERUP' command.
//   - if we observe incoming messages and not being initialised, we know that the B&G NETWORK was already
//     running when we experienced our startup. We will initialise and send our own 0A command
//  - Realise that when we write some message, we get out own output back into the receiver. Remember that the
//    Rx and Tx pins are connected? So, for every received message we check if it's OK for us to process:
//    - ignore any message FROM ourselves (could be multiple addresses)
//    - allow any message TO 'all' (0xFF) and ourselves (could be multiple addresses)
//    - there is a peculiar situation with PILOT sending a KEEPLIVE message to 0x00. What's up? Could this be
//      the ACP unit? I have not a ACP unit at my disposal and can not analyse the traffic between PILOT and ACP. 
// - Our objective is that we read signals from the NMEA2000 bus (or ethernet SignalK) and send this out on the
//   B&G NETWORK bus. This allows us to use the B&G devices as 'Display only' on our steering console. Modern
//   instrumentation with a 'classic' look. We are not interested in doing NMEA input/output, as the NMEA0183 version
//   supported by the NETWORK series is obsolete and incompatible with modern instruments.
// - Normally, when there is traffic on the IO bus, we synchronize our behaviour against existing traffic. When only
//   'passive' devices are on the network (modules without sensor input like DATA, other devices configured as 'repeater')
//    there is nothing and we have no heartbeat to adjust to.
//


//#define BGNET_RXFRAME_SIZE          1024   // send/receive buffer frame length

// Data is send in 'frames'. A frame has a HEADER (with Checksum), and optionally a BODY (also with a checksum). The seed
// of the 2 checksums is different. The BODY (if present) can be of variable size; the size if in the HEADER.
// Frames are separated by 'silence', typically 40-50 msec.
// When a device sends 2 frames, these can be combined into 1 transmission.

// FRAME definitions

// realize that the CR uses a different seed for HEADER or BODY
#define BGNET_FRAME_CR_HEAD           0x00
#define BGNET_FRAME_CR_FRAME          0x56

// HEADER layout
#define BGNET_FRAME_HEAD              5   //bytes in the HEADER
#define BGNET_FRAME_HEAD_TO           0   //position of TO byte
#define BGNET_FRAME_HEAD_FROM         1   //position of FROM byte
#define BGNET_FRAME_HEAD_SIZE         2   //position of SIZE byte
#define BGNET_FRAME_HEAD_CMD          3   //position of CMD byte
#define BGNET_FRAME_HEAD_CS           4   //position of CS byte

// BODY layout

// layout of the TAIL
#define BGNET_FRAME_TAIL_CS           1   //bytes in the TAIL (checksum)

// FROM/TO adresses

// values for the TO field (broadcast, byte 0
#define BGNET_FRAME_TO_NONE           0x00 // this TO address is observed with PILOT, sending KEEPALIVE command.
#define BGNET_FRAME_TO_2020           0xFA
#define BGNET_FRAME_TO_FFDHALYCON     0xFB
#define BGNET_FRAME_TO_FFDPILOT       0xFC
#define BGNET_FRAME_TO_PROCNODE       0xFD
#define BGNET_FRAME_TO_FFD            0xFE
#define BGNET_FRAME_TO_ALL            0xFF

// values for the FROM field (byte 1)
#define BGNET_FRAME_FROM_DEPTHCPU     0x01
#define BGNET_FRAME_FROM_WINDCPU      0x02
#define BGNET_FRAME_FROM_PERFCPU      0x09
#define BGNET_FRAME_FROM_EXPANSION1   0x0D
#define BGNET_FRAME_FROM_EXPANSION2   0x0E
#define BGNET_FRAME_FROM_EXPANSION3   0x0F
#define BGNET_FRAME_FROM_H2000GYRO    0x10
#define BGNET_FRAME_FROM_GYROACP      0x11
#define BGNET_FRAME_FROM_GYROACPPILOT 0x12

// base address, ?0-?F
#define BGNET_FRAME_FROM_UNKNOWN      0x00 // could this be the ACP unit on the network? (it's also connected to the tails)
#define BGNET_FRAME_FROM_NETQUAD      0x30
#define BGNET_FRAME_FROM_NETPILOT     0x50
#define BGNET_FRAME_FROM_NETDATA      0xD0
#define BGNET_FRAME_FROM_NETWIND      0x70

// commands (byte 3)
#define BGNET_COMMAND_MAX             0xFF
#define BGNET_FRAME_CMD_DATA          0x01 //DATA broadcast, variable length, instrument readings.
#define BGNET_FRAME_CMD_VALUE         0x02 //at startup some stations (QUAD/30, WIND/70) broadcast (ALL/FF) the memnonics of their measurements. Message format:
                                         //  Ch#+3x bytes numerical+ascii string (variable length)+CS
                                         //  QUAD:  20(TEMPC),31(ECHOSNR),41(BOATSPEEDKT),42(DEPTHM),4A(DEPTHM),64(AVGSPEEDKT),65(MAXSPEEDKT),75(TIMER),81(DRDISTANCENM),
                                         //         C4(DEPTHM),CD(LOGNM),CF(TRIPNM),D3(DRCOURSEM),FC(SHALLOWALARM),FD(DEPTHALARM),FE(ANCHORALARM),FF(DEPTHCAL)
                                         //  WIND:  50(AWINDSPEEDKT),51(AWINDANGLE),57(TWINDSPEEDKT),59(TWINDANGLE),7F(VMGKT),8D(BATTVOLTS)
                                         //Note: Command is not send when device (QUAD/30, WIND/70) is in 'repeater' mode.
                                         //In demo mode the follwing (simulated) data is send:
                                         //  WIND:  4E(AWINDSPEEDRAW),50(AWINDSPEEDKT),51(AWINDANGLE),52(TWINDANGLERAW),8D(BATTVOLTS)
                                         //  PILOT: 0B(RUDDER),41(BOATSPEEDKT),49(HEADING),51(AWINDANGLE)
#define BGNET_FRAME_CMD_LATLON        0x03 //DATA broadcast, variable length. ASCII transmission, for LAT/LON messages
#define BGNET_FRAME_CMD_SUBSCRIBE     0x06 //at startup some stations (DATA/60) broadcast to providers (QUAD/30) a subscription request, payload= Ch#, CS
                                         // DATA/60 to QUAD/30: FE(ANCHORALARM),FC(SHALLOWALARM),C4(DEPTHM),FD(DEPTHALARM),41(SPEEDKT),65(MAXSPEEDKT),
                                         //                     CD(LOGNM),64(AVGSPEEDKT),CF(TRIPNM),81(DRDISTANCENM),D3(DRCOURSEM)
                                         //Note: Command is not send when device (QUAD/30) is in 'repeater' mode.
#define BGNET_FRAME_CMD_POWERUP       0x0A //at startup all stations (QUAD/30, PILOT/51, DATA/60, WIND/70) broadcast (ALL/FF) their presence. Header only, no payload. Message format:
                                         //QUAD: "FF-30-00-0A-C7", PILOT: "FF-51-00-0A-A6", DATA: "FF-D6-00-0A-97", WIND: "FF 70 00 0A 87"
#define BGNET_FRAME_CMD_0C            0x0C //unknown
#define BGNET_FRAME_CMD_KEEPALIVE     0x0E //continously, a PILOT/50 station broadcasts (TO 0x00!) every 3-4 seconds. Message format:
                                         // PILOT: "00-51-00-0E-A1"
                                         // ACP: payload: 8E 01 07 2C FF D8 00 00 5A
                                         //               -- ACP status reply     -- Checksum
                                         //                  -- MODE=Auto {STBY|AUTO|TRACK|WIND}
                                         //                     ----- 072C= Heading = 185.6 (actual, 0-360)
                                         //                           ----- FFD8= Rudder -4.0 (-45/+45)
                                         //                                 -- 00= Alarms= No {off-course|drive fail|compass fail}
                                         //                                    -- 00=Drive = Idle {port|starboard|idle}
#define BGNET_FRAME_CMD_POWEROFF      0xC8 //shutdown system, no payload. Does not work on NETWORK series devices "FF 70 00 C8 C9"
#define BGNET_FRAME_CMD_BACKLIGHT     0xC9 //LIGHTS button pressed (any module) -> header+2 bytes: {0|1|2|3},0x20. Message format:
                                         // FF 70 01 C9 C7 00 AA or FF 70 01 C9 C7 00 AA
                                         // at powerup, initializes to 0 (off), when pressed the 1st time (after power) it starts with 3, then
                                         // down to 2,1,0 (off) and wrap back to 3
#define BGNET_FRAME_CMD_RESET         0xFE //reset adresss modules to factory default, no data. Does not work on NETWORK series devices.
                                         // FF 70 00 FE C9

// layout of the VALUE (1 or more in a BODY, dynamic length)
#define BGN_VALUE_HEAD              2  // number of bytes in VALUE header
#define BGN_VALUE_CNR               0  // position of channel byte
#define BGN_VALUE_FMT               1  // position of FMT byte
#define BGN_VALUE_DATA              2  // position where DATA starts (multiple bytes, formatted as per FMT byte)

// DATA/LATLON commands (CMD={1|3}) sends instrument data by 'Channel', each idetified by a 'Channel Number' 
#define BGN_CHANNEL_MAX                 0xFF // for sizing purposes
#define BGN_CNR_00_NODERESET        0X00
#define BGN_CNR_01_UNKNOWN          0X01
#define BGN_CNR_03_UNKNOWN          0X03
#define BGN_CNR_06_UNKNOWN          0X06
#define BGN_CNR_0B_RUDDER           0X0B
#define BGN_CNR_0C_LINEAR5          0x0C
#define BGN_CNR_0D_LINEAR6          0x0D
#define BGN_CNR_0E_LINEAR7          0x0E
#define BGN_CNR_0F_LINEAR8          0x0F
#define BGN_CNR_10_LINEAR9          0x10
#define BGN_CNR_11_LINEAR10         0x11
#define BGN_CNR_12_LINEAR11         0x12
#define BGN_CNR_13_LINEAR12         0x13
#define BGN_CNR_14_LINEAR13         0x14
#define BGN_CNR_15_LINEAR14         0x15
#define BGN_CNR_16_LINEAR15         0x16
#define BGN_CNR_17_LINEAR16         0x17
#define BGN_CNR_1C_AIRTEMPF         0x1C
#define BGN_CNR_1D_AIRTEMPC         0x1D
#define BGN_CNR_1E_SEATEMPF         0x1E
#define BGN_CNR_1F_SEATEMPC         0x1F
#define BGN_CNR_20_SEATEMPQ         0x20 //as displayed on QUAD module, selectable C/F
#define BGN_CNR_27_HEADLIFTTREND    0x27
#define BGN_CNR_29_OFFCOURSE        0x29
#define BGN_CNR_31_ECHO_SNR         0x31 // ascii text, status, " T " or " F "
#define BGN_CNR_32_TACKPERF         0x32
#define BGN_CNR_33_REACHPERF        0x33
#define BGN_CNR_34_HEELANGLE        0x34
#define BGN_CNR_35_OPTWINDANGL      0x35
#define BGN_CNR_36_DEPTHGAIN        0x36
#define BGN_CNR_37_DEPTHNOISE       0x37
#define BGN_CNR_38_LINEAR1          0x38
#define BGN_CNR_39_LINEAR2          0x39
#define BGN_CNR_3A_LINEAR3          0x3A
#define BGN_CNR_3B_LINEAR4          0x3B
#define BGN_CNR_3C_ROLLRATE         0x3C
#define BGN_CNR_41_BOATSPEEDKTS     0x41
#define BGN_CNR_42_BOATSPEEDRAW     0x42
#define BGN_CNR_43_BOATSPEEDMS      0x43
#define BGN_CNR_44_YAWRATE          0x44
#define BGN_CNR_46_APSPEEDKTS       0x46
#define BGN_CNR_47_LONLAT           0x47 //send in ASCII via CMD03
#define BGN_CNR_48_LONLAT           0x48 //send in ASCII via CMD03
#define BGN_CNR_49_HEADING          0x49
#define BGN_CNR_4A_HEADINGRAW       0x4A
#define BGN_CNR_4D_AWINDSPEEDKTS    0x4D
#define BGN_CNR_4E_AWINDSPEEDRAW    0x4E
#define BGN_CNR_4F_AWINDSPEEDMS     0x4F
#define BGN_CNR_50_AWINDSPEEDKT     0x50
#define BGN_CNR_51_AWINDANGLE       0x51
#define BGN_CNR_52_AWINDANGLERAW    0x52
#define BGN_CNR_53_TARGET_TWA       0x53 // send by PILOT upon change/button pressed, Target True Wind Angle
#define BGN_CNR_55_TWINDSPEEDKT     0x55
#define BGN_CNR_56_TWINDSPEEDMS     0x56
#define BGN_CNR_57_TWINDSPEEDKT     0x57
#define BGN_CNR_59_TWINDANGLE       0x59
#define BGN_CNR_5A_MEASWINDANGLEDEG 0x5A
#define BGN_CNR_64_SPEEDAVGKTS      0x64
#define BGN_CNR_65_SPEEDAVGRAW      0x65
#define BGN_CNR_68_REQFORDATA       0x68
#define BGN_CNR_69_COURSE           0x69
#define BGN_CNR_6A_ACTFORDATA       0x6A
#define BGN_CNR_6D_TRUEWINDDIR      0x6D
#define BGN_CNR_6F_NEXTLEGAWANGLE   0x6F
#define BGN_CNR_70_NEXTLEGBOATSPEED 0x70
#define BGN_CNR_71_NEXTLEGAWSPEED   0x71
#define BGN_CNR_75_TIMER            0x75
#define BGN_CNR_76_UNKNOWN          0x76
#define BGN_CNR_7C_POLARPERFORMANCE 0x7C
#define BGN_CNR_7D_TARGETBOATSPEED  0x7D
#define BGN_CNR_7F_VMGKT            0x7F
#define BGN_CNR_80_UNKNOWN          0x80
#define BGN_CNR_81_DR_DIST          0x81 //shown on SPEED and DATA, reset via QUAD
#define BGN_CNR_82_LEEWAY           0x82
#define BGN_CNR_83_TIDALDRIFT       0x83
#define BGN_CNR_84_TIDALSET         0x84
#define BGN_CNR_85_UPWASH           0x85
#define BGN_CNR_86_BAROTREND        0x86
#define BGN_CNR_87_BARORAW          0x87
#define BGN_CNR_8A_UNKNOWN          0x8A
#define BGN_CNR_8D_BATTVOLTS        0x8D
#define BGN_CNR_9A_HEADINGNEXTTACK  0x9A
#define BGN_CNR_9B_FOREAFTTRIM      0x9B
#define BGN_CNR_9C_MASTANGLE        0x9C
#define BGN_CNR_9D_WANGLETOMAST     0x9D
#define BGN_CNR_3A_LINEAR3          0x3A
#define BGN_CNR_3B_LINEAR4          0x3B
#define BGN_CNR_3C_ROLLRATE         0x3C
#define BGN_CNR_9E_PITCHRATE        0x9E
#define BGN_CNR_A6_AP_CTARGET       0xA6 // send by PILOT upon change/button pressed, Target Course (in AUTO or memory when STBY)
#define BGN_CNR_A7_AP_WPB           0xA7 // send by PILOT upon change/button pressed, WayPoint Bearing
#define BGN_CNR_AF_AP_OFFCOURSE     0xAF
#define BGN_CNR_B4_UNKNOWN          0xB4
#define BGN_CNR_B5_AP_MODE          0xB5 // send by PILOT upon change/button pressed, 2bytes (byte0: 0=STBY,1=AUTO; byte1: 1=VANE;3=COMP;4=NAV)
#define BGN_CNR_BA_UNKNOWN          0xBA
#define BGN_CNR_C1_DEPTHM           0xC1
#define BGN_CNR_C2_DEPTHFT          0xC2
#define BGN_CNR_C3_DEPTHF           0xC3
#define BGN_CNR_C4_DEPTHM           0xC4 // shows some "---" or ?
#define BGN_CNR_CD_STORLOGNM        0xCD
#define BGN_CNR_CF_TRIPLOGNM        0xCF
#define BGN_CNR_D3_DR_COURSE        0xD3 //shown on SPEED and DATA, reset via QUAD
#define BGN_CNR_DC_LOCALTIME        0xDC
#define BGN_CNR_DD_UTCTIME          0xDD
#define BGN_CNR_DE_UNKNOWN          0xDE
#define BGN_CNR_E0_BRGW2WRUMBTRUE   0xE0
#define BGN_CNR_E1_BRGW2WRUMBMAG    0xE1
#define BGN_CNR_E2_LAYLINEDIST      0xE2
#define BGN_CNR_E3_BRG2WRUMBTRUE    0xE3
#define BGN_CNR_E4_BRG2WRUMBMAG     0xE4
#define BGN_CNR_E5_BRG2WGYROTRUE    0xE5
#define BGN_CNR_E6_BRG2WGYROMAG     0xE6
#define BGN_CNR_E7_WAYPDISTRUMB     0xE7
#define BGN_CNR_E8_WAYPDISTGYRO     0xE8
#define BGN_CNR_E9_COGTRUE          0xE9
#define BGN_CNR_EA_COGMAG           0xEA
#define BGN_CNR_EB_SOG              0xEB
#define BGN_CNR_EC_VMG2W            0xEC
#define BGN_CNR_ED_TIME2W           0xED
#define BGN_CNR_EE_CROSSTRACKERR    0xEE
#define BGN_CNR_EF_REMOTE0          0xEF
#define BGN_CNR_F0_REMOTE1          0xF0
#define BGN_CNR_F1_REMOTE2          0xF1
#define BGN_CNR_F2_REMOTE3          0xF2
#define BGN_CNR_F3_REMOTE4          0xF3
#define BGN_CNR_F4_REMOTE5          0xF4
#define BGN_CNR_F5_REMOTE6          0xF5
#define BGN_CNR_F6_REMOTE7          0xF6
#define BGN_CNR_F7_REMOTE8          0xF7
#define BGN_CNR_F8_REMOTE9          0xF8
#define BGN_CNR_F9_COURSETOSAIL     0xF9
#define BGN_CNR_FA_NEXTWAYPDIST     0xFA
#define BGN_CNR_FB_TIMETOLAYLINE    0xFB
#define BGN_CNR_FC_SHALLOWALARM     0xFC
#define BGN_CNR_FD_DEPTHALARM       0xFD //OFF or ?, might be WIND
#define BGN_CNR_FE_ANCHORALARM      0xFE //OFF or ?, might be WIND
#define BGN_CNR_FF_DEPTH_CAL        0xFF //DEPTH calibration, difference between the deepest point of the boat and the
                                         // sensor (Depth_sensor - Depth_keel, usually this value is negative)

// In our protocol handler, we mirror received channel data
// In principal we could support every B&G NETWORK device, albeit we do not had them available when making this protcol
// decoder/encoder, so for safely reasons we will ignore traffic from devices we could not test; hence the iEnable flag.
// The 'iactive' flag is set when we have received data FROM that device. As per B&G recommendation we only expect
// 1 'active' device from every type. Possible double units are to be set in 'repeater' mode and will not transmit.
#define BGNET_DEVICE_MAX            18
// our 'managed' devices
typedef struct {
    char sProdName[20];
    char sModelName[20];
    char sModelNr[20];
    char sSerialNr[20];
    char sVersion[20];
    int iEnable;
    int iDeviceType;
} BGNET_DEV;
// channel label
typedef struct {
    char sLabel[20];
    int iNMEA_ID;
} BGNET_CHL;

typedef struct {
    unsigned char ucTO;
    unsigned char ucFROM;
    unsigned char ucFMT;
    unsigned char ucBYTES;
    unsigned char ucRAW[16]; // raw data (for diagnostic purposes)
    char cValue[16]; // ascii formatted value string (for display purposes)
    time_t tTimeStamp;
} BGN_CHANNELDATA;

// in this structure we count how many times we have encountred
// this command
typedef struct {
    int iCounter;
    time_t tTimeStamp;
} BGN_COMMAND;

// We might need some device-specific values during processing. Please put them here so they overlap
union BGNET_DEVDATA {
    // here we declare all ' common' parameters (used by every device)
    int iKeepAliveCounter; // i know only the PILOT sends the 'keepalive' command, but i still put it here
    int iActive;
    unsigned char ucDeviceAddress;
    // this is the ' consolidated' channel list. We also write the same channels to the device that send it for diagnostic reasons
    BGN_CHANNELDATA DATA[BGN_CHANNEL_MAX]; // Channels received
    BGN_COMMAND CMD[BGNET_COMMAND_MAX]; // Commands received
    struct {
        int iSendTest;
        int iSendACP;
        int iCounter; // number of channels received
     } bgn_NETPILOT;
    struct {
        int iCounter; // number of channels received
    } bgn_NETDATA;
    struct {
        int iCounter; // number of channels received
    } bgn_NETQUAD;
    struct {
        int iCounter; // number of channels received
    } bgn_NETWIND;
};

#define BGNET_CTRL_INIT            1
#define BGNET_CTRL_INIT_RESET      0
#define BGNET_CTRL_TXBUFFERSIZE    1024
#define BGNET_CTRL_DEVCNTR_INIT    1000
typedef struct {
    int iInitialized;
    int iNumberOfDevices;
    // the timer service decrements this value to zero (and stops); if zero the 'device processing' takes place
    int iDeviceCounter; // count down by timer
    int iDeviceRun; // number of runs (0-1000) with wrap-around
    //
    BGNET_DEV* pDev; // pointer to a structure containing device definition (all static, program shal not write here)
    BGNET_CHL* pChl; // pointer to a structure containing channel definition (all static, program shal not write here)
    Config* pCfg; // pointer to config structure (all static, program shal not write here)

    // this is where the device processor puts the Tx messages
    size_t tx_cnt;
    unsigned char tx_buf[BGNET_CTRL_TXBUFFERSIZE]; // must be big enough to hold all our output messages
    // here is where a dynamic 'device data' is stored
    union BGNET_DEVDATA BGNET_devdata[BGNET_DEVICE_MAX];
} BGNET_CTRL;

// prototypes
unsigned int frame_decoder(BGNET_CTRL*, unsigned char*, size_t);

#endif
