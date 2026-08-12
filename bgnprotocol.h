#ifndef BGNPROTODEF_H
#define BGNPROTODEF_H

// notes:
// - The NETWORK series system by B&G was the pinnacle for marine instrumentation. Introduced somewhere
//   around 1985 it was the bread-and-butter for B&G until the introduction of the H1000 system in
//   2002. The network, part of this se// this is the first found ACPries was is the precursor of what later would be called 'fastnet'
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
// - NET WIND: has a 'demo mode' (td). When in demo mode, only APP data is displayed (TRUE wind data is not displayed)
// - NET WIND: has a 'repeater mode' (tr) and a 'demo mode' (td)
// - NET QUAD: has a 'repeater mode' (tYPEr)
// - NET QUAD-R: does have NMEA output capabilities that the QUAD module has not. You can configure a QUAD as
//   a repeater, but that does not give it the NMEA capability.
// - NET QUAD has a 'demo mode' (press <TIMER> to display stored LOG, press <SETUP>, select 'TYPEd'; you must first put the QUAD in 'TYPEr')
// - NET PILOT: has a 'demo mode' (TYPE D)
// - NET PILOT: has a connector at the back for NMEA0183 input. Could be a 2-pin connector (use cable 612-OA-014)
//   or a 3-pin connector (use cable 612-OA-053). Cables are no longer produced.
//    - 2-pin cable: pin ?/blue = NMEA In-, pin 2/red = NMEA In+ (no info)
//    - 3-pin cable: pin 1/blue = NMEA In-, pin 3/red = NMEA In+
//   A limited set of NMEA messages are supported, see user manual, that are outdated by now. Interconnectivity with modern
//   equipment is not possible.
// - NET PILOT: Needs a connection with a ACP to work, unless it's in 'demo mode'
// - NET PILOT: appears to use the address range 0x50-0x5F (at least the NETWORK series)
// - NET PILOT: when it observes a duplicate address it immediately increments its own address (0x50-0x5F) and
//   continues as normal
// - NET PILOT: in demo mode sends 4 channels: 0x49/Heading, 0x0B/Rudder, 0x51/AWA and 0x41/Speed
// - NET PILOT: at startup (without ACP), the message "No Pilot" is displayed and it only sends a 0x0E/HELLO command TO 0x00. This is
//   repeated every 3.8sec. If a reply is not received (within 5sec), "No Pilot" is displayed.
//   If the PILOT receives a DATA command with atleast 0xB5/AP_STATUS, the message "No Pilot" disappears. You might seeb some errors
//    ("WND OFF", "NO COMP"), because some critical channels are not present at the PILOT.
// - NET PILOT sends a CMD (3x) when a button is pressed:
//    - AUTO or OFF button:   TO FROM 05 D3 CS 00 B5 01 00 00 CS
//    - MODE button:          TO FROM 05 D4 CS 00 A6 01 00 01 CS
//    - <10,>10,<1,>1 button: TO FROM 02 DC CS 00 A6 CS
//   if no response message is received, PILOT starts to beep. The response shall be a ACK command with 1 payload byte: the first byte of the
//    received command shall be returned as confirmation. This number increments every time it is used and after powerup it's 0x00.
//   The 2nd byte of the payload is a channel number of the affected channel. Observe that the FMT is different than used in DATA commands. If
//   this is intentional or not could not be found out. DATA transmission use FMT_08, D3/D4/DC commands use FMT_01
// - NET PILOT: WA (Watch Alarm) and OC (Off Course Alarm) seems a 'local alarm'. In only beeps on PILOT and can not be reset
//   on other (even on 'alarm capable' units). No alarm data is send over the network
// - NET PILOT: has a 2 (or 3) pins female connector at the backside for NMEA 0183/RS442/4800/N/8/1 interface. There is a limited set of supported
//   sentences and is build against specification v1.5 (about 1985). Newer revisions have made the most important sentences incompatible and useless.
//   Pin arrangement (looking from the backside): 1 (Left) = Tx-, 2 (Top) = Shield, 3 (Right) = Tx+
//   It might take a few sec before the the channel value is transmitted on the network (via a 0x01/DATA command). Once the NMEA stream stops,
//   it may take some 10-15 sec before the network retransmits is stopped.
// - Sending NMEA sentences for testingi found the easiest way to make a textfile with sentences and send them via a command prompt:
//   on a windows machine: ECHO {sentence}  > COM6
// - NET PILOT: it has been observed that somtimes PILOT sends erroneous FRAME CHECKSUMs. resetting the PILOT cleared the phenomenon.
// - NET PILOT: in some cases, after pressing a button, "ERR 18" is observed. This has found to be caused by incorrect CMD/ACK handling
//   This may be due to incorrect programming by me (the PILOT does not receive the extected return) to Checksum errors, causing our
//   application to throw-out received commands (and thus never replying what was expected by PILOT)
// - When a DATA or QUAD-r does not receive a value (every 10 seconds) it displays "OFF" 
// - NET ACP: supports a joystick, directly connected to the ACP, engage/disengaged by a switch connected to the ACP. When engaged, the
//   joystick controls the rudder directly. When engaged, the PILOT displays "JOYSTICK".
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
// - Command 01 = used to communicated DATA (values from instruments). Data is send as "Channel". The length of the channel (number of bytes)
//   depends on the DataType. The same channel could me send as different DataTypes. If that DataType has another length, sometimes the channels
//   may have different length in one case or another.
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

// Data is send in 'frames'. A frame has a HEADER (with Checksum), and optionally a PAYLOAD (also with a checksum). The seed
// of the 2 checksums is different. The PAYLOAD (if present) can be of variable size; the size is in the HEADER.
// Frames are separated by 'silence', typically 40-50 msec.
// When 1 device sends 2 frames, these can be combined into 1 transmission. We should expect that received frames may consist
// of multiple 'sub-frames'.

// Note for my future me:
// A future project could be an interface between PILOT and PYPILOT. PYPILOT does not have a documented API. It does have
// some features that run over port 21311. To interface with the appliction. Refer to the following scripts how this could
// be archieved:
//  - pypilot/ui/autopilot_control.py    python control client
//  - pypilot/lcd/lcd.py                 spi lcd, gpio keypad and IR remote
//  - pypilot/webapp/webapp.py           python/javascript control from website, web interface for pypilot using web sockets
// see pypilot on github: https://github.com/pypilot
// when installing openplotter, pypilot is usually found in /usr/local/lib/python3.11/dist-packages/pypilot
// All pypilot features are also utilised in the opencpn plugin. This is implemented in C++ and might be easier to connect
// to this B&G gateway: https://github.com/pypilot/pypilot_pi

// GENERIC defines

// use this macro to convert a floating point value (double) to integer with round-off
// sometimes it's trivial, but using defines makes it easier to locate where what conversions are done
#define DTOIROUND(d)                    ((int)(d >= 0.0 ? (d + 0.5) : (d - 0.5)))
#define ITOD(i)                         (double)i
#define BGNET_PI                        3.14159
#define BGNET_MIN(a, b) (((a) < (b)) ? (a) : (b))
#define BGNET_TIME_FMT                 "%Y-%m-%d %H:%M:%S" 

// FRAME definitions

// realize that the CS uses a different seed for HEADER or BODY
#define BGNET_FRAME_CR_HEAD             0x00
#define BGNET_FRAME_CR_FRAME            0x56

#define BGNET_FRAME_HEAD                5   //size of HEADER (fixed)
#define BGNET_FRAME_TAIL_CS             1   //bytes in the TAIL (payload checksum)
typedef struct {        // for the FRAME header we use a pointer
    uint8_t TO;
    uint8_t FROM;
    uint8_t PL;         // payload length
    uint8_t CMD;        // Command
    uint8_t CSH;        // Checksum HEAD
    uint8_t payload[];  // payload (variable size)
    // uint8_t CSP;     // Checksum payload is placed after the last payload byte
} FRAME_H;

// maximum number of possible devices. B&G advises a maximum of around 20 devices on the same network. Because
// for us it's much faster to use the address as the index to the device, we use 0xFF as the maximum
#define BGNET_DEVICE_MAX                0xFF

// FROM/TO address range (0x?0-0x?F) for device types. There is some address negotiations going on. At one point the NETDATA is 0xD6, another time it's 0x60.
// The same with NETPILOT. Normally 0x50, but if another NETPILOT is detected, it's 0x51. As long as you have 1 of each it's simple. I do not have multiple
// units of the same type, so i was never able to reverse-engineer the address negotiations.
#define BGNET_TYPE_MASK                 0xF0 // mask the device type from the address
#define BGNET_TYPE_NETACP               0x00 // this seems to be the ACP, as the PILOT is sending it's HELLO message to (and is happy with the reply)
#define BGNET_TYPE_NETQUAD              0x30 // confirmed
#define BGNET_TYPE_NETPILOT             0x50 // confirmed
#define BGNET_TYPE_NETDEPTH             0x10 //?
#define BGNET_TYPE_NETSPEED             0x40 //?
#define BGNET_TYPE_NETDATA              0x60 // confirmed
#define BGNET_TYPE_NETWIND              0x70 // confirmed
#define BGNET_TYPE_ALL                  0xFF // used for broadcast (to ALL)

#define BGNET_COMMAND_MAX               0xFF
#define BGNET_COMMAND_INIT              0x00

#define BGNET_CMD_01_DATA               0x01 //DATA broadcast, variable length, instrument readings.
#define BGNET_CMD_02_LABEL              0x02 //at startup some stations (QUAD/30, WIND/70) broadcast (ALL/FF) the memnonics of their measurements. Message format:
                                             //  Ch#+3x bytes numerical+ascii string (variable length)+CS
                                             //  QUAD:  20(TEMPC),31(ECHOSNR),41(BOATSPEEDKT),42(DEPTHM),4A(DEPTHM),64(AVGSPEEDKT),65(MAXSPEEDKT),75(TIMER),81(DRDISTANCENM),
                                             //         C4(DEPTHM),CD(LOGNM),CF(TRIPNM),D3(DRCOURSEM),FC(SHALLOWALARM),FD(DEPTHALARM),FE(ANCHORALARM),FF(DEPTHCAL)
                                             //  WIND:  50(AWINDSPEEDKT),51(AWINDANGLE),57(TWINDSPEEDKT),59(TWINDANGLE),7F(VMGKT),8D(BATTVOLTS)
                                             //Note: Command is not send when device (QUAD/30, WIND/70) is in 'repeater' mode.
                                             //In demo mode the follwing (simulated) data is send:
                                             //  WIND:  4E(AWINDSPEEDRAW),50(AWINDSPEEDKT),51(AWINDANGLE),52(TWINDANGLERAW),8D(BATTVOLTS)
                                             //  PILOT: 0B(RUDDER),41(BOATSPEEDKT),49(HEADING),51(AWINDANGLE)
                                             //         (when AUTO) 53(TARGET TWA),A6 (COURSETARGET),A7 (AP WP BEARING),B5 (AP MODE)
                                             //  QUAD:  20(SEATEMP),31(ECHOSND),41(BOATSPEEDKT),42(BOATSPEEDRAW),64(SPEEDAVG),65(SPEEDMAX),75(TIMER),
                                             //         81(DRDIST),C4(DEPTH),CD(STORLOG),CF(TRIPLOG),D3(DRCOURSE),FC(SHALLOWALM),FD(DEPTHALM),FE(ANCHORALM)
                                             //valid characters: 0-9, A-Z, and symbols (-/.,%). % is represented as "degree"
                                             // QUAD: if you reconfigure the TEMP units, the new label is send with as 0x02/0x20 command to ALL (0xFF), making the DATA
                                             //       units to display the corresponding value & units, but the channel 0x20 still transmits the value as degreeC.
    #define BGNET_CMD_LABEL_MAX     22       // buffer size for storing data in the LABEL/0x02 command
#define BGNET_CMD_06_SUBSCRIBE          0x06 //at startup some stations (DATA/60) broadcast to providers (QUAD/30) a subscription request, payload= Ch#, CS
                                             // DATA/60 to QUAD/30: FE(ANCHORALARM),FC(SHALLOWALARM),C4(DEPTHM),FD(DEPTHALARM),41(SPEEDKT),65(MAXSPEEDKT),
                                             //                     CD(LOGNM),64(AVGSPEEDKT),CF(TRIPNM),81(DRDISTNM),D3(DRCOURSEM)
                                             //Note: Command is not send by DATA/60 when QUAD/30 is in 'repeater' mode.
#define BGNET_CMD_0A_POWERUP            0x0A //at startup all stations (QUAD/30, PILOT/50, DATA/60, WIND/70) broadcast (ALL/FF) their presence. Header only, no payload. Message format:
                                             //QUAD: "FF-30-00-0A-C7", PILOT: "FF-50-00-0A-A6", DATA: "FF-D6-00-0A-97", WIND: "FF 70 00 0A 87"
                                             //(no firmware version number is included in NETWORK series)
#define BGNET_CMD_0E_PILOT_HELLO        0x0E //continously, a PILOT/50 station broadcasts (TO 0x00) every 3-4 seconds. Message format:
                                               // PILOT: "00 51 00 0E A1" (no payload)
                                               // PILOT broadcasts this command in 'normal' mode, and also in 'demo' mode.
                                               // if PILOT receives a DATA command (with some minimal channels), it leaves the "NO PILOT" state
#define BGNET_CMD_20_PILOT_OCDAT        0x20 // 5 bytes payload after SET of WA minutes, CH = AF
                                               // 3 or 5 bytes payload for ANCHOR ALARM status (on QUAD): On/Off (3 bytes) and level (in meters, 5 bytes)
#define BGNET_CMD_21_PILOT_WADAT        0x21 // 5 bytes payload after SET of WA minutes, CH = 76
#define BGNET_CMD_22_DEPTH_DADAT        0x22 // 3 or 5 bytes payload for DEEP ALARM status (on QUAD): On/Off (3 bytes) and level (in meters, 5 bytes)
#define BGNET_CMD_B1_PILOT_DELTADAT     0xB1 // STEP/ADJUST -10/-1/+1/+10, 5 bytes payload (00 A6 01 FF F6) = -10
#define BGNET_CMD_C9_BACKLIGHT          0xC9 // LIGHTS button pressed (all units) -> header + 1 bytes payload: {0|1|2|3}
                                               // At powerup, initializes to 0 (off). When pressed (1st time), it displays the current setting, when presed again it
                                               // cycles though 0-3-2-1-0
#define BGNET_CMD_CB_QUAD_RESET         0xCB // 2 bytes payload (00 CF), request to reset channel CF (triplognm, send by QUADr (QUADt or QUADd seems not to accept this reset)
                                                // also used by PILOT when some bits in the STATE byte are received
#define BGNET_CMD_CD_PILOT_DMPREQ       0xCD // QUAD repeater: 2 bytes payload (00 CF, 00 41), request to reset channel CF(=TRIPLOGNM) or 41(=BOATSPEEDKT > DR_DIST/DR_COURSE)
                                               // Note: DRD is displayed on SPEED and DATA, not on QUAD.
                                               // Possibly also the reset command to a QUADt, but could not test this as i have no working QUADt
                                               // PILOT: 2 bytes payload (ACKSEQ, 0x49)
#define BGNET_CMD_CE_PILOT_DMPDAT       0xCE
#define BGNET_CMD_CF_PILOT_MANREQ       0xCF // 2 bytes payload (00 46), 46=AP_SPEED -> Response FST/MID/SLW

#define BGNET_CMD_D0_PILOT_SPDREQ       0xD0 // 2 bytes payload (00 46), 46= AP_SPEED
#define BGNET_CMD_D1_PILOT_RESPREQ      0xD1 // 2 bytes payload (00 A6), Steering Response N/E/D (Normal, Economy, Downwind)
#define BGNET_CMD_D2_PILOT_MANREQ       0xD2 // 2 bytes payload (00 46), MAN speed selection (FST, MED, SLW) via SPEED key
#define BGNET_CMD_D3_PILOT_AM           0xD3 // 5 bytes payload (00 B5 01 00 XX), B5= channel, XX: {AUTO|STBY}, reply with B5?
#define BGNET_CMD_D4_PILOT_MODE         0xD4 // 5 bytes payload (00 A6 01 00 XX), A6= channel, XX: {COMP|VANE|NAV}
                                               //                 (00 46 01 00 XX), 46= channel, XX: SPEED MODE (0=TRAnsmitter, 1=MANual, 2=SOG)
#define BGNET_CMD_D5_PILOT_RESPDAT      0xD5 // 5 bytes payload (00 A6 01 00 XX), A6= channel, XX: RESP MODE (0=Normal, 1=Eco, 2=Downwind)
#define BGNET_CMD_D6_PILOT_SLWDAT       0xD6 // 5 bytes payload (00 46 01 00 XX), 46= channel, XX: MANSLW speed (0-990)
#define BGNET_CMD_DC_PILOT_STEPREQ      0xDC // 2 bytes payload (00 A6), A6= channel, is followed by command 0xB1/ADJUST to speficy the actual step (+/-, 10/1)
#define BGNET_CMD_DD_QUAD_SHAREQ        0xDD // 2 bytes payload (00 C4), C4=DEPTH
#define BGNET_CMD_DE_PILOT_WAREQ        0xDE // 2 bytes payload (00 76), 76=WA ALARM, command send to ACP during SETUP
#define BGNET_CMD_DF_PILOT_OCREQ        0xDF // 2 bytes payload (00 AF), AF=OC ALARM, command send to ACP during SETUP
#define BGNET_CMD_EA_QUAD_AUTOCAL       0xEA // sometimes we get this CMD: 00 30 02 EA E4 03 01 A6 while stepping though SPEED SETUP (4th option)
                                               // and i do not know what it does, so we just send a CMD-ACK
#define BGNET_CMD_ACK                   0xFD // 1 byte payload

// layout of the VALUE (1 or more in the payload of a DATA command)
#define BGNET_CHN_CID                   0   // position of channel byte
// FMT byte is encoded as follows
// MSB bit 7,6    : divisor (00=1, 01=10, 10=100, 11=1000)
//     bit 5,4    : size/padding (00=4, 01=3, 10=2, 11=1)
// LSB bit 3,2,1,0: datatype (0000-1111, see decoding routine)
    #define BGNET_FMT_DIVPAD      0xF0    // bits 7,6,5,4  - Divisor & Padding
    #define BGNET_FMT_DIVPAD_DIV  0xC0    // bits 7,6      - Divisor
    #define BGNET_FMT_DIVPAD_PAD  0x30    // bits 5,4      - Padding
    #define BGNET_FMT_DT          0x0F    // bits 3,2,1,0  - DataType

// DATA/LATLON commands (CMD={1|3}) sends instrument data by 'Channel', each idetified by a 'Channel Number' 
#define BGNET_CHANNEL_MAX           0xFF // for sizing purposes (range 00-FF, both including) add 1 for sizing purpose

#define BGNET_CHN_00_NODERESET        0X00
#define BGNET_CHN_01_UNKNOWN          0X01
#define BGNET_CHN_03_UNKNOWN          0X03
#define BGNET_CHN_07_AP_FR            0X07  // FR - Fault Reporting. When PILOT receives this channel, causes PILOT to display an error message and to 'freeze'
                                          // 0x00 = "UNCOMIS"      This channelID is also used in PILOT/SETUP/FR option SELECT.
                                          // 0x01 = "INT CMP"      To ' 'un-freeze' you need to cycle power on the PILOT
                                          // 0x02 = "RUD ERR"
                                          // 0x03 = "RUD DRV"
                                          // 0x04 = "NO SPD"
                                          // 0x05 = "EXT CMP"
                                          // 0x06 = "NO NMEA"
                                          // 0x07 = "TYP RES"
                                          // 0x08 = "BAD XTE"
                                          // 0x09 = "NO WIND"
                                          // 0x0A = "NO OPTW"
                                          // 0x0B = "TRIP"
                                          // 0x0C = "NET ERR"
                                          // 0x0D = "ERR ERR"
                                          // 0x0E = "ERR ERR"
                                          // 0x0F = "ERR ERR"
#define BGNET_CHN_0B_RUDDER           0X0B  // range (-,PORT)<99 or >99(+,STARBOARD)
//#define BGNET_CHN_0C_LINEAR5          0x0C
//#define BGNET_CHN_0D_LINEAR6          0x0D
//#define BGNET_CHN_0E_LINEAR7          0x0E
//#define BGNET_CHN_0F_LINEAR8          0x0F
//#define BGNET_CHN_10_LINEAR9          0x10
//#define BGNET_CHN_11_LINEAR10         0x11
//#define BGNET_CHN_12_LINEAR11         0x12
//#define BGNET_CHN_13_LINEAR12         0x13
//#define BGNET_CHN_14_LINEAR13         0x14
//#define BGNET_CHN_15_LINEAR14         0x15
//#define BGNET_CHN_16_LINEAR15         0x16
//#define BGNET_CHN_17_LINEAR16         0x17
//#define BGNET_CHN_1C_AIRTEMPF         0x1C
//#define BGNET_CHN_1D_AIRTEMPC         0x1D
//#define BGNET_CHN_1E_SEATEMPF         0x1E
//#define BGNET_CHN_1F_SEATEMPC         0x1F
#define BGNET_CHN_20_SEATEMPQ         0x20 //as displayed on QUAD module, selectable C/F
//#define BGNET_CHN_27_HEADLIFTTREND    0x27
//#define BGNET_CHN_29_OFFCOURSE        0x29
#define BGNET_CHN_31_ECHO_SNR         0x31 // DEPTH/QUAD: signal return strengh A-F (" A " is excellent, " F " is poor)
//#define BGNET_CHN_32_TACKPERF         0x32
//#define BGNET_CHN_33_REACHPERF        0x33
//#define BGNET_CHN_34_HEELANGLE        0x34
//#define BGNET_CHN_35_OPTWINDANGL      0x35
//#define BGNET_CHN_36_DEPTHGAIN        0x36
//#define BGNET_CHN_37_DEPTHNOISE       0x37
//#define BGNET_CHN_38_LINEAR1          0x38
//#define BGNET_CHN_39_LINEAR2          0x39
//#define BGNET_CHN_3A_LINEAR3          0x3A
//#define BGNET_CHN_3B_LINEAR4          0x3B
//#define BGNET_CHN_3C_ROLLRATE         0x3C
//#define BGNET_CHN_40_FORESTAY         0x40
#define BGNET_CHN_41_STW              0x41 // from SPEED or QUAD (corrected), but also from PILOT/NMEA/VHW/#5
#define BGNET_CHN_42_STWRAW           0x42 // from SPEED or QUAD (uncorrected), but also from PILOT/NMEA/VHW/#5
//#define BGNET_CHN_43_BOATSPEEDMS      0x43
//#define BGNET_CHN_44_YAWRATE          0x44
#define BGNET_CHN_46_AP_SPEED         0x46 // SPEED selected (0=TRAnsmitter, 1= MANual, 2=SOG); send by PILOT using command 0x4D/46 
//#define BGNET_CHN_47_LONLAT           0x47 //send in ASCII via CMD03
//#define BGNET_CHN_48_LONLAT           0x48 //send in ASCII via CMD03
#define BGNET_CHN_49_HEADING          0x49 // prominently displayed on PILOT (when in STBY, copied to CTARGET when switched to AUTO)
#define BGNET_CHN_4A_DEPTHM           0x4A // referenced by DATA via a LABEL/0x02 command, appears to be "DEPTH M", but seems incorrect
//#define BGNET_CHN_4D_AWINDSPEEDKT     0x4D
#define BGNET_CHN_4E_AWSRAW           0x4E // signal from the paddlewheel, prior correction or calibration 
//#define BGNET_CHN_4F_AWINDSPEEDMS     0x4F
#define BGNET_CHN_50_AWS              0x50 // displayed by WIND (small display, when APP selected)
                                         // displayed by DATA (when WIND button is pressed)
#define BGNET_CHN_51_AWA              0x51 // displayed by WIND (dial, when APP selected)
                                         // displayed by PILOT (when in VANE mode)
#define BGNET_CHN_52_AWARAW           0x52 // signal from the direction sensor, prior correction, calibration or alignment
#define BGNET_CHN_53_TARGET_TWA       0x53 // send by PILOT upon change/button pressed, Target True Wind Angle, fmt=0x01
//#define BGNET_CHN_55_TWS              0x55 // in KT
//#define BGNET_CHN_56_TWS              0x56 // in M/S
#define BGNET_CHN_57_TWS              0x57
#define BGNET_CHN_59_TWA              0x59
#define BGNET_CHN_5A_MEASWINDANGLE    0x5A
#define BGNET_CHN_64_SPEEDAVG         0x64 // displayed by QUAD
#define BGNET_CHN_65_SPEEDMAX         0x65 // displayed by QUAD
#define BGNET_CHN_66_KEELANGLE        0x66
#define BGNET_CHN_67_CANARDANGLE      0x67
#define BGNET_CHN_68_REQFORDATA       0x68
#define BGNET_CHN_69_COURSE           0x69
#define BGNET_CHN_6A_ACTFORDATA       0x6A
#define BGNET_CHN_6D_TRUEWINDDIR      0x6D
#define BGNET_CHN_6F_NEXTLEGAWA       0x6F
#define BGNET_CHN_70_NEXTLEGBOATSPEED 0x70
#define BGNET_CHN_71_NEXTLEGAWSPEED   0x71
#define BGNET_CHN_75_TIMER            0x75
#define BGNET_CHN_76_AP_WA_ALARM      0x76
#define BGNET_CHN_77_ROLLTIMER        0x77
#define BGNET_CHN_7C_POLARPERFORMANCE 0x7C
#define BGNET_CHN_7D_TARGETBOATSPEED  0x7D
#define BGNET_CHN_7F_VMG              0x7F // calculted by WIND, displayed on DATA (only when WIND is present) under the WIND button
#define BGNET_CHN_80_UNKNOWN          0x80
#define BGNET_CHN_81_DRD              0x81 // QUAD/SPEED: Dead Recogning Distance travelled, resettable
#define BGNET_CHN_82_LEEWAY           0x82
#define BGNET_CHN_83_TIDALDRIFT       0x83
#define BGNET_CHN_84_TIDALSET         0x84
#define BGNET_CHN_85_UPWASH           0x85
#define BGNET_CHN_86_BAROTREND        0x86
#define BGNET_CHN_87_BARORAW          0x87 // signal from the barometric sensor, prior calibration
#define BGNET_CHN_8A_UNKNOWN          0x8A
#define BGNET_CHN_8D_BATTERY          0x8D // displayed by WIND (if 'VOLTS' button pressed), fmt=0x48 (120=12.0)
                                         // displayed by DATA (if 'OPTIONS' button is pressed)
#define BGNET_CHN_98_UNKNOWN          0x98 // When a PILOT/NMEA/APB message is received this channel is observed with FMT_00 and
                                         // and only 2 bytes, no data. I do not know if this is on purpose or a undesired feature
                                         // if the XTE parameters are left empty; this channel does not appear.
#define BGNET_CHN_9A_HEADINGNEXTTACK  0x9A
#define BGNET_CHN_9B_FOREAFTTRIM      0x9B
#define BGNET_CHN_9C_MASTANGLE        0x9C
#define BGNET_CHN_9D_WANGLETOMAST     0x9D
#define BGNET_CHN_9E_PITCHRATE        0x9E
#define BGNET_CHN_A2_UNKNOWN          0xA2
#define BGNET_CHN_A4_BOOMPOS          0xA4
#define BGNET_CHN_A6_AP_CTS           0xA6 // Course To Steer send by PILOT upon change/button pressed, Target Course (in AUTO or memory when STBY), fmt=0x08
                                           // seems to be used to communicate multiple type of values/status from ACP to PILOT
#define BGNET_CHN_A7_AP_WPB           0xA7 // send by PILOT upon change/button pressed, WayPoint Bearing, shown as "MEM xxx" (stored previous course), fmt=0x08
#define BGNET_CHN_AF_AP_OC_ALARM      0xAF
#define BGNET_CHN_B3_COMPASSCAL       0xB3
#define BGNET_CHN_B4_UNKNOWN          0xB4
#define BGNET_CHN_B5_AP_STATUS        0xB5 // send by PILOT upon change/button pressed, 2bytes (byte0: 0=STBY,1=AUTO; byte1: 1=VANE;3=COMP;4=NAV; 6=JOYSTICK), fmt=0x08
                                           // 0x50 = STBY, 0x51= AUTO, 0x59=COMPASS steering
                                           // 0x5001 = STBY/COMPASS, 0x5002=STBY/POWER, 0x5102=AUTO/POWER, 0x5901=COMPASS/POWER
    #define BGN_AP_MODE1_MASK_STATE 0x01   // DATA1 - 0 STATE, engaged/not enganged STATE (0=STBY, 1=AUTO)
    #define BGN_AP_MODE1_STBY       0x00   //         1 seems unused
    #define BGN_AP_MODE1_AUTO       0x01   //         2 makes "MEM 000" displayed, PILOT sends command: 00 50 02 CB E3 1E A6 E6, expects MEM value back. When pressed OFF
    #define BGN_AP_MODE1_MASK_INIT  0x04   //           goes into STBY, but you are unable to go back into AUTO.
    #define BGN_AP_MODE1_EXPERIM    0x00   //           Normaly you would press AUTO 2x (within 4 seconds) to make the ACP accept the MEMorized course as CTS
                                           //         3 ??, PILOT sends command: 00 50 02 CB E3 0A A7 F9, expects some value back. When press OFF, goes into STBY
                                           //           but you are unable to go back into AUTO
                                           //         4 seems unused
                                           //         5 DISPLAY, SETUP and SPEED buttons seems deactivated
                                           //         6 seems unused
                                           //         7 seems unused
    #define BGN_AP_MODE2_MASK_MODE  0x07   // DATA2 - 0-2 MODE (1=COMP(+MEM, when in AUTO), 3=NAV(+XTE), 4=VANE(+WND), 6=JOYSTICK)
    #define BGN_AP_MODE2_COMP       0x01   //         3 seems unused                     note: if there is no WIND signal, you can not select VANE mode
    #define BGN_AP_MODE2_NAV        0x03   //         4 PILOT SETUP option appears: "WIND A/T", behaviour unknown
    #define BGN_AP_MODE2_VANE       0x04   //         5 makeds "MEM 000" displayed, but no command observed
    #define BGN_AP_MODE2_JOYSTICK   0x06   //         6-7 seems unused
    #define BGN_AP_MODE2_EXPERIM    0x04   // used for experiments
#define BGNET_CHN_BA_UNKNOWN          0xBA
#define BGNET_CHN_BB_LATITUDE         0xBB
#define BGNET_CHN_BC_LONGITUDE        0xBC
#define BGNET_CHN_C1_DEPTHM           0xC1
#define BGNET_CHN_C2_DEPTHFT          0xC2
#define BGNET_CHN_C3_DEPTHFM          0xC3
#define BGNET_CHN_C4_DEPTH            0xC4 // displayed by QUAD and DATA, or "---" incase no measurement
#define BGNET_CHN_CD_STORLOG          0xCD // QUAD/SPEED: saved in non-volitile memory; retained over the lifespan of the unit
    #define BGNET_DIVISOR_STORLOG       10.0
#define BGNET_CHN_CF_TRIPLOG          0xCF // QUAD/SPEED/DATA: TripLog, reset via QUAD
    #define BGNET_DIVISOR_TRIPLOG       10.0
#define BGNET_CHN_D3_DRC              0xD3 // displayed by SPEED and DATA, reset via QUAD
#define BGNET_CHN_DC_LOCALTIME        0xDC
#define BGNET_CHN_DD_UTCTIME          0xDD
#define BGNET_CHN_DE_WPB              0xDE // WP Bearing (via PILOT/NMEA/APB/#11 en PILOT/NMEA/RMB/#11)
#define BGNET_CHN_E0_BRGW2WRUMBTRUE   0xE0
#define BGNET_CHN_E1_BRGW2WRUMBMAG    0xE1
#define BGNET_CHN_E2_LAYLINEDIST      0xE2
#define BGNET_CHN_E3_BRG2WRUMBTRUE    0xE3
#define BGNET_CHN_E4_BRG2WRUMBMAG     0xE4
#define BGNET_CHN_E5_BRG2WGYROTRUE    0xE5
#define BGNET_CHN_E6_BRG2WGYROMAG     0xE6
#define BGNET_CHN_E7_WAYPDISTRUMB     0xE7
#define BGNET_CHN_E8_WPD              0xE8 // WP Distance (via PILOT/NMEA/RMB/#10)
#define BGNET_CHN_E9_COGTRUE          0xE9 // Course Over Ground, True
#define BGNET_CHN_EA_COGMAG           0xEA // Course Over Ground, Magnetic
#define BGNET_CHN_EB_SOGKT            0xEB // Speed Over Ground, KT (via PILOT/NMEA/VTG/#5)
                                         // Is also observed with command 0xD5 (with a 3 byte payload) after setting FST/MED/SLW (Manual Speed selection on PILOT)
                                         // It is send when you page though the selection option (from FST>MED>SLW>FST), but not the 1st time you press the <SPEED>
                                         // button. The first 2 bytes stays the same (0xEB 0x0F), but the 3rd payload byte varies between 0x00-0xFF (and sometimes
                                         // it does not change). The meaning of this command is unclear. The 3rd byte also does not seem te be a ASCII character.
                                         // The occurance of this channel causes this application to log a message, because the 2nd bye is not a recognised FMT byte.
                                         // Also, the frame with this perculiar command has a broadcast TO address 0xFF, so it is not meant for the whole network.
                                         // example: FF 50 03 D5 D9 0C EB 0F A4, note: "0F" might mean "OFF" ; we suspect that something is switch OFF                                        
#define BGNET_CHN_ED_TIMETOWP         0xED
#define BGNET_CHN_EE_XTE              0xEE
#define BGNET_CHN_EF_REMOTE0          0xEF
#define BGNET_CHN_F0_REMOTE1          0xF0
#define BGNET_CHN_F1_REMOTE2          0xF1
#define BGNET_CHN_F2_REMOTE3          0xF2
#define BGNET_CHN_F3_REMOTE4          0xF3
#define BGNET_CHN_F4_REMOTE5          0xF4
#define BGNET_CHN_F5_REMOTE6          0xF5
#define BGNET_CHN_F6_REMOTE7          0xF6
#define BGNET_CHN_F7_REMOTE8          0xF7
#define BGNET_CHN_F8_REMOTE9          0xF8
#define BGNET_CHN_F9_COURSETOSAIL     0xF9
#define BGNET_CHN_FA_NEXTWPDIST       0xFA
#define BGNET_CHN_FB_TIMETOLAYLINE    0xFB
#define BGNET_CHN_FC_SHALLOWALARM     0xFC // displayed on QUAD, FMT_01= value (0-999) or FMT_04= "OFF"
#define BGNET_CHN_FD_DEEPALARM        0xFD // displayed on QUAD, FMT_01= value (0-999) or FMT_04= "OFF"
                                         // When alarm limit is set, the alarm fires "C4 17 00 C4" channel value (transmitted via 0x01/DATA command)
#define BGNET_CHN_FE_ANCHORALARM      0xFE // displayed on QUAD, FMT_01= value (0-999) or FMT_04= "OFF". The ANCHOR alarms levels
                                         // consists of 2 values (shallow & deep) that are alternating on this channel.
                                         // At the moment you set the limits, these are relative to the measured depth at that moment.
#define BGNET_CHN_FF_DEPTH_CAL        0xFF // displayed on QUAD, FMT_01= value (0-999)
                                         // difference between the deepest point of the boat and the depth of the sensor, usually negative


// Channel data can be of variable length and variable datatype. We create a structure/union to pack/unpack the
// contents of the channel data. Note that some compilers may not correctly align them.
// use htons/htonl (host to network short/long) to write and ntohs/ntohl (network to host short/long) to read/write
// to defeat possible endianess conflicts.
#pragma pack(1)
// this is the value portion of the channel structure. The maximum legnth we have observed was 11 bytes (a FMT_07)
// channel with variable length. To be on the safe side, we use a maximum size of 16.
union CV {
    uint8_t uB[12];     // binary data (status, pattern, mask)
    char uC[12];        // ASCII characters (like alarm text)
    int8_t sB[12];      // signed bytes (byte values with +/- range like depth)
    uint16_t uW[6];     // unsigned 16-bit register
    int16_t sW[6];      // signed 16-bit registers
    uint32_t uL[3];     // unsigned 32-bit register
    int32_t sL[3];      // signed 32-bit register
};
// this is the complete channel structure
union CH {
    uint8_t ucRAW[14]; // sufficient space for the largest one (2 bytes header + max 12 bytes in data)
    uint32_t ulRAW[3];
    struct {
        uint8_t CH;             // channel number
        uint8_t FMT;            // format byte
        union CV cv;            // packed data
    } s;
};
// change detection: when receiving lots of frames, decoding and conversion causes much processing. When we receive a channel (via a DATA frame)
// we will compare the new channel with the previous channel via the RAW element in our union. We could also do a 'memcmp', but again, this will
// cost also resources. If we do compare the new 'RAW' vs the existing we will compare 8 bytes. This covers everything, except the FMT_07, but that
// risk i'm willing to take.
// if we compare to many bytes (most of the channels are only 4 bytes), the ultimate consequence is that we are processing a channel unnecscery,
// what we also would do if we would not attempt to reduce processing channels that dit not habe a significant change.
// NOTE: this only works for 64bit CPU's. Because we want to be compabitble with 32bits CPU, we compare by uL[0]
//
// conclusion: upon processing new channel data we do a "RAW compare" existing vs new. When both RAW values are the same, we skip processing
#pragma pack()

// divisor may be used to scale the value to more digits
#define BGN_FMT_00      0x00        // FMT initialized
#define BGN_FMT_01      0x01        // 16 bit signed register, is used for a value that falls in range -255/+255 (rudder position, wind angle, temperature)
                                    // optionally scaled down to 25.5, 2.55, 0.255 using the divisor
#define BGN_FMT_91      0x91
#define BGN_FMT_03      0x03        // Timer value (5 bytes, 0x75 only), last 3 bytes are HH:MM:SS
#define BGN_FMT_02      0x02        // 32 bit unsigned register (6 bytes) 
#define BGN_FMT_42      0x42        // used by channel 0xCD, STORLOGNM (with divisor = 10)
#define BGN_FMT_04      0x04        // 4 bytes, byte 1 not used (0x00), bytes 2-4 are ASCII, like " T ", " F ", "OFF", "ON " or "---"
                                    // observed: 0x31: " F ", " A " (DEPTH signal quality, F=bad, A=exccelent)
                                    //           alarm setting: "OFF" (when set a 'numeric datatype' is used)
                                    //           alarm state:   "ON " (=alarm), "OFF" (=OK)
                                    //           bad signal: "---" (depth only)
#define BGN_FMT_07      0x07        // variable length, ascii, look for the null-terminator. This appears to be the ALARM trigger for this
                                    // Channel. All devices that have ALARM capability (all but WIND and ?) will start to beep. All devices
                                    // that are able to display the message (PILOT, DATA) show this text. The ALARM beep continues untill
                                    // confirmed (press any key on a ALARM capable device) or the alarm state is returned to normal
                                    // (a null-string) is send.
#define BGN_FMT_08      0x08        // 16 bit unsigned register (0-999), optionally scaled down to 99.9, 9.9 0r 0.9 using the divisor
#define BGN_FMT_48      0x48        // used by channel 0xCF, TRIPLOGNM (with divisor = 10)
#define BGN_FMT_18      0x18        // used by WA and OC
#define BGN_FMT_0C      0x0C        // 3 letter error message, like "INV"
#define BGN_FMT_0F      0x0F        // Boolean/switch value: 0x0F= Off, 0x1F= On
#define BGN_FMT_9D      0x9D        // send by QUAD/DEPTH/SETUP anchor alarm (7 bytes payload)
#define BGN_FMT_0D      0x0D        // used for dual values (Anchor Alarm Shallow & Deep, relative)


// FMT Definition Table
typedef struct {
    int iIndex;
    int iLen;
    char cLabel[64];
} BGNET_FMTDT;

// 16 NETWORK devices exist, the first one (index=0) is used for devices we are unable to identify
#define BGNET_TYPE_MAX            17
// Device Definition Table
typedef struct {
    char sProdName[BGNET_CMD_LABEL_MAX];
    char sModelName[BGNET_CMD_LABEL_MAX];
    char sModelNr[BGNET_CMD_LABEL_MAX];
    char sSerialNr[BGNET_CMD_LABEL_MAX];
    char sVersion[BGNET_CMD_LABEL_MAX];
    int iEnable;
    int iDeviceType;
} BGNET_DEVDT;


// allow import/export definitions for NMEA & Signal K. import or export is exclusive. When you import from one, you can not
// export to the other You can't have it both ways.
#define BGNET_CHN_IMPEXP_NONE         0       // no provisions for import/export
#define BGNET_CHN_IMPEXP_NMEA         1       // import from or export to NMEA2000
#define BGNET_CHN_IMPEXP_SIGNALK      2       // import from or export to SignalK

// Channel Definition Table
typedef struct {
    uint8_t ucChannelIndex;         // Channel index
    char cName[BGNET_CMD_LABEL_MAX];   // the label, variable name
    uint8_t ucFMT;                  // default formatting
    uint8_t ucFROM;                 // instrument type that produces this channel
    uint8_t ucFreq;                 // frequency (0= none, 1=every run, 2= every other run, 3 = every 2nd run)
    uint8_t ucPhase;                // phase, how to overlap (only when freq>1)
    char cLabel[BGNET_CMD_LABEL_MAX];   // the label, variable name
    uint8_t ucCMD_06;               // DeviceType that might SUBSCRIBE to this channel (so we can replicate that)
    int iImpExpEnable;              // allow Import or Export
    int iImportFrom;                // Import from NMEA2000 or SignalK
    int iExportTo;                  // Export to NMEA2000 of SignalK
    int iNMEA_PGN;                  // NMEA2000 reference PGN
    char cSKdef[50];                // SignalK reference string 
} BGNET_CHNDT;

typedef struct {
    uint8_t ucCommandIndex;         // Channel index
    char cLabel[BGNET_CMD_LABEL_MAX];   // the label, variable name
} BGNET_CMDDT;

#define BGN_FRAME_SIZE    120       // a BGNET frame usually does not exceed 120 characters

// register device statistics
typedef struct {
    int iPowerUp; // POWERUP command seen
    int iNewDevice; // New DEVICE data might have been received
    BGNET_COUNT CounterRx;
    int iCSError_Header;
    int iCSError_Frame;
    union {
        unsigned char ucFrame[BGN_FRAME_SIZE];
        FRAME_H FH;
    } Rx;
    //unsigned char ucFrameRx[BGN_FRAME_SIZE]; // most recent Rx frame (binary)
    BGNET_COUNT CounterTx;
    union {
        unsigned char ucFrame[BGN_FRAME_SIZE];
        FRAME_H FH;
    } Tx;
    //unsigned char ucFrameTx[BGN_FRAME_SIZE]; // most recent Tx frame (binary)
    size_t source;
} BGN_DEVICE;

// register Command statistics
typedef struct {
    int iNewCommand; // new COMMAND was received
    BGNET_COUNT CounterRx;
    union {
        unsigned char ucFrame[BGN_FRAME_SIZE];
        FRAME_H FH;
    } Rx;
    //unsigned char ucFrameRx[BGN_FRAME_SIZE]; // most recent Rx frame (binary)
    BGNET_COUNT CounterTx;
    union {
        unsigned char ucFrame[BGN_FRAME_SIZE];
        FRAME_H FH;
    } Tx;
    //unsigned char ucFrameTx[BGN_FRAME_SIZE]; // most recent Tx frame (binary)
} BGN_COMMAND;

typedef struct {
    int iNewChannel; // some new channel data was received
    struct {
        unsigned char ucFROM; // the station that transmitted the channel
        // we store the raw byte sequence for this channel. From the FMT character, you can derive the length of the channel sequence
        unsigned char ucRAW[sizeof(union CH)]; // raw data (for diagnostic purposes)
        // we store the channel value in ASCII for display purposes
        char cValue[sizeof(union CV)]; // ascii formatted value string (for display purposes)
        BGNET_COUNT CounterRx;
    } x01; // DATA command
    struct {
        unsigned char ucRAW[30]; // raw data (for diagnostic purposes), usually 18 bytes payload (=24 bytes complete frame)
        char cValue[24]; // payload - 4 bytes + 1 null terminator, usually 0x12 bytes maximum      
        BGNET_COUNT CounterRx;
    } x02; // LABEL command
    struct {
        unsigned char ucTO;
        unsigned char ucFROM;
        unsigned char ucRAW[10]; // raw data (for diagnostic purposes), usually 1 bytes payload (=7 bytes complete frame)
        BGNET_COUNT CounterRx;
    } x06; // SUBSCRIBE command
    struct {
        unsigned char ucTO;
        unsigned char ucFROM;
        unsigned char ucCMD;
        unsigned char ucRAW[10]; // raw data (for diagnostic purposes)
        // we store the channel value in ASCII for display purposes
        char cValue[sizeof(union CV)]; // ascii formatted value string (for display purposes)
        BGNET_COUNT CounterRx;
    } x00; // Other commands, usually PILOT/ACP handshakes
} BGN_CHANNEL;

typedef struct {
    int iNewACP;
    time_t tTimestamp;          // timestamp of startup
    // we have observed weird behaviour......
    // the ACKSEQ number goes from 00.....0C and then back to 0A when we would expect the 0D as the ACKSEQ.
    // at the same time we get CS errors that are 3 off.... just the difference between 0A and 0D.
    // realize we are dealing with unsigned char, we can not use negiative numbers.
    // Normally a received ACKSEQ is the same (withing the current handshake), or the next one (1 bigger). But
    // when a new received ACKSEQ is less then the previous one, it seems that we have to adjust the CS-SEED
    // This is an experiment to see if it is what it seems to be.
    struct {
        unsigned char ACKSEQprev; // last ACKSEQ received from PILOT
        unsigned char ACKSEQ; // last ACKSEQ received from PILOT
        //unsigned char FRAME_SEED; // SEED to use
    } weird;
    

    // parameters, we need to manage our service
    struct {
        NMEA2K_VALUE SystemTime;           // system time (our computer time)
        NMEA2K_VALUE NetworkTime;          // network time
#define BGNET_PILOTWD_SET   10
        NMEA2K_VALUE PILOTWD;       // PILOT WatchDog: flunk timer. With every "PILOT_HELLO" we set this to 100, the timer counts it down (every device run)
                                // when 0, ACP generates an error and falls back to STBY/COMP mode.
    } SERVICE;
    struct {
    // parameters, usually from the network (any device)
        NMEA2K_VALUE BackLight;     // 0=Off, 3-1=ON
    } NETWORK;
    // parameters, usually hosted by ACP. How do we know what paramaters are hosted by what nodes? Some seem logical, others not.
    // if we look the the command 0x06 that the DATA uses to register it's values, you see that it adresses the ACP/PILOT/QUAD/WIND node
    // this is a clear indication where what parameters lives.
    struct {
        NMEA2K_VALUE STATE;
        NMEA2K_VALUE MODE;
#define BGN_JOYSTICK_ON     1
#define BGN_JOYSTICK_OFF    0
        NMEA2K_VALUE Joystick;      // 0=Off, 1=ON, overrides control mode (COMP/NAV/VANE)
        NMEA2K_VALUE EBS;           // Effective Boat Speed, result of speed selection, used as initializer for SpeedMAX and SpeedAVG
        NMEA2K_VALUE EBS_SEL;       // Effective Boat Speed, selector
        NMEA2K_VALUE CTS;           // Course To Steer (Setpoint for AutoPilot)
        NMEA2K_VALUE Rudder;        // Rudder position (-45/+45 degrees)   
        NMEA2K_VALUE COG;
        NMEA2K_VALUE SOG;
        NMEA2K_VALUE SHA_ALARM;
        NMEA2K_VALUE DEEP_ALARM;     // WIND Selection Type (0=T, 1 =A)
    } ACP;
    // parameters, usually hosted by PILOT
    struct {
        NMEA2K_VALUE Heading;       // Compass Heading
        struct {
            // PILOT/SETUP parameters
            NMEA2K_VALUE SPD;     // 0=TRA (Transmitter); 1=Manual (SLW/MED/FST); 2=SOG (from NMEA/GPS)
            NMEA2K_VALUE RESP;    // Steering response (Normal/Downwind/Eco)
            NMEA2K_VALUE WIND_SEL;              NMEA2K_VALUE WA_MIN;  // Watch Alarm (minutes)
            NMEA2K_VALUE WA_STAT; // Watch Alarm (status on/off)
            NMEA2K_VALUE OC_DEG;  // OffCourse Alarm (degrees)
            NMEA2K_VALUE OC_STAT; // OffCourse Alarm (status on/off)
            NMEA2K_VALUE JSM;     // Joystick Steering Mode (Normal or Proportional)
            NMEA2K_VALUE DMP;     // Compass heading display damping (1-99 sec, defaut =4)
            NMEA2K_VALUE FR;      // Fault Reporting ON/OFF
        } SETUP;
    // PILOT/SPEED/SETUP - Manual speed selection
#define BGNET_SPD_TRA           0
#define BGNET_SPD_MAN_FST       1
#define BGNET_SPD_SOG           2
#define BGNET_SPD_MAN_MED       3
#define BGNET_SPD_MAN_SLW       4
        struct {
            NMEA2K_VALUE SEL;  // selected manual speed (FST/MED/SLW)
            NMEA2K_VALUE FST;  // speed FST x10
            NMEA2K_VALUE MED;  // speed MED x10
            NMEA2K_VALUE SLW;  // speed SLW x10
        } SPEED;
    } PILOT;
    // SPEED parameters
    struct {
        // Note: if you change from C/F only a new LABEL command is broadcast. No CMD-REQ/CMD-DAT/CMD-ACK sequence is involved
        NMEA2K_VALUE Temperature;  // Measured TEMP (current value)
        // Note: if you change from KT/MPH only a new LABEL command is broadcast. No CMD-REQ/CMD-DAT/CMD-ACK sequence is involved
        NMEA2K_VALUE STW;          // Speed Through Water (Measured SPEED)
        NMEA2K_VALUE StoredLogNM;  // channel CD (stored log) is stored as 3-byte register
        NMEA2K_VALUE TripLogNM;    // channel CF (trip log NM) is stored as a 2 byte register
        NMEA2K_VALUE DRD;          // channel 81 (Dead Reconing Distance) is stored as a 2 byte register
        NMEA2K_VALUE DRC;          // channel D3 (Dead Reconing Course) is stored as a 2 byte register
        NMEA2K_VALUE SpeedMAX;     // MAX speed (since DRD reset)
        NMEA2K_VALUE SpeedAVG;     // AVG speed (since DRD reset)
        NMEA2K_VALUE TIMER;        // Timer (no yet implemented, but lives here)
        struct {
            NMEA2K_VALUE DMP;      // QUAD/SPEED/SETUP/#1/DMP, response time of speed value
            //                     // QUAD/SPEED/SETUP/#2 causes the units to change (with revised LABEL/0x02 CMD)
            NMEA2K_VALUE CAL;      // QUAD/SPEED/SETUP/#3, #4 CALibration, boatspeed adjustment by speed (#3) or by distance (#4)
            NMEA2K_VALUE AUTOCAL;  // QUAD/SPEED/SETUP/#5/AUTOCAL, 0=Off, 1=On
            //                     // QUAD/SPEED/SETUP/#6 is the DRD-RESET, no value set, just resetting the DRD (and speed AVG/MAX)
        } SETUP;
    } SPEED;
    // DEPTH parameters
    struct {
        NMEA2K_VALUE Depth;        // Measured DEPTH (current value)
        NMEA2K_VALUE ACH_ALARM;
        struct {
            NMEA2K_VALUE SA_M;     // Shallow Alarm limit (meter, absolute)
            NMEA2K_VALUE SA_STAT;  // Shallow Alarm ON/OFF
            NMEA2K_VALUE DA_M;     // Deep Alarm limit (meter, absolute)
            NMEA2K_VALUE DA_STAT;  // Deep Alarm ON/OFF
            NMEA2K_VALUE ASA_M;    // Anchor Shallow Alarm limit (meter, relative to the depth at the moment of setting)
            NMEA2K_VALUE ADA_M;    // Anchor Deep Alarm limit (meter, relative to the depth at the moment of setting)
            NMEA2K_VALUE AMA_M;    // Anchor Measured depth at the moment of setting
            NMEA2K_VALUE AA_STAT;  // Anchor Alarm ON/OFF
        } SETUP;
    } DEPTH;
    struct {
        NMEA2K_VALUE AWS;
        NMEA2K_VALUE AWA;
        NMEA2K_VALUE TWS;
        NMEA2K_VALUE TWA;
        NMEA2K_VALUE VMG;
        NMEA2K_VALUE BATTERY;

    } WIND;
} BGNET;

#define BGNET_CTRL_INIT            1
#define BGNET_CTRL_INIT_RESET      0
// we count down from 1000 before we do the device processing
#define BGNET_CTRL_DEVCNTR_INIT    1000

typedef struct {
    int iInitialized;
    // the timer service decrements this value to zero (and stops); if zero the 'device processing' takes place
    int iDeviceCounter; // count down by timer
    int iDeviceRun; // number of runs (0-1000) with wrap-around

    // pointers for control structures. Do not write into
    BGNET_DEVDT* pDEVDT; // device definitions
    BGNET_CHNDT* pCHNDT; // channel definitions
    BGNET_FMTDT* pFMTDT; // format definitions
    BGNET_CMDDT* pCMDDT; // format definitions
    Config* pCfg;    // config structure


    BGNET BGNET; // ACP status
    BGN_DEVICE DEV[BGNET_DEVICE_MAX]; // device statistics
    BGN_COMMAND CMD[BGNET_COMMAND_MAX]; // commands statistics
    BGN_CHANNEL CHN[BGNET_CHANNEL_MAX]; // channels statistics
#define BGN_NEW_SET     1
#define BGN_NEW_RESET   0
    int iNewDevice;     // when 1: new DEVICE data was received (scan though DEV structure)
    int iNewCommand;    // when 1: new COMMAND was received (scan though CMD structure)
    int iNewChannel;    // when 1: new CHANNEL data was received (scan though CHANNEL structure)
} BGNET_CTRL;

// prototypes
size_t bgnet_decode_frame(BGNET_CTRL *, unsigned char *, size_t);
size_t bgnet_process(BGNET_CTRL *, unsigned char *, size_t);
void bgnet_initialize_reset(BGNET_CTRL *);
void bgnet_initialize(BGNET_CTRL *);
int bgnet_deviceindex(BGNET_CTRL *, unsigned char);
size_t bgnet_sizeof_channel(BGNET_CTRL *, union CH *);
size_t bgnet_sizeof_frame(FRAME_H *);
char* strCommandLabel(BGNET_CTRL *, char *, unsigned char);
char* strChannelLabel(BGNET_CTRL *, char *, unsigned char);
size_t bgnet_encode_frame(FRAME_H *, int, int, int, unsigned char *, int);
int makeRESPresponse(BGNET_CTRL *, unsigned char *);
int makeWAresponse(BGNET_CTRL *, unsigned char *);
int makeOCresponse(BGNET_CTRL *, unsigned char *);
size_t register_DATA_frame(BGNET_CTRL *, FRAME_H *);
uint16_t EffectiveBoatSpeed(BGNET_CTRL *, NMEA2K_VALUE *);

void CounterInit(BGNET_COUNT*, time_t);
void CounterUp(BGNET_COUNT*, time_t);
void NMEAinit(NMEA2K_VALUE*, time_t);

// convert Command Frame to ASCII for display purposes
#define BGN_CMD_ASCII              48
#define BGN_CMD_ASCII_SEP   " /.../ "
#define BGN_CMD_DATA_TAIL           6
char* CommandToAscii(char*, size_t, FRAME_H*);

#endif
