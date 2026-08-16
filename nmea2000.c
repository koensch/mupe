#include "nmea2000.h"
#include "app_state.h"
#include "logger.h"
#include "bgnprotocol.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
//#include <net/if.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <arpa/inet.h>

/* notes on NMEA 2000 interfaces
    if you use a CANable interface (like i do), you will observe the phenomenon that when your CANable experiense a short power-down (<5sec, like
    what happens with a reboot!) that your device goes into the DFU mode ("bootloader mode"), just the red 'power' LED on.
    For normal operations you what to have your device in the "normal mode" (LED red 'power' ON and blue 'state' flashing).
    If you have some other CAN interface, this might be irrelevant for you (or might have other phenomenons)

    You could solve this:
    1 - power-off the USB device at shutdown time (custom script in .etc/rc6.d) and after bootup (with a delay of 10sec)  run a startup script to
        power it back-on.
        You can use the utility 'uhubctl', it's great. But your USB controller (in your computer) must support the feature to enable/disable power of
        individual ports. My computer does not have that (or my older VMWare hypervisor does not support it)
        Running uhubctl returns: "No compatible devices detected!"
    2 - Some USB contrtollers have the capability to powerdown the whole hub (all USB devices). Be careful you your computer boots from a USB boot drive.
        You don't want to lock yourself out.
    3 - On the CANable USB device, "disable the pin BOOT0" via the flash utility.
    4 - Manually #1:
        - CAN adapter is running fine (RED/power on, BLUE/state blinking), candump command returns data
        - reboot computer (sudo shutdown now -r), after reboot CANable adapter shows all 3 LED on (RED, BLUE, GREEN). command 'ip link ls' shows can adapter
          absent.
        - unplug the USB adapter from the computer (all LED are off)
        - wait at least 5 seconds
        - plugin USB adapter, RED/power ON, GREEN/work on (BLUE/state is off)
        - observe that canbus does not appear in "sudo ip link ls"
        - reinitialise can adapter with commands:
            - ip link set down can0
            - ip link set can0 type can bitrate 250000
            - ip link set up can0
        - CAN adapter is running fine (RED/power on, BLUE/state blinking), candump command returns data
    5 - Manually #2:
        - CAN adapter is running fine (RED/power on, BLUE/state blinking), candump command returns data
        - in VMWare remove USB device from virtual machine
        - all LED on, RED/power, BLUE/state, GREEN/work
        - reboot computer (sudo shutdown now -r)
        - in VMWare, add USB device to virtual machine
        - all LED on, RED/power, BLUE/state, GREEN/work
        - observe that canbus appears in "sudo ip link ls", but is "down"
        - reinitialise can adapter with commands:
            - ip link set down can0
            - ip link set can0 type can bitrate 250000
            - ip link set up can0
        - CAN adapter is running fine (RED/power on, BLUE/state blinking), candump command returns data
    
    If you are running on a Raspberry Pi with some HAT providing you with a 'real' CANbus, like the OpenMarine 'MacArthur HAT' you'll save yourself a lot
    of troubles.
*/
/* Helpers for N2K “not available” values */
static int16_t n2k_i16(uint8_t lo, uint8_t hi) {
    return (int16_t)(lo | (hi << 8));
}
static uint16_t n2k_u16(uint8_t lo, uint8_t hi) {
    return (uint16_t)(lo | (hi << 8));
}
static uint32_t n2k_u32(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    return (uint32_t)b0 | ((uint32_t)b1 << 8) | ((uint32_t)b2 << 16) | ((uint32_t)b3 << 24);
}

//============================== PGN handling

/* 126992 – System Time
 * byte 0       Sequence ID     to sync with other fast data
 * byte 1       Reserved
 * byte 2-3     Date            days since 1970-01-01
 * byte 4-7     Time            seconds past midnight
 */
static void parse_pgn_126992(BGNET_CTRL *pCtrl, const struct can_frame *f)
{
    uint16_t days = ntohs(n2k_u16(f->data[3], f->data[2])); // days since 1970-01-01
    uint32_t seconds = ntohl(n2k_u32(f->data[7], f->data[6],f->data[5], f->data[4])) * 1e-4; // seconds since midnight * 1000
    time_t time_raw = days * 86400 + seconds;

    pthread_mutex_lock(&g_data_mutex);
    g_n2kdata.boat.time.value.tTime = time_raw;
    g_n2kdata.boat.time.source = BGN_SOURCE_NMEA;
    CounterUp(&g_n2kdata.boat.time.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
    pthread_mutex_unlock(&g_data_mutex);

    struct tm *ptmInfo;
    char asciibuffer[40];
    ptmInfo = localtime(&g_n2kdata.boat.time.value.tTime);
    strftime(asciibuffer, sizeof(asciibuffer), BGNET_TIME_FMT, ptmInfo);

    //log_debug("N2K 126992: received time= %s (localtime)", asciibuffer);

    switch(pCtrl->BGNET.SERVICE.NetworkTime.source) {
        case BGN_SOURCE_NONE:
        case BGN_SOURCE_ACP:
        case BGN_SOURCE_NMEA:
            pCtrl->BGNET.SERVICE.NetworkTime.value.tTime = time_raw;
            pCtrl->BGNET.SERVICE.NetworkTime.source = BGN_SOURCE_NMEA;
            CounterUp(&pCtrl->BGNET.SERVICE.NetworkTime.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
    }
}

/* 126993 – Heartbeat
 * byte 0       Reserved        no data
 */
static void parse_pgn_126993()
{
    //log_debug("N2K 126993: Heartbeat");
}

/* 127245 – Rudder Angle
 * Angle: -PI (-180) to +Pi (+180) degrees, range 0-30000
 * mechanical limits are restricted between +/-45 and +/-90 degrees
 * byte 0       instance        in case you have multiple rudders (0x00=Main/Single, 0xFC=Port, 0xFD=Starboard)
 * byte 1       order           ordered direction (0x00=No Order, 0x01=Move to Port, 0x02=Move to Starboard
 * byte 2       not used
 * byte 3-4     order position  angle order, 0.0001 rad/bit or 0xFFFF (no order)
 * byte 5-6     position        measured position, 0.0001 rad/bit, += starboard, -=port
 */
static void parse_pgn_127245(BGNET_CTRL *pCtrl, const struct can_frame *f)
{
    //int16_t order_raw = (int16_t)(ntohs(ntohs(n2k_i16(f->data[4], f->data[3])))); 
    int16_t rudder_raw = (int16_t)(ntohs(ntohs(n2k_i16(f->data[6], f->data[5])))); 

    //double order_rad  = order_raw * 1e-4;  /* devide by 1000 to get rads */
    double rudder_rad  = rudder_raw * 1e-4;  /* devide by 1000 to get rads */

    pthread_mutex_lock(&g_data_mutex);
    g_n2kdata.boat.rudder.value.dValue = rudder_rad;
    g_n2kdata.boat.rudder.source = BGN_SOURCE_NMEA;
    CounterUp(&g_n2kdata.boat.rudder.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
    pthread_mutex_unlock(&g_data_mutex);

    //log_debug("N2K 127245: rudder %d, Order/Dir= %x, Order/Pos= %d, Actual/Pos %d [deg]", f->data[0], (f->data[1] & 0x03), (int)(order_rad*180/M_PI), (int)(rudder_rad*180/M_PI));

    switch(pCtrl->BGNET.ACP.Rudder.source) {
        case BGN_SOURCE_NONE:
        case BGN_SOURCE_ACP:
        case BGN_SOURCE_NMEA:
            pCtrl->BGNET.ACP.Rudder.value.nLong = DTOIROUND(rudder_rad * 180/M_PI);
            pCtrl->BGNET.ACP.Rudder.source = BGN_SOURCE_NMEA;
            CounterUp(&pCtrl->BGNET.ACP.Rudder.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
            break;
    }
}

/* 127250 – Heading
 */
static void parse_pgn_127250(BGNET_CTRL *pCtrl, const struct can_frame *f)
{
    //u_int16_t heading_raw = ntohs(n2k_i16(f->data[3], f->data[2])); // heading, magnetic, range 0-2PI rad, 0-259 Deg
    //int16_t deviation_raw = (int16_t)ntohs(n2k_i16(f->data[5], f->data[4])); // deviation, range -PI-+PI, -180/+180 Deg
    //int16_t variation_raw = (int16_t)ntohs(n2k_i16(f->data[7], f->data[6])); // deviation, range -PI-+PI, -180/+180 Deg
    u_int16_t heading_raw = ntohs(n2k_i16(f->data[2], f->data[1])); // heading, magnetic, range 0-2PI rad, 0-259 Deg
    int16_t deviation_raw = (int16_t)ntohs(n2k_i16(f->data[4], f->data[3])); // deviation, range -PI-+PI, -180/+180 Deg
    int16_t variation_raw = (int16_t)ntohs(n2k_i16(f->data[6], f->data[5])); // deviation, range -PI-+PI, -180/+180 Deg

    double heading = 0.0;
    double deviation = 0.0;
    double variation = 0.0;
    if (heading_raw == 0xFFFF) {
        // no data do not update
    } else {
        heading = heading_raw * 1e-4;  /* devide by 1000 to get rads */
        pthread_mutex_lock(&g_data_mutex);
        g_n2kdata.boat.headingM.value.dValue = heading; // Magnetic (no compensation)
        g_n2kdata.boat.headingM.source = BGN_SOURCE_NMEA;
        CounterUp(&g_n2kdata.boat.headingM.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
        pthread_mutex_unlock(&g_data_mutex);

        if ((deviation_raw == 0x7FFF) | (variation_raw == 0x7FFF)) {
            // no data do not update
        } else {
            deviation = deviation_raw * 1e-4; /* device by 1000 to get rads */
            variation = variation_raw * 1e-4; /* device by 1000 to get rads */
            pthread_mutex_lock(&g_data_mutex);
            g_n2kdata.boat.headingT.value.dValue = (heading + deviation + variation); // True (with compensation)
            g_n2kdata.boat.headingT.source = BGN_SOURCE_NMEA;
            CounterUp(&g_n2kdata.boat.headingT.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
            pthread_mutex_unlock(&g_data_mutex);

            switch(pCtrl->BGNET.PILOT.Heading.source) {
                case BGN_SOURCE_NONE:
                case BGN_SOURCE_ACP:
                case BGN_SOURCE_NMEA:
                    pCtrl->BGNET.PILOT.Heading.value.nLong = DTOIROUND(heading * 180/M_PI);
                    pCtrl->BGNET.PILOT.Heading.source = BGN_SOURCE_NMEA;
                    CounterUp(&pCtrl->BGNET.PILOT.Heading.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
                    break;
            }
        }
    }
    log_debug("N2K 127250: Heading= %d, Deviation= %d, Variation= %d", DTOIROUND(heading * 180/M_PI), DTOIROUND(deviation * 180/M_PI), DTOIROUND(variation * 180/M_PI));
}

/* 128259 – Speed, Water Referenced (STW, Speed Through Water)
 * Water speed: 0.01 m/s
 */
static void parse_pgn_128259(BGNET_CTRL *pCtrl, const struct can_frame *f)
{
    uint16_t speed_raw = ntohs(n2k_u16(f->data[3], f->data[2]));

    if (speed_raw == 0xFFFF) {
        // no data do not update
    } else {

        double stw = speed_raw * 0.01; // m/s

        pthread_mutex_lock(&g_data_mutex);
        g_n2kdata.boat.stw.value.dValue = stw;
        g_n2kdata.boat.stw.source = BGN_SOURCE_NMEA;
        CounterUp(&g_n2kdata.boat.stw.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
        pthread_mutex_unlock(&g_data_mutex);

        log_debug("N2K 128259: STW=%.2f m/s", stw);

        switch(pCtrl->BGNET.SPEED.STW.source) {
            case BGN_SOURCE_NONE:
            case BGN_SOURCE_ACP:
            case BGN_SOURCE_NMEA:
                pCtrl->BGNET.SPEED.STW.value.dValue = (stw * 1.94384449 * 10); // convert m/s to KT*10
                pCtrl->BGNET.SPEED.STW.source = BGN_SOURCE_NMEA;
                CounterUp(&pCtrl->BGNET.SPEED.STW.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
        }
    }
}

/* 128267 – Water Depth
 * Depth: 0.01 m, Offset: 0.01 m
 * We publish depth below transducer (no offset correction here).
 */
static void parse_pgn_128267(BGNET_CTRL *pCtrl, const struct can_frame *f)
{
    uint16_t depth_raw  = ntohs(n2k_u16(f->data[1], f->data[0]));
    uint16_t offset_raw = ntohs(n2k_u16(f->data[3], f->data[2]));

    if (depth_raw == 0xFFFF) {
        // no data do not update
    } else {
        double depth  = depth_raw * 0.01;  /* meters */
        double offset = (offset_raw == 0xFFFF) ? 0.0 : (int16_t)offset_raw * 0.01;

        pthread_mutex_lock(&g_data_mutex);
        g_n2kdata.boat.depth.value.dValue = (depth + offset);
        g_n2kdata.boat.depth.source = BGN_SOURCE_NMEA;
        CounterUp(&g_n2kdata.boat.depth.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
        pthread_mutex_unlock(&g_data_mutex);

        log_debug("N2K 128267: depth=%.2f M", (depth + offset));
    }
}

/* 129025 – Position, Rapid Update (only included lat/lon, no other parameters)
 * Lat/Lon in 1e-7 degrees, signed 32-bit
 */
static void parse_pgn_129025(BGNET_CTRL *pCtrl, const struct can_frame *f)
{
    int32_t lat_raw = (int32_t)ntohl(n2k_u32(f->data[3], f->data[2], f->data[1], f->data[0]));
    int32_t lon_raw = (int32_t)ntohl(n2k_u32(f->data[7], f->data[6], f->data[5], f->data[4]));

    if (lat_raw == 0x7FFFFFFF || lon_raw == 0x7FFFFFFF) {
        // no data do not update
    } else {

        double lat = lat_raw * 1e-7;
        double lon = lon_raw * 1e-7;

        pthread_mutex_lock(&g_data_mutex);
        g_n2kdata.pos.lat.value.dValue = lat;
        g_n2kdata.pos.lat.source = BGN_SOURCE_NMEA;
        CounterUp(&g_n2kdata.pos.lat.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);

        g_n2kdata.pos.lon.value.dValue = lon;
        g_n2kdata.pos.lon.source = BGN_SOURCE_NMEA;
        CounterUp(&g_n2kdata.pos.lon.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
        pthread_mutex_unlock(&g_data_mutex);

        //log_debug("N2K 129025: lat=%.7f lon=%.7f", lat, lon);
    }
}

/* 129026 – COG & SOG, Rapid Update
 * COG: 0.0001 rad, SOG: 0.01 m/s
 */
static void parse_pgn_129026(BGNET_CTRL *pCtrl, const struct can_frame *f)
{
    uint16_t cog_raw = ntohs(n2k_u16(f->data[3], f->data[2]));
    uint16_t sog_raw = ntohs(n2k_u16(f->data[5], f->data[4]));

    if (cog_raw == 0xFFFF) {
        // no data do not update
    } else {
        double cog = cog_raw * 1e-4; /* radians */
        pthread_mutex_lock(&g_data_mutex);
        g_n2kdata.pos.cog.value.dValue = cog;
        g_n2kdata.pos.cog.source = BGN_SOURCE_NMEA;
        CounterUp(&g_n2kdata.pos.cog.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
        pthread_mutex_unlock(&g_data_mutex);

        switch(pCtrl->BGNET.PILOT.NMEA.VTG.COG.source) {
            case BGN_SOURCE_NONE:
            case BGN_SOURCE_ACP:
            case BGN_SOURCE_NMEA:
                pCtrl->BGNET.PILOT.NMEA.VTG.COG.value.nLong = DTOIROUND(cog * 180/BGNET_PI); // convert rad to deg
                pCtrl->BGNET.PILOT.NMEA.VTG.COG.source = BGN_SOURCE_NMEA;
                CounterUp(&pCtrl->BGNET.PILOT.NMEA.VTG.COG.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
        }
        //log_debug("N2K 129026: COG=%.4f rad", cog);
    }

    if (sog_raw == 0xFFFF) {
        // no data do not update
    } else {
        double sog = sog_raw * 0.01; /* m/s */
        pthread_mutex_lock(&g_data_mutex);
        g_n2kdata.pos.sog.value.dValue = sog;
        g_n2kdata.pos.sog.source = BGN_SOURCE_NMEA;
        CounterUp(&g_n2kdata.pos.sog.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
        pthread_mutex_unlock(&g_data_mutex);

        switch(pCtrl->BGNET.PILOT.NMEA.VTG.SOG.source) {
            case BGN_SOURCE_NONE:
            case BGN_SOURCE_ACP:
            case BGN_SOURCE_NMEA:
                pCtrl->BGNET.PILOT.NMEA.VTG.SOG.value.nLong = DTOIROUND(sog * 1.94384 *10); // convert m/s to KT
                pCtrl->BGNET.PILOT.NMEA.VTG.SOG.source = BGN_SOURCE_NMEA;
                CounterUp(&pCtrl->BGNET.PILOT.NMEA.VTG.SOG.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
        }
        //log_debug("N2K 129026: SOG=%.2f m/s", sog);
    }

}
/* 129029 – GNSS position data
 * byte 0       SID             Sequence ID to group 'fast packages'
 * byte 1,2     Position Date   days since 1970-01-01
 * byte 3,6     Position Time   seconds since midnight, 0.0001s/bit
 * byte 7-14    Latitude        double precision float 1x10^-16 deg/bit
 * byte 15-22   Longitude       double precision float 1x10^-16 deg/bit
 * byte 23-26   Altitude        long integer, signed 1x10^-6 m/bit
 * byte 27      Type            0=GPS, 1=GLONASS, 2=GPS+GLONASS, 3=Galileo
 */
static void parse_pgn_129029(BGNET_CTRL *pCtrl, const struct can_frame *f)
{
    int32_t alt_raw = (int32_t)n2k_u32(f->data[23], f->data[24], f->data[25], f->data[26]);

    if (alt_raw == 0xFFFF) {
        // no data do not update
    } else{
        double alt = alt_raw * 1e-6;
        pthread_mutex_lock(&g_data_mutex);
        g_n2kdata.pos.alt.value.dValue = alt;
        g_n2kdata.pos.alt.source = BGN_SOURCE_NMEA;
        CounterUp(&g_n2kdata.pos.alt.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
        pthread_mutex_unlock(&g_data_mutex);

        //log_debug("N2K 129029: ALT=%.4f M", alt);
    }
}

/* 130306 – Wind Data
 * Wind speed: 0.01 m/s
 * Wind angle: 0.0001 rad
 * Reference:
 *   0 = True (ground)
 *   1 = Magnetic
 *   2 = Apparent
 *   3 = True (water)
 */
static void parse_pgn_130306(BGNET_CTRL *pCtrl, const struct can_frame *f)
{
    uint8_t ref      = f->data[0] & 0x07;
    uint16_t speed_r = n2k_u16(f->data[1], f->data[2]);
    uint16_t angle_r = n2k_u16(f->data[3], f->data[4]);

    if (speed_r == 0xFFFF || angle_r == 0xFFFF) {
        // no data do not update
    } else {
        double speed = speed_r * 0.01;   /* m/s */
        double angle = angle_r * 1e-4;   /* rad */

        pthread_mutex_lock(&g_data_mutex);
        switch (ref) {
            case 2: /* Apparent */
                g_n2kdata.wind.aws.value.dValue = speed;
                g_n2kdata.wind.aws.source = BGN_SOURCE_NMEA;
                CounterUp(&g_n2kdata.wind.aws.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
                g_n2kdata.wind.awa.value.dValue = angle;
                g_n2kdata.wind.awa.source = BGN_SOURCE_NMEA;
                CounterUp(&g_n2kdata.wind.awa.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
                break;
            case 0: /* True (ground) */
            case 3: /* True (water) */
                g_n2kdata.wind.tws.value.dValue = speed;
                g_n2kdata.wind.tws.source = BGN_SOURCE_NMEA;
                CounterUp(&g_n2kdata.wind.tws.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
                g_n2kdata.wind.twa.value.dValue = angle;
                g_n2kdata.wind.twa.source = BGN_SOURCE_NMEA;
                CounterUp(&g_n2kdata.wind.twa.Counter, pCtrl->BGNET.SERVICE.SystemTime.value.tTime);
                break;
            default:
                break;
        }
        pthread_mutex_unlock(&g_data_mutex);

        log_debug("N2K 130306: ref=%u speed=%.2f m/s angle=%.4f rad", ref, speed, angle);
    }
}

// ========================================== worker thread

void *nmea2000_thread(void *arg)
{
    BGNET_CTRL *pCtrl = (BGNET_CTRL *)arg;
    int iValid;

    struct ifreq ifr;

    // to resolve the phenomenon that the nmea2000 thread was 'hanging' and the application would not
    // terminate with <Ctrl><C> i have made the socket non-blocking
    // KS 2026-07-29
    int s = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    //int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        log_error("CAN socket: %s", strerror(errno));
        return NULL;
    }

    memset(&ifr, 0, sizeof(ifr));
    snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", &pCtrl->pCfg->can_interface[0]);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        log_error("CAN ioctl: %s", strerror(errno));
        close(s);
        return NULL;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        log_error("CAN bind: %s", strerror(errno));
        close(s);
        return NULL;
    }

    log_info("CAN bus: listening on %s for NMEA2000 messages", &pCtrl->pCfg->can_interface[0]);

    struct can_frame frame;

    while (g_running) {
        int n = read(s, &frame, sizeof(frame));
        if (n < 0) {
            switch (errno) {
                case EINTR:
                case EAGAIN:    // no data, non-blocking
                    continue;
                    break;
                default:
                    log_error("CAN read: %s (%d)", strerror(errno), errno);
                    break;
            }
        } else {
            if (n > 0) {
                if (n > (int)sizeof(struct can_frame)) {
                    log_error("CAN bus: Receive buffer overflow, received %d bytes", n);
                } else {
                    // this must be safe to handle
                    /* Extract PGN from 29-bit ID: bits 8..25 */
                    uint32_t pgn = (frame.can_id >> 8) & 0x1FFFF;

                    iValid = 1;
                    switch (pgn) {  // ADD: SEATEMP, HEADING
                        case 126992: parse_pgn_126992(pCtrl, &frame); break;   // system time
                        case 126993: parse_pgn_126993(); break;   // heartbeat, no data
                        case 127245: parse_pgn_127245(pCtrl, &frame); break;   // rudder position
                        case 127250: parse_pgn_127250(pCtrl, &frame); break;   // Heading and Deviation (Magnetic Heading, True Heading)
                        case 128259: parse_pgn_128259(pCtrl, &frame); break;   // STW (Speed Through Water)
                        case 128267: parse_pgn_128267(pCtrl, &frame); break;   // Depth
                        case 129025: parse_pgn_129025(pCtrl, &frame); break;   // GPS/LAT, LON (can not be handled by NETWORK devices)
                        case 129026: parse_pgn_129026(pCtrl, &frame); break;   // GPS/COG, SOG
                        case 129029: parse_pgn_129029(pCtrl, &frame); break;   // GNSS position data
                        case 130306: parse_pgn_130306(pCtrl, &frame); break;   // AWS, AWA, TWS, TWA
                        default:
                            iValid = 0;
                        break;
                    }
                    if (iValid > 0) {
                        //log_debug("NMEA2000: PGN %d received and handled", pgn);
                    } else {
                        log_debug("NMEA2000: unhandled PGN %d received", pgn);
                    }
                }
            } else {
                // no data received from device
            }
        }
        usleep(MSEC1);
    }
    close(s);
    log_info("NMEA2000: thread exiting");
    return NULL;
}
