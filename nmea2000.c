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

/* 129025 – Position, Rapid Update (only included lat/lon, no other parameters)
 * Lat/Lon in 1e-7 degrees, signed 32-bit
 */
static void parse_pgn_129025(const struct can_frame *f)
{
    time_t t;
    int32_t lat_raw = (int32_t)n2k_u32(f->data[0], f->data[1], f->data[2], f->data[3]);
    int32_t lon_raw = (int32_t)n2k_u32(f->data[4], f->data[5], f->data[6], f->data[7]);

    if (lat_raw == 0x7FFFFFFF || lon_raw == 0x7FFFFFFF)
        return; /* not available */

    time(&t);

    double lat = lat_raw * 1e-7;
    double lon = lon_raw * 1e-7;

    pthread_mutex_lock(&g_data_mutex);
    g_n2kdata.pos.lat.value.dValue = lat;
    g_n2kdata.pos.lat.timestamp = t;
    g_n2kdata.pos.lat.source = BGN_SOURCE_NMEA;
    g_n2kdata.pos.lon.value.dValue = lon;
    g_n2kdata.pos.lon.timestamp = t;
    g_n2kdata.pos.lon.source = BGN_SOURCE_NMEA;
    pthread_mutex_unlock(&g_data_mutex);

    log_debug("N2K 129025: lat=%.7f lon=%.7f", lat, lon);
}

/* 129026 – COG & SOG, Rapid Update
 * COG: 0.0001 rad, SOG: 0.01 m/s
 */
static void parse_pgn_129026(const struct can_frame *f)
{
    time_t t;
    uint16_t cog_raw = n2k_u16(f->data[2], f->data[3]);
    uint16_t sog_raw = n2k_u16(f->data[4], f->data[5]);

    time(&t);

    if (cog_raw != 0xFFFF) {
        double cog = cog_raw * 1e-4; /* radians */
        pthread_mutex_lock(&g_data_mutex);
        g_n2kdata.pos.cog.value.dValue = cog;
        g_n2kdata.pos.cog.timestamp = t;
        g_n2kdata.pos.cog.source = BGN_SOURCE_NMEA;
        pthread_mutex_unlock(&g_data_mutex);
        log_debug("N2K 129026: COG=%.4f rad", cog);
    }

    if (sog_raw != 0xFFFF) {
        double sog = sog_raw * 0.01; /* m/s */
        pthread_mutex_lock(&g_data_mutex);
        g_n2kdata.pos.sog.value.dValue = sog;
        g_n2kdata.pos.sog.timestamp = t;
        g_n2kdata.pos.sog.source = BGN_SOURCE_NMEA;
        pthread_mutex_unlock(&g_data_mutex);
        log_debug("N2K 129026: SOG=%.2f m/s", sog);
    }
}

/* 128259 – Speed, Water Referenced
 * Water speed: 0.01 m/s
 */
static void parse_pgn_128259(const struct can_frame *f)
{
    time_t t;
    uint16_t speed_raw = n2k_u16(f->data[2], f->data[3]);
    if (speed_raw == 0xFFFF)
        return;

    time(&t);

    double stw = speed_raw * 0.01; /* m/s */

    pthread_mutex_lock(&g_data_mutex);
    g_n2kdata.boat.stw.value.dValue = stw;
    g_n2kdata.boat.stw.timestamp = t;
    g_n2kdata.boat.stw.source = BGN_SOURCE_NMEA;
    pthread_mutex_unlock(&g_data_mutex);

    log_debug("N2K 128259: STW=%.2f m/s", stw);
}

/* 128267 – Water Depth
 * Depth: 0.01 m, Offset: 0.01 m
 * We publish depth below transducer (no offset correction here).
 */
static void parse_pgn_128267(const struct can_frame *f)
{
    time_t t;
    uint16_t depth_raw  = n2k_u16(f->data[0], f->data[1]);
    uint16_t offset_raw = n2k_u16(f->data[2], f->data[3]);

    time(&t);

    if (depth_raw == 0xFFFF)
        return;

    double depth  = depth_raw * 0.01;  /* meters */
    double offset = (offset_raw == 0xFFFF) ? 0.0 : (int16_t)offset_raw * 0.01;

    /* For now, just store raw depth below transducer; you can apply offset if desired */
    (void)offset;

    pthread_mutex_lock(&g_data_mutex);
    g_n2kdata.boat.depth.value.dValue = depth;
    g_n2kdata.boat.depth.timestamp = t;
    g_n2kdata.boat.depth.source = BGN_SOURCE_NMEA;
    pthread_mutex_unlock(&g_data_mutex);

    log_debug("N2K 128267: depth=%.2f m offset=%.2f m", depth, offset);
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
static void parse_pgn_130306(const struct can_frame *f)
{
    time_t t;
    uint8_t ref      = f->data[0] & 0x07;
    uint16_t speed_r = n2k_u16(f->data[1], f->data[2]);
    uint16_t angle_r = n2k_u16(f->data[3], f->data[4]);

    if (speed_r == 0xFFFF || angle_r == 0xFFFF)
        return;

    time(&t);

    double speed = speed_r * 0.01;   /* m/s */
    double angle = angle_r * 1e-4;   /* rad */

    pthread_mutex_lock(&g_data_mutex);
    switch (ref) {
        case 2: /* Apparent */
            g_n2kdata.wind.aws.value.dValue = speed;
            g_n2kdata.wind.aws.timestamp = t;
            g_n2kdata.wind.aws.source = BGN_SOURCE_NMEA;
            g_n2kdata.wind.awa.value.dValue = angle;
            g_n2kdata.wind.awa.timestamp = t;
            g_n2kdata.wind.awa.source = BGN_SOURCE_NMEA;
            break;
        case 0: /* True (ground) */
        case 3: /* True (water) */
            g_n2kdata.wind.tws.value.dValue = speed;
            g_n2kdata.wind.tws.timestamp = t;
            g_n2kdata.wind.tws.source = BGN_SOURCE_NMEA;
            g_n2kdata.wind.twa.value.dValue = angle;
            g_n2kdata.wind.twa.timestamp = t;
            g_n2kdata.wind.twa.source = BGN_SOURCE_NMEA;
            break;
        default:
            break;
    }
    pthread_mutex_unlock(&g_data_mutex);

    log_debug("N2K 130306: ref=%u speed=%.2f m/s angle=%.4f rad", ref, speed, angle);
}

/* 127245 – Rudder Angle
 * Angle: -PI (-180) to +Pi (+180) degrees, range 0-30000
 * mechinical limits are restricted to +/-45 to +/-90 degrees
 */
static void parse_pgn_127245(const struct can_frame *f)
{
    time_t t;
    uint8_t  rudder_instance = f->data[0]; // identify rudder, when multiple rudders or sonsors
    uint8_t  rudder_orderdir = f->data[1]; // order direction (0= no order, 1=turn port, 2= turn starboard)
    // byte 2 not used
    int16_t rudder_orderpos = n2k_i16(f->data[4], f->data[3]); // angle order 0.0001 rad/bit or 0xFFFF (no order)
    int16_t rudder_raw      = n2k_i16(f->data[6], f->data[5]); // position, 0.0001 rad/bit, += starboard, -=port

    //if (rudder_raw == 0xFFFF)
    //    return;

    time(&t);

    double rudder_rad  = rudder_raw * 1e-4;  /* devide by 1000 to get rads */

    /* For now, just store position, destroy others */
    (void) rudder_instance;
    (void) rudder_orderdir;
    (void) rudder_orderpos;

    pthread_mutex_lock(&g_data_mutex);
    g_n2kdata.boat.rudder.value.dValue = rudder_rad;
    g_n2kdata.boat.rudder.timestamp = t;
    g_n2kdata.boat.rudder.source = BGN_SOURCE_NMEA;
    pthread_mutex_unlock(&g_data_mutex);

    log_debug("N2K 127245: rudder angle=%.4f [rad] (or %d degrees) %X", rudder_rad, (int)(rudder_rad*180/M_PI), rudder_raw);
}

void *nmea2000_thread(void *arg)
{
    BGNET_CTRL *pCtrl= (BGNET_CTRL *)arg;
    Config *cfg = pCtrl->pCfg;
    int iValid;

    struct ifreq ifr;

    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        log_error("CAN socket: %s", strerror(errno));
        return NULL;
    }

    memset(&ifr, 0, sizeof(ifr));
    snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", cfg->can_interface);
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

    log_info("NMEA2000 listening on %s", cfg->can_interface);

    struct can_frame frame;

    while (g_running) {
        int n = read(s, &frame, sizeof(frame));
        if (n < 0) {
            if (errno == EINTR) continue;
            log_error("CAN read: %s", strerror(errno));
            break;
        }
        if (n < (int)sizeof(struct can_frame))
            continue;

        /* Extract PGN from 29-bit ID: bits 8..25 */
        uint32_t pgn = (frame.can_id >> 8) & 0x1FFFF;

        iValid = 1;
        switch (pgn) {
            case 129025: parse_pgn_129025(&frame); break;
            case 129026: parse_pgn_129026(&frame); break;
            case 128259: parse_pgn_128259(&frame); break;
            case 128267: parse_pgn_128267(&frame); break;
            case 130306: parse_pgn_130306(&frame); break;
            case 127245: parse_pgn_127245(&frame); break;
            default:
                iValid = 0;
                break;
        }
        if (iValid > 0) {
            log_debug("NMEA2000: PGN %d received and handled", pgn);
        } else {
            log_debug("NMEA2000: unhandled PGN %d received", pgn);
        }
        usleep(MSEC1);
    }
    close(s);
    log_info("NMEA2000: thread exiting");
    return NULL;
}
