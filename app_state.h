#ifndef APP_STATE_H
#define APP_STATE_H

#include <pthread.h>
#include <stdint.h>

#define ApplicationName "Marine Gateway"
// the lower the usleep, the higher the CPU load. 1000 (1 msec) works Ok, 5000 (5 msec) also works OK,
// but 10000 (10 msec) lo longer works. Use this to tune behaviour vs resources.
// for the time being, i'll leave it to 1000 (1 msec) 
#define MSEC1           1000 // usleep 1msec

// generic defines
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

// an incoming stream is captured every 1 msec. This buffer in only to
// hold what 11008 baud can produce (= ca 1100 bytes/sec). 256 bytes should be OK
#define BGNET_RXTXRAW_SIZE            1024    // max size readbuffer 

// thread control
extern volatile int g_running;

extern pthread_mutex_t g_data_mutex;
extern unsigned char g_last_serial_dataRx[BGNET_RXTXRAW_SIZE];
extern unsigned char g_last_serial_dataTx[BGNET_RXTXRAW_SIZE];
extern char g_last_ethernet_data[BGNET_RXTXRAW_SIZE];
extern char g_last_signalk_delta[BGNET_RXTXRAW_SIZE];

/* Serial Protocol timing */
#define SerialCounter_Reset     0
#define SerialCounter_Set       10
#define SerialCounter_Bottom    -999
extern int  g_serial_data_lenRx;      // number of bytes in serial buffer
extern int  g_serial_data_lenTx;      // number of bytes in serial buffer
extern int  g_serial_data_counter;    // receive counter
#define BGN_SERIAL_MODE_RO      0
#define BGN_SERIAL_MODE_RW      1
extern int  g_serial_mode;

// NMEA2K value definition. We keep a timestamp for every individual value as we can not predict
// that all parameters come from a sensor at the same time, from the same sensor, or how the sensor
// behaves when there is a malfunction. This allows us to apply 'quality' check on a per-parameter basis.
#define BGN_SOURCE_NONE     0
#define BGN_SOURCE_API      1
#define BGN_SOURCE_BGNET    2
#define BGN_SOURCE_NMEA     3
#define BGN_SOURCE_SIGNALK  4
// used when a derived/calculated value used mixed sources
#define BGN_SOURCE_MIXED    5
#define BGN_SOURCE_ACP      6

union VALUE {
    double dValue;    // used for floatingpoint values
    uint8_t nByte;    // state/byte
    uint64_t nLong;   // large integer value
    char cString[12]; // character string
};
// most values we store are 'real' (floating point) values, but sometimes 
// we need other datatype.
typedef struct {
    uint8_t datatype;
    time_t timestamp;
    size_t source;
    union VALUE value;
} NMEA2K_VALUE;
typedef struct {
    NMEA2K_VALUE lat;
    NMEA2K_VALUE lon;
    NMEA2K_VALUE alt;
    NMEA2K_VALUE cog;
    NMEA2K_VALUE sog;
} NMEA2K_POSITION;
typedef struct {
    NMEA2K_VALUE stw;
    NMEA2K_VALUE depth;
    NMEA2K_VALUE seatemp;
    NMEA2K_VALUE rudder;
    NMEA2K_VALUE heading;
} NMEA2K_BOAT;
typedef struct {
    NMEA2K_VALUE twa;
    NMEA2K_VALUE tws;
    NMEA2K_VALUE awa;
    NMEA2K_VALUE aws;
    NMEA2K_VALUE airtemp;
    NMEA2K_VALUE baro;
} NMEA2K_WIND;
typedef struct {
    NMEA2K_VALUE cts;
    NMEA2K_VALUE triplog;
    NMEA2K_VALUE vmg;
} NMEA2K_PILOT;
typedef struct {
    NMEA2K_POSITION pos;
    NMEA2K_BOAT boat;
    NMEA2K_WIND wind;
    NMEA2K_PILOT pilot;
} NMEA2K_DATA;

extern NMEA2K_DATA g_n2kdata;

void app_state_init(void);
void app_state_shutdown(void);

#endif
