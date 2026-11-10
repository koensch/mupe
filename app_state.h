#ifndef APP_STATE_H
#define APP_STATE_H

#include <pthread.h>
#include <stdint.h>

#define ApplicationName "mupe"
// the lower the usleep, the higher the CPU load. 1000 (1 msec) works Ok, 5000 (5 msec) also works OK,
// but 10000 (10 msec) no longer works. Use this to balance behaviour/performance vs resources.
// for the time being, i'll leave it to 2500 (2.5 msec). Lowered it to 1500 because of collisions when ACP is present 
#define MSEC1           1500     // usleep 1.5 msec
// for the NMEA2000 pacemaker 
#define MSEC10          10000    // usleep 10 msec
// for the NMEA0183 pacemaker, i do'n want to send data too fast. 
#define MSEC2500        250000  // usleep 2500 msec
// for the main() 
#define SEC5            5       // sleep 5 sec

// generic defines
// convert double to int/long (not roundoff, just cutoff the fractional part)
#define BGNET_DTOIROUND(d)                  ((int)(d))
// convert int/long to double
#define BGNET_ITOD(i)                       ((double)i)
// if we need PI
#define BGNET_PI                            3.14159
// get the lowest of 2 values
#define BGNET_MIN(a, b)                     (((a) < (b)) ? (a) : (b))
// get the highest of 2 values
#define BGNET_MAX(a, b)                     (((a) > (b)) ? (a) : (b))
// get the high-byte from a int/long
#define BGNET_HIBYTE(w)                     (((uint16_t)(w) >> 8) & 0xFF)
// get the low-byte from a int/long
#define BGNET_LOBYTE(w)                     ((uint8_t)((w) & 0xFF))
// combine 2 bytes (high- and low byte) to a 16 bits register
#define BGNET_BYTES2WORD(a, b)              ((uint8_t)(a)*0x100+(uint8_t)(b))
// WRAP and increment around a range. a= value, b= increment, c= range 
#define BGNET_INCREMENTWRAP(a, b, c, d)     ( (a) + (b) + ((d)-(c)) ) % ( (d)-(c) )
// whereever we need a time-formatted string
#define BGNET_TIME_FMT                      "%Y-%m-%d %H:%M:%S"
// convert RAD to DEG
#define BGNET_RAD2DEG(r)                    (r * 180.0/M_PI)
// convert DEG to RAD
#define BGNET_DEG2RAD(r)                    ((d * M_PI)/180.0)

#define BGNET_RXTXRAW_SIZE              1024    // max size read/write buffer (ethernet, fastnet) to hold the complete frames

// ASCII characters
#define BGNET_ASC_NULL                  0x00
#define BGNET_ASC_SPACE                 0x20
#define BGNET_ASC_PERIOD                0x2E
#define BGNET_ASC_0                     0x30
#define BGNET_ASC_1                     0x31
#define BGNET_ASC_2                     0x32
#define BGNET_ASC_3                     0x33
#define BGNET_ASC_4                     0x34
#define BGNET_ASC_5                     0x35
#define BGNET_ASC_6                     0x36
#define BGNET_ASC_7                     0x37
#define BGNET_ASC_8                     0x38
#define BGNET_ASC_9                     0x39
#define BGNET_ASC_QUEST                 0x3F
#define BGNET_ASC_A                     0x41
#define BGNET_ASC_B                     0x42
#define BGNET_ASC_C                     0x43
#define BGNET_ASC_D                     0x44
#define BGNET_ASC_E                     0x45
#define BGNET_ASC_F                     0x46
#define BGNET_ASC_M                     0x4D
#define BGNET_ASC_N                     0x4E
#define BGNET_ASC_O                     0x4F
#define BGNET_ASC_P                     0x50
#define BGNET_ASC_W                     0x57
#define BGNET_ASC_X                     0x58
#define BGNET_ASC_a                     0x61
#define BGNET_ASC_b                     0x62
#define BGNET_ASC_c                     0x63
#define BGNET_ASC_d                     0x64
#define BGNET_ASC_e                     0x65
#define BGNET_ASC_f                     0x66
#define BGNET_ASC_x                     0x78

// thread control
extern volatile int g_running;

extern pthread_mutex_t g_data_mutex;
extern unsigned char g_last_fastnet_dataRx[BGNET_RXTXRAW_SIZE];
extern unsigned char g_last_fastnet_dataTx[BGNET_RXTXRAW_SIZE];
extern char g_last_ethernet_data[BGNET_RXTXRAW_SIZE];
extern char g_last_signalk_delta[BGNET_RXTXRAW_SIZE];
extern unsigned char g_last_nmea0183_dataRx1[BGNET_RXTXRAW_SIZE];
extern unsigned char g_last_nmea0183_dataTx1[BGNET_RXTXRAW_SIZE];
extern unsigned char g_last_nmea0183_dataRx2[BGNET_RXTXRAW_SIZE];
extern unsigned char g_last_nmea0183_dataTx2[BGNET_RXTXRAW_SIZE];

/* Serial Protocol timing */
#define SerialCounter_Reset     0
#define SerialCounter_Set       10
#define SerialCounter_Bottom    -999
extern int  g_fastnet_data_lenRx;      // number of bytes in serial buffer fastnet
extern int  g_fastnet_data_lenTx;      // number of bytes in serial buffer fastnet
extern int  g_fastnet_data_counter;    // receive counter
#define BGN_FASTNET_MODE_RO      0
#define BGN_FASTNET_MODE_RW      1
#define BGN_FASTNET_MODE_PASSIVE 2
extern int  g_fastnet_mode;

extern int  g_nmea0183_data_lenRx1;      // number of bytes in serial buffer nmea0183
extern int  g_nmea0183_data_lenTx1;      // number of bytes in serial buffer nmea0183
extern int  g_nmea0183_data_lenRx2;      // number of bytes in serial buffer nmea0183
extern int  g_nmea0183_data_lenTx2;      // number of bytes in serial buffer nmea0183

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
#define BGN_SOURCE_BLANK    7

#define BGNET_SAFECOPY_MAX  12
union VALUE {
    double dValue;      // used for floating point values
    uint8_t nByte;      // state/byte (unsigned)
    int64_t nLong;      // long integer value (signed)
    char cString[BGNET_SAFECOPY_MAX];   // character string (use bgn_safecopy to copy)
    time_t tTime;       // time format
};

typedef struct {
    size_t iCounter;
    time_t tTimestamp;
    time_t tCounterReset;
} BGNET_COUNT;

typedef struct {
    time_t tPreset;
    time_t tTimestamp;
} BGNET_TIMER;

// some values are stored as bytes, double, long, string of time, please be consistent
typedef struct {
    BGNET_COUNT Counter;
    size_t source;
    union VALUE value;
} NMEA2K_VALUE;

typedef struct {
    NMEA2K_VALUE log_level;
} NMEA2K_CFG;
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
    NMEA2K_VALUE compass;
    NMEA2K_VALUE time;  // absolute time
    NMEA2K_VALUE timer; // countdown/countup timer
} NMEA2K_BOAT;
typedef struct {
    NMEA2K_VALUE twa;
    NMEA2K_VALUE tws;
    NMEA2K_VALUE awa;
    NMEA2K_VALUE aws;
    NMEA2K_VALUE airtemp;
    NMEA2K_VALUE baro;
    NMEA2K_VALUE vmg;
} NMEA2K_WIND;
typedef struct {
    NMEA2K_VALUE cts;
    NMEA2K_VALUE triplog;
} NMEA2K_PILOT;
typedef struct {
    NMEA2K_CFG cfg;
    NMEA2K_POSITION pos;
    NMEA2K_BOAT boat;
    NMEA2K_WIND wind;
    NMEA2K_PILOT pilot;
} NMEA2K_DATA;

extern NMEA2K_DATA g_n2kdata;

void app_state_init(void);
void app_state_shutdown(void);
void CounterInit(BGNET_COUNT*, time_t);
void CounterUp(BGNET_COUNT*, time_t);
void NMEAinit(NMEA2K_VALUE*, time_t);

#endif
