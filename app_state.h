#ifndef APP_STATE_H
#define APP_STATE_H

#include <pthread.h>
#include <stdint.h>

#define ApplicationName "Marine Gateway"
// the lower the usleep, the higher the CPU load. 1000 (1 msec) works Ok, 5000 (5 msec) also works OK,
// but 10000 (10 msec) lo longer works. Use this to tune behaviour vs resources.
// for the time being, i'll leave it to 1000 (1 msec) 
#define MSEC1           2500 // usleep 2.5msec

// generic defines
//#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define DTOIROUND(d)                    ((int)(d))
#define ITOD(i)                         ((double)i)
#define BGNET_PI                        3.14159
#define BGNET_MIN(a, b) (((a) < (b)) ? (a) : (b))
#define BGNET_MAX(a, b) (((a) > (b)) ? (a) : (b))
#define BGNET_HIBYTE(w) (((uint16_t)(w) >> 8) & 0xFF)
#define BGNET_LOBYTE(w) ((uint8_t)((w) & 0xFF))
#define BGNET_BYTES2WORD(a, b) ((uint8_t)(a)*0xFF+(uint8_t)(b))
//#define BGNET_REMAINDER(a, b) ((a) % (b))
// WRAP and increment around a range. a= value, b= increment, c= range 
#define BGNET_INCREMENTWRAP(a, b, c, d) ( (a) + (b) + ((d)-(c)) ) % ( (d)-(c) )
#define BGNET_TIME_FMT                 "%Y-%m-%d %H:%M:%S" 



// an incoming stream is captured every 1 msec. This buffer in only to
// hold what 11008 baud can produce (= ca 1100 bytes/sec). 256 bytes should be OK
#define BGNET_RXTXRAW_SIZE            1024    // max size readbuffer 

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

/* Serial Protocol timing */
#define SerialCounter_Reset     0
#define SerialCounter_Set       10
#define SerialCounter_Bottom    -999
extern int  g_fastnet_data_lenRx;      // number of bytes in serial buffer
extern int  g_fastnet_data_lenTx;      // number of bytes in serial buffer
extern int  g_fastnet_data_counter;    // receive counter
#define BGN_FASTNET_MODE_RO      0
#define BGN_FASTNET_MODE_RW      1
#define BGN_FASTNET_MODE_PASSIVE 2
extern int  g_fastnet_mode;

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
    int iCounter;
    time_t tTimestamp;
    time_t tCounterReset;
} BGNET_COUNT;

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
    NMEA2K_VALUE headingM;
    NMEA2K_VALUE headingT;
    NMEA2K_VALUE time;
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
