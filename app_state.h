#ifndef APP_STATE_H
#define APP_STATE_H

#include <pthread.h>

#define ApplicationName "Marine Gateway"
// the lower the usleep, the higher the CPU load. 1000 (1 msec) works Ok, 5000 (5 msec) also works OK,
// but 10000 (10 msec) lo longer works. Use this to tune behaviour vs resources.
// for the time being, i'll leave it to 1000 (1 msec) 
#define MSEC1           1000 // usleep 1msec

// generic defines
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

// thread control
extern volatile int g_running;

extern pthread_mutex_t g_data_mutex;
extern char g_last_serial_dataRx[1024];
extern char g_last_serial_dataTx[1024];
extern char g_last_ethernet_data[1024];
extern char g_last_signalk_delta[1024];

/* Serial Protocol timing */
#define SerialCounter_Reset     0
#define SerialCounter_Set       10
#define SerialCounter_Bottom    -999
extern int  g_serial_data_lenRx;      // number of bytes in serial buffer
extern int  g_serial_data_lenTx;      // number of bytes in serial buffer
extern int  g_serial_data_counter;  // receive counter
#define BGN_SERIAL_MODE_RO      0
#define BGN_SERIAL_MODE_RW      1
extern int  g_serial_mode;

/* NMEA2000-derived values */
extern double g_lat;
extern double g_lon;
extern double g_cog;
extern double g_stw;
extern double g_depth;
extern double g_twa;
extern double g_tws;
extern double g_awa;
extern double g_aws;
extern double g_rudder;

void app_state_init(void);
void app_state_shutdown(void);

#endif
