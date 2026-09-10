#include "app_state.h"
#include <string.h>

volatile int g_running = 1;

pthread_mutex_t g_data_mutex;

// our strategy about putting data in global space and locking is as follows:
// - when data is accessed (read or write) from multiple threads, it's better to place it in global common
// - when data is written from 2 threads (or more), thread locking/unlocking shall be used
// - when doing locking, only do memory transfer. No I/O, no suspended operations.
// - if doing read access to global common and you must guarentee a consistent dataset, do a memcpy of the
//   data you need.

unsigned char g_last_serial_dataRx[BGNET_RXTXRAW_SIZE];
unsigned char g_last_serial_dataTx[BGNET_RXTXRAW_SIZE];
int  g_serial_data_lenRx = 0;
int  g_serial_data_lenTx = 0;
int  g_serial_data_counter = SerialCounter_Reset;
int  g_serial_mode = BGN_SERIAL_MODE_RO;

char g_last_ethernet_data[BGNET_RXTXRAW_SIZE];
char g_last_signalk_delta[BGNET_RXTXRAW_SIZE];

NMEA2K_DATA g_n2kdata;

void app_state_init(void)
{
    time_t t;
    time(&t);

    pthread_mutex_init(&g_data_mutex, NULL);
    memset(g_last_serial_dataRx, 0, sizeof(g_last_serial_dataRx));
    memset(g_last_serial_dataTx, 0, sizeof(g_last_serial_dataRx));
    memset(g_last_ethernet_data, 0, sizeof(g_last_ethernet_data));
    memset(g_last_signalk_delta, 0, sizeof(g_last_signalk_delta));
    g_serial_data_lenRx = 0;
    g_serial_data_lenTx = 0;
    g_serial_data_counter = SerialCounter_Reset;

    // erase datablock
    memset(&g_n2kdata, 0, sizeof(g_n2kdata));

    NMEAinit(&g_n2kdata.pos.lat, t);
    NMEAinit(&g_n2kdata.pos.lon, t);
    NMEAinit(&g_n2kdata.pos.alt, t);
    NMEAinit(&g_n2kdata.pos.cog, t);
    NMEAinit(&g_n2kdata.pos.sog, t);

    NMEAinit(&g_n2kdata.boat.stw, t);
    NMEAinit(&g_n2kdata.boat.depth, t);
    NMEAinit(&g_n2kdata.boat.seatemp, t);
    NMEAinit(&g_n2kdata.boat.rudder, t);
    NMEAinit(&g_n2kdata.boat.headingM, t);
    NMEAinit(&g_n2kdata.boat.headingT, t);

    NMEAinit(&g_n2kdata.wind.twa, t);
    NMEAinit(&g_n2kdata.wind.tws, t);
    NMEAinit(&g_n2kdata.wind.awa, t);
    NMEAinit(&g_n2kdata.wind.aws, t);
    NMEAinit(&g_n2kdata.wind.airtemp, t);
    NMEAinit(&g_n2kdata.wind.baro, t);
    NMEAinit(&g_n2kdata.wind.vmg, t);

    NMEAinit(&g_n2kdata.pilot.cts, t);
    NMEAinit(&g_n2kdata.pilot.triplog, t);
}

void app_state_shutdown(void)
{
    pthread_mutex_destroy(&g_data_mutex);
}
