#include "app_state.h"
#include <string.h>

volatile int g_running = 1;

pthread_mutex_t g_data_mutex;
char g_last_serial_dataRx[1024];
char g_last_serial_dataTx[1024];
char g_last_ethernet_data[1024];
char g_last_signalk_delta[1024];
int  g_serial_data_lenRx = 0;
int  g_serial_data_lenTx = 0;
int  g_serial_data_counter = SerialCounter_Reset;
int  g_serial_mode = BGN_SERIAL_MODE_RO;

double g_lat = 0.0;
double g_lon = 0.0;
double g_cog = 0.0;
double g_stw = 0.0;
double g_depth = 0.0;
double g_twa = 0.0;
double g_tws = 0.0;
double g_awa = 0.0;
double g_aws = 0.0;
double g_rudder = 0.0;

void app_state_init(void)
{
    pthread_mutex_init(&g_data_mutex, NULL);
    memset(g_last_serial_dataRx, 0, sizeof(g_last_serial_dataRx));
    memset(g_last_serial_dataTx, 0, sizeof(g_last_serial_dataRx));
    memset(g_last_ethernet_data, 0, sizeof(g_last_ethernet_data));
    memset(g_last_signalk_delta, 0, sizeof(g_last_signalk_delta));
    g_serial_data_lenRx = 0;
    g_serial_data_lenTx = 0;
    g_serial_data_counter = SerialCounter_Reset;
}

void app_state_shutdown(void)
{
    pthread_mutex_destroy(&g_data_mutex);
}
