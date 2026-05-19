#include "timer.h"
#include "app_state.h"
#include "logger.h"
#include <unistd.h>
#include <stdio.h>
#include <string.h>

void *timer_thread(void *arg)
{
    Config *cfg = (Config *)arg;
    unsigned char readbuf[1024];
    char asciibuf[2048];
    int readbuflen = 0;
    int counter = 0;

    while (g_running) {
        pthread_mutex_lock(&g_data_mutex);
        if (g_serial_data_len>0) {
            // data is present, wait for frame completion
            if (g_serial_data_counter>SerialCounter_Reset) {
                // wait for more, decrement counter
                g_serial_data_counter--;
            } else {
                // timeout, end of frame, get data into local buffer
                memcpy(readbuf,g_last_serial_data,g_serial_data_len);
                readbuflen = g_serial_data_len;
                g_serial_data_len = 0; // reset receiving buffer
            }
        } else {
            // no data present, count down to the bottom (prevent underflow)
            if (g_serial_data_counter>SerialCounter_Bottom) {
                g_serial_data_counter--;
            }
            counter = g_serial_data_counter;
        }
        pthread_mutex_unlock(&g_data_mutex);

        // show what we got
        if (readbuflen>0) {
            byte2ascii(asciibuf, sizeof(asciibuf), readbuf, readbuflen, BYTE2ASCII_PLAIN);
            log_debug("Timer %d ms: serial data -> %s", counter, asciibuf);
        }
        readbuflen=0;

        // no racing allowed
        usleep(cfg->timer_interval_ms * MSEC1);
    }
    log_info("Timer thread exiting");
    return NULL;
}
