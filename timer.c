#include "timer.h"
#include "app_state.h"
#include "logger.h"
#include <unistd.h>
#include <stdio.h>

void *timer_thread(void *arg)
{
    Config *cfg = (Config *)arg;

    while (g_running) {
        usleep(cfg->timer_interval_ms * 1000);

        int has_data = 0;
        char buf[256];

        pthread_mutex_lock(&g_data_mutex);
        if (g_serial_data_available) {
            has_data = 1;
            g_serial_data_available = 0;
            snprintf(buf, sizeof(buf), "%s", g_last_serial_data);
        }
        pthread_mutex_unlock(&g_data_mutex);

        if (has_data) {
            log_debug("Timer %d ms: serial data -> %s",
                      cfg->timer_interval_ms, buf);
        }
    }

    log_info("Timer thread exiting");
    return NULL;
}
