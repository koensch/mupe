#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>

#include "config.h"
#include "logger.h"
#include "app_state.h"
#include "serial.h"
#include "ethernet.h"
#include "timer.h"
#include "webserver.h"
#include "nmea2000.h"

// signal handling to make a soft landing
//  - ps ax | grep {processname} > gives you the process name and processID
//  - kill -s SIGINT {processID} > sends <Ctrl><C> to the service
void handle_sigint(int sig) {
    if (sig == SIGINT) { // received <Ctrl><C>
        g_running = 0;
    }
}

int main(void)
{
    // assign signal handler
    signal(SIGINT, handle_sigint);

    Config cfg;
    if (load_config("config.json", &cfg) != 0) {
        fprintf(stderr, "Failed to load config.json\n");
        return 1;
    }

    logger_init(cfg.log_file, cfg.log_level);
    log_info("Starting %s", ApplicationName);

    app_state_init();

    // activate all out threads
    pthread_t th_serial, th_eth, th_timer, th_web, th_n2k;
    if (
            (pthread_create(&th_serial, NULL, serial_thread, &cfg) != 0 ||
            pthread_create(&th_eth,    NULL, ethernet_thread, &cfg) != 0 ||
            pthread_create(&th_timer,  NULL, timer_thread, &cfg) != 0 ||
            pthread_create(&th_web,    NULL, webserver_thread, &cfg) != 0 ||
            pthread_create(&th_n2k,    NULL, nmea2000_thread, &cfg) != 0)) {
        log_error("Failed to create threads");
        g_running = 0;
        app_state_shutdown();
        logger_close();
        return 1;
    }

    // Run forever. If needed, we can do some bookkeeping now and then. Loop exit via signal handler
    while(g_running) {
        sleep(10);
    }

    // deactivate threads
    pthread_join(th_serial, NULL);
    pthread_join(th_eth, NULL);
    pthread_join(th_timer, NULL);
    pthread_join(th_web, NULL);
    pthread_join(th_n2k, NULL);

    // close main process. bye, bye
    log_info("Exiting %s, bye-bye......", ApplicationName);
    logger_close();
    app_state_shutdown();
    return 0;
}
