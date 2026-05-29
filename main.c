#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>

#include "config.h"
#include "logger.h"
#include "app_state.h"
#include "serial.h"
#include "bgnprotocol.h"
#include "ethernet.h"
#include "timer.h"
#include "webserver.h"
#include "nmea2000.h"

// signal handling to make a soft landing
//  - ps ax | grep {processname} > gives you the process name and processID
//  - kill -s SIGINT {processID} > sends <Ctrl><C> to the service
// or press <Ctrl><C> if running from the commandline
void handle_sigint(int sig) {
    if (sig == SIGINT) {
        g_running = 0;
    }
}

// please consider this as 'constants'; do not write into this structure.
// Use is as some kind of bookkeeping table. We only 'enable' the devices we have tested.
BGNET_DEV BGNET_dev[BGNET_DEVICES] = { \
// ProdName     Model       ModelNr       SerialNr       Version Enable DeviceType             
{"NET DEPTH",   "NET-DPDU", "610-00-021", "NDEP9410109", "0000", 0,     BGNET_FRAME_FROM_UNKNOWN},\
{"NET SPEED",   "NET-SPDU", "610-00-020", "NSPDG485015", "0000", 0,     BGNET_FRAME_FROM_UNKNOWN},\
{"NET WIND",    "NET-WDDU", "611-00-007", "NWINB06505",  "0000", 0,     BGNET_FRAME_FROM_NETWIND},\
{"NET GPSplus", "NET-GPDU", "615-00-021", "",            "0000", 0,     BGNET_FRAME_FROM_UNKNOWN},\
{"NET LORAN",   "NET-LORAN","",           "",            "0000", 0,     BGNET_FRAME_FROM_UNKNOWN},\
{"NET CHART",   "NET-CHART","",           "CHT350327",   "0000", 0,     BGNET_FRAME_FROM_UNKNOWN},\
{"NET COMPASS", "NET-CPDU", "545-00-031", "",            "0000", 0,     BGNET_FRAME_FROM_UNKNOWN},\
{"NET TACK",    "NET-TADU", "610-00-086", "",            "0000", 0,     BGNET_FRAME_FROM_UNKNOWN},\
{"NET PILOT",   "NET-PLDU", "610-00-010", "NPLDB06217",  "0000", 1,     BGNET_FRAME_FROM_NETPILOT},\
{"NET QUAD",    "NET-QDDU", "610-00-023", "NQDUB08432",  "0000", 0,     BGNET_FRAME_FROM_NETQUAD},\
{"NET DATA",    "NET-DTDU", "610-00-024", "NDAT9417101", "0000", 0,     BGNET_FRAME_FROM_NETDATA},\
{"NET NAV",     "NET-NADU", "610-00-086", "",            "0000", 0,     BGNET_FRAME_FROM_UNKNOWN},\
{"NET ENGINE",  "NET-ENGI", "610-00-132", "",            "0000", 0,     BGNET_FRAME_FROM_UNKNOWN},\
{"NET QUAD-R",  "NET-QDRU", "610-00-022", "",            "0000", 0,     BGNET_FRAME_FROM_UNKNOWN},\
{"NET VHF",     "NET-VHF",  "613-10-003", "",            "0000", 0,     BGNET_FRAME_FROM_UNKNOWN},\
{"NET ACP1",    "NET-ACP1", "360-00-002", "NP1CB15613",  "0000", 0,     BGNET_FRAME_FROM_UNKNOWN},\
{"NET ACP2",    "NET-ACP2", "545-00-035", "HACP29617101","0000", 0,     BGNET_FRAME_FROM_UNKNOWN}};

int main(void)
{
    // assign signal handler
    signal(SIGINT, handle_sigint);

    // first we need the configuration, otherwise we cannot log errors
    Config cfg;
    if (load_config("config.json", &cfg) != 0) {
        // this is the first and only printf we use in this application; all other
        // console-output is send to the logfile as specified in the configuration
        fprintf(stderr, "Failed to load config.json\n");
        return 1;
    }
    logger_init(cfg.log_file, cfg.log_level);
    log_info("Starting %s", ApplicationName);

    //allocate space for shared data
    BGNET_CTRL* pBGNET_ctrl;
    pBGNET_ctrl = malloc(sizeof(BGNET_CTRL));
    if (pBGNET_ctrl == NULL) {
        log_error("Unable to allocale memory for pBGNET_ctrl");
        exit(1);
    }
    BGNET_DEV* pBGNET_dev;
    pBGNET_dev = malloc(sizeof(BGNET_DEV)*BGNET_DEVICES);
    if (pBGNET_dev == NULL) {
        log_error("Unable to allocale memory for pBGNET_dev");
        exit(1);
    }
    // setup pointers to the structures
    //pBGNET_ctrl->pDev = pBGNET_dev; // pointer to BGNET devices
    pBGNET_ctrl->pDev = BGNET_dev; // pointer to BGNET devices
    pBGNET_ctrl->pCfg = &cfg; // pointer to Config structure

#ifdef EXCLUDE_THIS
    // shared data, declaration
    BGNET_CTRL BGNET_ctrl;
#endif

    app_state_init();

    // activate all out threads
    pthread_t th_serial, th_eth, th_timer, th_web, th_n2k;
    if (
           (pthread_create(&th_serial, NULL, serial_thread, pBGNET_ctrl) != 0 ||
            pthread_create(&th_eth,    NULL, ethernet_thread, pBGNET_ctrl) != 0 ||
            pthread_create(&th_timer,  NULL, timer_thread, pBGNET_ctrl) != 0 ||
            pthread_create(&th_web,    NULL, webserver_thread, pBGNET_ctrl) != 0 ||
            pthread_create(&th_n2k,    NULL, nmea2000_thread, pBGNET_ctrl) != 0)) {
        // not all threads could be started, close the ones that did start and shutdown with error
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

    // free allocated memory
    free(pBGNET_ctrl);
    free(pBGNET_dev);
    
    // child threads are all closed, make a soft landing, bye, bye
    log_info("Exiting %s, bye-bye......", ApplicationName);
    logger_close();
    app_state_shutdown();
    return 0;
}
