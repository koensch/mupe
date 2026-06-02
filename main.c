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
// if we do a lookup and there is no match, we return 0 and pick the first one
BGNET_DEV BGNET_dev[BGNET_DEVICE_MAX] = { \
// ProdName     Model       ModelNr       SerialNr       Version Enable DeviceType             
{"????",        "????",     "????",       "????",        "????", 0,     BGNET_FRAME_FROM_UNKNOWN},\
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

// this lookup table allows us to obtain the channel label
// (for display purpose) and the reference to a NMEA 2000 value
// to allow us to link NMEA2K values and BGNET Channels
BGNET_CHL BGNET_chl[BGN_CHANNEL_MAX+1] = { \
    {"UNKNOWN 00", 0},\
    {"UNKNOWN 01", 0},\
    {"UNKNOWN 02", 0},\
    {"UNKNOWN 03", 0},\
    {"UNKNOWN 04", 0},\
    {"UNKNOWN 05", 0},\
    {"UNKNOWN 06", 0},\
    {"UNKNOWN 07", 0},\
    {"UNKNOWN 08", 0},\
    {"UNKNOWN 09", 0},\
    {"UNKNOWN 0A", 0},\
    {"RUDDERANGLE %", 0},\
    {"UNKNOWN 0C", 0},\
    {"UNKNOWN 0D", 0},\
    {"UNKNOWN 0E", 0},\
    {"UNKNOWN 0F", 0},\
    {"UNKNOWN 10", 0},\
    {"UNKNOWN 11", 0},\
    {"UNKNOWN 12", 0},\
    {"UNKNOWN 13", 0},\
    {"UNKNOWN 14", 0},\
    {"UNKNOWN 15", 0},\
    {"UNKNOWN 16", 0},\
    {"UNKNOWN 17", 0},\
    {"UNKNOWN 18", 0},\
    {"UNKNOWN 19", 0},\
    {"UNKNOWN 1A", 0},\
    {"UNKNOWN 1B", 0},\
    {"UNKNOWN 1C", 0},\
    {"UNKNOWN 1D", 0},\
    {"UNKNOWN 1E", 0},\
    {"UNKNOWN 1F", 0},\
    {"UNKNOWN 20", 0},\
    {"UNKNOWN 21", 0},\
    {"UNKNOWN 22", 0},\
    {"UNKNOWN 23", 0},\
    {"UNKNOWN 24", 0},\
    {"UNKNOWN 25", 0},\
    {"UNKNOWN 26", 0},\
    {"UNKNOWN 27", 0},\
    {"UNKNOWN 28", 0},\
    {"UNKNOWN 29", 0},\
    {"UNKNOWN 2A", 0},\
    {"UNKNOWN 2B", 0},\
    {"UNKNOWN 2C", 0},\
    {"UNKNOWN 2D", 0},\
    {"UNKNOWN 2E", 0},\
    {"UNKNOWN 2F", 0},\
    {"UNKNOWN 30", 0},\
    {"UNKNOWN 31", 0},\
    {"UNKNOWN 32", 0},\
    {"UNKNOWN 33", 0},\
    {"UNKNOWN 34", 0},\
    {"UNKNOWN 35", 0},\
    {"UNKNOWN 36", 0},\
    {"UNKNOWN 37", 0},\
    {"UNKNOWN 38", 0},\
    {"UNKNOWN 39", 0},\
    {"UNKNOWN 3A", 0},\
    {"UNKNOWN 3B", 0},\
    {"UNKNOWN 3C", 0},\
    {"UNKNOWN 3D", 0},\
    {"UNKNOWN 3E", 0},\
    {"UNKNOWN 3F", 0},\
    {"UNKNOWN 40", 0},\
    {"BOATSPEED KT", 0},\
    {"UNKNOWN 42", 0},\
    {"UNKNOWN 43", 0},\
    {"UNKNOWN 44", 0},\
    {"UNKNOWN 45", 0},\
    {"UNKNOWN 46", 0},\
    {"UNKNOWN 47", 0},\
    {"UNKNOWN 48", 0},\
    {"HEADING", 0},\
    {"UNKNOWN 4A", 0},\
    {"UNKNOWN 4B", 0},\
    {"UNKNOWN 4C", 0},\
    {"UNKNOWN 4D", 0},\
    {"AWS RAW", 0},\
    {"UNKNOWN 4F", 0},\
    {"AWS KT", 0},\
    {"AWINDANGLE", 0},\
    {"AWA RAW", 0},\
    {"TARGET TWA", 0},\
    {"UNKNOWN 54", 0},\
    {"UNKNOWN 55", 0},\
    {"UNKNOWN 56", 0},\
    {"UNKNOWN 57", 0},\
    {"UNKNOWN 58", 0},\
    {"UNKNOWN 59", 0},\
    {"UNKNOWN 5A", 0},\
    {"UNKNOWN 5B", 0},\
    {"UNKNOWN 5C", 0},\
    {"UNKNOWN 5D", 0},\
    {"UNKNOWN 5E", 0},\
    {"UNKNOWN 5F", 0},\
    {"UNKNOWN 60", 0},\
    {"UNKNOWN 61", 0},\
    {"UNKNOWN 62", 0},\
    {"UNKNOWN 63", 0},\
    {"UNKNOWN 64", 0},\
    {"UNKNOWN 65", 0},\
    {"UNKNOWN 66", 0},\
    {"UNKNOWN 67", 0},\
    {"UNKNOWN 68", 0},\
    {"UNKNOWN 69", 0},\
    {"UNKNOWN 6A", 0},\
    {"UNKNOWN 6B", 0},\
    {"UNKNOWN 6C", 0},\
    {"UNKNOWN 6D", 0},\
    {"UNKNOWN 6E", 0},\
    {"UNKNOWN 6F", 0},\
    {"UNKNOWN 70", 0},\
    {"UNKNOWN 71", 0},\
    {"UNKNOWN 72", 0},\
    {"UNKNOWN 73", 0},\
    {"UNKNOWN 74", 0},\
    {"UNKNOWN 75", 0},\
    {"UNKNOWN 76", 0},\
    {"UNKNOWN 77", 0},\
    {"UNKNOWN 78", 0},\
    {"UNKNOWN 79", 0},\
    {"UNKNOWN 7A", 0},\
    {"UNKNOWN 7B", 0},\
    {"UNKNOWN 7C", 0},\
    {"UNKNOWN 7D", 0},\
    {"UNKNOWN 7E", 0},\
    {"UNKNOWN 7F", 0},\
    {"UNKNOWN 80", 0},\
    {"UNKNOWN 81", 0},\
    {"UNKNOWN 82", 0},\
    {"UNKNOWN 83", 0},\
    {"UNKNOWN 84", 0},\
    {"UNKNOWN 85", 0},\
    {"UNKNOWN 86", 0},\
    {"UNKNOWN 87", 0},\
    {"UNKNOWN 88", 0},\
    {"UNKNOWN 89", 0},\
    {"UNKNOWN 8A", 0},\
    {"UNKNOWN 8B", 0},\
    {"UNKNOWN 8C", 0},\
    {"BATTERY VOLT", 0},\
    {"UNKNOWN 8E", 0},\
    {"UNKNOWN 8F", 0},\
    {"UNKNOWN 90", 0},\
    {"UNKNOWN 91", 0},\
    {"UNKNOWN 92", 0},\
    {"UNKNOWN 93", 0},\
    {"UNKNOWN 94", 0},\
    {"UNKNOWN 95", 0},\
    {"UNKNOWN 96", 0},\
    {"UNKNOWN 97", 0},\
    {"UNKNOWN 98", 0},\
    {"UNKNOWN 99", 0},\
    {"UNKNOWN 9A", 0},\
    {"UNKNOWN 9B", 0},\
    {"UNKNOWN 9C", 0},\
    {"UNKNOWN 9D", 0},\
    {"UNKNOWN 9E", 0},\
    {"UNKNOWN 9F", 0},\
    {"UNKNOWN A0", 0},\
    {"UNKNOWN A1", 0},\
    {"UNKNOWN A2", 0},\
    {"UNKNOWN A3", 0},\
    {"UNKNOWN A4", 0},\
    {"UNKNOWN A5", 0},\
    {"AP COURSETARGET", 0},\
    {"AP WP BEARING", 0},\
    {"UNKNOWN A8", 0},\
    {"UNKNOWN A9", 0},\
    {"UNKNOWN AA", 0},\
    {"UNKNOWN AB", 0},\
    {"UNKNOWN AC", 0},\
    {"UNKNOWN AD", 0},\
    {"UNKNOWN AE", 0},\
    {"UNKNOWN AF", 0},\
    {"UNKNOWN B0", 0},\
    {"UNKNOWN B1", 0},\
    {"UNKNOWN B2", 0},\
    {"UNKNOWN B3", 0},\
    {"UNKNOWN B4", 0},\
    {"AP MODE", 0},\
    {"UNKNOWN B6", 0},\
    {"UNKNOWN B7", 0},\
    {"UNKNOWN B8", 0},\
    {"UNKNOWN B9", 0},\
    {"UNKNOWN BA", 0},\
    {"UNKNOWN BB", 0},\
    {"UNKNOWN BC", 0},\
    {"UNKNOWN BD", 0},\
    {"UNKNOWN BE", 0},\
    {"UNKNOWN BF", 0},\
    {"UNKNOWN C0", 0},\
    {"UNKNOWN C1", 0},\
    {"UNKNOWN C2", 0},\
    {"UNKNOWN C3", 0},\
    {"UNKNOWN C4", 0},\
    {"UNKNOWN C5", 0},\
    {"UNKNOWN C6", 0},\
    {"UNKNOWN C7", 0},\
    {"UNKNOWN C8", 0},\
    {"UNKNOWN C9", 0},\
    {"UNKNOWN CA", 0},\
    {"UNKNOWN CB", 0},\
    {"UNKNOWN CC", 0},\
    {"UNKNOWN CD", 0},\
    {"UNKNOWN CE", 0},\
    {"UNKNOWN CF", 0},\
    {"UNKNOWN D0", 0},\
    {"UNKNOWN D1", 0},\
    {"UNKNOWN D2", 0},\
    {"UNKNOWN D3", 0},\
    {"UNKNOWN D4", 0},\
    {"UNKNOWN D5", 0},\
    {"UNKNOWN D6", 0},\
    {"UNKNOWN D7", 0},\
    {"UNKNOWN D8", 0},\
    {"UNKNOWN D9", 0},\
    {"UNKNOWN DA", 0},\
    {"UNKNOWN DB", 0},\
    {"UNKNOWN DC", 0},\
    {"UNKNOWN DD", 0},\
    {"UNKNOWN DE", 0},\
    {"UNKNOWN DF", 0},\
    {"UNKNOWN E0", 0},\
    {"UNKNOWN E1", 0},\
    {"UNKNOWN E2", 0},\
    {"UNKNOWN E3", 0},\
    {"UNKNOWN E4", 0},\
    {"UNKNOWN E5", 0},\
    {"UNKNOWN E6", 0},\
    {"UNKNOWN E7", 0},\
    {"UNKNOWN E8", 0},\
    {"UNKNOWN E9", 0},\
    {"UNKNOWN EA", 0},\
    {"UNKNOWN EB", 0},\
    {"UNKNOWN EC", 0},\
    {"UNKNOWN ED", 0},\
    {"UNKNOWN EE", 0},\
    {"UNKNOWN EF", 0},\
    {"UNKNOWN F0", 0},\
    {"UNKNOWN F1", 0},\
    {"UNKNOWN F2", 0},\
    {"UNKNOWN F3", 0},\
    {"UNKNOWN F4", 0},\
    {"UNKNOWN F5", 0},\
    {"UNKNOWN F6", 0},\
    {"UNKNOWN F7", 0},\
    {"UNKNOWN F8", 0},\
    {"UNKNOWN F9", 0},\
    {"UNKNOWN FA", 0},\
    {"UNKNOWN FB", 0},\
    {"UNKNOWN FC", 0},\
    {"UNKNOWN FD", 0},\
    {"UNKNOWN FE", 0},\
    {"UNKNOWN FF", 0}\
};

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
    pBGNET_ctrl->pDev = BGNET_dev; // pointer to BGNET device definition
    pBGNET_ctrl->pChl = BGNET_chl; // pointer to BGNET channel definition
    pBGNET_ctrl->pCfg = &cfg; // pointer to Config structure

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
    //free(pBGNET_dev);
    
    // child threads are all closed, make a soft landing, bye, bye
    log_info("Exiting %s, bye-bye......", ApplicationName);
    logger_close();
    app_state_shutdown();
    return 0;
}
