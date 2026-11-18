#ifndef CONFIG_H
#define CONFIG_H

#define _GNU_SOURCE

#include <stdint.h>

// at present we only support the netio bus used by NETWORK series. we may open the door
// for other protocols, like Fastnet to be added later.
#define BGNET_HANDLER_UNDEF         0
#define BGNET_HANDLER_NETIO         1
#define BGNET_HANDLER_TYPE_NETIO    "netio"

typedef struct {
    struct {
        char  port[128];
        int   baudrate;
        int   databits;
        int   stopbits;
        char  parity[16];
        char  mode[16];
        char  protocol[16];
        int   handler;
    } fastnet;

    struct {
        char  eth_device[64];
        char  protocol[8];      /* "tcp" or "udp" */
        char  remote_ip[64];
        int   remote_port;
        int   local_port;
    } ethernet;

    // In our test setup, only PILOT has NMEA input capabilities. It accepts a limited set of sentences
    // We use a bitpattern to select for 1 or or more for each port.
    // Leave the 'portx' or 'navx' field empty to disable the port
    #define BGNET_NMEA0183_OFF          0x00
    #define BGNET_NMEA0183_APA          0x01
    #define BGNET_NMEA0183_APB          0x02
    #define BGNET_NMEA0183_RMB          0x04
    #define BGNET_NMEA0183_XTE          0x08
    #define BGNET_NMEA0183_VTG          0x10
    #define BGNET_NMEA0183_BWR          0x20
    #define BGNET_NMEA0183_BWC          0x40
    #define BGNET_NMEA0183_VHW          0x80

    // this is the strucure where we store the config parameters
    struct {
        int   baudrate;
        int   databits;
        int   stopbits;
        char  parity[16];
        struct {
            char  port[128];
            char  nav[16];
            int   enable;
        } port1;
        struct {
            char  port[128];
            char  nav[16];
            int   enable;
        } port2;
    } nmea0183;

    struct {
        char  can_interface[16]; // don't make this bigger; you might get compiler warnings, 16 bytes should be big enough.
    } nmea2000;

    struct {
        int   timer_interval_ms;
    } timer;

    struct {
        int   web_port;
        int   web_autorefresh;
    } webserver;

    struct {
        char  str_log_file[256];
        char  str_log_level[16];
    } logger;
} Config;

int load_config(const char *filename, Config *cfg);

#endif
