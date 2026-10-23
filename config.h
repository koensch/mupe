#ifndef CONFIG_H
#define CONFIG_H

#define _GNU_SOURCE

#include <stdint.h>

typedef struct {
    struct {
        char  port[128];
        int   baudrate;
        int   databits;
        int   stopbits;
        char  parity[16];
        char  mode[16];
        char  protocol[16];
    } fastnet;

    struct {
        char  eth_device[64];
        char  protocol[8];      /* "tcp" or "udp" */
        char  remote_ip[64];
        int   remote_port;
        int   local_port;
    } ethernet;

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
