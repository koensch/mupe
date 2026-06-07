#ifndef CONFIG_H
#define CONFIG_H

#define _GNU_SOURCE

#include <stdint.h>

typedef struct {
    char  serial_port[128];
    int   baudrate;
    int   databits;
    int   stopbits;
    char  parity[16];
    char  mode[16];

    char  eth_device[64];
    char  protocol[8];      /* "tcp" or "udp" */
    char  remote_ip[64];
    int   remote_port;
    int   local_port;

    char  can_interface[16]; // don't make this bigger; you might get compiler warnings, 16 bytes should be big enough.

    int   timer_interval_ms;
    int   web_port;

    char  log_file[256];
    char  log_level[16];
} Config;

int load_config(const char *filename, Config *cfg);

#endif
