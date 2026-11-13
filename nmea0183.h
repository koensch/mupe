#ifndef NMEA0183_H
#define NMEA0183_H

#include "config.h"

void *nmea0183_thread(void *);
char *bgnet_nmea_sentences(char *, unsigned char);
void bgnet_nmea_ctrl_init(BGNET_NMEA_CTRL *, uint8_t, time_t);
uint8_t bgnet_nmea_ctrl_pop(BGNET_NMEA_CTRL *, time_t);

#endif
