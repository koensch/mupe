#ifndef NMEA0183_H
#define NMEA0183_H

#include "config.h"

void *nmea0183_thread(void *);
char *strSentences(char *, unsigned char);

#endif
