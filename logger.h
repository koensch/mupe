#ifndef LOGGER_H
#define LOGGER_H

// lower-value logging is included in higher level
#define LOGGER_LEVEL_UNDEFINED  -1
#define LOGGER_LEVEL_NONE   0
#define LOGGER_LEVEL_ERROR  1
#define LOGGER_LEVEL_WARN   2
#define LOGGER_LEVEL_INFO   3
#define LOGGER_LEVEL_DEBUG  4

int log_level(const char *);
void log_init(const char *filename, const char *level);
void log_close(void);

void log_error(const char *fmt, ...);
void log_warn(const char *fmt, ...);
void log_info(const char *fmt, ...);
void log_debug(const char *fmt, ...);

int16_t hex2bin(unsigned char); // convert a single HEX/ASCII character ('0' ... 'F') into the binary equivalent (0-15 or -1 if the characters is not HEX)
int16_t bin2hex(unsigned char); // convert a binary value (0-15) into the HEX/ASCII quivalent ('0' ... 'F')

#endif
