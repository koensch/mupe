#ifndef LOGGER_H
#define LOGGER_H

// lower-value logging is included in higher level
#define LOGGER_LEVEL_ERROR  0
#define LOGGER_LEVEL_WARN   1
#define LOGGER_LEVEL_INFO   2
#define LOGGER_LEVEL_DEBUG  3

void logger_init(const char *filename, const char *level);
void logger_close(void);

void log_error(const char *fmt, ...);
void log_warn(const char *fmt, ...);
void log_info(const char *fmt, ...);
void log_debug(const char *fmt, ...);

// we use a option string to allow for spacing, and output formatting
#define BYTE2ASCII_PLAIN    0   // continous string:        00010203 .... FEFF
#define BYTE2ASCII_SPACED   1   // space between bytes:     00 01 02 03 .... FE FF 
#define BYTE2ASCII_HEXED    2   // every byte preceeded by  0x00 0x01 0x02 0x03 .... 0xFE 0xFF
char* byte2ascii(char*, size_t, unsigned char*, size_t, int);

#endif
