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

#endif
