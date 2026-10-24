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

#endif
