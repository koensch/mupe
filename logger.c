#include "logger.h"
#include <stdio.h>
#include <pthread.h>
#include <stdarg.h>
#include <string.h>

static FILE *logf = NULL;
static int log_level = LOGGER_LEVEL_INFO;
static pthread_mutex_t log_mutex;

void logger_init(const char *filename, const char *level)
{
    int log;

    pthread_mutex_init(&log_mutex, NULL);

    if (strcmp(level, "error") == 0) log_level = LOGGER_LEVEL_ERROR;
    else if (strcmp(level, "warn") == 0) log_level = LOGGER_LEVEL_WARN;
    else if (strcmp(level, "info") == 0) log_level = LOGGER_LEVEL_INFO;
    else if (strcmp(level, "debug") == 0) log_level = LOGGER_LEVEL_DEBUG;

    logf = fopen(filename, "a");
    if (!logf) {
        logf = stderr;
    }
    log = log_level;
    log_level = LOGGER_LEVEL_INFO;
    log_info("***** Logging started (level %d)",log);
    log_level = log;
}

void logger_close(void)
{
    int log;
    log = log_level;
    log_level = LOGGER_LEVEL_INFO;
    log_info("***** Logging stopped (level %d)",log);
    log_level = log;

    if (logf && logf != stderr)
        fclose(logf);
    pthread_mutex_destroy(&log_mutex);
}

static void write_log(int level, const char *prefix, const char *fmt, va_list ap)
{
    if (level > log_level) return;

    pthread_mutex_lock(&log_mutex);
    fprintf(logf, "%s ", prefix);
    vfprintf(logf, fmt, ap);
    fprintf(logf, "\n");
    fflush(logf);
    pthread_mutex_unlock(&log_mutex);
}

void log_error(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    write_log(LOGGER_LEVEL_ERROR, "[ERROR]", fmt, ap);
    va_end(ap);
}

void log_warn(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    write_log(LOGGER_LEVEL_WARN, "[WARN ]", fmt, ap);
    va_end(ap);
}

void log_info(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    write_log(LOGGER_LEVEL_INFO, "[INFO ]", fmt, ap);
    va_end(ap);
}

void log_debug(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    write_log(LOGGER_LEVEL_DEBUG, "[DEBUG]", fmt, ap);
    va_end(ap);
}
