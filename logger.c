#include "app_state.h"
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

// note: we place the nibbles and characters in the same order that we receive them
//       so there should be no problem on a OS with other endianess
size_t byte2ascii(char* string, size_t size, unsigned char* input, size_t len, int option) {
    char char1,char2;
    size_t i,r;
    const char hex[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
    switch (option) {
        case  BYTE2ASCII_PLAIN:
            for (i=0;i<MIN(size/2-1,len);i++){      // leave 1 byte in the target string for the null-terminator
                char1 = hex[(input[i]&0xF)];        // low nibble
                char2 = hex[((input[i]>>4)&0xF)];   // high nibble
                string[i*2+1]=char1;    // low nibble
                string[i*2+0]=char2;    // high nibble
            }
            string[i*2+0] = '\0'; // null byte
            r=i*2;
            break;
        case  BYTE2ASCII_SPACED:
        case  BYTE2ASCII_HEXED:
            // not yet implemented
        default:
            strcpy(string,"byte2ascii() - invalid option");
            r=strlen(string);
            break;
    }
    return(r); // number of characters in ascii string
}
