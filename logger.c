#include "app_state.h"
#include "logger.h"
#include <stdio.h>
#include <limits.h>
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

// convert a ASCII/HEX character to binary (0-15), or -1 if not hex
// note we use a signed character to be able to return a error condition
int16_t hex2bin(unsigned char ucHex) {
    int16_t r;
    switch (ucHex) {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            r = ucHex - '0';
            break;
        case 'A':
        case 'B':
        case 'C':
        case 'D':
        case 'E':
        case 'F':
            r = ucHex - 'A' + 10;
            break;
        case 'a':
        case 'b':
        case 'c':
        case 'd':
        case 'e':
        case 'f':
            r = ucHex - 'a' + 10;
            break;
        default:
            r = -1;
    }
    return(r);
} 

int16_t ascii2byte(unsigned char *OutByte, char *InString, size_t size) {
    size_t i;
    int16_t r, ucLo, ucHi;
    // initialize return value
    r = 0;
    // for every 2 ASCII characters (bytes), we need at least 1 binary byte
    if (strlen(InString) > (size * 2)) {
        r = -1; // too many ascii characters to fit into the binary array
    } else {
        if (strlen(InString) & 0x01) {
            // we must always have an even number of ASCII characters
            r = -2; // odd number of ascii/hex
        } else {
            if (size > SHRT_MAX) {
                r = -3; // we return the bytecount as a signed byte so we are limited
            } else {
                for (i=0; i< (strlen(InString)/2); i++) {
                    ucHi = hex2bin(InString[i*2 + 0]);
                    ucLo = hex2bin(InString[i*2 + 1]);
                    if ((ucHi < 0) | (ucLo < 0)) {
                        r = -4; // not hex
                        break;
                    } else {
                        // we work per byte, so we do not deal with endieness
                        OutByte[i] = ((ucHi & 0x0F) << 4) | (ucLo & 0x0F);
                    }
                }
                if (r == 0) {
                    r = i - 1; // number of bytes in OutByte
                }
            }
        }
    }
    return(r);
}

// note: we place the nibbles and characters in the same order that we receive them
//       so there should be no problem on a OS with other endianess
// example: 
//       byte2ascii(asciibuf, sizeof(asciibuf), rawbuf, n, BYTE2ASCII_SPACED);
//       log_info("Received: %s (%d bytes)", asciibuf, n);
char* byte2ascii(char *string, size_t size, unsigned char *input, size_t len, int option) {
    size_t i;
    const char hex[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
    switch (option) {
        case  BYTE2ASCII_PLAIN:
            for (i=0;i<BGNET_MIN(size/2-1,len);i++){        // leave 1 byte in the target string for the null-terminator
                string[i*2+0] = hex[((input[i]>>4)&0xF)];   // high nibble
                string[i*2+1] = hex[(input[i]&0xF)];        // low nibble
            }
            string[i*2+0] = BGNET_ASC_NULL;
            break;
        case  BYTE2ASCII_SPACED:
            for (i=0;i<BGNET_MIN(size/3-1,len);i++){        // leave 1 byte in the target string for the null-terminator
                string[i*3+0] = hex[((input[i]>>4)&0xF)];   // high nibble
                string[i*3+1] = hex[(input[i]&0xF)];        // low nibble
                string[i*3+2] = BGNET_ASC_SPACE;
            }
            string[i*3+0] = BGNET_ASC_NULL;
            break;
        case  BYTE2ASCII_HEXED:
            for (i=0;i<BGNET_MIN(size/5-1,len);i++){        // leave 1 byte in the target string for the null-terminator
                string[i*5+0] = BGNET_ASC_0;
                string[i*5+1] = BGNET_ASC_x;
                string[i*5+2] = hex[((input[i]>>4)&0xF)];   // high nibble
                string[i*5+3] = hex[(input[i]&0xF)];        // low nibble
                string[i*5+4] = BGNET_ASC_SPACE;
            }
            string[i*5+0] = BGNET_ASC_NULL;
            break;
        default:
            strcpy(string,"byte2ascii(): invalid option");
            break;
    }
    return(string); // result string
}