#include "app_state.h"
#include "config.h"
#include "bgnprotocol.h"
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

// convert a ASCII/HEX character to binary (0-15), return -1 if not hex
// note we use a signed character to be able to return a error condition
int16_t hex2bin(unsigned char ucHex) {
    int16_t r;
    switch (ucHex) {
        case BGNET_ASC_0:
        case BGNET_ASC_1:
        case BGNET_ASC_2:
        case BGNET_ASC_3:
        case BGNET_ASC_4:
        case BGNET_ASC_5:
        case BGNET_ASC_6:
        case BGNET_ASC_7:
        case BGNET_ASC_8:
        case BGNET_ASC_9:
            r = ucHex - BGNET_ASC_0;
            break;
        case BGNET_ASC_A:
        case BGNET_ASC_B:
        case BGNET_ASC_C:
        case BGNET_ASC_D:
        case BGNET_ASC_E:
        case BGNET_ASC_F:
            r = ucHex - BGNET_ASC_A + 10;
            break;
        case BGNET_ASC_a:
        case BGNET_ASC_b:
        case BGNET_ASC_c:
        case BGNET_ASC_d:
        case BGNET_ASC_e:
        case BGNET_ASC_f:
            r = ucHex - BGNET_ASC_a + 10;
            break;
        default:
            r = -1;
    }
    return(r);
} 

int16_t ascii2byte(unsigned char *OutByte, char *InString, size_t size) {
    size_t i;
    int16_t r, ucLo, ucHi;
    r = 0;  // initialize return value
    // input string must be a multiple of 2
    if ((strlen(InString)%2) == 0) {
        for (i=0; i<BGNET_MIN((strlen(InString)*2), size); i++) {
            ucHi = hex2bin(InString[i*2 + 0]);
            ucLo = hex2bin(InString[i*2 + 1]);
            if ((ucHi < 0) | (ucLo < 0)) {
                r = BYTE2ASCII_NOTHEX;
                break;
            } else {
                // we work per byte, so we do not deal with endieness, we can shift the nibbles as we need
                OutByte[i] = ((ucHi & 0x0F) << 4) | (ucLo & 0x0F);
            }
        }
        if (r == 0) {
            r = i; // number of bytes in OutByte. Note: if r>size, the ascii string might be missing the last portion
        }
    } else {
        // we must always have an even number of ASCII characters
        r = BYTE2ASCII_NOTEVEN; // odd number of ascii/hex
    }
    return(r);
}
int16_t ascii2frame(FRAME_H *pF, char *pString, size_t *pOffset) {
    int16_t r, len_msg;
    size_t offset = 0;
    if (pOffset != NULL) offset = *pOffset;
    // convert the ascii/hex string into a binary frame (just the header)
    r = ascii2byte((unsigned char *)pF, &pString[offset], BGNET_FRAME_HEAD);
    if (r == BYTE2ASCII_EMPTY) {
        // no more data in source string, just return it
    } else {
        if (r > 0) {
            len_msg = bgnet_sizeof_frame(pF);
            if (len_msg > r) {
                // frame is bigger than just the header; re-read the frame with the header + payload
                // (i know i could just add reading the payload to the exsiting header, but that is too complex at this point)
                r = ascii2byte((unsigned char *)pF, &pString[offset], len_msg);
                if (r > 0) {
                    if (pOffset != NULL) *pOffset = *pOffset + len_msg; // (incase we come back to read a buffer with subframes)
                } else {
                    // r must contain a negative error code; just return it
                }
            } else {
                // seems to be a frame without payload, it already have it
            }
        } else {
            // r must contain a negative error code; just return it
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
    const char hex[16]={BGNET_ASC_0,BGNET_ASC_1,BGNET_ASC_2,BGNET_ASC_3,BGNET_ASC_4,BGNET_ASC_5,BGNET_ASC_6,BGNET_ASC_7,BGNET_ASC_8,BGNET_ASC_9,BGNET_ASC_A,BGNET_ASC_B,BGNET_ASC_C,BGNET_ASC_D,BGNET_ASC_E,BGNET_ASC_F};
    switch (option) {
        case  BYTE2ASCII_PLAIN:
            for (i=0;i<BGNET_MIN(size/2-1,len);i++){        // leave 1 byte in the target string for the null-terminator
                string[i*2+0] = hex[((input[i]>>4)&0x0F)];  // high nibble
                string[i*2+1] = hex[(input[i]&0x0F)];       // low nibble
            }
            string[i*2+0] = BGNET_ASC_NULL;
            break;
        case  BYTE2ASCII_SPACED:
            for (i=0;i<BGNET_MIN(size/3-1,len);i++){        // leave 1 byte in the target string for the null-terminator
                string[i*3+0] = hex[((input[i]>>4)&0x0F)];  // high nibble
                string[i*3+1] = hex[(input[i]&0x0F)];       // low nibble
                string[i*3+2] = BGNET_ASC_SPACE;
            }
            string[i*3+0] = BGNET_ASC_NULL;
            break;
        case  BYTE2ASCII_HEXED:
            for (i=0;i<BGNET_MIN(size/5-1,len);i++){        // leave 1 byte in the target string for the null-terminator
                string[i*5+0] = BGNET_ASC_0;
                string[i*5+1] = BGNET_ASC_x;
                string[i*5+2] = hex[((input[i]>>4)&0x0F)];  // high nibble
                string[i*5+3] = hex[(input[i]&0x0F)];       // low nibble
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