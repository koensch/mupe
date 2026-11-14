#include "app_state.h"
#include "config.h"
#include "bgnet.h"
#include "logger.h"
#include <stdio.h>
#include <pthread.h>
#include <stdarg.h>
#include <string.h>

static FILE *logf = NULL;
//static int log_level = LOGGER_LEVEL_INFO;
static pthread_mutex_t log_mutex;

// set the numerical loglevel as specified by the string
int log_level(const char *pStr) {
    int r = LOGGER_LEVEL_UNDEFINED;
    if (strcmp(pStr, "none") == 0) r = LOGGER_LEVEL_NONE;
    else if (strcmp(pStr, "error") == 0) r = LOGGER_LEVEL_ERROR;
    else if (strcmp(pStr, "warn") == 0) r = LOGGER_LEVEL_WARN;
    else if (strcmp(pStr, "info") == 0) r = LOGGER_LEVEL_INFO;
    else if (strcmp(pStr, "debug") == 0) r = LOGGER_LEVEL_DEBUG;
    else r = LOGGER_LEVEL_WARN; // default
    return(r);
}
void log_init(const char *filename, const char *level)
{
    int log;
    time_t t;
    time(&t);

    pthread_mutex_init(&log_mutex, NULL);

    g_n2kdata.cfg.log_level.value.nByte = log_level(level);
    g_n2kdata.cfg.log_level.source = BGN_SOURCE_ACP;
    CounterInit(&g_n2kdata.cfg.log_level.Counter, t);

    logf = fopen(filename, "a");
    if (!logf) {
        logf = stderr;
    }
    log = g_n2kdata.cfg.log_level.value.nByte;
    g_n2kdata.cfg.log_level.value.nByte = LOGGER_LEVEL_INFO;
    log_info("***** Logging started (level %d), output to %s, git reference: %s", log, filename, GIT_VER);
    g_n2kdata.cfg.log_level.value.nByte = log;
}

void log_close(void)
{
    int log;
    log = g_n2kdata.cfg.log_level.value.nByte;
    g_n2kdata.cfg.log_level.value.nByte = LOGGER_LEVEL_INFO;
    log_info("***** Logging stopped (level %d)",log);
    g_n2kdata.cfg.log_level.value.nByte = log;

    if (logf && logf != stderr)
        fclose(logf);
    pthread_mutex_destroy(&log_mutex);
}

static void write_log(int level, const char *prefix, const char *fmt, va_list ap)
{
    if (level <= g_n2kdata.cfg.log_level.value.nByte) {
        pthread_mutex_lock(&log_mutex);
        fprintf(logf, "%s ", prefix);
        vfprintf(logf, fmt, ap);
        fprintf(logf, "\n");
        fflush(logf);
        pthread_mutex_unlock(&log_mutex);
    } else {
        // do not log higher levels
    }
    return;
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
            r = (ucHex - BGNET_ASC_0);
            break;
        case BGNET_ASC_A:
        case BGNET_ASC_B:
        case BGNET_ASC_C:
        case BGNET_ASC_D:
        case BGNET_ASC_E:
        case BGNET_ASC_F:
            r = (ucHex - BGNET_ASC_A) + 10;
            break;
        case BGNET_ASC_a:
        case BGNET_ASC_b:
        case BGNET_ASC_c:
        case BGNET_ASC_d:
        case BGNET_ASC_e:
        case BGNET_ASC_f:
            r = (ucHex - BGNET_ASC_a) + 10;
            break;
        default:
            r = -1;
    }
    return(r);
}
// convert 1 byte binary into 2 bytes HEX characters
int16_t bin2hex(unsigned char ucBin) {
    char hex[17] = {"0123456789ABCDEF"}; // 1 extra for the null character
    return ((hex[((ucBin & 0xF0) >> 4)] << 8) | hex[(ucBin & 0x0F)]);
} 

int16_t ascii2byte(unsigned char *OutByte, char *InString, size_t size) {
    size_t i, iHexType;
    int16_t r, ucLo, ucHi;
    r = 0;  // initialize return value

    // determine what type of hex-string we are dealing with
    // usually a user does not specify the trailing terminator (otherwise you do not arrive at a multiple of 5 or 3), so we assume it's not there
    iHexType = BYTE2ASCII_NONE;
    // if the length is a multiple of 5, and it starts with a '0x'/'0X' and has a space, most likely it's a 'hexed' hex-string
    if ((InString[0] == BGNET_ASC_0) & ((InString[1] == BGNET_ASC_X)|(InString[1] == BGNET_ASC_x))  & (InString[4]==BGNET_ASC_SPACE) & ((strlen(InString)+1)%BYTE2ASCII_HEXED == 0)) {
        iHexType = BYTE2ASCII_HEXED;
    // if the length is a multiple of 3, and it has a space as the 3rd character, most likely it's a 'spaced' hex-string
    } else if ((InString[2]==BGNET_ASC_SPACE) & ((strlen(InString)+1)%BYTE2ASCII_SPACED == 0)) {
        iHexType = BYTE2ASCII_SPACED;
    // if the length is a multiple of2, most likely it is a 'plain' hex-string
    } else if (strlen(InString)%BYTE2ASCII_PLAIN == 0) {
        iHexType = BYTE2ASCII_PLAIN;    
    }

    // we must have a good idea how the hex-string is formatted
    if (iHexType == BYTE2ASCII_NONE) {
        // we must always have an even number of ASCII characters
        r = BYTE2ASCII_NOTEVEN; // odd number of ascii/hex
    } else {
        for (i=0; i<BGNET_MIN((strlen(InString)*iHexType), size); i++) {
            switch (iHexType) {
                case BYTE2ASCII_HEXED:
                    ucHi = hex2bin(InString[i*iHexType + 2]);
                    ucLo = hex2bin(InString[i*iHexType + 3]);
                    break;
                case BYTE2ASCII_SPACED:
                    ucHi = hex2bin(InString[i*iHexType + 0]);
                    ucLo = hex2bin(InString[i*iHexType + 1]);
                    break;
                case BYTE2ASCII_PLAIN:
                    ucHi = hex2bin(InString[i*iHexType + 0]);
                    ucLo = hex2bin(InString[i*iHexType + 1]);
                    break;
            }
            if ((ucHi < 0) | (ucLo < 0)) {
                r = BYTE2ASCII_NOTHEX;
                break;
            } else {
                // we work per byte, so we do not deal with endieness, we can shift the nibbles as we need
                OutByte[i] = ((ucHi & 0x0F) << 4) | (ucLo & 0x0F);
            }
        }
        if (r == 0) {
            r = i; // number of bytes in OutByte. Note: if r>size, not all hex-ascii was read
        }
    }
    return(r);
}
// with the optional offset parameter, we can use this routine to iterate through a multi-frame buffer
int16_t ascii2frame(FRAME_H *pF, char *pString, size_t *pOffset) {
    int16_t r, len_msg;
    size_t offset = 0; // default is 0 -> start of string
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