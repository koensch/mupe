#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>

#include "config.h"
#include "logger.h"
#include "app_state.h"
#include "serial.h"
#include "bgnprotocol.h"
#include "ethernet.h"
#include "timer.h"
#include "webserver.h"
#include "nmea2000.h"

// signal handling to make a soft landing
//  - ps ax | grep {processname} > gives you the process name and processID
//  - kill -s SIGINT {processID} > sends <Ctrl><C> to the service
// or press <Ctrl><C> if running from the commandline
void handle_sigint(int sig) {
    if (sig == SIGINT) {
        g_running = 0;
    }
}

// please consider this as 'constant'; do not write into this structure.
// Use is as some kind of bookkeeping table. We only 'enable' the devices we have tested.
// if we do a lookup and there is no match, we return 0 and pick the first one (the ACP)
BGNET_DEVDT DEVDT[BGNET_TYPE_MAX] = { \
// ProdName     Model       ModelNr       SerialNr       Version Enable DeviceType
{"??????",      "??????",   "??????",     "??????",      "????", 0,     0xFF},\
{"NET ACP",     "NET-ACP",  "360-00-002", "NP1CB15613",  "0000", 0,     BGNET_FRAME_FROM_NETACP},\
{"NET DEPTH",   "NET-DPDU", "610-00-021", "NDEP9410109", "0000", 0,     BGNET_FRAME_FROM_NETACP},\
{"NET SPEED",   "NET-SPDU", "610-00-020", "NSPDG485015", "0000", 0,     BGNET_FRAME_FROM_NETACP},\
{"NET WIND",    "NET-WDDU", "611-00-007", "NWINB06505",  "0000", 1,     BGNET_FRAME_FROM_NETWIND},\
{"NET GPSplus", "NET-GPDU", "615-00-021", "",            "0000", 0,     BGNET_FRAME_FROM_NETACP},\
{"NET LORAN",   "NET-LORAN","",           "",            "0000", 0,     BGNET_FRAME_FROM_NETACP},\
{"NET CHART",   "NET-CHART","",           "CHT350327",   "0000", 0,     BGNET_FRAME_FROM_NETACP},\
{"NET COMPASS", "NET-CPDU", "545-00-031", "",            "0000", 0,     BGNET_FRAME_FROM_NETACP},\
{"NET TACK",    "NET-TADU", "610-00-086", "",            "0000", 0,     BGNET_FRAME_FROM_NETACP},\
{"NET PILOT",   "NET-PLDU", "610-00-010", "NPLDB06217",  "0000", 1,     BGNET_FRAME_FROM_NETPILOT},\
{"NET QUAD",    "NET-QDDU", "610-00-023", "NQDUB08432",  "0000", 1,     BGNET_FRAME_FROM_NETQUAD},\
{"NET DATA",    "NET-DTDU", "610-00-024", "NDAT9417101", "0000", 0,     BGNET_FRAME_FROM_NETDATA},\
{"NET NAV",     "NET-NADU", "610-00-086", "",            "0000", 0,     BGNET_FRAME_FROM_NETACP},\
{"NET ENGINE",  "NET-ENGI", "610-00-132", "",            "0000", 0,     BGNET_FRAME_FROM_NETACP},\
{"NET QUAD-R",  "NET-QDRU", "610-00-022", "",            "0000", 0,     BGNET_FRAME_FROM_NETACP},\
{"NET VHF",     "NET-VHF",  "613-10-003", "",            "0000", 0,     BGNET_FRAME_FROM_NETACP}};



// allow import/export definitions for NMEA & Signal K. import or export is exclusive. When you import from one, you can not
// export to the other You can't have it both ways.
#define BGN_CNR_IMPEXP_NONE 0       // no provisions for import/export
#define BGN_CNR_IMPEXP_PGN  1       // import from or export to NMEA2000
#define BGN_CNR_IMPEXP_SK   2       // import from or export to SignalK

// this lookup table allows us:
// - to lookup the channel label (looks nice in styatistics report)
// - to obtain the format specifier (when making a output message, see note below)
// - enable/disable items for import or export
// - import/export data to/from NMEA of Signal K interface
// - get the NMEA PGN for import/export (if NMEA2000 enabled)
// - SignalK reference key(if SignalK enabled)
// i would prefer this to do in some kind of ini-file, but that might be done later (if ever).
// i know it's kind of redundant to have the channel number in this table (it's also the index),
// but this table gives me a quick x-ref when i'm reverse-engineering the protocol.
// Note: The DataType is determined by the lower nibble. This must be a valid number (use the defines to be sure).
//       If the high nibble is 0 we do NOT perform scaling when making a output message. If the high nibble is non-zero,
//       the application attempts to adjust the value and divisor to get the compatible display representation.
//       This FMT is only used for creating output (write) DATA commands (0x01) for DataType 1 and 8. For incoming
//       messages, the sender(s) alread did the scaling for us.

    uint8_t ubChannelIndex;     // Channel index
    char cLabel[20];            // the label, variable name
    uint8_t ucFMT;              // default formatting
    int iChannelLen;            // number of bytes in channel
    int iImpExpEnable;          // allow Import or Export
    int iImportFrom;            // Import from NMEA2000 or SignalK
    int iExportTo;              // Export to NMEA2000 of SignalK
    int iNMEA_PGN;              // NMEA2000 reference PGN
    char cSKdef[32];            // SignalK reference string 

BGNET_CHNDT CHNDT[BGN_CHANNEL_MAX+1] = { \
    //Index                    Label              FMT       n/a  I/E enable    Import From Export To          PGN SK
    {0,                        "UNKNOWN 00",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 01",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 02",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 03",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 04",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 05",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 06",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 07",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 08",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 09",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 0A",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_0B_RUDDER,        "RUDDER"    ,      BGN_FMT_01, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 0C",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 0D",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 0E",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 0F",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 10",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 11",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 12",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 13",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 14",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 15",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 16",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 17",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 18",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 19",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 1A",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 1B",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 1C",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 1D",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 1E",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 1F",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_20_SEATEMPQ,      "SEA TEMP",        BGN_FMT_01, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 21",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 22",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 23",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 24",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 25",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 26",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 27",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 28",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 29",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 2A",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 2B",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 2C",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 2D",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 2E",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 2F",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 30",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_31_ECHO_SNR,      "ECHO SND"  ,      BGN_FMT_04, 6, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 32",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 33",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 34",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 35",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 36",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 37",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 38",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 39",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 3A",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 3B",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 3C",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 3D",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 3E",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 3F",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 40",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_41_BOATSPEEDKTS,  "BOATSPEED KT",    BGN_FMT_08, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_42_BOATSPEEDRAW,  "BOATSPEED RAW",   BGN_FMT_08, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 43",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 44",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 45",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 46",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 47",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 48",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_49_HEADING,       "HEADING",         BGN_FMT_08, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 4A",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 4B",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 4C",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 4D",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_4E_AWINDSPEEDRAW, "AWS RAW",         BGN_FMT_08, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 4F",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_50_AWINDSPEEDKT,  "AWS KT",          BGN_FMT_08, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_51_AWINDANGLE,    "AWA",             BGN_FMT_01, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_52_AWINDANGLERAW, "AWA RAW",         BGN_FMT_08, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_53_TARGET_TWA,    "TARGET TWA",      BGN_FMT_01, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 54",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 55",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 56",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_57_TWINDSPEEDKT,  "TWS KT",          BGN_FMT_08, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 58",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_59_TWINDANGLE,    "TWA",             BGN_FMT_01, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 5A",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 5B",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 5C",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 5D",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 5E",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 5F",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 60",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 61",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 62",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 63",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_64_SPEEDAVGKTS,   "SPEED AVG KT",    BGN_FMT_08, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_65_SPEEDAVGRAW,   "SPEED AVG RAW",   BGN_FMT_08, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 66",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 67",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 68",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 69",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 6A",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 6B",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 6C",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 6D",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 6E",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 6F",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 70",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 71",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 72",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 73",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 74",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_75_TIMER,         "TIMER",           BGN_FMT_03, 5, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 76",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 77",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 78",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 79",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 7A",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 7B",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 7C",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 7D",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 7E",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_7F_VMGKT,         "VMG KT",          BGN_FMT_08, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 80",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_81_DR_DIST,       "DR DISTANCE",     BGN_FMT_08, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 82",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 83",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 84",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 85",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 86",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 87",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 88",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 89",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 8A",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 8B",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 8C",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_8D_BATTVOLT,      "BATT VOLT",       BGN_FMT_08, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 8E",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 8F",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 90",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 91",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 92",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 93",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 94",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 95",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 96",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 97",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 98",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 99",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 9A",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 9B",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 9C",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 9D",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 9E",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN 9F",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN A0",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN A1",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_A2_UNKNOWN,       "FROM WIND A2",    BGN_FMT_01, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN A3",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN A4",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN A5",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_A6_AP_CTARGET,    "AP COURSETARGET", BGN_FMT_08, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_A7_AP_WPB,        "AP WP BEARING",   BGN_FMT_08, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN A8",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN A9",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN AA",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN AB",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN AC",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN AD",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN AE",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN AF",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN B0",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN B1",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN B2",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN B3",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN B4",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_B5_AP_MODE,       "AP MODE",         BGN_FMT_08, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN B6",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN B7",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN B8",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN B9",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN BA",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN BB",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN BC",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN BD",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN BE",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN BF",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN C0",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN C1",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN C2",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN C3",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_C4_DEPTHM,        "DEPTH M",         BGN_FMT_01, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN C5",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN C6",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN C7",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN C8",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN C9",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN CA",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN CB",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN CC",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_CD_STORLOGNM,     "STORED LOG NM",   BGN_FMT_42, 6, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN CE",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_CF_TRIPLOGNM,     "TRIP LOG NM",     BGN_FMT_08, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN D0",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN D1",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN D2",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_D3_DR_COURSE,     "DR COURSE",       BGN_FMT_01, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN D4",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN D5",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN D6",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN D7",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN D8",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN D9",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN DA",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN DB",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN DC",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN DD",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN DE",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN DF",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN E0",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN E1",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN E2",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN E3",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN E4",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN E5",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN E6",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN E7",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN E8",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN E9",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN EA",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN EB",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN EC",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN ED",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN EE",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN EF",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN F0",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN F1",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN F2",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN F3",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN F4",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN F5",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN F6",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN F7",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN F8",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN F9",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN FA",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {0,                        "UNKNOWN FB",      0x00,       0, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_FC_SHALLOWALARM,  "SHALLOW ALARM",   BGN_FMT_04, 6, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_FD_DEPTHALARM,    "DEPTH ALARM",     BGN_FMT_01, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_FE_ANCHORALARM,   "ANCHOR ALARM",    BGN_FMT_04, 6, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""},\
    {BGN_CNR_FF_DEPTH_CAL,     "DEPTH CAL",       BGN_FMT_01, 4, 0,  BGN_CNR_IMPEXP_NONE, BGN_CNR_IMPEXP_NONE, 0, ""}\
};

// for every DataType (0-15) we check the validity (>0) and the number of bytes used in the frame
BGNET_FMTDT FMTDT[16] = { {-1, ""}, \
                                { 4, "2 bytes, signed 16bit register"}, \
                                { 6, "4 bytes, unsigned 32bit register (used by STOREDLOG)"}, \
                                { 5, "3 bytes, timer value (HH,MM,SS, displayed as 'H:MM:SS')"}, \
                                { 6, "4 bytes, text (00+3 bytes, like ' T ',' F ','OFF','ON ','---')"}, \
                                {-1, ""}, \
                                {-1, ""}, \
                                {-1, ""}, \
                                { 4, "2 bytes, unsigned 16bit register"}, \
                                {-1, ""}, \
                                {-1, ""}, \
                                {-1, ""}, \
                                {-1, ""}, \
                                {-1, ""}, \
                                {-1, ""}, \
                                {-1, ""}
                                };

int main(void)
{
    int i;
    // assign signal handler
    signal(SIGINT, handle_sigint);

    // first we need the configuration, otherwise we cannot log errors
    Config cfg;
    if (load_config("config.json", &cfg) != 0) {
        // this is the first and only printf we use in this application; all other
        // logging is done through the logfile as specified in the configuration
        fprintf(stderr, "Failed to load config.json\n");
        return 1;
    }
    logger_init(cfg.log_file, cfg.log_level);
    log_info("Starting %s", ApplicationName);

    //allocate space for shared data
    BGNET_CTRL* pBGNET_ctrl;
    pBGNET_ctrl = malloc(sizeof(BGNET_CTRL));
    if (pBGNET_ctrl == NULL) {
        log_error("Unable to allocale memory for pBGNET_ctrl");
        exit(1);
    }
    // setup pointers to definition structures
    pBGNET_ctrl->pDEVDT = DEVDT; // pointer to BGNET device definitions
    pBGNET_ctrl->pCHNDT = CHNDT; // pointer to BGNET channel definitions
    pBGNET_ctrl->pFMTDT = FMTDT; // pointer to BGNET format defintions
    pBGNET_ctrl->pCfg = &cfg; // pointer to Config structure
    bgn_net_initialize_reset(pBGNET_ctrl);
    bgn_net_initialize(pBGNET_ctrl);

    app_state_init();

    // activate all out threads
    pthread_t th_serial, th_eth, th_timer, th_web, th_n2k;
    if (
           (pthread_create(&th_serial, NULL, serial_thread, pBGNET_ctrl) != 0 ||
            pthread_create(&th_eth,    NULL, ethernet_thread, pBGNET_ctrl) != 0 ||
            pthread_create(&th_timer,  NULL, timer_thread, pBGNET_ctrl) != 0 ||
            pthread_create(&th_web,    NULL, webserver_thread, pBGNET_ctrl) != 0 ||
            pthread_create(&th_n2k,    NULL, nmea2000_thread, pBGNET_ctrl) != 0)) {
        // not all threads could be started, close the ones that did start and shutdown with error
        log_error("Failed to create threads");
        g_running = 0;
        app_state_shutdown();
        logger_close();
        return 1;
    }

    // Run forever. If needed, we can do some bookkeeping now and then. Loop exit via signal handler
    while(g_running) {
        sleep(1);
        pthread_mutex_lock(&g_data_mutex);
        //bgn_net_initialize(pBGNET_ctrl);
        pthread_mutex_unlock(&g_data_mutex);
    }

    // deactivate threads
    pthread_join(th_serial, NULL);
    pthread_join(th_eth, NULL);
    pthread_join(th_timer, NULL);
    pthread_join(th_web, NULL);
    pthread_join(th_n2k, NULL);

    // print report
    log_info("Statistics report");
    log_info("   FROM       CounterRx    CounterTx  DeviceType");
    for (i=0; i<BGNET_DEVICE_MAX; i++) {
        if ((pBGNET_ctrl->DEV[i].iCounterRx < 1) & (pBGNET_ctrl->DEV[i].iCounterTx < 1)) {
            // it was never touched
        } else {
            log_info("  0x%02X        %8d     %8d     %s", i, pBGNET_ctrl->DEV[i].iCounterRx, pBGNET_ctrl->DEV[i].iCounterTx, pBGNET_ctrl->pDEVDT[bgn_FindDeviceIndex(pBGNET_ctrl, i)].sProdName);
        }
    }
    log_info("    CMD       CounterRx    CounterTx");
    for (i=0; i<BGNET_COMMAND_MAX; i++) {
        if ((pBGNET_ctrl->CMD[i].iCounterRx < 1) & (pBGNET_ctrl->CMD[i].iCounterTx < 1)) {
            // it was never touched
        } else {
            log_info("  0x%02X        %8d     %8d", i, pBGNET_ctrl->CMD[i].iCounterRx, pBGNET_ctrl->CMD[i].iCounterTx, pBGNET_ctrl->CMD[i].iCounterTx);
        }
    }
    log_info("Channel         Counter    Label");
    for (i=0; i<BGN_CHANNEL_MAX; i++) {
        if (pBGNET_ctrl->DATA[i].iCounter < 1) {
            // it was never touched
        } else {
            log_info("  0x%02X        %8d     %s", i, pBGNET_ctrl->DATA[i].iCounter, pBGNET_ctrl->pCHNDT[i].cLabel);
        }
    }
    log_info("End of report");

    // free allocated memory
    free(pBGNET_ctrl);
    
    // child threads are all closed, make a soft landing, bye, bye
    log_info("Exiting %s, bye-bye......", ApplicationName);
    logger_close();
    app_state_shutdown();
    return 0;
}
