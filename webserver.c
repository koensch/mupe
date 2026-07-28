#define _GNU_SOURCE

#include "webserver.h"
#include "app_state.h"
#include "logger.h"
#include "bgnprotocol.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <cjson/cJSON.h>
#include <openssl/sha.h>
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <stdbool.h>
#include <ctype.h>
#include <openssl/evp.h>
#include <openssl/buffer.h>
#include <fcntl.h>

#define MAX_WS_CLIENTS 10

static int ws_clients[MAX_WS_CLIENTS];
static pthread_mutex_t ws_mutex = PTHREAD_MUTEX_INITIALIZER;

static void ws_add_client(int fd)
{
    pthread_mutex_lock(&ws_mutex);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (ws_clients[i] == 0) {
            ws_clients[i] = fd;
            break;
        }
    }
    pthread_mutex_unlock(&ws_mutex);
}

char* strSource(char* buffer, size_t nSource) {
    char cSOURCE[8][8] = {"NONE","API","BGNET","NMEA","SIGNALK","MIXED","ACP","NA"};
    if (nSource > 8) {
        strcpy(buffer, cSOURCE[0]);
    } else {
        strcpy(buffer, cSOURCE[nSource]);
    }
    return(buffer);
}

// appears to be not used
//static void ws_remove_client(int fd)
//{
//    pthread_mutex_lock(&ws_mutex);
//    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
//        if (ws_clients[i] == fd) {
//            ws_clients[i] = 0;
//            break;
//        }
//    }
//    pthread_mutex_unlock(&ws_mutex);
//}

static void ws_broadcast_text(const char *msg)
{
    uint8_t header[10];
    size_t len = strlen(msg);
    size_t hlen = 0;

    header[0] = 0x81; // FIN + text
    if (len <= 125) {
        header[1] = (uint8_t)len;
        hlen = 2;
    } else if (len <= 65535) {
        header[1] = 126;
        header[2] = (len >> 8) & 0xFF;
        header[3] = len & 0xFF;
        hlen = 4;
    } else {
        // not expected here
        return;
    }

    pthread_mutex_lock(&ws_mutex);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        int fd = ws_clients[i];
        if (fd > 0) {
            if (send(fd, header, hlen, 0) < 0 ||
                send(fd, msg, len, 0) < 0) {
                // drop on error
                close(fd);
                ws_clients[i] = 0;
            }
        }
    }
    pthread_mutex_unlock(&ws_mutex);
}

/* Base64 + SHA1 for WebSocket accept key */

static char *base64_encode(const unsigned char *input, int length)
{
    BIO *bmem = NULL, *b64 = NULL;
    BUF_MEM *bptr;

    b64 = BIO_new(BIO_f_base64());
    BIO_set_flags(b64, BIO_FLAGS_BASE64_NO_NL);
    bmem = BIO_new(BIO_s_mem());
    b64 = BIO_push(b64, bmem);
    BIO_write(b64, input, length);
    BIO_flush(b64);
    BIO_get_mem_ptr(b64, &bptr);

    char *buff = malloc(bptr->length + 1);
    memcpy(buff, bptr->data, bptr->length);
    buff[bptr->length] = 0;

    BIO_free_all(b64);
    return buff;
}

static int handle_ws_handshake(int client_fd, const char *req)
{
    const char *key_hdr = "Sec-WebSocket-Key:";
    const char *p = strcasestr(req, key_hdr);
    if (!p) return -1;

    p += strlen(key_hdr);
    while (*p == ' ') p++;
    char key[128];
    int i = 0;
    while (*p && *p != '\r' && *p != '\n' && i < (int)sizeof(key)-1) {
        key[i++] = *p++;
    }
    key[i] = '\0';

    const char *guid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    char concat[256];
    snprintf(concat, sizeof(concat), "%s%s", key, guid);

    unsigned char sha[SHA_DIGEST_LENGTH];
    SHA1((unsigned char *)concat, strlen(concat), sha);

    char *accept = base64_encode(sha, SHA_DIGEST_LENGTH);

    char resp[512];
    snprintf(resp, sizeof(resp),
             "HTTP/1.1 101 Switching Protocols\r\n"
             "Upgrade: websocket\r\n"
             "Connection: Upgrade\r\n"
             "Sec-WebSocket-Accept: %s\r\n\r\n",
             accept);

    free(accept);

    if (send(client_fd, resp, strlen(resp), 0) < 0)
        return -1;

    ws_add_client(client_fd);
    log_info("webserver: WebSocket client connected");
    return 0;
}

// to read data via API interface, use 'curl':
// curl http://localhost:8080/api/state
// this returns the most recent received and transmitted serial message
static void handle_api_state(int client_fd)
{
    unsigned char serial_dataRx[BGNET_RXTXRAW_SIZE];
    unsigned char serial_dataTx[BGNET_RXTXRAW_SIZE];
//    char ethernet_data[BGNET_RXTXRAW_SIZE];
    char asciibufRx[BGNET_RXTXRAW_SIZE*2], asciibufTx[BGNET_RXTXRAW_SIZE*2];

    // lock source, copy data
    pthread_mutex_lock(&g_data_mutex);
    memcpy(serial_dataRx, g_last_serial_dataRx, sizeof(serial_dataRx));
    memcpy(serial_dataTx, g_last_serial_dataTx, sizeof(serial_dataRx));
 //   memcpy(ethernet_data, g_last_ethernet_data, sizeof(ethernet_data));
    pthread_mutex_unlock(&g_data_mutex);

    // convert Rx/Tx data to human readable format (haven't checked if the bytes are really a BGNET frame)
    CommandToAscii(asciibufRx, sizeof(asciibufRx), (FRAME_H*)serial_dataRx);
    CommandToAscii(asciibufTx, sizeof(asciibufTx), (FRAME_H*)serial_dataTx);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "SerialDataRx", asciibufRx);
    cJSON_AddStringToObject(root, "SerialDataTx", asciibufTx);
//    cJSON_AddStringToObject(root, "EthernetData", ethernet_data);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (!json) {
        const char *err = "HTTP/1.1 500 Internal Server Error\r\n\r\n";
        send(client_fd, err, strlen(err), 0);
        return;
    }

    char header[256];
    snprintf(header, sizeof(header),
             "HTTP/1.1 200 OK\r\n"
             "Content-Type: application/json\r\n"
             "Content-Length: %zu\r\n\r\n",
             strlen(json));

    send(client_fd, header, strlen(header), 0);
    send(client_fd, json, strlen(json), 0);
    free(json);
}

// to read data via API interface, use 'curl':
// curl http://localhost:8080/api/signalk
// this returns the most recent received signalk message
static void handle_api_signalk(int client_fd)
{
    char delta[1024];

    pthread_mutex_lock(&g_data_mutex);
    snprintf(delta, sizeof(delta), "%s", g_last_signalk_delta);
    pthread_mutex_unlock(&g_data_mutex);

    cJSON *root = cJSON_Parse(delta[0] ? delta : "{}");
    if (!root) root = cJSON_CreateObject();

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    char header[256];
    snprintf(header, sizeof(header),
             "HTTP/1.1 200 OK\r\n"
             "Content-Type: application/json\r\n"
             "Content-Length: %zu\r\n\r\n",
             strlen(json));

    send(client_fd, header, strlen(header), 0);
    send(client_fd, json, strlen(json), 0);
    free(json);
}

// to send data via API interface, use 'curl':
// curl --data "{datastring}" http://localhost:8080/api/update
// where {json} is like:
//  {\"depth\":15.6}
// good result returns "OK"
static void handle_api_update(int client_fd, const char *body)
{
    time_t t;
    cJSON *root = cJSON_Parse(body);

    // obtain timestamp value
    time(&t);

    if (!root) {
        const char *err = "HTTP/1.1 400 Bad Request\r\n\r\n";
        send(client_fd, err, strlen(err), 0);
        return;
    }

    log_debug("webserver: api_update: %s", body);

    // GPS information
    cJSON *latitude  = cJSON_GetObjectItem(root, "latitude");  // lattitude +=North of equator, -=South of equator
    cJSON *longitude = cJSON_GetObjectItem(root, "longitude"); // longitude +=East of prime meridian, -=West of prime meridian
    cJSON *altitude  = cJSON_GetObjectItem(root, "altitude");  // altitude [M], 0=sealevel, +=airplane, -=submarine
    cJSON *cog       = cJSON_GetObjectItem(root, "cog");       // course over ground [degree] (0-359)
    cJSON *sog       = cJSON_GetObjectItem(root, "sog");       // speed over ground [M/S]
    // boat measured parameters
    cJSON *stw       = cJSON_GetObjectItem(root, "stw");       // speed through water [M/S] (boatspeed)
    cJSON *depth     = cJSON_GetObjectItem(root, "depth");     // water depth below transducer [M]
    cJSON *seatemp   = cJSON_GetObjectItem(root, "seatemp");   // temperature of seawater [degree C]
    cJSON *rudder    = cJSON_GetObjectItem(root, "rudder");    // rudder angle sensor [radian], -Pi/+Pi (equals to -180/+180)
    cJSON *heading   = cJSON_GetObjectItem(root, "heading");   // heading [degree] (0-359, compass)
    // wind measured parameters
    cJSON *twa       = cJSON_GetObjectItem(root, "twa");       // True Wind Angle [degree] (-180/+180, derived from SOG, COG and AWA)
    cJSON *tws       = cJSON_GetObjectItem(root, "tws");       // True Wind Speed [M/S] (derived from SOG, COG and AWS)
    cJSON *awa       = cJSON_GetObjectItem(root, "awa");       // Appearant Wind Angle [degree] (-180/+180, measured by masttop sensor)
    cJSON *aws       = cJSON_GetObjectItem(root, "aws");       // Appearent Wind Speed [M/S] (measured by masttop sensor)
    cJSON *airtemp   = cJSON_GetObjectItem(root, "airtemp");   // temperature of air [degree C]
    cJSON *baro      = cJSON_GetObjectItem(root, "baro");      // barometric airpressure [mbar]
    // pilot/trip parameters
    cJSON *cts       = cJSON_GetObjectItem(root, "cts");       // Course To Steer (0-359, compass)
    cJSON *triplog   = cJSON_GetObjectItem(root, "triplog");   // triplog, distance covered [M]
    cJSON *vmg       = cJSON_GetObjectItem(root, "vmg");       // Velocity Made Good, calculated from SOG and angle to the target

    pthread_mutex_lock(&g_data_mutex);

    // position information, usually from GPS
    if (latitude) {
        g_n2kdata.pos.lat.value.dValue = latitude->valuedouble;
        g_n2kdata.pos.lat.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.pos.lat.Counter, t);
    } 
    if (longitude) {
        g_n2kdata.pos.lon.value.dValue = longitude->valuedouble;
        g_n2kdata.pos.lon.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.pos.lon.Counter, t);
    }
    if (altitude) {
        g_n2kdata.pos.alt.value.dValue = altitude->valuedouble;
        g_n2kdata.pos.alt.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.pos.alt.Counter, t);
    }
    if (cog) {
        g_n2kdata.pos.cog.value.dValue = cog->valuedouble;
        g_n2kdata.pos.cog.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.pos.cog.Counter, t);
    }
    if (sog) {
        g_n2kdata.pos.sog.value.dValue = sog->valuedouble;
        g_n2kdata.pos.sog.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.pos.sog.Counter, t);
    }

    // boat information from instruments onboard
    if (stw) {
        g_n2kdata.boat.stw.value.dValue = stw->valuedouble;
        g_n2kdata.boat.stw.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.boat.stw.Counter, t);
    }
    if (depth) {
        g_n2kdata.boat.depth.value.dValue = depth->valuedouble;
        g_n2kdata.boat.depth.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.boat.depth.Counter, t);
    }
    if (seatemp) {
        g_n2kdata.boat.seatemp.value.dValue = seatemp->valuedouble;
        g_n2kdata.boat.seatemp.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.boat.seatemp.Counter, t);
    }
    if (rudder) {
        g_n2kdata.boat.rudder.value.dValue = rudder->valuedouble;
        g_n2kdata.boat.rudder.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.boat.rudder.Counter, t);
    }
    if (heading) {
        g_n2kdata.boat.heading.value.dValue = heading->valuedouble;
        g_n2kdata.boat.heading.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.boat.heading.Counter, t);
    }

    // wind information from masttop sensor
    if (twa) {
        g_n2kdata.wind.twa.value.dValue = twa->valuedouble;
        g_n2kdata.wind.twa.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.wind.twa.Counter, t);
    }
    if (tws) {
        g_n2kdata.wind.tws.value.dValue = tws->valuedouble;
        g_n2kdata.wind.tws.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.wind.tws.Counter, t);
    }
    if (awa) {
        g_n2kdata.wind.awa.value.dValue = awa->valuedouble;
        g_n2kdata.wind.awa.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.wind.awa.Counter, t);
    }
    if (aws) {
        g_n2kdata.wind.aws.value.dValue = aws->valuedouble;
        g_n2kdata.wind.aws.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.wind.aws.Counter, t);
    }
    if (airtemp) {
        g_n2kdata.wind.airtemp.value.dValue = airtemp->valuedouble;
        g_n2kdata.wind.airtemp.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.wind.airtemp.Counter, t);
    }
    if (baro) {
        g_n2kdata.wind.baro.value.dValue = baro->valuedouble;
        g_n2kdata.wind.baro.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.wind.baro.Counter, t);
    }

    // pilot information from navigation systems, autopilot, trip planners
    if (cts) {
        g_n2kdata.pilot.cts.value.dValue = cts->valuedouble;
        g_n2kdata.pilot.cts.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.pilot.cts.Counter, t);
    }
    if (triplog) {
        g_n2kdata.pilot.triplog.value.dValue = triplog->valuedouble;
        g_n2kdata.pilot.triplog.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.pilot.triplog.Counter, t);
    }
    if (vmg) {
        g_n2kdata.pilot.vmg.value.dValue = vmg->valuedouble;
        g_n2kdata.pilot.vmg.source = BGN_SOURCE_API;
        CounterUp(&g_n2kdata.pilot.vmg.Counter, t);
    }

    pthread_mutex_unlock(&g_data_mutex);

    cJSON_Delete(root);

    const char *ok = "HTTP/1.1 200 OK\r\n\r\n";
    send(client_fd, ok, strlen(ok), 0);
}

static char* html_head(char *Buffer, size_t MaxBuf) {
    const char *cHead =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n\r\n"
        "<html>\r\n"
        "<head>"
            "<style>body, table, td {font-family: Verdana; font-size: 14px;}\r\nh3 {margin-left: 20px;}\r\ntable {border-collapse: collapse; width: 95%; margin: 5px auto;}\r\nth, td {border:1px solid #ccc; padding: 5px; text-align: left;}</style>"
            "<title>%s</title>"
            "<link rel='shortcut icon' href='#' />"
            "</head>\r\n"
        "<body>\r\n";
    snprintf(Buffer, MaxBuf, cHead, ApplicationName);
    return(Buffer);
}

static char* html_tail(char *Buffer, size_t MaxBuf) {
    const char *cTail =
        "</body>%s</html>\r\n";
    snprintf(Buffer, MaxBuf, cTail, "");
    return(Buffer);
}

static char* html_title(char *Buffer, size_t MaxBuf) {
    const char *cTitle =
        "<table border='0px';><tr><th><a href='http://localhost:8080/'>Home</a></th><th><a href='http://localhost:8080/bgnet/'>Statistics Report</a></th><th>%s</th</tr></table>"
        "<h1>%s</h1>";
    char cTime[256];
    time_t t;
    struct tm* tm_info;
    time (&t);
    tm_info = localtime(&t);
    strftime(cTime, sizeof(cTime),"%Y-%m-%d %H:%M:%S", tm_info);
    snprintf(Buffer, MaxBuf, cTitle, cTime, ApplicationName);
    return(Buffer);
}

static char* html_help(char *Buffer, size_t MaxBuf, int iPort) {
    const char *cHelp =
        "<h2>Help</h2>"
        "<p>REST:</p>"
        "<ul>"
        "<li>GET /api/state</li>"
        "<li>GET /api/signalk</li>"
        "<li>POST /api/update</li>"
        "</ul>"
        "<p>WebSocket: ws://host:%d/ws</p>"
        "<p>for configuration parameters check file 'config.json'</p>";
    snprintf(Buffer, MaxBuf, cHelp, iPort);
    return(Buffer);
}

static char* html_interface_top(char *Buffer) {
    const char *cDevTop =
        "<h2>B&G Network Statistics</h2>"
        "<h3>Serial Interface</h3>"
        "<table>"
        "<tr><th rowspan='2'>Interface</th><th rowspan='2'>Communication Parameters</th><th rowspan='2'>Service</th><th colspan='2'>Device</th></tr>"
        "<tr><th>Node</th><th>Type</th></tr>";
    strcpy(Buffer, cDevTop);
    return(Buffer);
}
static char* html_interface_row(char *Buffer, BGNET_CTRL *pCtrl) {
    char cLine[512];
    const char *cInfRow =
        "<tr><td>%s</td><td>Baudrate: %d, Parity: %s, Databits: %d, Stopbits: %d</td><td>%s</td><td>0x%02X</td><td>%s</td></tr>";
    snprintf(cLine, sizeof(cLine), cInfRow,
                                            pCtrl->pCfg->serial_port,
                                            pCtrl->pCfg->baudrate,
                                            pCtrl->pCfg->parity,
                                            pCtrl->pCfg->databits,
                                            pCtrl->pCfg->stopbits,
                                            pCtrl->pCfg->mode,
                                            BGNET_ADDRESS_ME,
                                            pCtrl->pDEVDT[bgnet_deviceindex(pCtrl, BGNET_ADDRESS_ME)].sProdName
                                        );
    strcpy(Buffer, cLine);
    return(Buffer);
}
static char* html_interface_end(char *Buffer) {
    const char *cInfEnd =
        "</table>";
    strcpy(Buffer, cInfEnd);
    return(Buffer);
}

static char* html_device_top(char *Buffer) {
    const char *cDevTop =
        "<h3>External Devices</h3>"
        "<table>"
        "<tr><th colspan='2'>Device</th><th colspan='5'>Rx (input FROM interface)</th><th colspan='3'>Tx (output TO interface)</th></tr>"
        "<tr><th>Node</th><th>Type</th><th>Count</th><th>Sec</th><th>LastRaw</th><th>Err/Head</th><th>Err/Frame</th><th>Count</th><th>Sec</th><th>LastRaw</th></tr>";
    strcpy(Buffer, cDevTop);
    return(Buffer);
}

static char* html_device_row(char *Buffer, BGNET_CTRL *pCtrl) {
    int i;
    time_t t;
    char cLine[256], asciibufRx[BGN_CMD_ASCII], asciibufTx[BGN_CMD_ASCII];
    const char *cDevRow =
        "<tr %s><td>0x%02X</td><td>%s</td><td>%d</td><td>%d</td><td>%s</td><td>%d</td><td>%d</td><td>%d</td><td>%d</td><td>%s</td></tr>";

    strcpy(cLine, "");
    strcpy(Buffer, "");
    time(&t);
    for (i=0; i<BGNET_DEVICE_MAX; i++) {
        if ((pCtrl->DEV[i].CounterRx.iCounter < 1) & (pCtrl->DEV[i].CounterTx.iCounter < 1)) {
            // it was never touched
        } else {
            CommandToAscii(asciibufRx, sizeof(asciibufRx), &pCtrl->DEV[i].Rx.FH);
            CommandToAscii(asciibufTx, sizeof(asciibufTx), &pCtrl->DEV[i].Tx.FH);
            snprintf(cLine, sizeof(cLine), cDevRow,
                        ((pCtrl->DEV[i].source == BGN_SOURCE_BGNET)?"":"bgcolor='#dbdbdb'"),
                        i,
                        pCtrl->pDEVDT[bgnet_deviceindex(pCtrl, i)].sProdName,
                        pCtrl->DEV[i].CounterRx.iCounter,
                        (t - pCtrl->DEV[i].CounterRx.tTimestamp),
                        asciibufRx,
                        pCtrl->DEV[i].iCSError_Header, 
                        pCtrl->DEV[i].iCSError_Frame,
                        pCtrl->DEV[i].CounterTx.iCounter,
                        (t - pCtrl->DEV[i].CounterTx.tTimestamp),
                        asciibufTx
                    );
            strcat(Buffer, cLine);
        }
    }
    return(Buffer);
}
static char* html_device_end(char *Buffer) {
    const char *cDevEnd =
        "</table>";
    strcpy(Buffer, cDevEnd);
    return(Buffer);
}

static char* html_ACP_top(char *Buffer) {
    const char *cACPTop =
        "<h3>ACP status</h3>"
        "<table>"
        "<tr><th rowspan='2'>Parameter</th><th rowspan='2'>Nomenclature</th><th colspan='3'>Display</th></tr>"
        "<tr><th>Sec</th><th>Source</th><th>Value</th></tr>";
    strcpy(Buffer, cACPTop);
    return(Buffer);
}
static char* html_ACP_row(char *Buffer, BGNET_CTRL *pCtrl) {
    char cLine[512];
    char cSource[32];
    time_t t;
    struct tm * ptmInfo;
    char asciibuffer[256];
    // some variables have different datatypes, so we use different format specifiers
    const char *cACPRowSSDSS = "<tr><td>%s</td><td>%s</td><td>%d</td><td>%s</td><td>%s</td></tr>";
    const char *cACPRowSSDSD = "<tr><td>%s</td><td>%s</td><td>%d</td><td>%s</td><td>%d</td></tr>";
    const char *cACPRowSSDSX = "<tr><td>%s</td><td>%s</td><td>%d</td><td>%s</td><td>0x%02X</td></tr>";
    const char *cACPRowSSDSF = "<tr><td>%s</td><td>%s</td><td>%d</td><td>%s</td><td>%.1f</td></tr>";

    strcpy(cLine, "");
    strcpy(Buffer, "");
    time(&t);


    ptmInfo = localtime (&pCtrl->ACP.tTimestamp);
    strftime(asciibuffer, sizeof(asciibuffer), "%Y-%m-%d %H:%M:%S", ptmInfo);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSS, "ACP: Running since", "Seconds since start", t-pCtrl->ACP.tTimestamp, strSource(cSource, BGN_SOURCE_NONE), asciibuffer);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSD, "ACP: PILOT WATCHDOG", "Countdown by timer, reset by PILOT HELLO", t-pCtrl->ACP.PILOT.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.PILOT.source), pCtrl->ACP.PILOT.value.nLong);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSD, "ACP: Joystick", "[0=OFF, 1=ON]", t-pCtrl->ACP.Joystick.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.Joystick.source), pCtrl->ACP.Joystick.value.nByte);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSX, "ACP: AP STATE", "[0=STBY, 1=AUTO]", t-pCtrl->ACP.STATE.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.STATE.source), pCtrl->ACP.STATE.value.nByte);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSX, "ACP: AP MODE", "[1=COMP, 3=NAV, 4=VANE]", t-pCtrl->ACP.MODE.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.MODE.source), pCtrl->ACP.MODE.value.nByte);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSD, "PILOT/SETUP: Speed Select", "[0=TRAnsmitter, 1=MANual, 2=SOG]", t-pCtrl->ACP.SETUP_SPD.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.SETUP_SPD.source), pCtrl->ACP.SETUP_SPD.value.nByte);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSD, "PILOT/SETUP: Steering Response", "[0=Normal, 1=Economic, 2=Downwind]", t-pCtrl->ACP.SETUP_RESP.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.SETUP_RESP.source), pCtrl->ACP.SETUP_RESP.value.nByte);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSD, "PILOT/SETUP: WATCH Alarm", "[1-360 Minutes]", t-pCtrl->ACP.SETUP_WA_MIN.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.SETUP_WA_MIN.source), pCtrl->ACP.SETUP_WA_MIN.value.nLong);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSD, "PILOT/SETUP: WATCH Alarm ON/OFF", "[0=OFF, 1=ON]", t-pCtrl->ACP.SETUP_WA_STAT.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.SETUP_WA_STAT.source), pCtrl->ACP.SETUP_WA_STAT.value.nLong);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSD, "PILOT/SETUP: OFF-COURSE Alarm", "[1-99 Degrees]", t-pCtrl->ACP.SETUP_OC_DEG.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.SETUP_OC_DEG.source), pCtrl->ACP.SETUP_OC_DEG.value.nLong);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSD, "PILOT/SETUP: OFF-COURSE Alarm ON/OFF", "[0=OFF, 1=ON]", t-pCtrl->ACP.SETUP_OC_STAT.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.SETUP_OC_STAT.source), pCtrl->ACP.SETUP_OC_STAT.value.nLong);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSD, "PILOT/SETUP: Compass Damping", "[1-99 sec, 4=default]", t-pCtrl->ACP.SETUP_DMP.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.SETUP_DMP.source), pCtrl->ACP.SETUP_DMP.value.nByte);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSD, "PILOT/SETUP: Joystick Steering", "[NORMal, PROPortional]", t-pCtrl->ACP.SETUP_JSM.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.SETUP_JSM.source), pCtrl->ACP.SETUP_JSM.value.nByte);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSD, "PILOT/SETUP: Fault Reporting Disable", "[0=OFF/NOT Disabled, 1=ON/Disabled]", t-pCtrl->ACP.SETUP_FR.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.SETUP_FR.source), pCtrl->ACP.SETUP_FR.value.nByte);
    strcat(Buffer, cLine);

    snprintf(cLine, sizeof(cLine), cACPRowSSDSD, "PILOT/SPEED: Manual Speed select", "[FST/MED/SLW]", t-pCtrl->ACP.MANSPEED_SEL.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.MANSPEED_SEL.source), pCtrl->ACP.MANSPEED_SEL.value.nByte);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSD, "PILOT/SPEED: Manual Speed FST", "[1-990] x10", t-pCtrl->ACP.MANSPEED_FST.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.MANSPEED_FST.source), pCtrl->ACP.MANSPEED_FST.value.nLong);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSD, "PILOT/SPEED: Manual Speed MED", "[1-990] x10", t-pCtrl->ACP.MANSPEED_MED.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.MANSPEED_MED.source), pCtrl->ACP.MANSPEED_MED.value.nLong);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSD, "PILOT/SPEED: Manual Speed SLW", "[1-990] x10", t-pCtrl->ACP.MANSPEED_FST.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.MANSPEED_SLW.source), pCtrl->ACP.MANSPEED_SLW.value.nLong);
    strcat(Buffer, cLine);

    snprintf(cLine, sizeof(cLine), cACPRowSSDSD, "NETWORK: Backlight", "[0=OFF, 3=HIGH, 2=MEDIUM, 1=LOW]", t-pCtrl->ACP.BackLight.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.BackLight.source), pCtrl->ACP.BackLight.value.nByte);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSD, "NETWORK: Course Target", "Autopilot Course setpoint (0-359 Degrees M)", t-pCtrl->ACP.CourseTarget.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.CourseTarget.source), pCtrl->ACP.CourseTarget.value.nLong);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSD, "NETWORK: Compass Heading", "Current compass reading (0-359 Degrees M)", t-pCtrl->ACP.Heading.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.Heading.source), pCtrl->ACP.Heading.value.nLong);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSF, "NETWORK: STORLOG", "Stored log (QUAD or SPEED), stored in EEPROM, no reset) [NM]", t-pCtrl->ACP.StoredLogNM.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.StoredLogNM.source), (ITOD(pCtrl->ACP.StoredLogNM.value.nLong)/BGNET_DIVISOR_STORLOG));
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowSSDSF, "NETWORK: TRIPLOG", "Trip log (QUAD or SPEED), stored in EEPROM, with reset) [NM]", t-pCtrl->ACP.TripLogNM.Counter.tTimestamp, strSource(cSource, pCtrl->ACP.TripLogNM.source), (ITOD(pCtrl->ACP.TripLogNM.value.nLong)/BGNET_DIVISOR_TRIPLOG));
    strcat(Buffer, cLine);

    return(Buffer);
}
static char* html_ACP_end(char *Buffer) { 
    const char *cACPEnd =
        "</table>";
    strcpy(Buffer, cACPEnd);
    return(Buffer);
}

static char* html_NMEA_top(char *Buffer) {
    const char *cNMEATop =
        "<h3>SignalK/NMEA variables</h3>"
        "<table>"
        "<tr><th>Parameter</th><th>Count</th><th>Sec</th><th>Source</th><th>Value</th></tr>";
    strcpy(Buffer, cNMEATop);
    return(Buffer);
}
static char* html_NMEA_row(char *Buffer) {
    char cLine[512];
    char cSource[32];

    time_t t;
    const char *cNMEARow = "<tr><td>%s</td><td>%d</td><td>%d</td><td>%s</td><td>%.2f</td></tr>";
 
    strcpy(cLine, "");
    strcpy(Buffer, "");
    time(&t);

    snprintf(cLine, sizeof(cLine), cNMEARow, "LATITUDE", g_n2kdata.pos.lat.Counter.iCounter, (t - g_n2kdata.pos.lat.Counter.tTimestamp), strSource(cSource, g_n2kdata.pos.lat.source), g_n2kdata.pos.lat.value.dValue);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cNMEARow, "LONGITUDE", g_n2kdata.pos.lon.Counter.iCounter, (t - g_n2kdata.pos.lon.Counter.tTimestamp), strSource(cSource, g_n2kdata.pos.lon.source), g_n2kdata.pos.lon.value.dValue);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cNMEARow, "ALTITUDE", g_n2kdata.pos.alt.Counter.iCounter, (t - g_n2kdata.pos.alt.Counter.tTimestamp), strSource(cSource, g_n2kdata.pos.alt.source), g_n2kdata.pos.alt.value.dValue);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cNMEARow, "COG", g_n2kdata.pos.cog.Counter.iCounter, (t - g_n2kdata.pos.cog.Counter.tTimestamp), strSource(cSource, g_n2kdata.pos.cog.source), g_n2kdata.pos.cog.value.dValue);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cNMEARow, "SOG", g_n2kdata.pos.sog.Counter.iCounter, (t - g_n2kdata.pos.sog.Counter.tTimestamp), strSource(cSource, g_n2kdata.pos.sog.source), g_n2kdata.pos.sog.value.dValue);
    strcat(Buffer, cLine);

    snprintf(cLine, sizeof(cLine), cNMEARow, "STW", g_n2kdata.boat.stw.Counter.iCounter, (t - g_n2kdata.boat.stw.Counter.tTimestamp), strSource(cSource, g_n2kdata.boat.stw.source), g_n2kdata.boat.stw.value.dValue);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cNMEARow, "DEPTH", g_n2kdata.boat.depth.Counter.iCounter, (t - g_n2kdata.boat.depth.Counter.tTimestamp), strSource(cSource, g_n2kdata.boat.depth.source), g_n2kdata.boat.depth.value.dValue);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cNMEARow, "SEAWATERTEMP", g_n2kdata.boat.seatemp.Counter.iCounter, (t - g_n2kdata.boat.seatemp.Counter.tTimestamp), strSource(cSource, g_n2kdata.boat.seatemp.source), g_n2kdata.boat.seatemp.value.dValue);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cNMEARow, "RUDDER", g_n2kdata.boat.rudder.Counter.iCounter, (t - g_n2kdata.boat.rudder.Counter.tTimestamp), strSource(cSource, g_n2kdata.boat.rudder.source), g_n2kdata.boat.rudder.value.dValue);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cNMEARow, "HEADING", g_n2kdata.boat.heading.Counter.iCounter, (t - g_n2kdata.boat.heading.Counter.tTimestamp), strSource(cSource, g_n2kdata.boat.heading.source), g_n2kdata.boat.heading.value.dValue);
    strcat(Buffer, cLine);

    return(Buffer);
}
static char* html_NMEA_end(char *Buffer) { 
    const char *cNMEAEnd =
        "</table>";
    strcpy(Buffer, cNMEAEnd);
    return(Buffer);
}

static char* html_command_top(char *Buffer) {
    const char *cDevTop =
        "<h3>Commands</h3>"
        "<table>"
        "<tr><th colspan='2'>Command</th><th colspan='3'>Rx (input FROM interface)</th><th colspan='3'>Tx (output TO interface)</th></tr>"
        "<tr><th>ID</th><th>Label</th><th>Count</th><th>Sec</th><th>LastRaw</th><th>Count</th><th>Sec</th><th>LastRaw</th></tr>";
    strcpy(Buffer, cDevTop);
    return(Buffer);
}
static char* html_command_row(char *Buffer, BGNET_CTRL *pCtrl) {
    int i;
    time_t t;
    char cLine[256], asciibufRx[BGN_CMD_ASCII], asciibufTx[BGN_CMD_ASCII], asciibufCMD[BGNET_CMD_LABEL];
    const char *cDevRow =
        "<tr><td>0x%02X</td><td>%s</td><td>%d</td><td>%d</td><td>%s</td><td>%d</td><td>%d</td><td>%s</td></tr>";

    strcpy(cLine, "");
    strcpy(Buffer, "");
    time(&t);
    for (i=0; i<BGNET_COMMAND_MAX; i++) {
        if ((pCtrl->CMD[i].CounterRx.iCounter < 1) & (pCtrl->CMD[i].CounterTx.iCounter < 1)) {
            // it was never touched
        } else {
            CommandToAscii(asciibufRx, sizeof(asciibufRx), &pCtrl->CMD[i].Rx.FH);
            CommandToAscii(asciibufTx, sizeof(asciibufTx), &pCtrl->CMD[i].Tx.FH);
            snprintf(cLine, sizeof(cLine), cDevRow,
                                                    i,
                                                    strCommandLabel(pCtrl, asciibufCMD, i),
                                                    pCtrl->CMD[i].CounterRx.iCounter,
                                                    t - pCtrl->CMD[i].CounterRx.tTimestamp,
                                                    asciibufRx,
                                                    pCtrl->CMD[i].CounterTx.iCounter,
                                                    t - pCtrl->CMD[i].CounterTx.tTimestamp,
                                                    asciibufTx
                                                );
            strcat(Buffer, cLine);
        }
    }
    return(Buffer);
}
static char* html_command_end(char *Buffer) {
    const char *cDevTop =
        "</table>";
    strcpy(Buffer, cDevTop);
    return(Buffer);
}

static char* html_channel_top(char *Buffer, BGNET_CTRL *pCtrl) {
    char cLine[1024], asciibuf01[24], asciibuf02[24], asciibuf06[24];
    const char *cChnTop =
        "\r<h3>Channel Information</h3>"
        "\r<table>"
        "<tr><th colspan='2'>Channel</th>"
            "<th colspan='5'>Command 0x01/%s</th>"
            "<th colspan='4'>Command 0x02/%s</th>"
            "<th colspan='3'>Command 0x06/%s</th>"
            "<th colspan='7'>Various Commands/BUTTONS, ALARMS, RESETS</th>"
            "</tr>"
        "<tr><th>ID</th><th>Label</th>"
            "<th>Count</th><th>Sec</th><th>LastRaw</th><th>Node</th><th>Display</th>"
            "<th>Count</th><th>Sec</th><th>LastRaw</th><th>Label</th>"
            "<th>Count</th><th>Sec</th><th>LastRaw</th>"
            "<th>Count</th><th>Sec</th><th>LastRaw</th><th>TO</th><th>FROM</th><th>CMD</th><th>Display</th>"
            "</tr>";
    snprintf(cLine, sizeof(cLine), cChnTop, strCommandLabel(pCtrl, asciibuf01, BGNET_FRAME_CMD_01_DATA), strCommandLabel(pCtrl, asciibuf02, BGNET_FRAME_CMD_02_LABEL), strCommandLabel(pCtrl, asciibuf06, BGNET_FRAME_CMD_06_SUBSCRIBE));
    strcpy(Buffer, cLine);
    return(Buffer);
}
static char* html_channel_row(char *Buffer, BGNET_CTRL *pCtrl) {
    int i,y, InUse;
    time_t t;
    char cLine[2048], bufferCHN[BGNET_CMD_LABEL];
    char cLine01[1024], cLine02[1024], cLine06[1024], cLine00[1024];
    char asciibuf01[256], asciibuf02[256], asciibuf06[256], asciibuf00[256];

    const char *cDevRow_Start = "\r<tr %s><td>0x%02X</td><td>%s</td>";
    const char *cDevRow_End = "</tr>";

    const char *cDevRow01 = "<td>%d</td><td>%d</td><td>%s</td><td>0x%02X</td><td>%s</td>";
    const char *cDevRow02 = "<td>%d</td><td>%d</td><td>%s</td><td>%s</td>";
    const char *cDevRow06 = "<td>%d</td><td>%d</td><td>%s</td>";
    const char *cDevRow00 = "<td>%d</td><td>%d</td><td>%s</td><td>0x%02X</td><td>0x%02X</td><td>0x%02X</td><td>0x%s</td>";

    const char *cDevRow01E = "<td></td><td></td><td></td><td></td><td></td>";
    const char *cDevRow02E = "<td></td><td></td><td></td><td></td>";
    const char *cDevRow06E = "<td></td><td></td><td></td>";
    const char *cDevRow00E = "<td></td><td></td><td></td><td></td><td></td><td></td><td></td>";

    strcpy(cLine01, "");
    strcpy(cLine02, "");
    strcpy(cLine06, "");
    strcpy(cLine00, "");
    strcpy(Buffer, "");
    time(&t);

    for (i=0; i<BGNET_CHANNEL_MAX; i++) {
        InUse = 0;
        if ((pCtrl->CHN[i].x01.CounterRx.iCounter < 1)) {
            strcpy(cLine01, cDevRow01E);
        } else {
            InUse++;
            y = bgnet_sizeof_channel(pCtrl, (union CH*)pCtrl->CHN[i].x01.ucRAW);
            byte2ascii(asciibuf01, sizeof(asciibuf01), pCtrl->CHN[i].x01.ucRAW, y, BYTE2ASCII_SPACED);
            snprintf(cLine01, sizeof(cLine01), cDevRow01,
                                                    pCtrl->CHN[i].x01.CounterRx.iCounter,
                                                    (t - pCtrl->CHN[i].x01.CounterRx.tTimestamp),
                                                    asciibuf01,
                                                    pCtrl->CHN[i].x01.ucFROM,
                                                    pCtrl->CHN[i].x01.cValue);
        }
        if ((pCtrl->CHN[i].x02.CounterRx.iCounter < 1)) {
            strcpy(cLine02, cDevRow02E);
        } else {
            InUse++;
            y = BGNET_FRAME_HEAD + 4; // only show frame + 4 bytes payload
            byte2ascii(asciibuf02, sizeof(asciibuf02), pCtrl->CHN[i].x02.ucRAW, y, BYTE2ASCII_SPACED);
            snprintf(cLine02, sizeof(cLine02), cDevRow02,
                                                    pCtrl->CHN[i].x02.CounterRx.iCounter,
                                                    (t - pCtrl->CHN[i].x02.CounterRx.tTimestamp),
                                                    asciibuf02,
                                                    pCtrl->CHN[i].x02.cValue);
        }
        if (pCtrl->CHN[i].x06.CounterRx.iCounter < 1) {
            strcpy(cLine06, cDevRow06E);
        } else {
            InUse++;
            y = BGNET_FRAME_HEAD + 4; // only show frame + 4 bytes payload
            byte2ascii(asciibuf06, sizeof(asciibuf06), pCtrl->CHN[i].x06.ucRAW, y, BYTE2ASCII_SPACED);
            snprintf(cLine06, sizeof(cLine06), cDevRow06,
                                                    pCtrl->CHN[i].x06.CounterRx.iCounter,
                                                    (t - pCtrl->CHN[i].x06.CounterRx.tTimestamp),
                                                    asciibuf06,
                                                    pCtrl->CHN[i].x06.ucTO,
                                                    pCtrl->CHN[i].x06.ucFROM);
        }
        if (pCtrl->CHN[i].x00.CounterRx.iCounter < 1) {
            strcpy(cLine00, cDevRow00E);
        } else {
            InUse++;
            y = bgnet_sizeof_channel(pCtrl, (union CH*)pCtrl->CHN[i].x00.ucRAW);
            byte2ascii(asciibuf00, sizeof(asciibuf00), pCtrl->CHN[i].x00.ucRAW, y, BYTE2ASCII_SPACED);
            snprintf(cLine00, sizeof(cLine00), cDevRow00,
                                                    pCtrl->CHN[i].x00.CounterRx.iCounter,
                                                    (t - pCtrl->CHN[i].x00.CounterRx.tTimestamp),
                                                    asciibuf00,
                                                    pCtrl->CHN[i].x00.ucTO,
                                                    pCtrl->CHN[i].x00.ucFROM,
                                                    pCtrl->CHN[i].x00.ucCMD,
                                                    pCtrl->CHN[i].x00.cValue);
        }
        if (InUse>0) {
            snprintf(cLine, sizeof(cLine), cDevRow_Start,
                                                    ((pCtrl->CHN[i].iNewChannel==BGN_NEW_RESET)?"":"bgcolor='lightblue'"),
                                                    i,
                                                    strChannelLabel(pCtrl, bufferCHN, i));
            strcat(cLine, cLine01);
            strcat(cLine, cLine02);
            strcat(cLine, cLine06);
            strcat(cLine, cLine00);
            strcat(cLine, cDevRow_End);
        
            strcat(Buffer, cLine);  // warning: this could overflow the buffer; you might wanna check the pagesize
            //log_debug("webserver/channel: strlen(Buffer)= %d, strlen(cLine)=%d", strlen(Buffer), strlen(cLine));
        }
    }
    return(Buffer);
}
static char* html_channel_end(char *Buffer) {
    const char *cDevTop =
        "</table>";
    strcpy(Buffer, cDevTop);
    return(Buffer);
}

static void handle_root_html(int client_fd, BGNET_CTRL *pCtrl)
{
    char htmlbuffer[1024], workbuffer[1024];

    strcpy(htmlbuffer, ""); // initialize the empty buffer

    strcat(htmlbuffer, html_head(workbuffer, sizeof(workbuffer)));
    strcat(htmlbuffer, html_title(workbuffer, sizeof(workbuffer)));

    strcat(htmlbuffer, html_help(workbuffer, sizeof(workbuffer), pCtrl->pCfg->web_port));

    strcat(htmlbuffer, html_tail(workbuffer, sizeof(workbuffer)));
    //log_debug("webserver: / page size: %d, %d", strlen(htmlbuffer), strlen(workbuffer));
    send(client_fd, htmlbuffer, strlen(htmlbuffer), 0);
}

// warning: if you make changes to the report, realise that output might get larger, overflowing the buffers
// 
static void handle_bgnet_html(int client_fd, BGNET_CTRL *pCtrl)
{
    char htmlbuffer[16384], workbuffer[16384];

    strcpy(htmlbuffer, ""); // initialize the empty buffer

    strcat(htmlbuffer, html_head(workbuffer, sizeof(workbuffer)));
    strcat(htmlbuffer, html_title(workbuffer, sizeof(workbuffer)));

    strcat(htmlbuffer, html_interface_top(workbuffer));
    strcat(htmlbuffer, html_interface_row(workbuffer, pCtrl));
    strcat(htmlbuffer, html_interface_end(workbuffer));

    send(client_fd, htmlbuffer, strlen(htmlbuffer), 0);
    strcpy(htmlbuffer, ""); // initialize the empty buffer

    strcat(htmlbuffer, html_device_top(workbuffer));
    strcat(htmlbuffer, html_device_row(workbuffer, pCtrl));
    strcat(htmlbuffer, html_device_end(workbuffer));


    strcat(htmlbuffer, html_ACP_top(workbuffer));
    strcat(htmlbuffer, html_ACP_row(workbuffer, pCtrl));
    strcat(htmlbuffer, html_ACP_end(workbuffer));

    strcat(htmlbuffer, html_NMEA_top(workbuffer));
    strcat(htmlbuffer, html_NMEA_row(workbuffer));
    strcat(htmlbuffer, html_NMEA_end(workbuffer));

    strcat(htmlbuffer, html_command_top(workbuffer));
    strcat(htmlbuffer, html_command_row(workbuffer, pCtrl));
    strcat(htmlbuffer, html_command_end(workbuffer));


    send(client_fd, htmlbuffer, strlen(htmlbuffer), 0);
    strcpy(htmlbuffer, ""); // initialize the empty buffer

    strcat(htmlbuffer, html_channel_top(workbuffer, pCtrl));
    strcat(htmlbuffer, html_channel_row(workbuffer, pCtrl));
    strcat(htmlbuffer, html_channel_end(workbuffer));

    strcat(htmlbuffer, html_tail(workbuffer, sizeof(workbuffer)));

    log_debug("webserver: /bgnet page size: %d, %d", strlen(htmlbuffer), strlen(workbuffer));
    send(client_fd, htmlbuffer, strlen(htmlbuffer), 0);

}

void *webserver_thread(void *arg)
{
    BGNET_CTRL *pCtrl= (BGNET_CTRL *)arg;
    Config *cfg = pCtrl->pCfg;

    // warning: server should be in non-blocking mode SOCK_NONBLOCK
    // at present, the thread will not close when the application exits

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        log_error("webserver: Web socket: %s", strerror(errno));
        return NULL;
    }
    // make listerner socket non-blocking
    int flags = fcntl(server_fd, F_GETFL, 0);
    fcntl(server_fd, F_SETFL, flags | O_NONBLOCK);

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(cfg->web_port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        log_error("webserver: Web bind: %s", strerror(errno));
        close(server_fd);
        return NULL;
    }

    if (listen(server_fd, 5) < 0) {
        log_error("webserver: Web listen: %s", strerror(errno));
        close(server_fd);
        return NULL;
    }

    log_info("webserver: Web listening on http://localhost:%d", cfg->web_port);

    while (g_running) {
        int client_fd = accept(server_fd, NULL, NULL);
        if (client_fd < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                usleep(MSEC1);
                continue;
            }
            if (!g_running) break;
            log_error("Web accept: %s", strerror(errno));
            continue;
        }
        // make client socket non-blocking
        int cflags = fcntl(client_fd, F_GETFL, 0);
        fcntl(client_fd, F_SETFL, cflags | O_NONBLOCK);

        // make recv non-blocking
        char req[2048];
        ssize_t n = recv(client_fd, req, sizeof(req) - 1, 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No data yet — try again later
                continue;
            }
            close(client_fd);
            continue;
        }
        if (n == 0) {
            // Client closed connection
            close(client_fd);
            continue;
        }
        req[n] = '\0';

        if (strstr(req, "Upgrade: websocket")) {
            if (handle_ws_handshake(client_fd, req) != 0) {
                close(client_fd);
            }
            continue;
        }

        char method[8], path[256], Xmethod[8], Xpath[256];
        if (sscanf(req, "%7s %255s", method, path) != 2) {
            log_error("webserver: req=%s, does not have the 'method' and 'path'", req);
            handle_root_html(client_fd, pCtrl);
            close(client_fd);
            continue;
        }

        if ((strcmp(method,Xmethod)==0) & (strcmp(path,Xpath)==0)) {
            // no change, no log
        } else {
            //log_debug("webserver: req=%s, method= %s, path= %s", req, method, path);
            strcpy(Xpath, path);
            strcpy(Xmethod, method);
        }

        if (strcmp(method, "GET") == 0 && strcmp(path, "/api/state") == 0) {
            handle_api_state(client_fd);
        } else if (strcmp(method, "GET") == 0 && strcmp(path, "/api/signalk") == 0) {
            handle_api_signalk(client_fd);
        } else if (strcmp(method, "POST") == 0 && strcmp(path, "/api/update") == 0) {
            char *body = strstr(req, "\r\n\r\n");
            if (body) {
                body += 4;
                handle_api_update(client_fd, body);
            } else {
                const char *err = "HTTP/1.1 400 Bad Request\r\n\r\n";
                send(client_fd, err, strlen(err), 0);
            }
        } else {
            //log_info("webserver: req=%s", req);
            //log_info("webserver: method=%s", method);
            //log_info("webserver: path=%s", path);
            if (strcmp(path,"/bgnet/") == 0) {
                handle_bgnet_html(client_fd, pCtrl);
            } else {
                handle_root_html(client_fd, pCtrl);
            }
        }
        usleep(MSEC1);
        close(client_fd);
    }

    close(server_fd);
    log_info("webserver: thread exiting");
    return NULL;
}

/* Call this from anywhere to push latest SK delta to WS clients */
void webserver_broadcast_signalk(void)
{
    char delta[1024];
    pthread_mutex_lock(&g_data_mutex);
    snprintf(delta, sizeof(delta), "%s", g_last_signalk_delta);
    pthread_mutex_unlock(&g_data_mutex);

    if (delta[0]) {
        ws_broadcast_text(delta);
    }
}
