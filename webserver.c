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
    log_info("WebSocket client connected");
    return 0;
}

static void handle_api_state(int client_fd)
{
    char serial_copy[1024];
    char eth_copy[1024];

    pthread_mutex_lock(&g_data_mutex);
    snprintf(serial_copy, sizeof(serial_copy), "%s", g_last_serial_dataRx);
    snprintf(eth_copy, sizeof(eth_copy), "%s", g_last_ethernet_data);
    pthread_mutex_unlock(&g_data_mutex);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "lastSerialData", serial_copy);
    cJSON_AddStringToObject(root, "lastEthernetData", eth_copy);

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

static void handle_api_update(int client_fd, const char *body)
{
    cJSON *root = cJSON_Parse(body);
    if (!root) {
        const char *err = "HTTP/1.1 400 Bad Request\r\n\r\n";
        send(client_fd, err, strlen(err), 0);
        return;
    }

    cJSON *lat    = cJSON_GetObjectItem(root, "lat");
    cJSON *lon    = cJSON_GetObjectItem(root, "lon");
    cJSON *cog    = cJSON_GetObjectItem(root, "cog");
    cJSON *stw    = cJSON_GetObjectItem(root, "stw");
    cJSON *depth  = cJSON_GetObjectItem(root, "depth");
    cJSON *twa    = cJSON_GetObjectItem(root, "twa");
    cJSON *tws    = cJSON_GetObjectItem(root, "tws");
    cJSON *awa    = cJSON_GetObjectItem(root, "awa");
    cJSON *aws    = cJSON_GetObjectItem(root, "aws");
    cJSON *rudder = cJSON_GetObjectItem(root, "rudder");

    pthread_mutex_lock(&g_data_mutex);
    if (lat)    g_lat    = lat->valuedouble;
    if (lon)    g_lon    = lon->valuedouble;
    if (cog)    g_cog    = cog->valuedouble;
    if (stw)    g_stw    = stw->valuedouble;
    if (depth)  g_depth  = depth->valuedouble;
    if (twa)    g_twa    = twa->valuedouble;
    if (tws)    g_tws    = tws->valuedouble;
    if (awa)    g_awa    = awa->valuedouble;
    if (aws)    g_aws    = aws->valuedouble;
    if (rudder) g_rudder = rudder->valuedouble;
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
        "<head><style>h3 {margin-left: 40px;}\r\ntable {border-collapse: collapse; width: 80%; margin: 20px auto;}\r\nth, td {border:1px solid #ccc; padding: 10px; text-align: left;}</style><title>%s</title></head>\r\n"
        "<body>\r\n";
    snprintf(Buffer, MaxBuf, cHead, ApplicationName);
    return(Buffer);
}

static char* html_tail(char *Buffer) {
    const char *cTail =
        "<h2>End of Report</h2></body></html>\r\n";
    strcpy(Buffer, cTail);
    return(Buffer);
}

static char* html_title(char *Buffer, size_t MaxBuf) {
    const char *cTitle =
        "<div style=\"float:right\"><p>%s</p></div>\r\n"
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
        "<tr><th>Address</th><th>Type</th></tr>";
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
                                            pCtrl->pDEVDT[bgn_FindDeviceIndex(pCtrl, BGNET_ADDRESS_ME)].sProdName
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
        "<h3>Devices</h3>"
        "<table>"
        "<tr><th colspan='2'>Device</th><th colspan='2'>Rx</th><th colspan='2'>Tx</th><th colspan='2'>Checksum Errors</th></th></tr>"
        "<tr><th>Address</th><th>Type</th><th>Counter</th><th>Sec</th><th>Counter</th><th>Sec</th><th>Head</th><th>Frame</th></tr>";
    strcpy(Buffer, cDevTop);
    return(Buffer);
}
static char* html_device_row(char *Buffer, BGNET_CTRL *pCtrl) {
    int i;
    time_t t;
    char cLine[256];
    const char *cDevRow =
        "<tr><td>0x%02X</td><td>%s</td><td>%d</td><td>%d</td><td>%d</td><td>%d</td><td>%d</td><td>%d</td></tr>";

    strcpy(cLine, "");
    strcpy(Buffer, "");
    time(&t);
    for (i=0; i<BGNET_DEVICE_MAX; i++) {
        if ((pCtrl->DEV[i].iCounterRx < 1) & (pCtrl->DEV[i].iCounterTx < 1)) {
            // it was never touched
        } else {
            snprintf(cLine, sizeof(cLine), cDevRow,
                                                     i,
                                                     pCtrl->pDEVDT[bgn_FindDeviceIndex(pCtrl, i)].sProdName,
                                                     pCtrl->DEV[i].iCounterRx,
                                                     t - pCtrl->DEV[i].tTimestampRx,
                                                     pCtrl->DEV[i].iCounterTx,
                                                     t - pCtrl->DEV[i].tTimestampTx,
                                                     pCtrl->DEV[i].iCSError_Header, 
                                                     pCtrl->DEV[i].iCSError_Frame
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
        "<table><tr><th>Status</th><th>Display</th></tr>";
    strcpy(Buffer, cACPTop);
    return(Buffer);
}
static char* html_ACP_row(char *Buffer, BGNET_CTRL *pCtrl) {
    char cLine[256];
    char cSTAT[2][5] = {"STBY","AUTO"};
    char cMODE[8][8] = {"---0","COMP","---2","NAV","VANE","---5","---6","JOYSTICK"};
    const char *cACPRowS =
        "<tr><td>%s</td><td>%s</td></tr>";
    const char *cACPRowD =
        "<tr><td>%s</td><td>%d</td></tr>";

    strcpy(cLine, "");
    strcpy(Buffer, "");

    snprintf(cLine, sizeof(cLine), cACPRowS, "Status AUTO/STBY", cSTAT[pCtrl->ACP.ucSTATE]);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowS, "MODE COMP/NAV/VANE", cMODE[pCtrl->ACP.ucMODE]);
    strcat(Buffer, cLine);
    snprintf(cLine, sizeof(cLine), cACPRowD, "Backlight 0/1/2/3", pCtrl->ACP.ucLight);
    strcat(Buffer, cLine);

    return(Buffer);
}
static char* html_ACP_end(char *Buffer) {
    const char *cACPEnd =
        "</table>";
    strcpy(Buffer, cACPEnd);
    return(Buffer);
}

static char* html_command_top(char *Buffer) {
    const char *cDevTop =
        "<h3>Commands</h3>"
        "<table>"
        "<tr><th rowspan='2'>Command</th><th colspan='3'>Rx</th><th colspan='3'>Tx</th></tr>"
        "<tr><th>Counter</th><th>Sec</th><th>Raw</th><th>Counter</th><th>Sec</th><th>Raw</th></tr>";
    strcpy(Buffer, cDevTop);
    return(Buffer);
}
static char* html_command_row(char *Buffer, BGNET_CTRL *pCtrl) {
    int i;
    time_t t;
    char cLine[256], asciibufRx[BGN_CMD_ASCII], asciibufTx[BGN_CMD_ASCII];
    const char *cDevRow =
        "<tr><td>0x%02X</td><td>%d</td><td>%d</td><td>%s</td><td>%d</td><td>%d</td><td>%s</td></tr>";

    strcpy(cLine, "");
    strcpy(Buffer, "");
    time(&t);
    for (i=0; i<BGNET_COMMAND_MAX; i++) {
        if ((pCtrl->CMD[i].iCounterRx < 1) & (pCtrl->CMD[i].iCounterTx < 1)) {
            // it was never touched
        } else {
            CommandToAscii(asciibufRx, pCtrl->CMD[i].ucFrameRx);
            CommandToAscii(asciibufTx, pCtrl->CMD[i].ucFrameTx);
            snprintf(cLine, sizeof(cLine), cDevRow,
                                                    i,
                                                    pCtrl->CMD[i].iCounterRx,
                                                    t - pCtrl->CMD[i].tTimestampRx,
                                                    asciibufRx,
                                                    pCtrl->CMD[i].iCounterTx,
                                                    t - pCtrl->CMD[i].tTimestampTx,
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

static char* html_channel_top(char *Buffer) {
    const char *cDevTop =
        "<h3>Channels</h3>"
        "<table>"
        "<tr><th rowspan='2'>Channel</th><th colspan='3'>Rx</th><th rowspan='2'>Label</th><th rowspan='2'>From Address</th><th rowspan='2'>Display</th></tr>"
        "<tr><th>Counter</th><th>Sec</th><td>Raw</td></tr>";
    strcpy(Buffer, cDevTop);
    return(Buffer);
}
static char* html_channel_row(char *Buffer, BGNET_CTRL *pCtrl) {
    int i,y;
    time_t t;
    char cLine[1024], asciibuf[256];
    const char *cDevRow =
        "<tr><td>0x%02X</td><td>%d</td><td>%d</td><td>%s</td><td>%s</td><td>0x%02X</td><td>%s</td></tr>";

    strcpy(cLine, "");
    strcpy(Buffer, "");
    time(&t);
    for (i=0; i<BGN_CHANNEL_MAX; i++) {
        if (pCtrl->DATA[i].iCounter < 1) {
            // it was never touched
        } else {
            y = ChannelByteLength(pCtrl, pCtrl->DATA[i].ucRAW[BGN_VALUE_FMT]);
            byte2ascii(asciibuf, sizeof(asciibuf), pCtrl->DATA[i].ucRAW, y, BYTE2ASCII_SPACED);
            snprintf(cLine, sizeof(cLine), cDevRow, i, 
                                                    pCtrl->DATA[i].iCounter,
                                                    t - pCtrl->DATA[i].tTimestamp,
                                                    asciibuf,
                                                    pCtrl->pCHNDT[i].cLabel,
                                                    pCtrl->DATA[i].ucFROM,
                                                    pCtrl->DATA[i].cValue);
            strcat(Buffer, cLine);  // warning: this could overflow the buffer
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
    char htmlbuffer[8192], workbuffer[4096];
    strcpy(htmlbuffer, ""); // initialize the empty buffer

    strcat(htmlbuffer, html_head(workbuffer, sizeof(workbuffer)));
    strcat(htmlbuffer, html_title(workbuffer, sizeof(workbuffer)));

    strcat(htmlbuffer, html_help(workbuffer, sizeof(workbuffer), pCtrl->pCfg->web_port));

    strcat(htmlbuffer, html_interface_top(workbuffer));
    strcat(htmlbuffer, html_interface_row(workbuffer, pCtrl));
    strcat(htmlbuffer, html_interface_end(workbuffer));

    strcat(htmlbuffer, html_device_top(workbuffer));
    strcat(htmlbuffer, html_device_row(workbuffer, pCtrl));
    strcat(htmlbuffer, html_device_end(workbuffer));

    send(client_fd, htmlbuffer, strlen(htmlbuffer), 0);
    strcpy(htmlbuffer, ""); // initialize the empty buffer

    strcat(htmlbuffer, html_ACP_top(workbuffer));
    strcat(htmlbuffer, html_ACP_row(workbuffer, pCtrl));
    strcat(htmlbuffer, html_ACP_end(workbuffer));

    strcat(htmlbuffer, html_command_top(workbuffer));
    strcat(htmlbuffer, html_command_row(workbuffer, pCtrl));
    strcat(htmlbuffer, html_command_end(workbuffer));

    send(client_fd, htmlbuffer, strlen(htmlbuffer), 0);
    strcpy(htmlbuffer, ""); // initialize the empty buffer

    strcat(htmlbuffer, html_channel_top(workbuffer));
    strcat(htmlbuffer, html_channel_row(workbuffer, pCtrl));
    strcat(htmlbuffer, html_channel_end(workbuffer));

    strcat(htmlbuffer, html_tail(workbuffer));

    send(client_fd, htmlbuffer, strlen(htmlbuffer), 0);
    // check in the logfiles how much buffer space is used/left.
    log_info("Webserver: pagesize %d bytes", strlen(htmlbuffer));
}

void *webserver_thread(void *arg)
{
    BGNET_CTRL *pCtrl= (BGNET_CTRL *)arg;
    Config *cfg = pCtrl->pCfg;

    // warning: server should be in non-blocking mode SOCK_NONBLOCK
    // at present, the thread will not close when the application exits

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        log_error("Web socket: %s", strerror(errno));
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
        log_error("Web bind: %s", strerror(errno));
        close(server_fd);
        return NULL;
    }

    if (listen(server_fd, 5) < 0) {
        log_error("Web listen: %s", strerror(errno));
        close(server_fd);
        return NULL;
    }

    log_info("Web listening on http://localhost:%d", cfg->web_port);

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

        char method[8], path[256];
        if (sscanf(req, "%7s %255s", method, path) != 2) {
            handle_root_html(client_fd, pCtrl);
            close(client_fd);
            continue;
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
            handle_root_html(client_fd, pCtrl);
        }
        usleep(MSEC1);
        close(client_fd);
    }

    close(server_fd);
    log_info("Webserver: thread exiting");
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
