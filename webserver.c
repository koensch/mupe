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

static void handle_root_html(int client_fd, Config *pCfg)
{
    const char *template =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n\r\n"
        "<html>\r\n"
        "<head><title>%s</title></head>\r\n"
        "<body>\r\n<div style=\"float:right\"><p>%s</p></div>\r\n"
        "<h1>%s</h1>"
        "<p>REST:</p>"
        "<ul>"
        "<li>GET /api/state</li>"
        "<li>GET /api/signalk</li>"
        "<li>POST /api/update</li>"
        "</ul>"
        "<p>WebSocket: ws://host:%d/ws</p>"
        "<p>for configuration parameters check file 'config.json'</p>"
        "</body></html>\r\n";
    char timebuffer[256];
    time_t rawtime;
    struct tm* tm_info;
    time (&rawtime);
    tm_info = localtime(&rawtime);
    strftime(timebuffer, sizeof(timebuffer),"%Y-%m-%d %H:%M:%S", tm_info);
    char htmlbuffer[4095];
    snprintf(htmlbuffer, sizeof(htmlbuffer), template, ApplicationName, timebuffer, ApplicationName, pCfg->web_port);
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
            handle_root_html(client_fd, cfg);
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
            handle_root_html(client_fd, cfg);
        }
        usleep(MSEC1);
        close(client_fd);
    }

    close(server_fd);
    log_info("Webserver thread exiting");
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
