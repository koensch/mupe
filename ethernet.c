#include "ethernet.h"
#include "app_state.h"
#include "logger.h"
#include "bgnprotocol.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <errno.h>
#include <stdlib.h>
#include <cjson/cJSON.h>

static int setup_tcp_socket(const Config *cfg, int *sockfd)
{
    *sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (*sockfd < 0) {
        log_error("TCP socket: %s", strerror(errno));
        return -1;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(cfg->remote_port);

    if (inet_pton(AF_INET, cfg->remote_ip, &addr.sin_addr) <= 0) {
        log_error("inet_pton: %s", strerror(errno));
        close(*sockfd);
        return -1;
    }

    if (connect(*sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        log_error("connect: %s", strerror(errno));
        close(*sockfd);
        return -1;
    }

    log_info("Ethernet TCP connected to %s:%d", cfg->remote_ip, cfg->remote_port);
    return 0;
}

static int setup_udp_socket(const Config *cfg, int *sockfd, struct sockaddr_in *remote)
{
    *sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (*sockfd < 0) {
        log_error("UDP socket: %s", strerror(errno));
        return -1;
    }

    memset(remote, 0, sizeof(*remote));
    remote->sin_family = AF_INET;
    remote->sin_port   = htons(cfg->remote_port);

    if (inet_pton(AF_INET, cfg->remote_ip, &remote->sin_addr) <= 0) {
        log_error("inet_pton: %s", strerror(errno));
        close(*sockfd);
        return -1;
    }

    log_info("Ethernet UDP ready to %s:%d", cfg->remote_ip, cfg->remote_port);
    return 0;
}

static char *build_signalk_delta(
    double lat,
    double lon,
    double cog,
    double stw,
    double depth,
    double twa,
    double tws,
    double awa,
    double aws,
    double rudder
) {
    cJSON *root = cJSON_CreateObject();
    cJSON *updates = cJSON_AddArrayToObject(root, "updates");
    cJSON *upd = cJSON_CreateObject();
    cJSON_AddItemToArray(updates, upd);

    cJSON *source = cJSON_CreateObject();
    cJSON_AddStringToObject(source, "label", "marine-gateway");
    cJSON_AddItemToObject(upd, "source", source);

    cJSON *values = cJSON_AddArrayToObject(upd, "values");

    #define ADD_VALUE(path, val) do { \
        cJSON *v = cJSON_CreateObject(); \
        cJSON_AddStringToObject(v, "path", path); \
        cJSON_AddNumberToObject(v, "value", val); \
        cJSON_AddItemToArray(values, v); \
    } while(0)

    ADD_VALUE("navigation.position.latitude", lat);
    ADD_VALUE("navigation.position.longitude", lon);
    ADD_VALUE("navigation.courseOverGroundTrue", cog);
    ADD_VALUE("navigation.speedThroughWater", stw);
    ADD_VALUE("environment.depth.belowTransducer", depth);
    ADD_VALUE("environment.wind.angleTrueWater", twa);
    ADD_VALUE("environment.wind.speedTrue", tws);
    ADD_VALUE("environment.wind.angleApparent", awa);
    ADD_VALUE("environment.wind.speedApparent", aws);
    ADD_VALUE("vessels.self.steering.rudderAngle", rudder);

    #undef ADD_VALUE

    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return out;
}

/* Called by webserver to get last SK delta; we just store it in app_state */

void *ethernet_thread(void *arg)
{
    BGNET_CTRL *pCtrl= (BGNET_CTRL *)arg;
    Config *cfg = pCtrl->pCfg;

    log_info("Ethernet Device=%s Protocol=%s Remote=%s:%d LocalPort=%d (Signal K)",cfg->eth_device, cfg->protocol,cfg->remote_ip, cfg->remote_port, cfg->local_port);

    int sockfd = -1;
    int is_tcp = (strcmp(cfg->protocol, "tcp") == 0);

    struct sockaddr_in udp_remote;
    memset(&udp_remote, 0, sizeof(udp_remote));

    if (is_tcp) {
        if (setup_tcp_socket(cfg, &sockfd) != 0) {
            return NULL;
        }
    } else {
        if (setup_udp_socket(cfg, &sockfd, &udp_remote) != 0) {
            return NULL;
        }
    }

    char buf[1024];
    int counter = 0;

    fd_set rfds;
    struct timeval tv;

    while (g_running) {
        counter++;

        if (counter % 200000 == 0) {
            // get all the values
            pthread_mutex_lock(&g_data_mutex);
            double lat   = g_n2kdata.pos.lat.value.dValue;
            double lon   = g_n2kdata.pos.lon.value.dValue;
            double cog   = g_n2kdata.pos.cog.value.dValue;
            double stw   = g_n2kdata.boat.stw.value.dValue;
            double depth = g_n2kdata.boat.depth.value.dValue;
            double twa   = g_n2kdata.wind.twa.value.dValue;
            double tws   = g_n2kdata.wind.tws.value.dValue;
            double awa   = g_n2kdata.wind.awa.value.dValue;
            double aws   = g_n2kdata.wind.aws.value.dValue;
            double rudder= g_n2kdata.boat.rudder.value.dValue;
            pthread_mutex_unlock(&g_data_mutex);

            // build json string
            char *json = build_signalk_delta(lat, lon, cog, stw, depth, twa, tws, awa, aws, rudder);
            if (json) {
                char line[2048];
                snprintf(line, sizeof(line), "%s\n", json);

                ssize_t sent;
                if (is_tcp) {
                    sent = send(sockfd, line, strlen(line), 0);
                } else {
                    sent = sendto(sockfd, line, strlen(line), 0, (struct sockaddr *)&udp_remote, sizeof(udp_remote));
                }
                if (sent < 0) {
                    log_error("Ethernet send SK: %s", strerror(errno));
                } else {
                    log_debug("Ethernet sent SK delta: %s", json);
                    pthread_mutex_lock(&g_data_mutex);
                    snprintf(g_last_signalk_delta, sizeof(g_last_signalk_delta), "%s", json);
                    pthread_mutex_unlock(&g_data_mutex);
                }
                free(json);
            }
        }

        FD_ZERO(&rfds);
        FD_SET(sockfd, &rfds);
        tv.tv_sec = 0;
        tv.tv_usec = 1000;

        int rv = select(sockfd + 1, &rfds, NULL, NULL, &tv);
        if (rv > 0 && FD_ISSET(sockfd, &rfds)) {
            ssize_t n;
            if (is_tcp) {
                n = recv(sockfd, buf, sizeof(buf) - 1, 0);
            } else {
                n = recvfrom(sockfd, buf, sizeof(buf) - 1, 0, NULL, NULL);
            }
            if (n > 0) {
                buf[n] = '\0';

                pthread_mutex_lock(&g_data_mutex);
                snprintf(g_last_ethernet_data, sizeof(g_last_ethernet_data),"%s", buf);
                pthread_mutex_unlock(&g_data_mutex);

                log_debug("Ethernet received: %s", buf);
            } else if (n == 0 && is_tcp) {
                log_warn("Ethernet TCP connection closed by peer");
                break;
            } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                log_error("Ethernet recv: %s", strerror(errno));
                break;
            }
        }
        usleep(MSEC1);
    }
    close(sockfd);
    log_info("Ethernet: thread exiting");
    return NULL;
}
