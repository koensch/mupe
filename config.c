#include "config.h"
#include "app_state.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <cjson/cJSON.h>

int load_config(const char *filename, Config *cfg)
{
    FILE *f = fopen(filename, "rb");
    if (!f) {
        perror("fopen config");
        return -1;
    }

    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *data = malloc(len + 1);
    if (!data) {
        fclose(f);
        return -1;
    }
    fread(data, 1, len, f);
    data[len] = BGNET_ASC_NULL;
    fclose(f);

    cJSON *root = cJSON_Parse(data);
    free(data);
    if (!root) {
        fprintf(stderr, "Failed to parse JSON\n");
        return -1;
    }

    // clear config structure (also makes all strings into NULL strings)
    memset(cfg, 0, sizeof(*cfg));
    // apply some defaults to make sure we can log some errors incase someone did not configure config.json correctly
    strcpy(cfg->logger.str_log_file, ApplicationName);
    strcpy(cfg->logger.str_log_level, "warn");

    cJSON *fastnet   = cJSON_GetObjectItem(root, "fastnet");
    cJSON *ethernet  = cJSON_GetObjectItem(root, "ethernet");
    cJSON *nmea2000  = cJSON_GetObjectItem(root, "nmea2000");
    cJSON *timer     = cJSON_GetObjectItem(root, "timer");
    cJSON *webserver = cJSON_GetObjectItem(root, "webserver");
    cJSON *logger    = cJSON_GetObjectItem(root, "logger");

    // I know that netio is not fastnet, but fastnet was developed out netio. Fastnet shares lots of channels, different
    // communication parameters (totally incompatible bus), but it seems that the same approach we use for netio could
    // also be used for fastnet based productlines (Hydra, Hercules, H1000, H2000 and H3000)
    // With other communications parameters and a other protocol handler, that would be possible.
    if (fastnet) {
        snprintf(cfg->fastnet.port, sizeof(cfg->fastnet.port), "%s", cJSON_GetObjectItem(fastnet, "port")->valuestring);
        cfg->fastnet.baudrate = cJSON_GetObjectItem(fastnet, "baudrate")->valueint;
        cfg->fastnet.databits = cJSON_GetObjectItem(fastnet, "databits")->valueint;
        cfg->fastnet.stopbits = cJSON_GetObjectItem(fastnet, "stopbits")->valueint;
        snprintf(cfg->fastnet.parity, sizeof(cfg->fastnet.parity), "%s", cJSON_GetObjectItem(fastnet, "parity")->valuestring);
        snprintf(cfg->fastnet.mode, sizeof(cfg->fastnet.mode), "%s", cJSON_GetObjectItem(fastnet, "mode")->valuestring);
        snprintf(cfg->fastnet.protocol, sizeof(cfg->fastnet.protocol), "%s", cJSON_GetObjectItem(fastnet, "protocol")->valuestring);
    }

    if (ethernet) {
        snprintf(cfg->ethernet.eth_device, sizeof(cfg->ethernet.eth_device), "%s", cJSON_GetObjectItem(ethernet, "device")->valuestring);
        snprintf(cfg->ethernet.protocol, sizeof(cfg->ethernet.protocol), "%s", cJSON_GetObjectItem(ethernet, "protocol")->valuestring);
        snprintf(cfg->ethernet.remote_ip, sizeof(cfg->ethernet.remote_ip), "%s", cJSON_GetObjectItem(ethernet, "remote_ip")->valuestring);
        cfg->ethernet.remote_port = cJSON_GetObjectItem(ethernet, "remote_port")->valueint;
        cfg->ethernet.local_port  = cJSON_GetObjectItem(ethernet, "local_port")->valueint;
    }

    if (nmea2000) {
        snprintf(cfg->nmea2000.can_interface, sizeof(cfg->nmea2000.can_interface), "%s", cJSON_GetObjectItem(nmea2000, "can_interface")->valuestring);
    }

    if (timer) {
        cfg->timer.timer_interval_ms = cJSON_GetObjectItem(timer, "interval_ms")->valueint;
    }

    if (webserver) {
        cfg->webserver.web_port = cJSON_GetObjectItem(webserver, "port")->valueint;
        cfg->webserver.web_autorefresh = cJSON_GetObjectItem(webserver, "autorefresh")->valueint;
        if ((cfg->webserver.web_autorefresh < -1) | (cfg->webserver.web_autorefresh > 30))  cfg->webserver.web_autorefresh = -1; // disable autorefresh
    }

    if (logger) {
        snprintf(cfg->logger.str_log_file, sizeof(cfg->logger.str_log_file), "%s", cJSON_GetObjectItem(logger, "file")->valuestring);
        snprintf(cfg->logger.str_log_level, sizeof(cfg->logger.str_log_level), "%s", cJSON_GetObjectItem(logger, "level")->valuestring);
    }

    cJSON_Delete(root);
    return 0;
}