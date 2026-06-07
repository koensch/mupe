#include "config.h"
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
    data[len] = '\0';
    fclose(f);

    cJSON *root = cJSON_Parse(data);
    free(data);
    if (!root) {
        fprintf(stderr, "Failed to parse JSON\n");
        return -1;
    }

    memset(cfg, 0, sizeof(*cfg));

    cJSON *serial = cJSON_GetObjectItem(root, "serial");
    cJSON *ether  = cJSON_GetObjectItem(root, "ethernet");
    cJSON *n2k    = cJSON_GetObjectItem(root, "nmea2000");
    cJSON *timer  = cJSON_GetObjectItem(root, "timer");
    cJSON *web    = cJSON_GetObjectItem(root, "webserver");
    cJSON *log    = cJSON_GetObjectItem(root, "logging");

    if (serial) {
        snprintf(cfg->serial_port, sizeof(cfg->serial_port), "%s", cJSON_GetObjectItem(serial, "port")->valuestring);
        cfg->baudrate = cJSON_GetObjectItem(serial, "baudrate")->valueint;
        cfg->databits = cJSON_GetObjectItem(serial, "databits")->valueint;
        cfg->stopbits = cJSON_GetObjectItem(serial, "stopbits")->valueint;
        snprintf(cfg->parity, sizeof(cfg->parity), "%s", cJSON_GetObjectItem(serial, "parity")->valuestring);
        snprintf(cfg->mode, sizeof(cfg->mode), "%s", cJSON_GetObjectItem(serial, "mode")->valuestring);
    }

    if (ether) {
        snprintf(cfg->eth_device, sizeof(cfg->eth_device), "%s", cJSON_GetObjectItem(ether, "device")->valuestring);
        snprintf(cfg->protocol, sizeof(cfg->protocol), "%s", cJSON_GetObjectItem(ether, "protocol")->valuestring);
        snprintf(cfg->remote_ip, sizeof(cfg->remote_ip), "%s", cJSON_GetObjectItem(ether, "remote_ip")->valuestring);
        cfg->remote_port = cJSON_GetObjectItem(ether, "remote_port")->valueint;
        cfg->local_port  = cJSON_GetObjectItem(ether, "local_port")->valueint;
    }

    if (n2k) {
        snprintf(cfg->can_interface, sizeof(cfg->can_interface), "%s", cJSON_GetObjectItem(n2k, "can_interface")->valuestring);
    }

    if (timer) {
        cfg->timer_interval_ms = cJSON_GetObjectItem(timer, "interval_ms")->valueint;
    }

    if (web) {
        cfg->web_port = cJSON_GetObjectItem(web, "port")->valueint;
    }

    if (log) {
        snprintf(cfg->log_file, sizeof(cfg->log_file), "%s", cJSON_GetObjectItem(log, "file")->valuestring);
        snprintf(cfg->log_level, sizeof(cfg->log_level), "%s", cJSON_GetObjectItem(log, "level")->valuestring);
    }

    cJSON_Delete(root);
    return 0;
}