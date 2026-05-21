#pragma once

void handle_gnss_nmea(const char *sentence);

int handle_gnss_raw(const uint8_t *data, size_t len);
void gnss_cfg_close_all(int fd);
void gnss_cfg_eable_onchange(int fd, char *type);
void gnss_cfg(int fd, char *type, uint8_t enable, uint8_t per_second);
int8_t gnss_bdd_enable();
int8_t gnss_bdd_disable();
