#pragma once
#include "esp_err.h"
#include <stddef.h>
typedef struct {int fd;const char *token;} httpd_req_t;
int httpd_req_to_sockfd(httpd_req_t*);
esp_err_t httpd_req_get_hdr_value_str(httpd_req_t*,const char*,char*,size_t);
