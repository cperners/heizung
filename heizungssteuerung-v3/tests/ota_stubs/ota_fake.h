#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#define ESP_ERR_TIMEOUT 0x107
typedef struct {size_t content_len;} httpd_req_t;
int httpd_req_recv(httpd_req_t*,char*,size_t);
typedef struct {uint8_t pad[15];uint16_t min_chip_rev_full;uint8_t rest[7];} __attribute__((packed)) esp_image_header_t;
typedef struct {uint32_t address,length;} esp_image_segment_header_t;
typedef struct {char pad[48];char project_name[32];} esp_app_desc_t;
const esp_app_desc_t *esp_app_get_description(void);
typedef struct {unsigned revision;} esp_chip_info_t;
void esp_chip_info(esp_chip_info_t*);
typedef struct {size_t size;const char *label;} esp_partition_t;
typedef enum {ESP_OTA_IMG_VALID,ESP_OTA_IMG_PENDING_VERIFY} esp_ota_img_states_t;
typedef unsigned esp_ota_handle_t;
#define OTA_WITH_SEQUENTIAL_WRITES 0xfffffffe
const esp_partition_t *esp_ota_get_running_partition(void);
const esp_partition_t *esp_ota_get_next_update_partition(const void*);
esp_err_t esp_ota_get_state_partition(const esp_partition_t*,esp_ota_img_states_t*);
esp_err_t esp_ota_mark_app_valid_cancel_rollback(void);
esp_err_t esp_ota_begin(const esp_partition_t*,size_t,esp_ota_handle_t*);
esp_err_t esp_ota_write(esp_ota_handle_t,const void*,size_t);
esp_err_t esp_ota_end(esp_ota_handle_t);
esp_err_t esp_ota_abort(esp_ota_handle_t);
esp_err_t esp_ota_set_boot_partition(const esp_partition_t*);
typedef unsigned esp_timer_handle_t;
typedef struct {void(*callback)(void*);const char *name;} esp_timer_create_args_t;
esp_err_t esp_timer_create(const esp_timer_create_args_t*,esp_timer_handle_t*);
esp_err_t esp_timer_start_once(esp_timer_handle_t,uint64_t);
int64_t esp_timer_get_time(void);
void esp_restart(void);
#define ESP_LOGI(tag,...) ((void)0)
#define ESP_LOGE(tag,...) ((void)0)
