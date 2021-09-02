#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_tls.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"
#include "esp_log.h"

#include "esp_http_client.h"
#include "maze_client.h"

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2048
static const char *TAG = "MAZE CLIENT";
static char *local_response_buffer = NULL;

extern const uint8_t root_CA_crt_start[] asm("_binary_root_CA_crt_start");
extern const uint8_t root_CA_crt_end[] asm("_binary_root_CA_crt_end");
extern const uint32_t root_CA_crt_length;

extern const uint8_t mot0_device_pem_start[] asm("_binary_mot0_device_pem_start");
extern const uint8_t mot0_device_pem_end[] asm("_binary_mot0_device_pem_end");
extern const uint32_t mot0_device_pem_length;

extern const uint8_t mot0_public_pem_start[] asm("_binary_mot0_public_pem_start");
extern const uint8_t mot0_public_pem_end[] asm("_binary_mot0_public_pem_end");
extern const uint32_t mot0_public_pem_length;

extern const uint8_t mot0_private_pem_start[] asm("_binary_mot0_private_pem_start");
extern const uint8_t mot0_private_pem_end[] asm("_binary_mot0_private_pem_end");
extern const uint32_t mot0_private_pem_length;

void maze_client_init(void)
{
    local_response_buffer = malloc(sizeof(char) * MAX_HTTP_OUTPUT_BUFFER);
    bzero(local_response_buffer, MAX_HTTP_OUTPUT_BUFFER);
    esp_err_t esp_ret = esp_tls_set_global_ca_store(root_CA_crt_start, root_CA_crt_length + 1);
    if (esp_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error in setting the global ca store: [%02X] (%s),could not complete the https_request using global_ca_store", esp_ret, esp_err_to_name(esp_ret));
        return;
    }

    //    xTaskCreatePinnedToCore(maze_client_task, "MazeClientTask", 2048, NULL, 1, &MazeClientHandle, 1);
}
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static char *output_buffer; // Buffer to store response of http request from event handler
    static int output_len;      // Stores number of bytes read
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        /*
             *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
             *  However, event handler can also be used in case chunked encoding is used.
             */
        if (!esp_http_client_is_chunked_response(evt->client))
        {
            // If user_data buffer is configured, copy the response into the buffer
            if (evt->user_data)
            {
        ESP_LOGI(TAG, "copying to user_data %s",(char*)evt->data);
                memcpy(evt->user_data + output_len, evt->data, evt->data_len);
                //(char*)(evt->user_data)[evt->data_len] = '\0';
            }
            else
            {
                if (output_buffer == NULL)
                {
                    output_buffer = (char *)malloc(esp_http_client_get_content_length(evt->client));
                    output_len = 0;
                    if (output_buffer == NULL)
                    {
                        ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                        return ESP_FAIL;
                    }
                }
                memcpy(output_buffer + output_len, evt->data, evt->data_len);
            }
            output_len += evt->data_len;
        }

        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        if (output_buffer != NULL)
        {
            // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
            // ESP_LOG_BUFFER_HEX(TAG, output_buffer, output_len);
            free(output_buffer);
            output_buffer = NULL;
        }
        output_len = 0;
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
        int mbedtls_err = 0;
        esp_err_t err = esp_tls_get_and_clear_last_error(evt->data, &mbedtls_err, NULL);
        if (err != 0)
        {
            if (output_buffer != NULL)
            {
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
            ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
        }
        break;
    }
    return ESP_OK;
}

void get_maze(int x, int y, int m[][x])
{

    /**
     * NOTE: All the configuration parameters for http_client must be spefied either in URL or as host and path parameters.
     * If host and path parameters are not set, query parameter will be ignored. In such cases,
     * query parameter should be specified in URL.
     *
     * If URL as well as host and path parameters are specified, values of host and path will be considered.
     */
    esp_http_client_config_t config = {
        //.url = "http://dl3to8c26ssxq.cloudfront.net/production/maze?x=14&y=14",
        .url = "http://dl3to8c26ssxq.cloudfront.net/production/maze?id=1630337075",
        .event_handler = _http_event_handler,
        .user_data = local_response_buffer, // Pass address of local buffer to get response
        .use_global_ca_store = true,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);


    // GET
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %d",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    }
    else
    {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "buff before");
    ESP_LOG_BUFFER_HEX(TAG, local_response_buffer, strlen(local_response_buffer));
    ESP_LOGI(TAG, "buff %s", local_response_buffer);
    ESP_LOGI(TAG, "buff after");
    int length = esp_http_client_get_content_length(client);

    local_response_buffer[length] = '\0';

    cJSON *maze = cJSON_Parse(local_response_buffer);

    cJSON *d = cJSON_GetObjectItemCaseSensitive(maze, "cells");

    bool isarray = cJSON_IsArray(d);

    cJSON *n_row;
    int r = 0;
    cJSON_ArrayForEach(n_row, d)
    {

        int c = 0;
        cJSON *n_val;
        cJSON_ArrayForEach(n_val, n_row)
        {
            m[r][c++] = n_val->valueint;
        }
        r += 1;
    }


    cJSON_Delete(maze);
    esp_http_client_cleanup(client);
}
