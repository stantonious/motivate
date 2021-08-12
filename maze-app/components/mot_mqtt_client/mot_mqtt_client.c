
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include "cJSON.h"
#include <sys/param.h>

#include "mot_mqtt_client.h"


static const char *TAG = "MOT_MQTT_CLIENT";
static const char *CONFIG_BROKER_URI = "mqtts://ahdizksxaeeun-ats.iot.us-east-2.amazonaws.com:8883";

extern const uint8_t root_CA_crt_start[] asm("_binary_root_CA_crt_start");
extern const uint8_t root_CA_crt_end[] asm("_binary_root_CA_crt_end");
extern const uint32_t root_CA_crt_length;
extern const uint8_t mot0_device_pem_start[] asm("_binary_mot0_device_pem_start");
extern const uint8_t mot0_device_pem_end[] asm("_binary_mot0_device_pem_end");
extern const uint32_t mot0_device_pem_length;

extern const uint8_t mot0_private_pem_start[] asm("_binary_mot0_private_pem_start");
extern const uint8_t mot0_private_pem_end[] asm("_binary_mot0_private_pem_end");
extern const uint32_t mot0_private_pem_length;

extern const uint8_t mot0_public_pem_start[] asm("_binary_mot0_public_pem_start");
extern const uint8_t mot0_public_pem_end[] asm("_binary_mot0_public_pem_end");
extern const uint32_t mot0_public_pem_length;

static int8_t op_x = -1;
static int8_t op_y = -1;

static const char* CLIENT_ID="basicPubSub";
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_subscribe(client, "topic_1", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);

        cJSON *message_json = cJSON_Parse(event->data);
        if (message_json == NULL)
        {
            fprintf(stderr, "Error with json");
            const char *error_ptr = cJSON_GetErrorPtr();
            if (error_ptr != NULL)
            {
                fprintf(stderr, "Error before: %s\n", error_ptr);
            }
        }
        else
        {
            cJSON *message = cJSON_GetObjectItemCaseSensitive(message_json, "message");
            cJSON *x = cJSON_GetObjectItemCaseSensitive(message_json, "x");
            cJSON *y = cJSON_GetObjectItemCaseSensitive(message_json, "y");
            printf("x =%d y = %d\r\n", x->valueint,y->valueint);

            if (x == NULL || y == NULL)
            {
                printf("x and y must be provided!");
            }
            else
            {
                op_x = x->valueint;
                op_y = y->valueint;
            }
            cJSON_Delete(message_json);
        }

        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
            ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            ESP_LOGI(TAG, "Last captured errno : %d (%s)", event->error_handle->esp_transport_sock_errno,
                     strerror(event->error_handle->esp_transport_sock_errno));
        }
        else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED)
        {
            ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
        }
        else
        {
            ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

void mot_mqtt_client_init(void)
{
    esp_err_t esp_ret = ESP_FAIL;

    esp_ret = esp_tls_set_global_ca_store(root_CA_crt_start, root_CA_crt_length + 1);
    if (esp_ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error in setting the global ca store: [%02X] (%s),could not complete the https_request using global_ca_store", esp_ret, esp_err_to_name(esp_ret));
        return;
    }

    const esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URI,
        .client_cert_pem = (const char *)mot0_device_pem_start,
        .client_key_pem = (const char *)mot0_private_pem_start,
        .protocol_ver = MQTT_PROTOCOL_V_3_1_1,
        .use_global_ca_store = true,
        .client_id = CLIENT_ID};

    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_DEBUG);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    esp_mqtt_client_start(client);
}

void get_op_x_y(int8_t *x, int8_t *y)
{
    *x = op_x;
    *y = op_y;
}