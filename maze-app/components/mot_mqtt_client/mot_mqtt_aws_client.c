

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_system.h"
#include "esp_event.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include "esp_log.h"
//#include "mqtt_client.h"
//#include "esp_tls.h"
#include <sys/param.h>
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"

#include "core2forAWS.h"

#include "mot_mqtt_aws_client.h"

static const char *TAG = "MOT_MQTT_CLIENT";
static const char *CONFIG_BROKER_URI = "wss://ahdizksxaeeun-ats.iot.us-east-2.amazonaws.com";

extern const uint8_t root_CA_crt_start[] asm("_binary_root_CA_crt_start");
extern const uint8_t root_CA_crt_end[] asm("_binary_root_CA_crt_end");
extern const uint32_t root_CA_crt_length;
extern const uint8_t mot0_device_pem_start[] asm("_binary_mot0_device_pem_start");
extern const uint8_t mot0_device_pem_end[] asm("_binary_mot0_device_pem_end");

extern const uint8_t mot0_private_pem_start[] asm("_binary_mot0_private_pem_start");
extern const uint8_t mot0_private_pem_end[] asm("_binary_mot0_private_pem_end");

extern const uint8_t mot0_public_pem_start[] asm("_binary_mot0_public_pem_start");
extern const uint8_t mot0_public_pem_end[] asm("_binary_mot0_public_pem_end");

void disconnect_callback_handler(AWS_IoT_Client *pClient, void *data)
{
    ESP_LOGW(TAG, "MQTT Disconnect");
    IoT_Error_t rc = FAILURE;

    if (pClient == NULL)
    {
        return;
    }

    if (aws_iot_is_autoreconnect_enabled(pClient))
    {
        ESP_LOGI(TAG, "Auto Reconnect is enabled, Reconnecting attempt will start now");
    }
    else
    {
        ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
        rc = aws_iot_mqtt_attempt_reconnect(pClient);
        if (NETWORK_RECONNECTED == rc)
        {
            ESP_LOGW(TAG, "Manual Reconnect Successful");
        }
        else
        {
            ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
        }
    }
}

void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params, void *pData) {
    ESP_LOGI(TAG, "Subscribe callback");
    ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);
}


void mot_client_init(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    //esp_tls_init_global_ca_store();
    //esp_tls_set_global_ca_store((const unsigned char *)root_CA_crt_start, root_CA_crt_length + 1);
    //ESP_ERROR_CHECK(nvs_flash_init());
    //ESP_ERROR_CHECK(esp_netif_init());
    //ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    //ESP_ERROR_CHECK(example_connect());
}

void mot_client_task(void *pvParameters)
{
    ESP_LOGI(TAG,"0");
    IoT_Error_t rc = FAILURE;

    AWS_IoT_Client client;
    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
    ESP_LOGI(TAG,"1");
    IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;
    ESP_LOGI(TAG,"1");

    ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    mqttInitParams.enableAutoReconnect = false; // We enable this later below
    mqttInitParams.pHostURL = "ahdizksxaeeun-ats.iot.us-east-2.amazonaws.com";
    mqttInitParams.port = 8883;
    mqttInitParams.pRootCALocation = (const char *)root_CA_crt_start;
    mqttInitParams.pDeviceCertLocation =(const char *)mot0_device_pem_start;
    mqttInitParams.pDevicePrivateKeyLocation = (const char*)mot0_private_pem_start ;

#define CLIENT_ID_LEN (ATCA_SERIAL_NUM_SIZE * 2)
#define SUBSCRIBE_TOPIC_LEN (CLIENT_ID_LEN + 3)
#define BASE_PUBLISH_TOPIC_LEN (CLIENT_ID_LEN + 2)

    char *client_id = malloc(CLIENT_ID_LEN + 1);
    ATCA_STATUS ret = Atecc608_GetSerialString(client_id);
    if (ret != ATCA_SUCCESS)
    {
        printf("Failed to get device serial from secure element. Error: %i", ret);
        abort();
    }

    //char subscribe_topic[SUBSCRIBE_TOPIC_LEN];
    char *subscribe_topic = "topic_1";
    char base_publish_topic[BASE_PUBLISH_TOPIC_LEN];
    //snprintf(subscribe_topic, SUBSCRIBE_TOPIC_LEN, "%s/#", client_id);
    snprintf(base_publish_topic, BASE_PUBLISH_TOPIC_LEN, "%s/", client_id);

    mqttInitParams.mqttCommandTimeout_ms = 20000;
    mqttInitParams.tlsHandshakeTimeout_ms = 5000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = disconnect_callback_handler;
    mqttInitParams.disconnectHandlerData = NULL;

    rc = aws_iot_mqtt_init(&client, &mqttInitParams);
    if (SUCCESS != rc)
    {
        ESP_LOGE(TAG, "aws_iot_mqtt_init returned error : %d ", rc);
        abort();
    }

    /* Wait for WiFI to show as connected */
    //TODO
    //xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                        //false, true, portMAX_DELAY);

    connectParams.keepAliveIntervalInSec = 10;
    connectParams.isCleanSession = true;
    connectParams.MQTTVersion = MQTT_3_1_1;

    //connectParams.pClientID = client_id;
    //connectParams.clientIDLen = CLIENT_ID_LEN;
    connectParams.pClientID = "basicPubSub";
    connectParams.clientIDLen = 11;
    connectParams.isWillMsgPresent = false;
    ESP_LOGI(TAG, "Connecting to AWS IoT Core at %s:%d", mqttInitParams.pHostURL, mqttInitParams.port);

    do
    {
        rc = aws_iot_mqtt_connect(&client, &connectParams);
        if (SUCCESS != rc)
        {
            ESP_LOGE(TAG, "Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } while (SUCCESS != rc);
    ESP_LOGI(TAG, "Successfully connected to AWS IoT Core!");
    /*
     * Enable Auto Reconnect functionality. Minimum and Maximum time for exponential backoff for retries.
     *  #AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL
     *  #AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL
     */
    rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
    if (SUCCESS != rc)
    {
        ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
        abort();
    }

    ESP_LOGI(TAG, "Subscribing to '%s'", subscribe_topic);
    rc = aws_iot_mqtt_subscribe(&client, subscribe_topic, strlen(subscribe_topic), QOS0, iot_subscribe_callback_handler, NULL);
    if (SUCCESS != rc)
    {
        ESP_LOGE(TAG, "Error subscribing : %d ", rc);
        abort();
    }
    else
    {
        ESP_LOGI(TAG, "Subscribed to topic '%s'", subscribe_topic);
    }

    ESP_LOGI(TAG, "\n****************************************\n*  AWS client Id - %s  *\n****************************************\n\n",
             client_id);

    while ((NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc))
    {

        //Max time the yield function will wait for read messages
        rc = aws_iot_mqtt_yield(&client, 100);
        if (NETWORK_ATTEMPTING_RECONNECT == rc)
        {
            // If the client is attempting to reconnect we will skip the rest of the loop.
            continue;
        }

        ESP_LOGI(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(pdMS_TO_TICKS(100));

        //publisher(&client, base_publish_topic, BASE_PUBLISH_TOPIC_LEN);
    }

    ESP_LOGE(TAG, "An error occurred in the main loop.");
    abort();
}