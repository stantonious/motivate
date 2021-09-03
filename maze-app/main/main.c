#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_freertos_hooks.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs_fat.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "globals.h"
#include "core2forAWS.h"
#include "game_tab.h"
#include "maze_tab.h"
#include "train_tab.h"
#include "pred_tab.h"
#include "tilt_maze_tab.h"
#include "button_handler.h"
#include "app_wifi.h"
#include "maze_client.h"
#include "mot-imu-tf.h"
#include "mot_mqtt_client.h"
#include "imu_task.h"
#include "etch_tab.h"

static const char *TAG = "MAIN";
static void ui_start(void);
static lv_obj_t *tab_view;

static void tab_event_cb(lv_obj_t *slider, lv_event_t event);

void heap_caps_alloc_failed_hook(size_t requested_size, uint32_t caps, const char *function_name)
{
    printf("%s was called but failed to allocate %d bytes with 0x%X capabilities. \n", function_name, requested_size, caps);
    int stackSize = uxTaskGetStackHighWaterMark(NULL);
    int minHeap = xPortGetMinimumEverFreeHeapSize();
    int heapSize = xPortGetFreeHeapSize();
    int capsSize = heap_caps_get_free_size(caps);
    ESP_LOGI(TAG, "MAIN STACK HWM %d", stackSize);
    ESP_LOGI(TAG, "MAIN HEAP HWM %d", heapSize);
    ESP_LOGI(TAG, "MAIN MIN HEAP HWM %d", minHeap);
    ESP_LOGI(TAG, "MAIN CAPS HEAP HWM %d", capsSize);
}

void app_main(void)
{
    heap_caps_register_failed_alloc_callback(heap_caps_alloc_failed_hook);
    ESP_LOGI(TAG, "\n***************************************************\n MOTIVE MAZE \n***************************************************");

    // Initialize NVS for Wi-Fi stack to store data
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_log_level_set("gpio", ESP_LOG_NONE);
    esp_log_level_set("ILI9341", ESP_LOG_NONE);

    Core2ForAWS_Init();

    init_imu();
    init_button_handlers();
    init_mot_imu();
    ui_start();

    int connected = init_wifi();

    //Core2ForAWS_Sk6812_SetSideColor(SK6812_SIDE_LEFT, (current_red << 16) + (current_green << 8) + (current_blue));
    static const char *btns[] = {"Close", ""};
    if (connected == true)
    {
        ESP_LOGI(TAG, "init maze client");
        maze_client_init();
        ESP_LOGI(TAG, "init mqtt client");
        mot_mqtt_client_init(game_id);
        ESP_LOGI(TAG, "get maze");
        get_maze(game_id,MAZE_HEIGHT, MAZE_LEN, MAZE,&x_entry,&y_entry,&x_exit,&y_exit);
        ESP_LOGI(TAG, "maze %d", MAZE[0][0]);
        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
        lv_obj_t *mbox1 = lv_msgbox_create(lv_scr_act(), NULL);
        lv_msgbox_set_text(mbox1, "Wifi Connected");
        lv_msgbox_add_btns(mbox1, btns);
        lv_obj_set_width(mbox1, 200);
        lv_obj_align(mbox1, NULL, LV_ALIGN_CENTER, 0, 0);
        xSemaphoreGive(xGuiSemaphore);
    }
    else
    {
        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
        lv_obj_t *mbox1 = lv_msgbox_create(lv_scr_act(), NULL);
        lv_msgbox_set_text(mbox1, "Wifi Failed");
        lv_msgbox_add_btns(mbox1, btns);
        lv_obj_set_width(mbox1, 200);
        lv_obj_align(mbox1, NULL, LV_ALIGN_CENTER, 0, 0);
        xSemaphoreGive(xGuiSemaphore);
    }

    Core2ForAWS_Display_SetBrightness(40); // Last since the display first needs time to finish initializing.
}

static void ui_start(void)
{
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    lv_obj_t *top_scr = lv_obj_create(NULL, NULL);

    lv_scr_load_anim(top_scr, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, false);
    //Tab setup
    tab_view = lv_tabview_create(top_scr, NULL);
    lv_obj_set_event_cb(tab_view, tab_event_cb);
    lv_tabview_set_btns_pos(tab_view, LV_TABVIEW_TAB_POS_NONE);

    xSemaphoreGive(xGuiSemaphore);

    display_game_tab(tab_view);
    //display_pred_tab(tab_view);
    display_maze_tab(tab_view);
    display_train_tab(tab_view);
    display_etch_tab(tab_view);
}

static void tab_event_cb(lv_obj_t *slider, lv_event_t event)
{
    if (event == LV_EVENT_VALUE_CHANGED)
    {
        lv_tabview_ext_t *ext = (lv_tabview_ext_t *)lv_obj_get_ext_attr(tab_view);
        const char *tab_name = ext->tab_name_ptr[lv_tabview_get_tab_act(tab_view)];
        ESP_LOGI(TAG, "Current Active Tab: %s\n", tab_name);

        vTaskSuspend(MAZE_handle);
        //vTaskSuspend(TILT_MAZE_handle);
        vTaskSuspend(Train_handle);
        //   vTaskSuspend(Pred_handle);
        vTaskSuspend(Etch_handle);

        if (strcmp(tab_name, MAZE_TAB_NAME) == 0)
        {
            vTaskResume(MAZE_handle);
        }
        else if (strcmp(tab_name, TILT_MAZE_TAB_NAME) == 0)
        {
            vTaskResume(TILT_MAZE_handle);
        }
        else if (strcmp(tab_name, TRAIN_TAB_NAME) == 0)
        {
            ESP_LOGI(TAG, "Resuming :%s", tab_name);
            vTaskResume(Train_handle);
        }
        else if (strcmp(tab_name, ETCH_TAB_NAME) == 0)
        {
            ESP_LOGI(TAG, "Resuming :%s", tab_name);
            vTaskResume(Etch_handle);
        }
        /*
        else if (strcmp(tab_name, PRED_TAB_NAME) == 0)
        {
            ESP_LOGI(TAG, "Resuming :%s",tab_name);
            vTaskResume(Pred_handle);
        }
        */
    }
}
