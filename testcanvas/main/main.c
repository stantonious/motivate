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

#include "core2forAWS.h"
#include "test.h"
#include "imu.h"
#include "button_handler.h"

static const char* TAG = "MAIN";

static void ui_start(void);
static lv_obj_t* tab_view;

static void tab_event_cb(lv_obj_t* slider, lv_event_t event);

void app_main(void)
{
  ESP_LOGI(TAG, "\n***************************************************\n M5Stack Core2 for AWS IoT EduKit Factory Firmware\n***************************************************");

  // Initialize NVS for Wi-Fi stack to store data
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK( ret );

  esp_log_level_set("gpio", ESP_LOG_NONE);
  esp_log_level_set("ILI9341", ESP_LOG_NONE);

  Core2ForAWS_Init();
  Core2ForAWS_Display_SetBrightness(80); // Last since the display first needs time to finish initializing.

  ui_start();
}

static void ui_start(void){
  xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
  lv_obj_t* top_scr = lv_obj_create(NULL,NULL);

  lv_scr_load_anim(top_scr, LV_SCR_LOAD_ANIM_MOVE_LEFT, 400, 0, false);
  //Tab setup
  tab_view = lv_tabview_create(top_scr, NULL);
  lv_obj_set_event_cb(tab_view, tab_event_cb);
  lv_tabview_set_btns_pos(tab_view, LV_TABVIEW_TAB_POS_NONE); 

  xSemaphoreGive(xGuiSemaphore);

  display_test_tab(tab_view);
  display_imu_tab(tab_view);

  init_button_handler();
}

static void tab_event_cb(lv_obj_t* slider, lv_event_t event){
    if(event == LV_EVENT_VALUE_CHANGED) {
        lv_tabview_ext_t* ext = (lv_tabview_ext_t*) lv_obj_get_ext_attr(tab_view);
        const char* tab_name = ext->tab_name_ptr[lv_tabview_get_tab_act(tab_view)];
        ESP_LOGI(TAG, "Current Active Tab: %s\n", tab_name);

        vTaskSuspend(IMU_handle);
        //vTaskSuspend(test_handle);


        if(strcmp(tab_name, TEST_TAB_NAME) == 0)
        {
          //TODO
        
        }
        else if(strcmp(tab_name, IMU_TAB_NAME) == 0)
        {
          vTaskResume(IMU_handle);
        }
    }
}
