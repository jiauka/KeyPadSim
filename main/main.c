
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
/* Enable this to show verbose logging for this file only. */
// #define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include "myBle.h"
#include "OwnN2K.h"

static const char* TAG = "main";

// Main routine.
void app_main(void)
{
  vTaskDelay(1000UL / portTICK_PERIOD_MS);
  OwnN2KInit();
  // esp32RfStart();

#ifdef CONFIG_USE_BLE_YES
  BleInit();
  // setup();
#endif
  while (1) {
    // loop();
    vTaskDelay(10UL / portTICK_PERIOD_MS);
    // ESP_LOGD(TAG, "free stack: %d", uxTaskGetStackHighWaterMark(NULL));
  }
}
