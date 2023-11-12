
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE /* Enable this to show verbose logging for this file only. */
#include "esp_log.h"
#include <N2kMsg.h>
#include <NMEA2000.h> // https://github.com/ttlappalainen/NMEA2000
#include "sdkconfig.h"
#ifdef CONFIG_ESP32_CAN_TX_PIN
#define ESP32_CAN_TX_PIN (gpio_num_t) CONFIG_ESP32_CAN_TX_PIN
#endif
#ifdef CONFIG_ESP32_CAN_RX_PIN
#define ESP32_CAN_RX_PIN (gpio_num_t) CONFIG_ESP32_CAN_RX_PIN
#endif
#include <NMEA2000_esp32xx.h> // https://github.com/ttlappalainen/NMEA2000_esp32
#include "N2kMessages.h"
#include "ESP32N2kStream.h"
#include "../../main/myBle.h"
#include "led_strip.h"

static const char* TAG = "OwnN2k";
tNMEA2000_esp32xx NMEA2000;
// tActisenseReader ActisenseReader;
static uint8_t apSource = 0;
static bool mustBlink = false;
static TaskHandle_t N2K_task_handle = NULL;
ESP32N2kStream Serial;

// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM = { 130850L, 0 };

#if 0
	    debug('Emulate: Simrad AC12 Autopilot')
     debug('Emulate: B&G OP10 Keypad')
      setInterval(PGN130822, 300000) // Every 5 minutes
      setInterval(OP10_PGN65305, 1000) // unsure
      setInterval(heartbeat, 60000) // Heart beat PGN
 	    break;
#endif

static void SendN2kPGN130850(AP10Keys_t key);

static led_strip_handle_t led_strip;

static void blink_led(uint8_t s_led_state)
{
  /* If the addressable LED is enabled */
  if (s_led_state) {
    /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
    led_strip_set_pixel(led_strip, 0, 0, 8, 0);
    /* Refresh the strip to send data */
    led_strip_refresh(led_strip);
  } else {
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
  }
}

static void configure_led(void)
{
  /* LED strip initialization with the GPIO and pixels number*/
  led_strip_config_t strip_config = {
    .strip_gpio_num = 8,
    .max_leds = 1, // at least one LED on board
  };
  led_strip_rmt_config_t rmt_config = {
    .resolution_hz = 10 * 1000 * 1000, // 10MHz
  };
  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
  /* Set all LED off to clear all pixels */
  led_strip_clear(led_strip);
}

static void Esp32RfCallback(AP10Keys_t key, AP10Action_t action)
{
  printf("[JCB Esp32RfCallback] k=%d,a=%d\n", (int)key, (int)action);
  mustBlink = true;
  SendN2kPGN130850(key);
}

// *****************************************************************************
// Call back for NMEA2000 open. This will be called, when library starts bus communication.
// See NMEA2000.SetOnOpen(OnN2kOpen); on setup()
void OnN2kOpen()
{
  // Start schedulers now.
}

void HandleStreamN2kMsg(const tN2kMsg& N2kMsg)
{
  // N2kMsg.Print(&Serial);
  NMEA2000.SendMsg(N2kMsg);
}

// NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg& N2kMsg)
{
  if (N2kMsg.PGN == 65480) {
    apSource = N2kMsg.Source;
    printf("HandleNMEA2000Msg PGN: %ld source %d \n\r", N2kMsg.PGN, apSource);

  } else if (N2kMsg.PGN == 65341) {
    apSource = N2kMsg.Source;
    printf("HandleNMEA2000Msg PGN: %ld source %d \n\r", N2kMsg.PGN, apSource);
  }
  // printf("HandleNMEA2000Msg PGN: %ld\n\r",N2kMsg.PGN);
}

/*

const op10_key_code = {
    "+1":      "02,ff,ff,02,0d,00,04",
    "+10":     "02,ff,ff,02,0d,00,05",
    "-1":      "02,ff,ff,02,0d,00,ff",
    "-10":     "02,ff,ff,0a,06,00,00",
    "auto":    "02,ff,ff,0a,09,00,00",
    "mode":    "02,ff,ff,0a,0f,00,00",
    "standby": "02,ff,ff,0a,1c,00,00",
    "-1-10":   "",
    "+1+10":   ""
}
AC12:
*   const message = "%s,2,130850,%s,255,0c,41,9f,ff,ff,64,00,2b,00,ff,ff,ff,ff,ff"
function AP10_PGN130850 () {
  const op10msg = '%s,2,130850,%s,255,0c,41,9f,%s,ff,ff,%s,00,00,00,ff';
  msg = util.format(op10msg, (new Date()).toISOString(), canbus.candevice.address, hexByte(canbus.candevice.address), op10_key_code[button]);
  canbus.sendPGN(msg)
}
*/

/*

function AC12_PGN130850 () {
  const message = "%s,2,130850,%s,255,0c,41,9f,ff,ff,64,00,2b,00,ff,ff,ff,ff,ff"
  msg = util.format(message, (new Date()).toISOString(), canbus.candevice.address)
  canbus.sendPGN(msg)
}
*/
const uint8_t* op10_key_codeOld[] = {
  (uint8_t*)"\x02\xff\xff\x02\x0d\x00\x04", // +1 41 9F 00 FF FF 02 0D 00 05 00 00 00 // release
                                            //     41 9F 00 FF FF 02 0D 00 FF 00 00 00
  (uint8_t*)"\x02\xff\xff\x02\x0d\x00\x05", //+10 41 9F 00 FF FF 02 0D 00 FF 00 00 00

  (uint8_t*)"\x02\xff\xff\x02\x0d\x00\xff", // -1 41 9F 00 FF FF 02 0D 00 FF 00 00 00
  (uint8_t*)"\x02\xff\xff\x0a\x06\x00\x00", //-10 41 9F 00 FF FF 02 0D 00 04 00 00 00
                                            //     41 9F 00 FF FF 02 0D 00 ff 00 00 00
  (uint8_t*)"\x02\xff\xff\x0a\x09\x00\x00", // auto 41 9F 00 FF FF 0A 09 00 FF FF FF FF
  (uint8_t*)"\x02\xff\xff\x0a\x0f\x00\x00", // mode 41 9F 00 FF FF 0A 06 00 00 00 00 00
  (uint8_t*)"\x02\xff\xff\x0a\x1c\x00\x00"  // standby 41 9F 00 FF FF 0A 06 00 00 00 00 00
};
const uint8_t* op10_key_code[] = {
  (uint8_t*)"\x0A\x1A\x00\x03\xAF\x00\x00", // +1 41 9F 00 FF FF 02 0D 00 05 00 00 00
                                            //      41 9F 00 FF FF 02 0D 00 FF 00 00 00
  (uint8_t*)"\x0A\x1A\x00\x03\xD1\x06\x00", //+10 41 9F 00 FF FF 0A 1A 00 03 D1 06 00

  (uint8_t*)"\x0A\x1A\x00\x02\xAF\x00\x00", // -1 41 9F 00 FF FF 02 0D 00 FF 00 00 00
  (uint8_t*)"\x0A\x1A\x00\x02\xD1\x06\x00", //-10 41 9F 00 FF FF 0A 1A 00 02 D1 06 00
                                            //     41 9F 00 FF FF 02 0D 00 ff 00 00 00
  (uint8_t*)"\x0A\x09\x00\xFF\xFF\xFF\xFF", // auto 41 9F 00 FF FF 0A 09 00 FF FF FF FF
  (uint8_t*)"\x0a\x0f\x00\x00\x00\x00\x00", // mode 41 9F 00 FF FF 0A 0F 00 00 00 00 00
  (uint8_t*)"\x0a\x06\x00\x00\x00\x00\x00"  // standby 41 9F 00 FF FF 0A 06 00 00 00 00 00
};

//*     const op10msg = '%s,2,130850,%s,255,0c,41,9f,%s,ff,ff,%s,00,00,00,ff';

static void SendN2kPGN130850(AP10Keys_t key)
{
  tN2kMsg N2kMsg;
  N2kMsg.SetPGN(130850L);
  N2kMsg.Priority = 2;
  N2kMsg.AddByte(0x41);
  N2kMsg.AddByte(0x9f);
  N2kMsg.AddByte(apSource); // NMEA2000.GetN2kSource());
  N2kMsg.AddByte(0xff);
  N2kMsg.AddByte(0xff);
  N2kMsg.AddBuf((const void*)op10_key_code[key], 7);
  NMEA2000.SendMsg(N2kMsg);
}

// This is a FreeRTOS task
void N2K_task(void* pvParameters)
{
  //  Serial=new ESP32N2kStream();
  ESP_LOGI(TAG, "Starting task");
  NMEA2000.SetProductInformation("000001",                    // Manufacturer's Model serial code
                                 381,                         // Manufacturer's product code
                                 "Metis Remote Pilot Keypad", // Manufacturer's Model ID
                                 "0.1",                       // Manufacturer's Software version code
                                 "",                          // Manufacturer's Model version
                                 1,                           // LoadEquivalency
                                 1200,                        // N2kVersion
                                 2,                           // CertificationLevel
                                 0                            // iDev

  );
  // Set device information
  NMEA2000.SetDeviceInformation(1,   // Unique number. Use e.g. Serial number.
                                140, // Function
                                40,  // Device Class
                                1857 // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );

  NMEA2000.SetDeviceInformationInstances(0, // DeviceInstanceLower
                                         0, // DeviceInstanceUpper
                                         0, // SystemInstance
                                         0  // iDev
  );

  // Uncomment 2 rows below to see, what device will send to bus. Use e.g. OpenSkipper or Actisense NMEA Reader
  // Serial.begin(115200);
  // NMEA2000.SetForwardStream(&Serial);
  // If you want to use simple ascii monitor like Arduino Serial Monitor, uncomment next line
  // NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 23);
  // NMEA2000.SetDebugMode(tNMEA2000::dm_Actisense); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  // NMEA2000.SetForwardStream(&Serial); // PC output on native port
  // NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);   // Show in clear text
  // NMEA2000.EnableForward(true);
  // Here we tell library, which PGNs we transmit
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  // Define OnOpen call back. This will be called, when CAN is open and system starts address claiming.
  NMEA2000.SetOnOpen(OnN2kOpen);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.Open();
  // I originally had problem to use same Serial stream for reading and sending.
  // It worked for a while, but then stopped. Later it started to work.
  SetEsp32BleCallback(Esp32RfCallback);
  for (;;) {
    //    SendN2kRudder();
    NMEA2000.ParseMessages();
    if (mustBlink) {
      mustBlink = false;
      blink_led(1);
      vTaskDelay(10UL / portTICK_PERIOD_MS);
      blink_led(0);
    } else {
      vTaskDelay(10UL / portTICK_PERIOD_MS);
    }
  }
  vTaskDelete(NULL); // should never get here...
}

// was setup() in Arduino example:
extern "C" int OwnN2KInit(void)
{
  esp_err_t result = ESP_OK;
  configure_led();
  led_strip_set_pixel(led_strip, 0, 8, 0, 0);
  /* Refresh the strip to send data */
  led_strip_refresh(led_strip);

  ESP_LOGV(TAG, "create task");
  xTaskCreate(&N2K_task,            // Pointer to the task entry function.
              "N2K_task",           // A descriptive name for the task for debugging.
              3072,                 // size of the task stack in bytes.
              NULL,                 // Optional pointer to pvParameters
              tskIDLE_PRIORITY + 3, // priority at which the task should run
              &N2K_task_handle      // Optional pass back task handle
  );
  if (N2K_task_handle == NULL) {
    ESP_LOGE(TAG, "Unable to create task.");
    result = ESP_ERR_NO_MEM;
    goto err_out;
  }

err_out:
  if (result != ESP_OK) {
    if (N2K_task_handle != NULL) {
      vTaskDelete(N2K_task_handle);
      N2K_task_handle = NULL;
    }
  }

  return result;
}
