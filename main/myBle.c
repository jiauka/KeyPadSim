
/*

 
m3 STOP
03 f4 01 52 03 01 00 
00 f4 01 52 03 01 00 

DY STOP

07 08 00 08 fc 0e 01 
00 08 00 08 fc 0e 00 
AUTO
07 07 70 07 70 07 01 
00 07 70 07 70 07 00 

UP

00 07 70 07 20 03 00 
00 07 70 07 10 03 00 
07 07 70 07 80 0c 01 
dy Key =07800c
07 07 70 07 80 0b 01 
dy Key =07800b
07 07 70 07 28 0a 01 
dy Key =07280a
07 07 70 07 28 09 01 
dy Key =072809
07 07 70 07 28 08 01 
dy Key =072808
07 07 70 07 d0 07 01 
dy Key =07d007
07 07 70 07 4c 06 01 
dy Key =074c06
07 07 70 07 4c 05 01 
dy Key =074c05
07 07 70 07 4c 04 01 
dy Key =074c04
07 07 70 07 20 03 01 
dy Key =072003
R
* 00 07 ea 0b 87 04 00 
00 07 ea 0b 87 04 00 
07 07 32 01 87 04 01 
dy Key =018704
07 07 8a 02 87 04 01 
dy Key =028704
[JCB Esp32RfCallback] k=0,a=0
07 07 e2 04 87 04 01 
dy Key =048704
07 07 c4 05 87 04 01 
dy Key =058704
07 07 6c 06 87 04 01 
dy Key =068704
07 07 40 07 87 04 01 
dy Key =078704
07 07 50 08 87 04 01 
dy Key =088704
07 07 70 09 87 04 01 
dy Key =098704
07 07 40 0a 87 04 01 
dy Key =0a8704
07 07 ea 0b 87 04 01 
dy Key =0b8704

DW
* 
07 07 70 07 f4 02 01 
07 07 70 07 20 03 01 
07 07 70 07 4c 04 01 
07 07 70 07 4c 05 01 
07 07 70 07 28 06 01 
07 07 70 07 d0 07 01 
07 07 70 07 4c 08 01 
07 07 70 07 4c 09 01 
07 07 70 07 28 0a 01 
07 07 70 07 54 0b 01 
00 07 70 07 54 0b 00 
00 07 70 07 10 0d 00 

 
*/

#include "esp_log.h"
#include "nvs_flash.h"
/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "blecent.h"
#include "myBle.h"

#define BLE_ENCRYPTION


#define R1_CODE_KEY_DW 0x02
#define R1_CODE_KEY_UP 0x01
#define R1_CODE_KEY_R 0x20
#define R1_CODE_KEY_L 0x10
#define R1_CODE_KEY_A 0x04
#define R1_CODE_KEY_STOP 0x250
#define R1_CODE_KEY_NONE 0x00
#define R1_CODE_KEY_STOP_END 0x50

#define M3_CODE_KEY_DW 0xf40120
#define M3_CODE_KEY_UP 0xf401c8
#define M3_CODE_KEY_R 0x20032c
#define M3_CODE_KEY_L 0x90012c
#define M3_CODE_KEY_ON 0xf401f4
#define M3_CODE_KEY_STOP 0xf40152

#define DY_CODE_KEY_DW 0x7007540b
#define DY_CODE_KEY_UP 0x70072003
#define DY_CODE_KEY_R 0xea0b8704
#define DY_CODE_KEY_L 0xbc028704
#define DY_CODE_KEY_ON 0x70077007
#define DY_CODE_KEY_STOP 0x0008fc0e

static const char* TAG = "myBle";

const char* SERVER_NAME = "Magicsee";
const char* SERVER_NAME2 = "BLE-M3";
const char* SERVER_NAME3 = "DY Shutter"; 

typedef enum
{
  BLE_KEYPAD_NONE = 0,
  BLE_KEYPAD_R1,
  BLE_KEYPAD_M3,
  BLE_KEYPAD_DY,
} BLE_KEYPAD_t;

typedef struct
{
  AP10Action_t action;
  uint64_t pressedTime;
  bool alernate;
  bool pressed;
  uint32_t prevCode;
  uint32_t pressedCode;
  uint32_t code;
} ble_event_t;

static BLE_KEYPAD_t ble_keypad = BLE_KEYPAD_NONE;
static ble_event_t bleEvent;

static bool bleConnected = true;
TaskHandle_t receiverTask = NULL;

static QueueHandle_t s_esp_BLE_queue = NULL;
static bool pressed = false;
static bool r1Mode = false;

static const char* tag = "NimBLE_BLE_CENT";
static int blecent_gap_event(struct ble_gap_event* event, void* arg);

static esp32BleCallback _callback = NULL;

void SetEsp32BleCallback(esp32BleCallback callback)
{
  _callback = callback;
}

void m3_receiver_ble(void* pvParameter)
{
  while (bleConnected) {
    if (xQueueReceive(s_esp_BLE_queue, &bleEvent, 250UL / portTICK_PERIOD_MS) == pdFALSE) {
      if (bleEvent.prevCode == M3_CODE_KEY_R) {
        if (bleEvent.code == M3_CODE_KEY_STOP) {
          ESP_LOGE(TAG, "CODE_KEY_R 1 %lx", bleEvent.code);
          _callback(AP10KeyPlusOne, AP10ActionStart);

        } else {
          ESP_LOGE(TAG, "CODE_KEY_R 10 %lx", bleEvent.code);
          _callback(AP10KeyPlusTen, AP10ActionStart);
        }
        memset(&bleEvent, 0, sizeof(ble_event_t));

      } else if (bleEvent.prevCode == M3_CODE_KEY_L) {
        if (bleEvent.code == M3_CODE_KEY_STOP) {
          ESP_LOGE(TAG, "CODE_KEY_L 1 %lx", bleEvent.code);
          _callback(AP10KeyMinusOne, AP10ActionStart);

        } else {
          ESP_LOGE(TAG, "CODE_KEY_L 10 %lx", bleEvent.code);
          _callback(AP10KeyMinusTen, AP10ActionStart);
        }
        memset(&bleEvent, 0, sizeof(ble_event_t));
      } else if (bleEvent.prevCode == M3_CODE_KEY_ON) {
        if (bleEvent.code == M3_CODE_KEY_STOP) {
          ESP_LOGE(TAG, "CODE_KEY_ON 1 %lx", bleEvent.code);
          _callback(AP10KeyAuto, AP10ActionStart);

        } else {
          ESP_LOGE(TAG, "CODE_KEY_ON 10 %lx", bleEvent.code);
          _callback(AP10KeyMode, AP10ActionStart);
        }
        memset(&bleEvent, 0, sizeof(ble_event_t));
      }
    } else { // before 250ms
      //      ESP_LOGE(TAG, "receiver_ble Received %04x action=%d delay %llu\n", bleEvent.code,bleEvent.action,esp_timer_get_time()- bleEvent.pressedTime);
      if (bleEvent.action == AP10ActionStart) {
        bleEvent.prevCode = bleEvent.code;
        if (bleEvent.code == M3_CODE_KEY_STOP) {
          //          ESP_LOGE(TAG, "CODE_KEY_STOP");
          _callback(AP10KeyStandby, AP10ActionStart);

          memset(&bleEvent, 0, sizeof(ble_event_t));
        } else if (bleEvent.code == M3_CODE_KEY_UP) {
          //          ESP_LOGE(TAG, "CODE_KEY_UP");
          _callback(AP10KeyPlusTen, AP10ActionStart);
          memset(&bleEvent, 0, sizeof(ble_event_t));
        } else if (bleEvent.code == M3_CODE_KEY_DW) {
          //          ESP_LOGE(TAG, "CODE_KEY_DW");
          _callback(AP10KeyMinusTen, AP10ActionStart);
          memset(&bleEvent, 0, sizeof(ble_event_t));
        }
      } else if (bleEvent.action == AP10ActionContinue) { // repeat key
        if (bleEvent.code == M3_CODE_KEY_R) {
          _callback(AP10KeyPlusOne, AP10ActionStart);
        } else if (bleEvent.code == M3_CODE_KEY_L) {
          _callback(AP10KeyMinusOne, AP10ActionStart);
        }
        memset(&bleEvent, 0, sizeof(ble_event_t));
      }
    }
  }
}

void ble_store_config_init(void);

/**
 * Application callback.  Called when the attempt to subscribe to notifications
 * for the ANS Unread Alert Status characteristic has completed.
 */
static int blecent_on_subscribe(uint16_t conn_handle, const struct ble_gatt_error* error, struct ble_gatt_attr* attr, void* arg)
{
  struct peer* peer;

  MODLOG_DFLT(INFO,
              "Subscribe complete; status=%d conn_handle=%d "
              "attr_handle=%d\n",
              error->status,
              conn_handle,
              attr->handle);

  peer = peer_find(conn_handle);
  if (peer == NULL) {
    MODLOG_DFLT(ERROR, "Error in finding peer, aborting...");
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
  }
  /* Subscribe to, write to, and read the custom characteristic*/
  //  blecent_custom_gatt_operations(peer);

  return 0;
}

/**
 * Application callback.  Called when the write to the ANS Alert Notification
 * Control Point characteristic has completed.
 */
static int blecent_on_write(uint16_t conn_handle, const struct ble_gatt_error* error, struct ble_gatt_attr* attr, void* arg)
{
  MODLOG_DFLT(INFO, "Write complete; status=%d conn_handle=%d attr_handle=%d\n", error->status, conn_handle, attr->handle);

  /* Subscribe to notifications for the Unread Alert Status characteristic.
   * A central enables notifications by writing two bytes (1, 0) to the
   * characteristic's client-characteristic-configuration-descriptor (CCCD).
   */
  const struct peer_dsc* dsc;
  uint8_t value[2];
  int rc;
  const struct peer* peer = peer_find(conn_handle);

  dsc = peer_dsc_find_uuid(peer, BLE_UUID16_DECLARE(HID_SERVICE), BLE_UUID16_DECLARE(HID_REPORT_DATA), BLE_UUID16_DECLARE(HID_CHAR_NOTIFY));
  if (dsc == NULL) {
    MODLOG_DFLT(ERROR,
                "Error: Peer lacks a CCCD for the Unread Alert "
                "Status characteristic\n");
    goto err;
  }

  value[0] = 1;
  value[1] = 0;
  rc = ble_gattc_write_flat(conn_handle, dsc->dsc.handle, value, sizeof value, blecent_on_subscribe, NULL);
  if (rc != 0) {
    MODLOG_DFLT(ERROR,
                "Error: Failed to subscribe to characteristic; "
                "rc=%d\n",
                rc);
    goto err;
  }

  return 0;
err:
  /* Terminate the connection. */
  return ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

/**
 * Application callback.  Called when the read of the ANS Supported New Alert
 * Category characteristic has completed.
 */
static int blecent_on_read(uint16_t conn_handle, const struct ble_gatt_error* error, struct ble_gatt_attr* attr, void* arg)
{
  MODLOG_DFLT(INFO, "Read complete; status=%d conn_handle=%d", error->status, conn_handle);
  if (error->status == 0) {
    MODLOG_DFLT(INFO, " attr_handle=%d value=", attr->handle);
    print_mbuf(attr->om);
  }
  MODLOG_DFLT(INFO, "\n");

  /* Write two bytes (99, 100) to the alert-notification-control-point
   * characteristic.
   */
  const struct peer_chr* chr;
  uint8_t value[2];
  int rc;
  const struct peer* peer = peer_find(conn_handle);

  chr = peer_chr_find_uuid(peer, BLE_UUID16_DECLARE(HID_SERVICE), BLE_UUID16_DECLARE(HID_REPORT_DATA));
  if (chr == NULL) {
    MODLOG_DFLT(ERROR,
                "Error: Peer doesn't support the Alert "
                "Notification Control Point characteristic\n");
    goto err;
  }

  value[0] = 99;
  value[1] = 100;
  rc = ble_gattc_write_flat(conn_handle, chr->chr.val_handle, value, sizeof value, blecent_on_write, NULL);
  if (rc != 0) {
    MODLOG_DFLT(ERROR, "Error: Failed to write characteristic; rc=%d\n", rc);
    goto err;
  }

  return 0;
err:
  /* Terminate the connection. */
  return ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

/**
 * Performs three GATT operations against the specified peer:
 * 1. Reads the ANS Supported New Alert Category characteristic.
 * 2. After read is completed, writes the ANS Alert Notification Control Point characteristic.
 * 3. After write is completed, subscribes to notifications for the ANS Unread Alert Status
 *    characteristic.
 *
 * If the peer does not support a required service, characteristic, or
 * descriptor, then the peer lied when it claimed support for the alert
 * notification service!  When this happens, or if a GATT procedure fails,
 * this function immediately terminates the connection.
 */
static void blecent_read_write_subscribe(const struct peer* peer)
{
  const struct peer_chr* chr;
  int rc;

  /* Read the supported-new-alert-category characteristic. */
  chr = peer_chr_find_uuid(peer, BLE_UUID16_DECLARE(HID_SERVICE), BLE_UUID16_DECLARE(HID_REPORT_DATA));
  if (chr == NULL) {
    MODLOG_DFLT(ERROR,
                "Error: Peer doesn't support the Supported New "
                "Alert Category characteristic\n");
    goto err;
  }

  rc = ble_gattc_read(peer->conn_handle, chr->chr.val_handle, blecent_on_read, NULL);
  if (rc != 0) {
    MODLOG_DFLT(ERROR, "Error: Failed to read characteristic; rc=%d\n", rc);
    goto err;
  }

  return;
err:
  /* Terminate the connection. */
  ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

/**
 * Called when service discovery of the specified peer has completed.
 */
static void blecent_on_disc_complete(const struct peer* peer, int status, void* arg)
{

  if (status != 0) {
    /* Service discovery failed.  Terminate the connection. */
    MODLOG_DFLT(ERROR,
                "Error: Service discovery failed; status=%d "
                "conn_handle=%d\n",
                status,
                peer->conn_handle);
    ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    return;
  }

  /* Service discovery has completed successfully.  Now we have a complete
   * list of services, characteristics, and descriptors that the peer
   * supports.
   */
  MODLOG_DFLT(INFO,
              "Service discovery complete; status=%d "
              "conn_handle=%d\n",
              status,
              peer->conn_handle);

  /* Now perform three GATT procedures against the peer: read,
   * write, and subscribe to notifications for the ANS service.
   */
  blecent_read_write_subscribe(peer);
}

/**
 * Initiates the GAP general discovery procedure.
 */
static void blecent_scan(void)
{
  uint8_t own_addr_type;
  struct ble_gap_disc_params disc_params;
  int rc;

  /* Figure out address to use while advertising (no privacy for now) */
  rc = ble_hs_id_infer_auto(0, &own_addr_type);
  if (rc != 0) {
    MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
    return;
  }

  /* Tell the controller to filter duplicates; we don't want to process
   * repeated advertisements from the same device.
   */
  disc_params.filter_duplicates = 1;

  /**
   * Perform a passive scan.  I.e., don't send follow-up scan requests to
   * each advertiser.
   */
  disc_params.passive = 1;

  /* Use defaults for the rest of the parameters. */
  disc_params.itvl = 0;
  disc_params.window = 0;
  disc_params.filter_policy = 0;
  disc_params.limited = 0;

  rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, blecent_gap_event, NULL);
  if (rc != 0) {
    MODLOG_DFLT(ERROR, "Error initiating GAP discovery procedure; rc=%d\n", rc);
  }
}

/**
 * Indicates whether we should try to connect to the sender of the specified
 * advertisement.  The function returns a positive result if the device
 * advertises connectability and support for the Alert Notification service.
 */

static int blecent_should_connect(const struct ble_gap_disc_desc* disc)
{
  struct ble_hs_adv_fields fields;
  int rc;

  /* The device has to be advertising connectability. */
  if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND && disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {

    return 0;
  }

  rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
  if (rc != 0) {
    return 0;
  }

  if (fields.name_len > 0) {
    char name[256];
    memset(name,0,sizeof(name));
    memcpy(name,fields.name,fields.name_len);
    printf("Trying %s\n\r",name);
    if (strstr((char*)fields.name, SERVER_NAME)) {
      ble_keypad = BLE_KEYPAD_R1;
      return 1;
    }
    if (strstr((char*)fields.name, SERVER_NAME2)) {
      ble_keypad = BLE_KEYPAD_M3;
      return 1;
    }
    if (strstr((char*)fields.name, SERVER_NAME3)) {
      ble_keypad = BLE_KEYPAD_DY;
      return 1;
    }
  
}
  return 0;
}

/**
 * Connects to the sender of the specified advertisement of it looks
 * interesting.  A device is "interesting" if it advertises connectability and
 * support for the Alert Notification service.
 */
static void blecent_connect_if_interesting(void* disc)
{
  uint8_t own_addr_type;
  int rc;
  ble_addr_t* addr;

  /* Don't do anything if we don't care about this advertiser. */
  if (!blecent_should_connect((struct ble_gap_disc_desc*)disc)) {
    return;
  }

  /* Scanning must be stopped before a connection can be initiated. */
  rc = ble_gap_disc_cancel();
  if (rc != 0) {
    MODLOG_DFLT(DEBUG, "Failed to cancel scan; rc=%d\n", rc);
    return;
  }

  /* Figure out address to use for connect (no privacy for now) */
  rc = ble_hs_id_infer_auto(0, &own_addr_type);
  if (rc != 0) {
    MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
    return;
  }

  /* Try to connect the the advertiser.  Allow 30 seconds (30000 ms) for
   * timeout.
   */
  addr = &((struct ble_gap_disc_desc*)disc)->addr;

  rc = ble_gap_connect(own_addr_type, addr, 30000, NULL, blecent_gap_event, NULL);
  if (rc != 0) {
    MODLOG_DFLT(ERROR,
                "Error: Failed to connect to device; addr_type=%d "
                "addr=%s; rc=%d\n",
                addr->type,
                addr_str(addr->val),
                rc);
    return;
  }
}



#if MYNEWT_VAL(BLE_POWER_CONTROL)
static void blecent_power_control(uint16_t conn_handle)
{
  int rc;

  rc = ble_gap_read_remote_transmit_power_level(conn_handle, 0x01); // Attempting on LE 1M phy
  assert(rc == 0);

  rc = ble_gap_set_transmit_power_reporting_enable(conn_handle, 0x01, 0x01);
  assert(rc == 0);

  rc = ble_gap_set_path_loss_reporting_param(conn_handle, 60, 10, 30, 10, 2); // demo values
  assert(rc == 0);

  rc = ble_gap_set_path_loss_reporting_enable(conn_handle, 0x01);
  assert(rc == 0);
}
#endif

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that is
 * established.  blecent uses the same callback for all connections.
 *
 * @param event                 The event being signalled.
 * @param arg                   Application-specified argument; unused by
 *                                  blecent.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int blecent_gap_event(struct ble_gap_event* event, void* arg)
{
  struct ble_gap_conn_desc desc;
  struct ble_hs_adv_fields fields;
  int rc;
  uint8_t data[128];

  switch (event->type) {
    case BLE_GAP_EVENT_DISC:
      rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
      if (rc != 0) {
        return 0;
      }

      /* An advertisment report was received during GAP discovery. */
      print_adv_fields(&fields);

      /* Try to connect to the advertiser if it looks interesting. */
      blecent_connect_if_interesting(&event->disc);
      return 0;

    case BLE_GAP_EVENT_CONNECT:
      /* A new connection was established or a connection attempt failed. */
      if (event->connect.status == 0) {
        /* Connection successfully established. */
        MODLOG_DFLT(INFO, "Connection established ");

        rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
        assert(rc == 0);
        print_conn_desc(&desc);
        MODLOG_DFLT(INFO, "\n");

        /* Remember peer. */
        rc = peer_add(event->connect.conn_handle);
        if (rc != 0) {
          MODLOG_DFLT(ERROR, "Failed to add peer; rc=%d\n", rc);
          return 0;
        }

#if MYNEWT_VAL(BLE_POWER_CONTROL)
        blecent_power_control(event->connect.conn_handle);
#endif

#if MYNEWT_VAL(BLE_HCI_VS)
#if MYNEWT_VAL(BLE_POWER_CONTROL)
        int8_t vs_cmd[10] = { 0, 0, -70, -60, -68, -58, -75, -65, -80, -70 };

        vs_cmd[0] = ((uint8_t)(event->connect.conn_handle & 0xFF));
        vs_cmd[1] = ((uint8_t)(event->connect.conn_handle >> 8) & 0xFF);

        rc = ble_hs_hci_send_vs_cmd(BLE_HCI_OCF_VS_PCL_SET_RSSI, &vs_cmd, sizeof(vs_cmd), NULL, 0);
        if (rc != 0) {
          MODLOG_DFLT(INFO, "Failed to send VSC  %x \n", rc);
          return 0;
        } else
          MODLOG_DFLT(INFO, "Successfully issued VSC , rc = %d \n", rc);
#endif
#endif

#ifdef BLE_ENCRYPTION

        /** Initiate security - It will perform
         * Pairing (Exchange keys)
         * Bonding (Store keys)
         * Encryption (Enable encryption)
         * Will invoke event BLE_GAP_EVENT_ENC_CHANGE
         **/
        rc = ble_gap_security_initiate(event->connect.conn_handle);
        if (rc != 0) {
          MODLOG_DFLT(INFO, "Security could not be initiated, rc = %d\n", rc);
          return ble_gap_terminate(event->connect.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        } else {
          MODLOG_DFLT(INFO, "Connection secured\n");
        }
#else
        /* Perform service discovery */
        rc = peer_disc_all(event->connect.conn_handle, blecent_on_disc_complete, NULL);
        if (rc != 0) {
          MODLOG_DFLT(ERROR, "Failed to discover services; rc=%d\n", rc);
          return 0;
        }
#endif
        bleConnected = true;
        if (receiverTask) {
          vTaskDelete(receiverTask);
          receiverTask = NULL;
        }
        if (ble_keypad == BLE_KEYPAD_M3) {
          xTaskCreate(&m3_receiver_ble, "m3_receiver_ble", 2048 * 2, NULL, 3, &receiverTask);
        }

      } else {
        /* Connection attempt failed; resume scanning. */
        MODLOG_DFLT(ERROR, "Error: Connection failed; status=%d\n", event->connect.status);
        blecent_scan();
      }

      return 0;

    case BLE_GAP_EVENT_DISCONNECT:
      /* Connection terminated. */
      MODLOG_DFLT(INFO, "disconnect; reason=%d ", event->disconnect.reason);
      print_conn_desc(&event->disconnect.conn);
      MODLOG_DFLT(INFO, "\n");

      /* Forget about peer. */
      peer_delete(event->disconnect.conn.conn_handle);

      /* Resume scanning. */
      blecent_scan();
      return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
      MODLOG_DFLT(INFO, "discovery complete; reason=%d\n", event->disc_complete.reason);
      return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
      /* Encryption has been enabled or disabled for this connection. */
      MODLOG_DFLT(INFO, "encryption change event; status=%d ", event->enc_change.status);
      rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
      assert(rc == 0);
      print_conn_desc(&desc);
#ifdef BLE_ENCRYPTION
      /*** Go for service discovery after encryption has been successfully enabled ***/
      rc = peer_disc_all(event->connect.conn_handle, blecent_on_disc_complete, NULL);
      if (rc != 0) {
        MODLOG_DFLT(ERROR, "Failed to discover services; rc=%d\n", rc);
        return 0;
      }
#endif
      return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
      /* Peer sent us a notification or indication. */
#if 1
      MODLOG_DFLT(INFO,
                  "received %s; conn_handle=%d attr_handle=%d "
                  "attr_len=%d\n",
                  event->notify_rx.indication ? "indication" : "notification",
                  event->notify_rx.conn_handle,
                  event->notify_rx.attr_handle,
                  OS_MBUF_PKTLEN(event->notify_rx.om));
#endif
      /* Attribute data is contained in event->notify_rx.om. Use
       * `os_mbuf_copydata` to copy the data received in notification mbuf */
      int len = OS_MBUF_PKTLEN(event->notify_rx.om);
      memset(data, 0, sizeof(data));
      os_mbuf_copydata(event->notify_rx.om, 0, len, data);
#if 0
      for (int i = 0; i < len; i++) {
        printf("%02x ", data[i]);
      }
      printf("\n\r");
#endif

      if (ble_keypad == BLE_KEYPAD_M3) {
        uint32_t tmpKey = data[1] << 16 | data[2] << 8 | data[3];
        if (data[0] == 0x03 && (!pressed || bleEvent.pressedCode == tmpKey)) { // and repeat key
          pressed = true;
          if (esp_timer_get_time() - bleEvent.pressedTime < 300 * 1000 && (bleEvent.pressedCode == tmpKey)) { // repeat key
            bleEvent.pressedTime = esp_timer_get_time();
            if (bleEvent.pressedCode == M3_CODE_KEY_L) {
              _callback(AP10KeyMinusOne, AP10ActionStart);
            } else if (bleEvent.pressedCode == M3_CODE_KEY_R) {
              _callback(AP10KeyPlusOne, AP10ActionStart);
            }

          } else {
            bleEvent.action = AP10ActionStart;
            bleEvent.pressedTime = esp_timer_get_time();
            bleEvent.code = tmpKey;
            bleEvent.pressedCode = tmpKey;
            xQueueSend(s_esp_BLE_queue, &bleEvent, (TickType_t)0);
          }
        }
        if (data[0] == 0x00) {
          pressed = false;
          bleEvent.code = tmpKey;
          bleEvent.action = AP10ActionStop;
          xQueueSend(s_esp_BLE_queue, &bleEvent, (TickType_t)0);
        }
      } //(ble_keypad == BLE_KEYPAD_M3)
      else if (ble_keypad == BLE_KEYPAD_R1) {
        uint32_t key = 0;
        for(int i= 0; i < len ; i++) {
          key= key << 8 |data[i];
        }
        if(_callback) {
          switch(key) {
            case R1_CODE_KEY_NONE:
              if(bleEvent.code == R1_CODE_KEY_R) {
                if (esp_timer_get_time() - bleEvent.pressedTime < 400 * 1000) {
                  _callback(AP10KeyPlusOne,AP10ActionStart);
                } else { // long press
                  _callback(AP10KeyPlusTen,AP10ActionStart);
                }
                
                memset(&bleEvent, 0, sizeof(ble_event_t));
              }
              else if(bleEvent.code == R1_CODE_KEY_L) {
                if (esp_timer_get_time() - bleEvent.pressedTime < 400 * 1000) {
                  _callback(AP10KeyMinusOne,AP10ActionStart);
                } else { // long press
                  _callback(AP10KeyMinusTen,AP10ActionStart);
                }
                
                memset(&bleEvent, 0, sizeof(ble_event_t));
              }
            break;
            case R1_CODE_KEY_STOP:
              _callback(AP10KeyStandby,AP10ActionStart);
              r1Mode=false;
            break;
            case R1_CODE_KEY_A:
              if(r1Mode) {
                _callback(AP10KeyMode,AP10ActionStart);
              } else {
                _callback(AP10KeyAuto,AP10ActionStart);
              }
              r1Mode=!r1Mode;
            break;
            case R1_CODE_KEY_UP:
              _callback(AP10KeyPlusTen,AP10ActionStart);
            break;
            case R1_CODE_KEY_DW:
              _callback(AP10KeyMinusTen,AP10ActionStart);
            break;
            case R1_CODE_KEY_R:
              pressed = true;
              bleEvent.action = AP10ActionStart;
              bleEvent.pressedTime = esp_timer_get_time();
              bleEvent.code = key;
              bleEvent.pressedCode = key;
              //xQueueSend(s_esp_BLE_queue, &bleEvent, (TickType_t)0);
            break;
            case R1_CODE_KEY_L:
              pressed = true;
              bleEvent.action = AP10ActionStart;
              bleEvent.pressedTime = esp_timer_get_time();
              bleEvent.code = key;
              bleEvent.pressedCode = key;
            break;
            default:
            break;
          }
        }
      } else if (ble_keypad == BLE_KEYPAD_DY) {
        uint32_t key = 0;
        key = data[2] << 24|data[3] << 16| data[4] << 8 |data[5];
        if(data[0] == 0x00) { // key stop
          memset(&bleEvent, 0, sizeof(ble_event_t));
        } else {
          MODLOG_DFLT(INFO,"dy Key =%08lx", key);
          if(_callback) {
            switch(key) {
              case DY_CODE_KEY_STOP:
                _callback(AP10KeyStandby,AP10ActionStart);
                r1Mode=false;
              break;
              case DY_CODE_KEY_ON:
                if(r1Mode) {
                  _callback(AP10KeyMode,AP10ActionStart);
                } else {
                  _callback(AP10KeyAuto,AP10ActionStart);
                }
                r1Mode=!r1Mode;
              break;
              case DY_CODE_KEY_UP:
                _callback(AP10KeyPlusTen,AP10ActionStart);
              break;
              case DY_CODE_KEY_DW:
                _callback(AP10KeyMinusTen,AP10ActionStart);
              break;
              case DY_CODE_KEY_R:
                _callback(AP10KeyPlusOne,AP10ActionStart);
              break;
              case DY_CODE_KEY_L:
                _callback(AP10KeyMinusOne,AP10ActionStart);
              break;
              default:
              break;
            }
          }
        }
      }
      return 0;

    case BLE_GAP_EVENT_MTU:
      MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n", event->mtu.conn_handle, event->mtu.channel_id, event->mtu.value);
      return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
      /* We already have a bond with the peer, but it is attempting to
       * establish a new secure link.  This app sacrifices security for
       * convenience: just throw away the old bond and accept the new link.
       */

      /* Delete the old bond. */
      rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
      assert(rc == 0);
      ble_store_util_delete_peer(&desc.peer_id_addr);

      /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
       * continue with the pairing operation.
       */
      return BLE_GAP_REPEAT_PAIRING_RETRY;

#if MYNEWT_VAL(BLE_POWER_CONTROL)
    case BLE_GAP_EVENT_TRANSMIT_POWER:
      MODLOG_DFLT(INFO,
                  "Transmit power event : status=%d conn_handle=%d reason=%d "
                  "phy=%d power_level=%d power_level_flag=%d delta=%d",
                  event->transmit_power.status,
                  event->transmit_power.conn_handle,
                  event->transmit_power.reason,
                  event->transmit_power.phy,
                  event->transmit_power.transmit_power_level,
                  event->transmit_power.transmit_power_level_flag,
                  event->transmit_power.delta);
      return 0;

    case BLE_GAP_EVENT_PATHLOSS_THRESHOLD:
      MODLOG_DFLT(INFO,
                  "Pathloss threshold event : conn_handle=%d current path loss=%d "
                  "zone_entered =%d",
                  event->pathloss_threshold.conn_handle,
                  event->pathloss_threshold.current_path_loss,
                  event->pathloss_threshold.zone_entered);
      return 0;
#endif
    default:
      return 0;
  }
}

static void blecent_on_reset(int reason)
{
  MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void blecent_on_sync(void)
{
  int rc;

  /* Make sure we have proper identity address set (public preferred) */
  rc = ble_hs_util_ensure_addr(0);
  assert(rc == 0);

#if !CONFIG_EXAMPLE_INIT_DEINIT_LOOP
  /* Begin scanning for a peripheral to connect to. */
  blecent_scan();
#endif
}

void blecent_host_task(void* param)
{
  ESP_LOGI(tag, "BLE Host Task Started");
  /* This function will return only when nimble_port_stop() is executed */
  nimble_port_run();

  nimble_port_freertos_deinit();
}

#if CONFIG_EXAMPLE_INIT_DEINIT_LOOP
/* This function showcases stack init and deinit procedure. */
static void stack_init_deinit(void)
{
  int rc;
  while (1) {

    vTaskDelay(1000);

    ESP_LOGI(tag, "Deinit host");

    rc = nimble_port_stop();
    if (rc == 0) {
      nimble_port_deinit();
    } else {
      ESP_LOGI(tag, "Nimble port stop failed, rc = %d", rc);
      break;
    }

    vTaskDelay(1000);

    ESP_LOGI(tag, "Init host");

    rc = nimble_port_init();
    if (rc != ESP_OK) {
      ESP_LOGI(tag, "Failed to init nimble %d ", rc);
      break;
    }

    nimble_port_freertos_init(blecent_host_task);

    ESP_LOGI(tag, "Waiting for 1 second");
  }
}
#endif

void BleInit(void)
{
  int rc;
  /* Initialize NVS — it is used to store PHY calibration data */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  nimble_port_init();

  /* Configure the host. */
  ble_hs_cfg.reset_cb = blecent_on_reset;
  ble_hs_cfg.sync_cb = blecent_on_sync;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

  /* Initialize data structures to track connected peers. */
  rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
  assert(rc == 0);

  /* Set the default device name. */
  rc = ble_svc_gap_device_name_set("nimble-blecent");
  assert(rc == 0);

  /* XXX Need to have template for store */
  ble_store_config_init();

  nimble_port_freertos_init(blecent_host_task);
  if (s_esp_BLE_queue == NULL) {
    s_esp_BLE_queue = xQueueCreate(1, sizeof(ble_event_t));
  }
}
