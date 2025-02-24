// NimBLE Client - Scan

#include "esp_log.h"
/* BLE */
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "blecent.h"

#include "myBle.h"
char* DEVICE_NAME = "MetisBLE";
const char* SERVER_NAME = "Magicsee";
const char* SERVER_NAME2 = "BLE-M3";
const char* SERVER_NAME3 = "PR BT 4917";

#define HID_SERVICE 0x1812
#define HID_REPORT_MAP 0x2A4B
#define HID_REPORT_DATA 0x2A4D

#if CONFIG_BT_NIMBLE_L2CAP_COC_MAX_NUM > 1

#define COC_BUF_COUNT (3 * MYNEWT_VAL(BLE_L2CAP_COC_MAX_NUM))
#define L2CAP_COC_UUID 0x1812

static uint16_t conn_handle_coc;
static uint16_t mtu = 512;
static os_membuf_t sdu_coc_mem[OS_MEMPOOL_SIZE(COC_BUF_COUNT, 500)];
static struct os_mempool sdu_coc_mbuf_mempool;
static struct os_mbuf_pool sdu_os_mbuf_pool;
static int blecent_l2cap_coc_event_cb(struct ble_l2cap_event* event, void* arg);

/**
 * This API is used to send data over L2CAP connection oriented channel.
 */
static void blecent_l2cap_coc_send_data(struct ble_l2cap_chan* chan)
{
  struct os_mbuf* sdu_rx_data;
  int rc = 0;
  int len = 512;
  uint8_t value[len];

  for (int i = 0; i < len; i++) {
    value[i] = i;
  }

  sdu_rx_data = os_mbuf_get_pkthdr(&sdu_os_mbuf_pool, 0);
  os_mbuf_append(sdu_rx_data, value, len);

  //    print_mbuf_data(sdu_rx_data);

  rc = ble_l2cap_send(chan, sdu_rx_data);
  if (rc == 0) {
    MODLOG_DFLT(INFO, "Data sent successfully");
  } else {
    MODLOG_DFLT(INFO, "Data sending failed, rc = %d", rc);
  }
}

/**
 * After connetion is established on GAP layer, service discovery is performed. On
 * it's completion, this API is called for making a connection is on L2CAP layer.
 */
static void blecent_l2cap_coc_on_disc_complete(const struct peer* peer, int status, void* arg)
{
  uint16_t psm = 0x1002;
  struct os_mbuf* sdu_rx;

  sdu_rx = os_mbuf_get_pkthdr(&sdu_os_mbuf_pool, 0);
  ble_l2cap_connect(conn_handle_coc, psm, mtu, sdu_rx, blecent_l2cap_coc_event_cb, NULL);

  os_mbuf_free(sdu_rx);
}

/**
 * The nimble host executes this callback when a L2CAP  event occurs.  The
 * application associates a L2CAP event callback with each connection that is
 * established.  blecent_l2cap_coc uses the same callback for all connections.
 *
 * @param event                 The event being signalled.
 * @param arg                   Application-specified argument; unused by
 *                                  blecent_l2cap_coc.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular L2CAP event being signalled.
 */
static int blecent_l2cap_coc_event_cb(struct ble_l2cap_event* event, void* arg)
{
  struct ble_l2cap_chan_info chan_info;

  switch (event->type) {
    case BLE_L2CAP_EVENT_COC_CONNECTED:
      if (event->connect.status) {
        console_printf("LE COC error: %d\n", event->connect.status);
        return 0;
      }

      if (ble_l2cap_get_chan_info(event->connect.chan, &chan_info)) {
        assert(0);
      }

      console_printf("LE COC connected, conn: %d, chan: %p, psm: 0x%02x,"
                     " scid: 0x%04x, "
                     "dcid: 0x%04x, our_mps: %d, our_mtu: %d,"
                     " peer_mps: %d, peer_mtu: %d\n",
                     event->connect.conn_handle,
                     event->connect.chan,
                     chan_info.psm,
                     chan_info.scid,
                     chan_info.dcid,
                     chan_info.our_l2cap_mtu,
                     chan_info.our_coc_mtu,
                     chan_info.peer_l2cap_mtu,
                     chan_info.peer_coc_mtu);
      blecent_l2cap_coc_send_data(event->connect.chan);
      return 0;

    case BLE_L2CAP_EVENT_COC_DISCONNECTED:
      console_printf("LE CoC disconnected, chan: %p\n", event->disconnect.chan);
      return 0;

    default:
      return 0;
  }
}

static void blecent_l2cap_coc_mem_init(void)
{
  int rc;
  rc = os_mempool_init(&sdu_coc_mbuf_mempool, COC_BUF_COUNT, mtu, sdu_coc_mem, "coc_sdu_pool");
  assert(rc == 0);
  rc = os_mbuf_pool_init(&sdu_os_mbuf_pool, &sdu_coc_mbuf_mempool, mtu, COC_BUF_COUNT);
  assert(rc == 0);
}
#endif // #if CONFIG_BT_NIMBLE_L2CAP_COC_MAX_NUM > 1
static int blecent_gap_event(struct ble_gap_event* event, void* arg);

void ble_store_config_init(void);

static esp32BleCallback _callback = NULL;

void SetEsp32BleCallback(esp32BleCallback callback)
{
  _callback = callback;
}

void BleInit(void);

/**
 * Application callback.  Called when the attempt to subscribe to notifications
 * for the ANS Unread Alert Status characteristic has completed.
 */
static int blecent_on_subscribe(uint16_t conn_handle, const struct ble_gatt_error* error, struct ble_gatt_attr* attr, void* arg)
{
  MODLOG_DFLT(INFO,
              "Subscribe complete; status=%d conn_handle=%d "
              "attr_handle=%d\n",
              error->status,
              conn_handle,
              attr->handle);

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

  dsc = peer_chr_find_uuid(peer, BLE_UUID16_DECLARE(HID_SERVICE), BLE_UUID16_DECLARE(HID_REPORT_DATA));
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

/* Application callback.  Called when the read of the ANS Supported New Alert
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

#if 1
  value[0] = 99;
  value[1] = 100;
  rc = ble_gattc_write_flat(conn_handle, chr->chr.val_handle, value, sizeof value, blecent_on_write, NULL);
  if (rc != 0) {
    MODLOG_DFLT(ERROR, "Error: Failed to write characteristic; rc=%d\n", rc);
    goto err;
  }
#endif
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
  const struct peer_dsc* dsc;

  SLIST_FOREACH(dsc, &chr->dscs, next)
  {
    MODLOG_DFLT(INFO, "char=%02x\n", (uint8_t)&dsc->dsc.uuid.u);
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

  // comm_start(peer);

  /* Now perform three GATT procedures against the peer: read,
   * write, and subscribe to notifications.
   */
  blecent_read_write_subscribe(peer);
}

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
  rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
  if (rc != 0) {
    MODLOG_DFLT(WARN, "can't parse fields");
    return rc;
  }

  for (int i = 0; i < fields.num_uuids16; i++) {
    if (ble_uuid_u16(&fields.uuids16[i].u) == 0x1812) {
      printf("GOt 1812\n\r");
    }
  }
  if (fields.name_len > 0) {
    if (strstr((char*)fields.name, SERVER_NAME) || strstr((char*)fields.name, SERVER_NAME2))
      return 1;
  }

  return 0;
}

/**
 * Connects to the sender of the specified advertisement of it looks
 * interesting.  A device is "interesting" if it advertises connectability and
 * support for the Alert Notification service.
 */
static void blecent_connect_if_interesting(const struct ble_gap_disc_desc* disc)
{
  uint8_t own_addr_type;
  int rc;

  /* Don't do anything if we don't care about this advertiser. */
  if (!blecent_should_connect(disc)) {
    MODLOG_DFLT(INFO, "shouldn't connect");
    return;
  } else {
    MODLOG_DFLT(INFO, "Try to connet");
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

  rc = ble_gap_connect(own_addr_type, &disc->addr, 30000, NULL, blecent_gap_event, NULL);
  if (rc != 0) {
    MODLOG_DFLT(ERROR,
                "Error: Failed to connect to device; addr_type=%d "
                "addr=%s; rc=%d\n",
                disc->addr.type,
                addr_str(disc->addr.val),
                rc);
    return;
  } else {
    MODLOG_DFLT(INFO, "OK");
    printf("OK\n\r");
  }
}

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
  peer_disc_fn* disc_cb = NULL;

  switch (event->type) {
    case BLE_GAP_EVENT_DISC:
      printf("BLE_GAP_EVENT_DIS\n\r");

      rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
      if (rc != 0) {
        return 0;
      }

      /* An advertisment report was received during GAP discovery. */
      print_adv_fields(&fields);

      if (fields.name_len > 0) {
        MODLOG_DFLT(INFO, "Check %.*s", fields.name_len, fields.name);
        printf("Check %.*s\n\r", fields.name_len, fields.name);
        /* Try to connect to the advertiser if it looks interesting. */
        blecent_connect_if_interesting(&event->disc);
      }

      return 0;

    case BLE_GAP_EVENT_CONNECT:
      /* A new connection was established or a connection attempt failed. */
      if (event->connect.status == 0) {
        /* Connection successfully established. */
        MODLOG_DFLT(INFO, "Connection established ");
        printf("Connection established\n\r ");

        rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
        assert(rc == 0);
        print_conn_desc(&desc);
        MODLOG_DFLT(INFO, "\n");

#if CONFIG_BT_NIMBLE_L2CAP_COC_MAX_NUM > 1
        conn_handle_coc = event->connect.conn_handle;
        disc_cb = blecent_l2cap_coc_on_disc_complete;
#else
        disc_cb = blecent_on_disc_complete;
#endif
        /* Remember peer. */
        rc = peer_add(event->connect.conn_handle);
        if (rc != 0) {
          MODLOG_DFLT(ERROR, "Failed to add peer; rc=%d\n", rc);
          return 0;
        }

        /* Perform service discovery. */
        rc = peer_disc_all(event->connect.conn_handle, disc_cb, NULL);
        if (rc != 0) {
          MODLOG_DFLT(ERROR, "Failed to discover services; rc=%d\n", rc);
          return 0;
        }
      } else {
        /* Connection attempt failed; resume scanning. */
        MODLOG_DFLT(ERROR, "Error: Connection failed; status=%d\n", event->connect.status);
        blecent_scan();
      }

      return 0;

    case BLE_GAP_EVENT_DISCONNECT:

      // comm_stop();

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
      return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
      /* Peer sent us a notification or indication. */
      MODLOG_DFLT(INFO,
                  "received %s; conn_handle=%d attr_handle=%d "
                  "attr_len=%d\n",
                  event->notify_rx.indication ? "indication" : "notification",
                  event->notify_rx.conn_handle,
                  event->notify_rx.attr_handle,
                  OS_MBUF_PKTLEN(event->notify_rx.om));
      printf("received %s; conn_handle=%d attr_handle=%d "
             "attr_len=%d\n\r",
             event->notify_rx.indication ? "indication" : "notification",
             event->notify_rx.conn_handle,
             event->notify_rx.attr_handle,
             OS_MBUF_PKTLEN(event->notify_rx.om));

      uint8_t data[2];
      /* Attribute data is contained in event->notify_rx.om. Use
       * `os_mbuf_copydata` to copy the data received in notification mbuf */
      os_mbuf_copydata(event->notify_rx.om, 0, 2, data);
      uint16_t key = data[0] << 8 | data[1];
      printf("key=%04x \n\r", key);
      if (_callback) {
        switch (key) {
          case 0x1000:
            _callback(AP10KeyMinusOne, AP10ActionStart);
            break;
          case 0x2000:
            _callback(AP10KeyPlusOne, AP10ActionStart);
            break;
          case 0x0200:
            _callback(AP10KeyMinusTen, AP10ActionStart);
            break;
          case 0x0100:
            _callback(AP10KeyPlusTen, AP10ActionStart);
            break;
          case 0x0250:
            _callback(AP10KeyAuto, AP10ActionStart);
            break;
          case 0x0150:
            _callback(AP10KeyStandby, AP10ActionStart);
            break;
          case 0x0400:
            _callback(AP10KeyMode, AP10ActionStart);
            break;

          default:
            break;
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

  /* Begin scanning for a peripheral to connect to. */
  blecent_scan();
}

void blecent_host_task(void* param)
{
  /* This function will return only when nimble_port_stop() is executed */
  nimble_port_run();

  nimble_port_freertos_deinit();
}
void BleInit(void)
{
  int rc;
  nimble_port_init();
  /* Configure the host. */
  ble_hs_cfg.reset_cb = blecent_on_reset;
  ble_hs_cfg.sync_cb = blecent_on_sync;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

#if CONFIG_BT_NIMBLE_L2CAP_COC_MAX_NUM > 1
  blecent_l2cap_coc_mem_init();
#endif

  /* Initialize data structures to track connected peers. */
  rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
  assert(rc == 0);

  /* Set the default device name. */
  rc = ble_svc_gap_device_name_set(DEVICE_NAME);
  assert(rc == 0);

  nimble_port_freertos_init(blecent_host_task);
}
