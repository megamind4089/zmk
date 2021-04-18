/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/types.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <sys/byteorder.h>

#include <logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/ble.h>
#include <zmk/split/bluetooth/uuid.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/matrix_transform.h>
#include <init.h>

static int start_scan(void);
static int stop_scan(void);

#define POSITION_STATE_DATA_LEN 16
// TODO TODO TODO figure out how to have array of devices. ignore name matching
// if not using zmk peripheral.
// #define LEFT CONFIG_ZMK_KEYBOARD_NAME##" Left"
// #define RIGHT CONFIG_ZMK_KEYBOARD_NAME##" Right"
char devices[][16] = {"Chocolad Left", "Chocolad Right"};
#define DEVICE_COUNT sizeof(devices)/sizeof(devices[0])

static struct bt_conn *peripheral_conns[DEVICE_COUNT];

const static struct bt_uuid_128 service_uuid = BT_UUID_INIT_128(ZMK_SPLIT_BT_SERVICE_UUID);
const static struct bt_uuid_128 characteristic_uuid = BT_UUID_INIT_128(ZMK_SPLIT_BT_CHAR_POSITION_STATE_UUID);
const static struct bt_uuid_16 ccc_uuid = BT_UUID_INIT_16(BT_UUID_GATT_CCC_VAL);

static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;

// Track whether central is currently scanning for peripherals.
static bool is_scanning = false;


K_MSGQ_DEFINE(peripheral_event_msgq, sizeof(struct zmk_position_state_changed),
              CONFIG_ZMK_SPLIT_BLE_CENTRAL_POSITION_QUEUE_SIZE, 4);

void peripheral_event_work_callback(struct k_work *work) {
    struct zmk_position_state_changed ev;
    while (k_msgq_get(&peripheral_event_msgq, &ev, K_NO_WAIT) == 0) {
        LOG_DBG("Trigger key position state change for %d", ev.position);
        ZMK_EVENT_RAISE(new_zmk_position_state_changed(ev));
    }
}

K_WORK_DEFINE(peripheral_event_work, peripheral_event_work_callback);

static uint8_t split_central_notify_func(struct bt_conn *conn,
                                         struct bt_gatt_subscribe_params *params, const void *data,
                                         uint16_t length) {
    static uint8_t position_state[POSITION_STATE_DATA_LEN];

    uint8_t changed_positions[POSITION_STATE_DATA_LEN];

    if (!data) {
        LOG_DBG("[UNSUBSCRIBED]");
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[NOTIFICATION] data %p length %u", data, length);

    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        changed_positions[i] = ((uint8_t *)data)[i] ^ position_state[i];
        position_state[i] = ((uint8_t *)data)[i];
    }

    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        for (int j = 0; j < 8; j++) {
            if (changed_positions[i] & BIT(j)) {
                uint32_t position = (i * 8) + j;
                bool pressed = position_state[i] & BIT(j);
                struct zmk_position_state_changed ev = {
                    .position = position, .state = pressed, .timestamp = k_uptime_get()};

                k_msgq_put(&peripheral_event_msgq, &ev, K_NO_WAIT);
                k_work_submit(&peripheral_event_work);
            }
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

static int split_central_subscribe(struct bt_conn *conn) {
    int err = bt_gatt_subscribe(conn, &subscribe_params);
    switch (err) {
    case -EALREADY:
        LOG_DBG("[ALREADY SUBSCRIBED]");
        break;
        // break;
        // bt_gatt_unsubscribe(conn, &subscribe_params);
        // return split_central_subscribe(conn);
    case 0:
        LOG_DBG("[SUBSCRIBED]");
        break;
    default:
        LOG_ERR("Subscribe failed (err %d)", err);
        break;
    }

    return 0;
}

static uint8_t split_central_discovery_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                            struct bt_gatt_discover_params *params) {
    int err;

    if (!attr) {
        LOG_DBG("Discover complete");
        (void)memset(params, 0, sizeof(*params));
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[ATTRIBUTE] handle %u", attr->handle);

    // After discovering service, discover using characteristic.
    if (!bt_uuid_cmp(discover_params.uuid, &service_uuid.uuid)) {
        discover_params.uuid = &characteristic_uuid.uuid;
        discover_params.start_handle = attr->handle + 1;
        discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

        err = bt_gatt_discover(conn, &discover_params);
        if (err) {
            LOG_DBG("Discover failed (err %d)", err);
        }
        return BT_GATT_ITER_STOP;
    }

    // After discovering characteristic, discover using descriptor.
    if (!bt_uuid_cmp(discover_params.uuid, &characteristic_uuid.uuid)) {
        discover_params.uuid = &ccc_uuid.uuid;
        discover_params.start_handle = attr->handle + 2;
        discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
        subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

        err = bt_gatt_discover(conn, &discover_params);
        if (err) {
            LOG_DBG("Discover failed (err %d)", err);
        }
        return BT_GATT_ITER_STOP;
    }

    // After discovering descriptor, enable notification on descriptor.
    subscribe_params.notify = split_central_notify_func;
    subscribe_params.value = BT_GATT_CCC_NOTIFY;
    subscribe_params.ccc_handle = attr->handle;
    split_central_subscribe(conn);

    return BT_GATT_ITER_STOP;
}

static void split_central_process_connection(struct bt_conn *conn) {
    int err;

    LOG_DBG("Current security for connection: %d", bt_conn_get_security(conn));

    err = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (err) {
        LOG_ERR("Failed to set security (reason %d)", err);
        return;
    }

    // TODO TODO TODO is this loop necessary?
    for (int i = 0; i < DEVICE_COUNT; i++) {
        // TODO TODO TODO need to check subscribe_params?
        if (conn == peripheral_conns[i] ) { // && !subscribe_params.value) {
            LOG_DBG("starting discover for %d", i);

            // Clear discovery parameters from previous run.
            (void)memset(&discover_params, 0, sizeof(discover_params));

            // Discover based on Bluetooth service.
            discover_params.uuid = &service_uuid.uuid;
            discover_params.func = split_central_discovery_func;
            discover_params.start_handle = 0x0001;
            discover_params.end_handle = 0xffff;
            discover_params.type = BT_GATT_DISCOVER_PRIMARY;

            err = bt_gatt_discover(peripheral_conns[i], &discover_params);
            if (err) {
                LOG_ERR("Discover failed(err %d)", err);
                return;
            }
            break;
        }
    }

    struct bt_conn_info info;

    bt_conn_get_info(conn, &info);

    LOG_DBG("New connection params: Interval: %d, Latency: %d, PHY: %d", info.le.interval,
            info.le.latency, info.le.phy->rx_phy);
}

static bool split_central_eir_found(struct bt_data *data, void *user_data) {
    // Continue parsing advertisement data if data is not device name.
    if (data->type != BT_DATA_NAME_COMPLETE) {
        return true;
    }

    // TODO TODO TODO ideally use whitelist. also dosn't seem like this setting
    // is actually used? we should actually store something to prevent a
    // malicious peripheral from imitating the real peripheral.
    // zmk_ble_set_peripheral_addr(addr);

    int device_id = -1;
    for (int i = 0; i < DEVICE_COUNT; i++) {
        if (data->data_len == strlen(devices[i])) {
            if (!memcmp(devices[i], data->data, strlen(devices[i]))) {
                LOG_DBG("[NAME MATCH for device %i]", i);
                device_id = i;
                break;
            }
        }
    }

    // Continue parsing if device name does not match.
    if (-1 == device_id) {
        return true;
    }

    // Stop scanning so we can connect to the peripheral device.
    LOG_DBG("Stopping peripheral scanning");
    int err = stop_scan();
    if (err) {
        return true;
    }

    // Create connection to peripheral with the given connection parameters.
    bt_addr_le_t *addr = user_data;
    struct bt_le_conn_param *param = BT_LE_CONN_PARAM(0x0006, 0x0006, 30, 400);
    err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &peripheral_conns[device_id]);
    if (err) {
        LOG_ERR("Create conn failed (err %d) (create conn? 0x%04x)", err, BT_HCI_OP_LE_CREATE_CONN);
        start_scan();
    }

    err = bt_conn_le_phy_update(peripheral_conns[device_id], BT_CONN_LE_PHY_PARAM_2M);
    if (err) {
        LOG_ERR("Update phy conn failed (err %d)", err);
        start_scan();
    }

    // Stop processing advertisement data.
    return false;
}

static void split_central_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                                       struct net_buf_simple *ad) {
    char dev[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(addr, dev, sizeof(dev));
    LOG_DBG("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i", log_strdup(dev), type, ad->len,
            rssi);

    /* We're only interested in connectable events */
    if (type == BT_GAP_ADV_TYPE_ADV_IND || type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        LOG_DBG("[DEVICE]: %s", log_strdup(dev));
        bt_data_parse(ad, split_central_eir_found, (void *)addr);
    }
}

static int start_scan(void) {
    // No action is necessary if central is already scanning.
    if (is_scanning) {
        LOG_DBG("scanning is already on");
        return 0;
    }

    // If all the devices are connected, there is no need to scan.
    bool has_unconnected = false;
    for (int i = 0; i < DEVICE_COUNT; i++) {
        if (peripheral_conns[i] == NULL) {
            has_unconnected = true;
            break;
        }
    }
    if (!has_unconnected) {
        LOG_DBG("all devices are connected");
        return 0;
    }

    // Start scanning otherwise.
    is_scanning = true;
    LOG_DBG("Starting peripheral scanning");
    int err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, split_central_device_found);
    if (err) {
        LOG_ERR("Scanning failed to start (err %d)", err);
        return err;
    }

    LOG_DBG("Scanning successfully started");
    return 0;
}

static int stop_scan(void) {
    int err = bt_le_scan_stop();
    if (err) {
        LOG_ERR("Stop LE scan failed (err %d)", err);
        return err;
    }

    is_scanning = false;
    return err;
}

static void split_central_connected(struct bt_conn *conn, uint8_t conn_err) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (conn_err) {
        LOG_ERR("Failed to connect to %s (%u)", log_strdup(addr), conn_err);

        // Unset peripheral connection on failure.
        for (int i = 0; i < DEVICE_COUNT; i++) {
            if (peripheral_conns[i] == conn) {
                bt_conn_unref(peripheral_conns[i]);
                peripheral_conns[i] = NULL;
                break;
            }
        }
    } else {
        // TODO TODO TODO I think this is called for hosts (phones, etc). skip
        // for those? do loop and ID lookup here?
        LOG_DBG("Connected: %s", log_strdup(addr));
        split_central_process_connection(conn);
    }

    start_scan();
}

static void split_central_disconnected(struct bt_conn *conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_DBG("Disconnected: %s (reason %d)", log_strdup(addr), reason);

    // Handle if connection is a peripheral connection.
    for (int device_id = 0; device_id < DEVICE_COUNT; device_id++) {
        if (peripheral_conns[device_id] == conn) {
            // Unset peripheral connection.
            bt_conn_unref(peripheral_conns[device_id]);
            peripheral_conns[device_id] = NULL;
            break;

            // TODO TODO TODO Release all keys that were held by the peripheral.
        }
    }

    start_scan();
}

static struct bt_conn_cb conn_callbacks = {
    .connected = split_central_connected,
    .disconnected = split_central_disconnected,
};

int zmk_split_bt_central_init(const struct device *_arg) {
    bt_conn_cb_register(&conn_callbacks);

    return start_scan();
}

SYS_INIT(zmk_split_bt_central_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);
