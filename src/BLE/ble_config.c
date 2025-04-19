#include "ble_config.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#define BT_UUID_ESS_VAL 0x181A
#define BT_UUID_TEMPERATURE_VAL 0x2A6E
#define BT_UUID_HUMIDITY_VAL 0x2A6F
#define BT_UUID_MAGNETIC_FLUX_DENSITY_3D_VAL 0x2AA0

#define BT_UUID_ESS BT_UUID_DECLARE_16(BT_UUID_ESS_VAL)
#define BT_UUID_TEMPERATURE BT_UUID_DECLARE_16(BT_UUID_TEMPERATURE_VAL)
#define BT_UUID_HUMIDITY BT_UUID_DECLARE_16(BT_UUID_HUMIDITY_VAL)
#define BT_UUID_MAGNETIC_FLUX_DENSITY_3D BT_UUID_DECLARE_16(BT_UUID_MAGNETIC_FLUX_DENSITY_3D_VAL)

static int16_t temp_value_ble;
static uint16_t humi_value_ble;
static int16_t mag_xyz_scaled[3];

static ssize_t read_temperature(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &temp_value_ble, sizeof(temp_value_ble));
}

static ssize_t read_humidity(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &humi_value_ble, sizeof(humi_value_ble));
}

static ssize_t read_magnetic_flux(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                  void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, mag_xyz_scaled, sizeof(mag_xyz_scaled));
}

static void temp_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    printk("Notifications temperature %s\n", (value == BT_GATT_CCC_NOTIFY) ? "activees" : "desactivees");
}

static void humi_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    printk("Notifications humidite %s\n", (value == BT_GATT_CCC_NOTIFY) ? "activees" : "desactivees");
}

BT_GATT_SERVICE_DEFINE(sensor_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_ESS),
    BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_temperature, NULL, &temp_value_ble),
    BT_GATT_CCC(temp_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_humidity, NULL, &humi_value_ble),
    BT_GATT_CCC(humi_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(BT_UUID_MAGNETIC_FLUX_DENSITY_3D,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_magnetic_flux, NULL, mag_xyz_scaled),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

#define SENSOR_TEMP_ATTR  (&sensor_svc.attrs[2])
#define SENSOR_HUMI_ATTR  (&sensor_svc.attrs[5])
#define SENSOR_MAG_ATTR   (&sensor_svc.attrs[8])

static void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        printk("Echec de connexion (err 0x%02x)\n", err);
        return;
    }
    printk("Connecte\n");
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    printk("Deconnecte (raison 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
};

void ble_config_init(void) {
    int err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    settings_load();

    struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, strlen(CONFIG_BT_DEVICE_NAME)),
    };

    struct bt_data sd[] = {
        BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0x1A, 0x18),
    };

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("D\'emarrage de la publicite echoue (err %d)\n", err);
    } else {
        printk("Publicite BLE demarree.\n");
    }
}

void ble_update_temperature(int16_t temp_100) {
    temp_value_ble = temp_100;
    uint8_t buf[2];
    sys_put_le16(temp_value_ble, buf);
    bt_gatt_notify(NULL, SENSOR_TEMP_ATTR, buf, sizeof(buf));
}

void ble_update_humidity(uint16_t humi_100) {
    humi_value_ble = humi_100;
    uint8_t buf[2];
    sys_put_le16(humi_value_ble, buf);
    bt_gatt_notify(NULL, SENSOR_HUMI_ATTR, buf, sizeof(buf));
}

void ble_update_magnetometer(int16_t x_100, int16_t y_100, int16_t z_100) {
    mag_xyz_scaled[0] = x_100;
    mag_xyz_scaled[1] = y_100;
    mag_xyz_scaled[2] = z_100;
    bt_gatt_notify(NULL, SENSOR_MAG_ATTR, mag_xyz_scaled, sizeof(mag_xyz_scaled));
}
