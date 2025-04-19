#ifndef BLE_CONFIG_H_
#define BLE_CONFIG_H_

#include <zephyr/types.h>

void ble_config_init(void);
void ble_update_temperature(int16_t temp_100);
void ble_update_humidity(uint16_t humi_100);
void ble_update_magnetometer(int16_t x_100, int16_t y_100, int16_t z_100);

#endif /* BLE_CONFIG_H_ */
