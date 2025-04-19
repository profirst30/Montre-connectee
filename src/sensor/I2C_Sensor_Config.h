#ifndef I2C_SENSOR_CONFIG_H
#define I2C_SENSOR_CONFIG_H

#include <stdio.h>
#include <math.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

////////////////////////////////////////////////////////////
// VARIABLES ET POINTEURS CAPTEURS
////////////////////////////////////////////////////////////

extern struct sensor_value temp1, hum;
#ifdef CONFIG_LSM6DSO_ENABLE_TEMP
extern struct sensor_value die_temp;
#endif
extern struct sensor_value die_temp2;
extern struct sensor_value accel1[3], gyro[3];
extern struct sensor_value magn[3];

extern const struct device *hts221;
extern const struct device *lis2mdl;
extern const struct device *lsm6dso;

////////////////////////////////////////////////////////////
// INITIALISATION DES CAPTEURS
////////////////////////////////////////////////////////////

void Sensor_Init(void);

////////////////////////////////////////////////////////////
// UTILITAIRES DE BOUSSOLE
////////////////////////////////////////////////////////////

double calculate_compass_heading(double raw_x, double raw_y);
const char* get_direction(double angle_deg);
void print_compass_info(double raw_x, double raw_y);
/* Définit la constante PI si elle n'est pas déjà définie */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

////////////////////////////////////////////////////////////
// DÉTECTION DES PAS ET DISTANCE
////////////////////////////////////////////////////////////

#define STEP_LENGTH              0.7f
#define MIN_STEP_INTERVAL_MS     600
#define PEAK_THRESHOLD           12.5f
#define VALLEY_THRESHOLD         10.2f

typedef struct {
    int steps;
    float distance_m;
} StepResult;

StepResult detect_step_from_data(float ax, float ay, float az);

#ifdef __cplusplus
}
#endif
#endif /* I2C_SENSOR_CONFIG_H */
