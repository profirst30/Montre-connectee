////////////////////////////////////////////////////////////
// FICHIER : i2c_sensor_config.c
////////////////////////////////////////////////////////////

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#include <stdio.h>
#include <math.h>

////////////////////////////////////////////////////////////
// Trigger
////////////////////////////////////////////////////////////
#ifdef CONFIG_LIS2MDL_TRIGGER
static int lis2mdl_trig_cnt;

static void lis2mdl_trigger_handler(const struct device *dev,
                    const struct sensor_trigger *trig)
{
    sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
    lis2mdl_trig_cnt++;
}
#endif

#ifdef CONFIG_LSM6DSO_TRIGGER
static int lsm6dso_acc_trig_cnt;
static int lsm6dso_gyr_trig_cnt;
static int lsm6dso_temp_trig_cnt;

static void lsm6dso_acc_trig_handler(const struct device *dev,
                    const struct sensor_trigger *trig)
{
    sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
    lsm6dso_acc_trig_cnt++;
}

static void lsm6dso_gyr_trig_handler(const struct device *dev,
                    const struct sensor_trigger *trig)
{
    sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
    lsm6dso_gyr_trig_cnt++;
}

static void lsm6dso_temp_trig_handler(const struct device *dev,
                    const struct sensor_trigger *trig)
{
    sensor_sample_fetch_chan(dev, SENSOR_CHAN_DIE_TEMP);
    lsm6dso_temp_trig_cnt++;
}
#endif


////////////////////////////////////////////////////////////
// Configuration
////////////////////////////////////////////////////////////

///Magnetometer and Temperature///
static void lis2mdl_config(const struct device *lis2mdl)
{
	struct sensor_value odr_attr;

	/* set LIS2MDL sampling frequency to 100 Hz */
	odr_attr.val1 = 100;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lis2mdl, SENSOR_CHAN_ALL,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for LIS2MDL\n");
		return;
	}

#ifdef CONFIG_LIS2MDL_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_MAGN_XYZ;
	sensor_trigger_set(lis2mdl, &trig, lis2mdl_trigger_handler);
#endif
}


///Accelorometer and Gyroscope///
static void lsm6dso_config(const struct device *lsm6dso)
{
	struct sensor_value odr_attr, fs_attr;

	/* set LSM6DSO accel sampling frequency to 208 Hz */
	odr_attr.val1 = 208;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lsm6dso, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for LSM6DSO accel\n");
		return;
	}

	sensor_g_to_ms2(16, &fs_attr);

	if (sensor_attr_set(lsm6dso, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
		printk("Cannot set fs for LSM6DSO accel\n");
		return;
	}

	/* set LSM6DSO gyro sampling frequency to 208 Hz */
	odr_attr.val1 = 208;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lsm6dso, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for LSM6DSO gyro\n");
		return;
	}

	sensor_degrees_to_rad(250, &fs_attr);

	if (sensor_attr_set(lsm6dso, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
		printk("Cannot set fs for LSM6DSO gyro\n");
		return;
	}

#ifdef CONFIG_LSM6DSO_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	sensor_trigger_set(lsm6dso, &trig, lsm6dso_acc_trig_handler);

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_GYRO_XYZ;
	sensor_trigger_set(lsm6dso, &trig, lsm6dso_gyr_trig_handler);

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_DIE_TEMP;
	sensor_trigger_set(lsm6dso, &trig, lsm6dso_temp_trig_handler);
#endif
}

////////////////////////////////////////////////////////////
// CAPTEURS ET VARIABLES GLOBALS
////////////////////////////////////////////////////////////

struct sensor_value temp1, hum, press;
#ifdef CONFIG_LSM6DSO_ENABLE_TEMP
	struct sensor_value die_temp;
#endif
struct sensor_value die_temp2;
struct sensor_value accel1[3], gyro[3];
struct sensor_value magn[3];

const struct device *const hts221 = DEVICE_DT_GET_ONE(st_hts221);
const struct device *const lis2mdl = DEVICE_DT_GET_ONE(st_lis2mdl);
const struct device *const lsm6dso = DEVICE_DT_GET_ONE(st_lsm6dso);

////////////////////////////////////////////////////////////
// INITIALISATION DES CAPTEURS
////////////////////////////////////////////////////////////

void Sensor_Init()
{
    printk(">> Checking HTS221...\n");
    if (!device_is_ready(hts221)) {
        printk("%s: device not ready.\n", hts221->name);
        return;
    }

    printk(">> Checking LIS2MDL...\n");
    if (!device_is_ready(lis2mdl)) {
        printk("%s: device not ready.\n", lis2mdl->name);
        return;
    }

    printk(">> Checking LSM6DSO...\n");
    if (!device_is_ready(lsm6dso)) {
        printk("%s: device not ready.\n", lsm6dso->name);
        return;
    }

    printk(">> Calling lis2mdl_config...\n");
    lis2mdl_config(lis2mdl);
    printk(">> Calling lsm6dso_config...\n");
    lsm6dso_config(lsm6dso);
    printk(">> Sensor_Init complete.\n");
}


/*-----------------------------------------------------------
* Boussole
*-----------------------------------------------------------*/
#define M_PI 3.14159265358979323846

// Fonction qui déduit la direction à partir de l'angle en degrés
const char* get_direction(double angle_deg) {
    // Les intervalles sont définis en degrés pour les directions
    if (angle_deg < 22.5 || angle_deg >= 337.5)
        return "Nord";
    else if (angle_deg < 67.5)
        return "Nord-Est";
    else if (angle_deg < 112.5)
        return "Est";
    else if (angle_deg < 157.5)
        return "Sud-Est";
    else if (angle_deg < 202.5)
        return "Sud";
    else if (angle_deg < 247.5)
        return "Sud-Ouest";
    else if (angle_deg < 292.5)
        return "Ouest";
    else // angle_deg < 337.5
        return "Nord-Ouest";
}
double calculate_compass_heading(double raw_x, double raw_y)
{
    // Calibrations mesurées (à ajuster selon ton environnement)
    double x_min = -0.535;
    double x_max = -0.153;
    double y_min = -0.336;
    double y_max =  0.043;

    // Calcul des offsets
    double offset_x = (x_max + x_min) / 2.0;
    double offset_y = (y_max + y_min) / 2.0;

    // Application des offsets
    double x = raw_x - offset_x;
    double y = raw_y - offset_y;

    // Calcul de l'angle (radians → degrés)
    double heading_rad = atan2(y, -x);
    double heading_deg = heading_rad * (180.0 / M_PI);

    // Normalisation dans [0, 360[
    if (heading_deg < 0) heading_deg += 360.0;

    // Correction manuelle (ex: rotation carte / nord magnétique)
    heading_deg += 100.0;
    if (heading_deg >= 360.0) heading_deg -= 360.0;

    return heading_deg;
}

void print_compass_info(double raw_x, double raw_y)
{
    double angle = calculate_compass_heading(raw_x, raw_y);
    const char* direction = get_direction(angle);

    printf("Angle boussole : %.2f°\n", angle);
    printf("Direction : %s\n", direction);
}


/*-----------------------------------------------------------
* Distance et Pas
*-----------------------------------------------------------*/
#define STEP_LENGTH              0.7f
#define MIN_STEP_INTERVAL_MS     600
#define PEAK_THRESHOLD           12.5f   // seuil pour un pic
#define VALLEY_THRESHOLD         10.2f   // retour au calme

//int step_count = 0;
//float distance = 0.0f;

typedef struct {
    int steps;
    float distance_m;
} StepResult;

StepResult detect_step_from_data(float ax, float ay, float az)
{
    static float prev_norm = 0;
    static bool looking_for_peak = true;
    static int64_t last_step_time = 0;
    static int step_count = 0;
    static float distance = 0.0f;

    int64_t now = k_uptime_get();
    float norm = sqrtf(ax * ax + ay * ay + az * az);

    if (looking_for_peak) {
        if (prev_norm < PEAK_THRESHOLD && norm >= PEAK_THRESHOLD &&
            (now - last_step_time) > MIN_STEP_INTERVAL_MS) {

            step_count++;
            distance += STEP_LENGTH;
            last_step_time = now;
            looking_for_peak = false;

            printk("✅ Pas détecté ! Total: %d | Distance: %.2f m\n", step_count, distance);
        }
    } else {
        if (norm < VALLEY_THRESHOLD) {
            looking_for_peak = true;
        }
    }

    prev_norm = norm;

    StepResult result = {
        .steps = step_count,
        .distance_m = distance
    };
    return result;
}