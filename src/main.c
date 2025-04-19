/*
 * Copyright (c) 2024 LVGL <felipe@lvgl.io>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #include <zephyr/kernel.h>
 #include <zephyr/drivers/display.h>
 #include <lvgl.h>
 #include <lvgl_mem.h>
 //#include <lv_demos.h>
 #include "ui/ui.h"
 #include <stdio.h>
 #include <math.h>
 
 #include <zephyr/device.h>
 #include <zephyr/drivers/sensor.h>
 #include <zephyr/sys/util.h>
 
 #include "sensor/I2C_Sensor_Config.h"
 
 #include <zephyr/drivers/rtc.h>
 #include <zephyr/input/input.h>

 #include <zephyr/logging/log.h>

// RTC CALLBACK INCLUDE
 #include "rtc_callbacks.h"

 //ble include
 #include "BLE/ble_config.h"

 #include <nrfx_timer.h>


LOG_MODULE_REGISTER(sample, LOG_LEVEL_INF);
 #define REFRESH_RATE 100

//dimension ecran
#define SCREEN_WIDTH 240 
#define SCREEN_HEIGHT 320

#define refresh_rate 100

#define LVGL_STACK_SIZE 2048
K_THREAD_STACK_DEFINE(lvgl_stack_area, LVGL_STACK_SIZE);
struct k_thread lvgl_thread;
void lvgl_loop(void *p1, void *p2, void *p3)
{
    while (1) {
        lv_timer_handler();
        k_msleep(10); // Appelle LVGL toutes les 10 ms (~100 FPS)
	}
}


static const struct device *const touch_dev = DEVICE_DT_GET(DT_NODELABEL(tsc2007_adafruit_2_8_tft_touch_v2));

//structure pour le point de contact du tactile
static struct {
    size_t x;
    size_t y;
    bool pressed;
} touch_point;

static struct k_sem sync;

static const struct device *display_dev;

/*-----------------------------------------------------------
  * Input/Output et Timer
  *-----------------------------------------------------------*/
 #define INACTIVITY_TIMEOUT_MS 20000  // 20 secondes
 #define LED_BLINK_MS         500     // Clignotement LED, si utilisé
// volatile bool touch_wake_event_pending = false;



//Timer2
//static const nrfx_timer_t timer2 = NRFX_TIMER_INSTANCE(2);


 /*
 //led 0
 #define LED0_NODE DT_ALIAS(led0)
 static const struct gpio_dt_spec led_0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

 //bouton 0
 #define BUTTON0_NODE DT_ALIAS(sw0)
 static const struct gpio_dt_spec button_0 = GPIO_DT_SPEC_GET(BUTTON0_NODE, gpios);


*/

/*-----------------------------------------------------------
* Fonctions de gestion du timer et de la mise en veille
*-----------------------------------------------------------*/

/*

volatile bool system_awake = true;
volatile bool is_sleeping = false;

struct k_timer inactivity_timer;  // déclare l'instance ici

static int64_t last_sleep_time = 0;
#define SLEEP_COOLDOWN_MS 500  // temps minimum pour éviter réveil accidentel
static bool timer2_initialized = false;

static bool timer2_running = false;

volatile bool system_awake = true;
volatile bool is_sleeping = false;
static bool timer2_started = false;

void timer2_handler(nrf_timer_event_t event_type, void *p_context)
{
    if (event_type == NRF_TIMER_EVENT_COMPARE0) {
        if (system_awake && !is_sleeping) {
            LOG_INF("⏲️ Inactivité détectée — mise en veille");

            nrfx_timer_disable(&timer2);
            is_sleeping = true;
            system_awake = false;

            if (device_is_ready(display_dev)) {
                display_blanking_on(display_dev);
                LOG_INF("🖥️ Écran éteint");
            }
        }
    }
}

void start_timer2(void)
{
    static bool already_initialized = false;

    if (already_initialized) {
        LOG_INF("✅ Timer2 déjà initialisé, on saute.");
        return;
    }

    nrfx_timer_config_t config = {
        .frequency          = NRF_TIMER_FREQ_1MHz,
        .mode               = NRF_TIMER_MODE_TIMER,
        .bit_width          = NRF_TIMER_BIT_WIDTH_32,
        .interrupt_priority = 6,
        .p_context          = NULL
    };

    nrfx_err_t err = nrfx_timer_init(&timer2, &config, timer2_handler);
    if (err == NRFX_ERROR_INVALID_STATE) {
        LOG_WRN("⚠️ NRFX_TIMER2 déjà utilisé par un autre module");
        already_initialized = true;  // pour éviter de réessayer
        return;
    } else if (err != NRFX_SUCCESS) {
        LOG_ERR("❌ nrfx_timer_init(TIMER2) a échoué: %lu", (unsigned long)err);
        return;
    }

    already_initialized = true;

    uint32_t ticks = nrfx_timer_ms_to_ticks(&timer2, INACTIVITY_TIMEOUT_MS);
    nrfx_timer_extended_compare(&timer2,
        NRF_TIMER_CC_CHANNEL0,
        ticks,
        NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
        true);

    nrfx_timer_enable(&timer2);
    LOG_INF("✅ Timer2 initialisé et lancé");
}

void reset_inactivity_timer2(void)
{
    if (!system_awake || is_sleeping) return;

    nrfx_timer_clear(&timer2);
    nrfx_timer_enable(&timer2);
    LOG_INF("🔁 Timer2 réinitialisé");
}

*/



/*
void button_pressed_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    wake_up_system();  // <---
}
*/



int flagView = 1;
static int64_t last_touch_time = 0;
#define TOUCH_DEBOUNCE_MS 500  // délai de rebond

static void touch_event_callback(struct input_event *evt, void *user_data)
{
    static int current_x = -1;
    static int current_y = -1;
    static int current_pressed = -1;

    if (evt->code == INPUT_ABS_X) {
        current_x = evt->value;
    }

    if (evt->code == INPUT_ABS_Y) {
        current_y = evt->value;
    }

    if (evt->code == INPUT_BTN_TOUCH) {
        current_pressed = evt->value;
    }

    if (evt->sync) {
        // On met à jour touch_point seulement ici
        touch_point.x = current_x;
        touch_point.y = current_y;
        touch_point.pressed = current_pressed;

        // On log les coordonnées pour debug
		LOG_INF("TOUCH %s X, Y: (%d, %d)", touch_point.pressed ? "PRESS" : "RELEASE", touch_point.x, touch_point.y);

        // Gestion des écrans uniquement à l'appui (pressed == 1)
        if (touch_point.pressed == 1) {

            int64_t now = k_uptime_get();
            if ((now - last_touch_time) >= TOUCH_DEBOUNCE_MS) {
                last_touch_time = now;

                if (flagView == 1 && touch_point.y < (SCREEN_WIDTH / 2)) {
                    //LOG_INF("Screen1 → Screen2");
                    lv_scr_load(ui_Screen2);
                    flagView = 2;
                } else if (flagView == 1 && touch_point.y > (SCREEN_WIDTH / 2)) {
                    lv_scr_load(ui_Screen5);
                    flagView = 5;
                } else if (flagView == 2 && touch_point.y < (SCREEN_WIDTH / 2)) {
                    //LOG_INF("Screen2 → Screen3");
                    lv_scr_load(ui_Screen3);
                    flagView = 3;
                } else if (flagView == 2 && touch_point.y > (SCREEN_WIDTH / 2)) {
                    //LOG_INF("Screen2 → Screen1");
                    lv_scr_load(ui_Screen1);
                    flagView = 1;
                } else if (flagView == 3 && touch_point.y < (SCREEN_WIDTH / 2)) {
                    //LOG_INF("Screen3 → Screen4");
                    lv_scr_load(ui_Screen4);
                    flagView = 4;
                } else if (flagView == 3 && touch_point.y > (SCREEN_WIDTH / 2)) {
                    //LOG_INF("Screen3 → Screen2");
                    lv_scr_load(ui_Screen2);
                    flagView = 2;
                } else if (flagView == 4 && touch_point.y > (SCREEN_WIDTH / 2) && touch_point.x < (SCREEN_HEIGHT / 2)) {
                    //LOG_INF("Screen4 → Screen3");
                    lv_scr_load(ui_Screen3);
                    flagView = 3;
                }
            }
        }

        // C’est uniquement ici que LVGL est notifié → séquence complète
        k_sem_give(&sync);
	}
}

 /*-----------------------------------------------------------
  * Définition threads, mutex et sémaphore
  *-----------------------------------------------------------*/
 
 // Augmentation de la taille de la pile pour éviter les débordements (passage de 512 à 1024)
 #define STACK_SIZE 1024
 
 K_THREAD_STACK_DEFINE(thread_stack_area_0, STACK_SIZE);
 K_THREAD_STACK_DEFINE(thread_stack_area_1, STACK_SIZE);
 K_THREAD_STACK_DEFINE(rtc_stack, STACK_SIZE);
 
 // Déclaration des threads
 struct k_thread thread_Sensor;
 struct k_thread thread_Sensor_processing;
 struct k_thread thread_rtc;
 
//init BLE
#define BLE_STACK_SIZE 1024
#define BLE_PRIORITY   4

K_THREAD_STACK_DEFINE(ble_stack, BLE_STACK_SIZE);
struct k_thread ble_thread;

 /*-----------------------------------------------------------
 * Définition des files de messages et sémaphores
 *-----------------------------------------------------------*/
 #define QUEUE_SIZE 5

 

 
 /*-----------------------------------------------------------
  * Structure de données capteurs et Message Queue
  *-----------------------------------------------------------*/
 //Structure des données capteurs (HTS221, LSM6DSL...)
 typedef struct{
	 double temperature;
	 double humidity;
	 double Mx,My,Mz;
	 double ax,ay,az, gx,gy,gz;
 } Sensor_Data;
 
 /// Init fonction message queue
 struct k_msgq sensor_msgq;
 struct k_msgq sensor_msgq_ble;

 // initialisation de la file de messages
 // struct k_msgq sensor_msgq_high;
 
 typedef struct {
	 float value;
	 Sensor_Data DataSensor;
	 uint8_t sensor_id;
 } sensor_msg_data;
 char Msg_queue_buffer[10 * sizeof(sensor_msg_data)];
 char Msg_queue_buffer_ble[10 * sizeof(sensor_msg_data)];

 
 /*-----------------------------------------------------------
  * init RTC
  *-----------------------------------------------------------*/
 void RTC_Init(void)
 {
	const struct device *rtc_dev = DEVICE_DT_GET(DT_NODELABEL(rv8263));
 
	 if (rtc_dev == NULL) {
		 printk("rtc_dev = NULL !\n");
		 return;
	 }
 
	 if (!device_is_ready(rtc_dev)) {
		 printk("RTC non prête\n");
		 return;
	 }
 
	 struct rtc_time nouvelle_date = {
		 .tm_year = 2025 - 1900,
		 .tm_mon  = 3,
		 .tm_mday = 3,
		 .tm_hour = 12,
		 .tm_min  = 30,
		 .tm_sec  = 0,
	 };
 
	 if (rtc_set_time(rtc_dev, &nouvelle_date) == 0) {
		 printk("RTC réglée à la nouvelle date/heure.\n");
	 }
 }
 
 /*-----------------------------------------------------------
  * Fonction pour RTC
  *-----------------------------------------------------------*/
 void Clock_Print_Thread(void *p1, void *p2, void *p3)
  {
    k_sleep(K_MSEC(200)); // Laisse le temps aux périphériques I2C de s'activer

    RTC_Init();
	init_edit_time_from_rtc();  // Pour charger l’heure/date de la RTC dans les variables globales

    const struct device *rtc_dev = DEVICE_DT_GET_ANY(microcrystal_rv_8263_c8);

    if (rtc_dev == NULL) {
        printk("rtc_dev = NULL dans Clock_Print_Thread !\n");
        return;
    }

    if (!device_is_ready(rtc_dev)) {
        printk("RTC toujours pas prête dans Clock_Print_Thread\n");
        return;
    }

    struct rtc_time now;
    char time_buf[16];
    char date_buf[16];

	while (1) {
        if (rtc_get_time(rtc_dev, &now) == 0) {
            //snprintf(time_buf, sizeof(time_buf), "%02d:%02d:%02d", now.tm_hour, now.tm_min, now.tm_sec);
			snprintf(time_buf, sizeof(time_buf), "%02d:%02d", now.tm_hour, now.tm_min);
            snprintf(date_buf, sizeof(date_buf), "%02d/%02d/%02d", now.tm_mday, now.tm_mon + 1, (now.tm_year + 1900) % 100);
            if(flagView == 1) {
                lv_label_set_text(ui_LabelTimeScreen1, time_buf);
                lv_label_set_text(ui_LabelDateScreen1, date_buf);
            }
            else if (flagView == 2) {
                lv_label_set_text(ui_LabelTimeScreen3, time_buf);
            }
            else if (flagView == 3) {
                lv_label_set_text(ui_LabelTimeScreen4, time_buf);
            }

            lv_label_set_text(ui_LabelTimeScreen1, time_buf);
            lv_label_set_text(ui_LabelDateScreen1, date_buf);

            printk("Heure affichée: %s | Date affichée: %s\n", time_buf, date_buf);
        } else {
            printk("Erreur de lecture RTC\n");
        }

        k_sleep(K_SECONDS(1));
    }
 }
 
 /*-----------------------------------------------------------
  * Affichage des données capteurs dans l'inteface LVGL
  *-----------------------------------------------------------*/
 char temp_buf[20];
 char hum_buf[20];
 char angle_buf[20];
 char direction_buf[20];
 char NbPas_buf[20];
 char distance_buf[20];

 void Sensor_Interface_Update(sensor_msg_data data)
 {
	static int64_t last_temp_hum_update = 0;
    static int64_t last_accel_update = 0;
    static int64_t last_mag_update = 0;

	int64_t now = k_uptime_get();


    // Capteur ID 1 = HTS221 (Temp & Hum)
    if (data.sensor_id == 1) {
        int64_t now = k_uptime_get();
        if ((now - last_temp_hum_update) >= 5000) {  // toutes les 5 secondes
            last_temp_hum_update = now;

            snprintf(temp_buf, sizeof(temp_buf), "%.1f", data.DataSensor.temperature-7.2); //avec offset
            snprintf(hum_buf, sizeof(hum_buf), "%.2f%%", data.DataSensor.humidity+10); //avec offset

            if (flagView == 1) {
                lv_label_set_text(ui_LabelTempScreen1, temp_buf);
                lv_label_set_text(ui_LabelHumScreen1, hum_buf);
            }

            printk("UI Screen1: Temp = %s, Hum = %s\n", temp_buf, hum_buf);
        }
    }

    // Capteur ID 2 = LSM6DSO (Accel & Gyro) → Distance & pas
    else if (data.sensor_id == 2) {
        StepResult step_data = detect_step_from_data(
            data.DataSensor.ax,
            data.DataSensor.ay,
            data.DataSensor.az
        );

        snprintf(NbPas_buf, sizeof(NbPas_buf), "%d", step_data.steps);
        snprintf(distance_buf, sizeof(distance_buf), "%.2f", step_data.distance_m);

        if ((now - last_accel_update) >= 1000) {  // toutes les 1s
            last_accel_update = now;

            if (flagView == 2) {
                lv_label_set_text(ui_LabelPasScreen2, NbPas_buf);
                lv_label_set_text(ui_LabelKmHScreen2, distance_buf);
            }

            printk("UI Screen2: Pas = %s, Distance = %s\n", NbPas_buf, distance_buf);
        }
    }

    // Capteur ID 3 = LIS2MDL (Magnétomètre) → Angle & direction
    else if (data.sensor_id == 3) {
        double heading = calculate_compass_heading(data.DataSensor.Mx, data.DataSensor.My);
        const char *direction = get_direction(heading);

        snprintf(angle_buf, sizeof(angle_buf), "%d", (int)heading);
        snprintf(direction_buf, sizeof(direction_buf), "%s", direction);

        if ((now - last_mag_update) >= 1000) {  // toutes les 1s
            last_mag_update = now;

            if (flagView == 3) {
                lv_label_set_text(ui_LabelGyroScreen3, angle_buf);
                lv_label_set_text(ui_LabelSudScreen3, direction_buf);
            }

            printk("UI Screen3: Angle = %s, Direction = %s\n", angle_buf, direction_buf);
        }
    }
}

/*-----------------------------------------------------------
  * Thread d'acquisition HTS221 / LSM6DSO / LIS2MDL
  *-----------------------------------------------------------*/
 void Sensor_task(void *p1, void *p2, void *p3)
 {
	 sensor_msg_data data = {0};
	 struct sensor_value temp1, hum;
	 struct sensor_value accel[3], gyro[3], mag[3];
 
	 while (1) {
		 if (sensor_sample_fetch(hts221) >= 0) {
			 sensor_channel_get(hts221, SENSOR_CHAN_AMBIENT_TEMP, &temp1);
			 sensor_channel_get(hts221, SENSOR_CHAN_HUMIDITY, &hum);
			 data.DataSensor.temperature = sensor_value_to_double(&temp1);
			 data.DataSensor.humidity = sensor_value_to_double(&hum);
			 data.sensor_id = 1;
			 k_msgq_put(&sensor_msgq, &data, K_NO_WAIT);
			 k_msgq_put(&sensor_msgq_ble, &data, K_NO_WAIT);
		 }
		 if (sensor_sample_fetch(lsm6dso) == 0) {
			 sensor_channel_get(lsm6dso, SENSOR_CHAN_ACCEL_XYZ, accel);
			 sensor_channel_get(lsm6dso, SENSOR_CHAN_GYRO_XYZ, gyro);
			 data.DataSensor.ax = sensor_value_to_double(&accel[0]);
			 data.DataSensor.ay = sensor_value_to_double(&accel[1]);
			 data.DataSensor.az = sensor_value_to_double(&accel[2]);
			 data.DataSensor.gx = sensor_value_to_double(&gyro[0]);
			 data.DataSensor.gy = sensor_value_to_double(&gyro[1]);
			 data.DataSensor.gz = sensor_value_to_double(&gyro[2]);
			 data.sensor_id = 2;
			 k_msgq_put(&sensor_msgq, &data, K_NO_WAIT);
		 }
		 if (sensor_sample_fetch(lis2mdl) == 0) {
			 sensor_channel_get(lis2mdl, SENSOR_CHAN_MAGN_XYZ, mag);
			 data.DataSensor.Mx = sensor_value_to_double(&mag[0]);
			 data.DataSensor.My = sensor_value_to_double(&mag[1]);
			 data.DataSensor.Mz = sensor_value_to_double(&mag[2]);
			 data.sensor_id = 3;
			 k_msgq_put(&sensor_msgq, &data, K_NO_WAIT);
			 k_msgq_put(&sensor_msgq_ble, &data, K_NO_WAIT);
		 }
		 k_sleep(K_MSEC(200));
	 }
 }

 /*-----------------------------------------------------------
  * Thread de traitement : récupère les données de la file et les traite et affiches
  *-----------------------------------------------------------*/
 void Sensor_processing(void *p1, void *p2, void *p3)
 {
	 sensor_msg_data data;
 
	 while (1) {
		 if (k_msgq_get(&sensor_msgq, &data, K_FOREVER) == 0) {
			 printk("Message récupéré dans la file\n");
			 Sensor_Interface_Update(data);
		 } else {
			 printk("Erreur lors de la récupération de la file de messages\n");
		 }
	 }
 }

  /*-----------------------------------------------------------
 * Bluetooth low energy
 *-----------------------------------------------------------*/
void ble_thread_fn(void *p1, void *p2, void *p3)
{
    ble_config_init();
    sensor_msg_data data;
    while (1) {
        if (k_msgq_get(&sensor_msgq_ble, &data, K_FOREVER) == 0) {
            if (data.sensor_id == 1) {
                int16_t temp_ble = (int16_t)(data.DataSensor.temperature * 100.0f);
                uint16_t humi_ble = (uint16_t)(data.DataSensor.humidity * 100.0f);
                ble_update_temperature(temp_ble);
                ble_update_humidity(humi_ble);
            } else if (data.sensor_id == 3) {
                int16_t mx = (int16_t)(data.DataSensor.Mx * 100.0f);
                int16_t my = (int16_t)(data.DataSensor.My * 100.0f);
                int16_t mz = (int16_t)(data.DataSensor.Mz * 100.0f);
                ble_update_magnetometer(mx, my, mz);
            }
        }
		k_sleep(K_MSEC(2000));

    }
}

 //definition du callback pour le tactile
 INPUT_CALLBACK_DEFINE(touch_dev, touch_event_callback, NULL);

 int main(void)
 {
	 /*-----------------------------------------------------------
	  * Initialisation globale
	 *-----------------------------------------------------------*/
	 k_sem_init(&sync, 0, 1);
	 Sensor_Init();
	 if (!hts221) {
		 printk("Capteur HTS221 non trouvé !\n");
	 } else {
		 printk("Capteur HTS221 initialisé avec succès\n");
	 }
     /*
     gpio_pin_configure_dt(&led_0, GPIO_OUTPUT_ACTIVE);
     gpio_pin_configure_dt(&button_0, GPIO_INPUT);
     gpio_pin_interrupt_configure_dt(&button_0, GPIO_INT_EDGE_TO_ACTIVE);
     gpio_init_callback(&button_cb_data, button_pressed_isr, BIT(button_0.pin));
     gpio_add_callback(button_0.port, &button_cb_data);
 
     */
 
	 /*-----------------------------------------------------------
	  * Initialisation de l'interface LVGL
	  *-----------------------------------------------------------*/
	 //const struct device *display_dev;
 
	 display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	 if (!device_is_ready(display_dev)) {
		 printk("Device not ready, aborting test\n");
		 return 0;
	 }
 
	 /* Place your UI init function here */
	 ui_init();


 
	 display_blanking_off(display_dev);
 #ifdef CONFIG_LV_Z_MEM_POOL_SYS_HEAP
	 lvgl_print_heap_info(false);
 #else
	 printf("lvgl in malloc mode\n");
 #endif
    


	 /*-----------------------------------------------------------
	 * Initialisation des threads et message queue
	 *-----------------------------------------------------------*/

	 k_thread_create(&lvgl_thread, lvgl_stack_area, LVGL_STACK_SIZE,lvgl_loop, NULL, NULL, NULL,0, 0, K_NO_WAIT);
	 printk("Thread LVGL lancé\n");
	 
	 k_sem_give(&sync);
	 k_msgq_init(&sensor_msgq, Msg_queue_buffer, sizeof(sensor_msg_data), 10);
	 k_msgq_init(&sensor_msgq_ble, Msg_queue_buffer_ble, sizeof(sensor_msg_data), 10);
	 printk("File de messages initialisée\n");
 
	 //k_msgq_init(&sensor_msgq_high, msg_queue_buffer_critical, sizeof(sensor_msg_data), QUEUE_SIZE);
	 //k_sem_init(&sem_high, 0, QUEUE_SIZE);
	 //k_sem_init(&sem_normal, 0, QUEUE_SIZE);
 
	 // Création de la tâche d'acquisition
	 k_thread_create(&thread_Sensor, thread_stack_area_0, STACK_SIZE, Sensor_task, (void *)1, NULL, NULL, 5, 0, K_NO_WAIT);
	 printk("Thread Sensor lancé\n");
 
	 // Création de la tâche de traitement et d'affichage
	 k_thread_create(&thread_Sensor_processing, thread_stack_area_1, STACK_SIZE, Sensor_processing, NULL, NULL, NULL, 5, 0, K_NO_WAIT);
	 printk("Thread Sensor_processing lancé\n");
 
	 // RTC thread
	 k_thread_create(&thread_rtc, rtc_stack, STACK_SIZE, Clock_Print_Thread, NULL, NULL, NULL, 3, 0, K_NO_WAIT);

	  
     //BLE thread
     k_thread_create(&ble_thread, ble_stack, BLE_STACK_SIZE, ble_thread_fn, NULL, NULL, NULL, BLE_PRIORITY, 0, K_NO_WAIT);
     	 
 
	 while (1) {
		 //This function has to be called periodically to update the UI
		 //Can be placed in a dedicated thread
 
		 /*-----------------------------------------------------------
		 * Rafraichissement de l'interface LVGL
		 *-----------------------------------------------------------*/
		 //k_msleep(refresh_rate);
            k_sem_take(&sync, K_FOREVER);
		 //uint32_t sleep_ms = lv_timer_handler();
		 //k_msleep(MIN(sleep_ms, 20));
	 }
	 return 0;
 }

 