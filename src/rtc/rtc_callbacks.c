#include <zephyr/device.h>
#include <zephyr/drivers/rtc.h>
#include <stdio.h>
#include <lvgl.h>
#include "../ui/ui.h"

extern int flagView;

// Variables globales modifiables
static int edit_hour = 0;
static int edit_minute = 0;
static int edit_second = 0;
static int edit_day = 1;
static int edit_month = 1;
static int edit_year = 2025;

// Mise à jour de l'affichage LVGL
void update_time_display() {
    char buffer[8];
    snprintf(buffer, sizeof(buffer), "%02d", edit_hour);
    lv_label_set_text(ui_LabelHeureScreen5, buffer);
    snprintf(buffer, sizeof(buffer), "%02d", edit_minute);
    lv_label_set_text(ui_LabelMinuteScreen5, buffer);
    snprintf(buffer, sizeof(buffer), "%02d", edit_second);
    lv_label_set_text(ui_LabelSecondeScreen5, buffer);
}

void update_date_display() {
    char buffer[8];
    snprintf(buffer, sizeof(buffer), "%02d", edit_day);
    lv_label_set_text(ui_LabelJourScreen5, buffer);
    snprintf(buffer, sizeof(buffer), "%02d", edit_month);
    lv_label_set_text(ui_LabelMoisScreen5, buffer);
    snprintf(buffer, sizeof(buffer), "%02d", edit_year % 100);
    lv_label_set_text(ui_LabelAnneeScreen5, buffer);
}

// Enregistrement RTC
void update_rtc_time() {
    const struct device *rtc_dev = DEVICE_DT_GET(DT_NODELABEL(rv8263));
    if (!device_is_ready(rtc_dev)) return;

    struct rtc_time new_time = {
        .tm_sec  = edit_second,
        .tm_min  = edit_minute,
        .tm_hour = edit_hour,
        .tm_mday = edit_day,
        .tm_mon  = edit_month - 1,
        .tm_year = edit_year - 1900,
    };

    rtc_set_time(rtc_dev, &new_time);
}

// Initialisation des variables depuis l'heure réelle
void init_edit_time_from_rtc() {
    const struct device *rtc_dev = DEVICE_DT_GET(DT_NODELABEL(rv8263));
    if (!device_is_ready(rtc_dev)) return;

    struct rtc_time now;
    if (rtc_get_time(rtc_dev, &now) == 0) {
        edit_hour = now.tm_hour;
        edit_minute = now.tm_min;
        edit_second = now.tm_sec;
        edit_day = now.tm_mday;
        edit_month = now.tm_mon + 1;
        edit_year = now.tm_year + 1900;
        update_time_display();
        update_date_display();
    }
}

// Callbacks LVGL

// Heure
void hour_plus_cb(lv_event_t * e) { edit_hour = (edit_hour + 1) % 24; update_time_display(); update_rtc_time(); }
void hour_minus_cb(lv_event_t * e) { edit_hour = (edit_hour + 23) % 24; update_time_display(); update_rtc_time(); }
// Minute
void minute_plus_cb(lv_event_t * e) { edit_minute = (edit_minute + 1) % 60; update_time_display(); update_rtc_time(); }
void minute_minus_cb(lv_event_t * e) { edit_minute = (edit_minute + 59) % 60; update_time_display(); update_rtc_time(); }
// Seconde
void second_plus_cb(lv_event_t * e) { edit_second = (edit_second + 1) % 60; update_time_display(); update_rtc_time(); }
void second_minus_cb(lv_event_t * e) { edit_second = (edit_second + 59) % 60; update_time_display(); update_rtc_time(); }

// Jour
void day_plus_cb(lv_event_t * e) { edit_day = (edit_day % 31) + 1; update_date_display(); update_rtc_time(); }
void day_minus_cb(lv_event_t * e) { edit_day = (edit_day + 29) % 31 + 1; update_date_display(); update_rtc_time(); }
// Mois
void month_plus_cb(lv_event_t * e) { edit_month = (edit_month % 12) + 1; update_date_display(); update_rtc_time(); }
void month_minus_cb(lv_event_t * e) { edit_month = (edit_month + 10) % 12 + 1; update_date_display(); update_rtc_time(); }
// Année
void year_plus_cb(lv_event_t * e) { edit_year += 1; update_date_display(); update_rtc_time(); }
void year_minus_cb(lv_event_t * e) { if (edit_year > 2000) edit_year -= 1; update_date_display(); update_rtc_time(); }

// Callbacks Home
void home_cb(lv_event_t * e) { lv_scr_load(ui_Screen1); flagView = 1; }

