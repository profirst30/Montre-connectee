// rtc_callbacks.h
#ifndef RTC_CALLBACKS_H
#define RTC_CALLBACKS_H

#include <lvgl.h>

// Initialise les variables depuis la RTC et met à jour l'affichage
void init_edit_time_from_rtc(void);

// Met à jour la RTC avec les valeurs courantes des variables
void update_rtc_time(void);

// Met à jour l'affichage LVGL de l'heure
void update_time_display(void);

// Met à jour l'affichage LVGL de la date
void update_date_display(void);

// Callbacks LVGL pour l'heure
void hour_plus_cb(lv_event_t * e);
void hour_minus_cb(lv_event_t * e);
void minute_plus_cb(lv_event_t * e);
void minute_minus_cb(lv_event_t * e);
void second_plus_cb(lv_event_t * e);
void second_minus_cb(lv_event_t * e);

// Callbacks LVGL pour la date
void day_plus_cb(lv_event_t * e);
void day_minus_cb(lv_event_t * e);
void month_plus_cb(lv_event_t * e);
void month_minus_cb(lv_event_t * e);
void year_plus_cb(lv_event_t * e);
void year_minus_cb(lv_event_t * e);

// Callback Home
void home_cb(lv_event_t * e);


#endif // RTC_CALLBACKS_H
