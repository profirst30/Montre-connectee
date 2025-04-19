#include "chrono.h"
#include <zephyr/kernel.h>
#include <stdio.h>

static lv_obj_t *sec_label = NULL;
static lv_obj_t *min_label = NULL;
static lv_obj_t *hour_label = NULL;

static int seconds = 0;

// Timer Zephyr
K_TIMER_DEFINE(chrono_timer, NULL, NULL);  // initialisé dans chrono_start

// Work item Zephyr pour mise à jour de l’UI
static struct k_work_delayable chrono_work;

// Fonction appelée toutes les secondes
static void chrono_work_handler(struct k_work *work)
{
    seconds++;

    int hrs = seconds / 3600;
    int mins = (seconds % 3600) / 60;
    int secs = seconds % 60;

    char buf[4];

    snprintf(buf, sizeof(buf), "%02d", secs);
    lv_label_set_text(sec_label, buf);

    snprintf(buf, sizeof(buf), "%02d", mins);
    lv_label_set_text(min_label, buf);

    snprintf(buf, sizeof(buf), "%02d", hrs);
    lv_label_set_text(hour_label, buf);

    // Replanifier pour la prochaine seconde
    k_work_schedule(&chrono_work, K_SECONDS(1));
}

void chrono_init(lv_obj_t *sec, lv_obj_t *min, lv_obj_t *hr)
{
    sec_label = sec;
    min_label = min;
    hour_label = hr;

    seconds = 0;

    k_work_init_delayable(&chrono_work, chrono_work_handler);
}

void chrono_start(void)
{
    // Lance le work delayable toutes les secondes
    k_work_schedule(&chrono_work, K_SECONDS(1));
}

void chrono_stop(void)
{
    // Arrête l'exécution future
    k_work_cancel_delayable(&chrono_work);
}

void chrono_reset(void)
{
    chrono_stop();
    seconds = 0;

    lv_label_set_text(sec_label, "00");
    lv_label_set_text(min_label, "00");
    lv_label_set_text(hour_label, "00");
}
