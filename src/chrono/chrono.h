#ifndef CHRONO_H
#define CHRONO_H

#include <lvgl.h>

// Initialise le chronomètre avec les pointeurs vers les labels LVGL
void chrono_init(lv_obj_t *sec, lv_obj_t *min, lv_obj_t *hr);

// Démarre le chronomètre
void chrono_start(void);

// Arrête le chronomètre
void chrono_stop(void);

// Réinitialise le chronomètre (remet à zéro les compteurs et l'affichage)
void chrono_reset(void);

#endif // CHRONO_H
