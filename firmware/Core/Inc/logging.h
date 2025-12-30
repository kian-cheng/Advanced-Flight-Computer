#ifndef LOGGING_H
#define LOGGING_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>

bool logging_init(void);
void logging_tick(void);

#endif