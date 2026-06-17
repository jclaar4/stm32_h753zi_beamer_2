#pragma once
#include <stdbool.h>
#include "main.h"

extern I2C_HandleTypeDef hi2c2;

void init_dacs();
void mute_dacs(bool mute);
void reset_dacs();
void mute_amps(bool mute);
