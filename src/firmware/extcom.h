/*
 * bbshd-fw
 *
 * Copyright (C) Daniel Nilsson, 2021.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _EXTCOM_H_
#define _EXTCOM_H_

#include "cfgstore.h"

void extcom_init();
void extcom_process();

#ifdef DEBUG_WITH_DISPLAY
void set_debug_value(uint8_t value);
#endif

#endif

