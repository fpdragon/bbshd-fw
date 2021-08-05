/*
 * bbshd-fw
 *
 * Copyright (C) Daniel Nilsson, 2021.
 *
 * Released under the GPL License, Version 3
 */

#include "throttle.h"
#include "stc15.h"
#include "pins.h"
#include "system.h"
#include "util.h"
#include "filter.h"

#include <stdbool.h>

static __xdata uint16_t min_voltage_x1000;
static __xdata uint16_t max_voltage_x1000;
static __xdata uint8_t start_percent;

static __xdata bool throttle_low_ok;
static __xdata bool throttle_hard_ok;
static __xdata uint32_t throttle_hard_limit_hit_at;
static __xdata int16_t throttle_last_voltage_x1000;

#define ADC_VOLTAGE_MV						5000

#define THROTTLE_HARD_LOW_LIMIT_MV			0500
#define THROTTLE_HARD_HIGH_LIMIT_MV			4500
#define THROTTLE_HARD_LIMIT_TOLERANCE_MS	1000


void throttle_init(__xdata uint16_t min_mv, __xdata uint16_t max_mv)
{
	min_voltage_x1000 = min_mv;
	max_voltage_x1000 = max_mv;
	start_percent = 0;
	throttle_low_ok = false;
	throttle_hard_ok = true;
	throttle_hard_limit_hit_at = 0;
	throttle_last_voltage_x1000 = 0;

	// Setup pin 3 as adc input
	SET_PIN_INPUT(PIN_THROTTLE);
	SET_PIN_LOW(PIN_THROTTLE);
	SET_BIT(P1ASF, GET_PIN_NUM(PIN_THROTTLE));

	ADC_RES = 0;

	// Arrange adc result for 8bit reading
	CLEAR_BIT(PCON2, 5);

	// enable adc power, set adc speed
	ADC_CONTR = (uint8_t)((1 << 7) | (1 << 5));
}

bool throttle_ok()
{
	return true;
	//return throttle_low_ok && throttle_hard_ok;
}

uint8_t throttle_read()
{
	int16_t value = 0;

	for (uint8_t i = 0; i < 4; ++i)
	{
		// Sample ADC on pin 3.
		ADC_CONTR = (uint8_t)((1 << 7) | (1 << 5) | (1 << 3) | (GET_PIN_NUM(PIN_THROTTLE) & 0x07));

		// as per specification
		NOP();
		NOP();
		NOP();
		NOP();

		while (!IS_BIT_SET(ADC_CONTR, 4));
		CLEAR_BIT(ADC_CONTR, 4);

		value += ADC_RES;

		// some delay between samples
		for (uint16_t t = 0; t < 4000; ++t) NOP();
	}

	value /= 4;

	value = (int16_t)((value * (uint32_t)ADC_VOLTAGE_MV) / 256);

	if (value < THROTTLE_HARD_LOW_LIMIT_MV || value > THROTTLE_HARD_HIGH_LIMIT_MV)
	{
		// allow invalid throttle input value for a number of milliseconds before reporting throttle error.
		value = throttle_last_voltage_x1000;
		if (throttle_hard_limit_hit_at != 0)
		{
			if (throttle_hard_ok && (system_ms() - throttle_hard_limit_hit_at) > THROTTLE_HARD_LIMIT_TOLERANCE_MS)
			{
				throttle_hard_ok = false;
			}
		}
		else
		{
			throttle_hard_limit_hit_at = system_ms();
		}
	}
	else
	{
		throttle_hard_limit_hit_at = 0;
		throttle_hard_ok = true;
		throttle_last_voltage_x1000 = value;
	}

	if (value < min_voltage_x1000)
	{
		// throttle is considered not working until it has given a signal below minimum value
		throttle_low_ok = true;
		return 0;
	}

	if (value > max_voltage_x1000)
	{
		value = max_voltage_x1000;
	}

	return (uint8_t)MAP(value, min_voltage_x1000, max_voltage_x1000, 0, 100);
}

