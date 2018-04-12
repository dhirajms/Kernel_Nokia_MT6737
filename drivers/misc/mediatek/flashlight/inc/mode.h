#ifndef _MODE_H
#define _MODE_H

typedef enum {
	FLASH_EN = 0,
	FLASH_CONCRL = 1,

} flash_pin;

typedef enum {
	PIN_HIGH = 0,
	PIN_LOW = 1,
	PIN_PWM = 2,

} pin_mod;

#endif