/* Linux i2c-dev platform for the ST VL53L7CX ULD (ECPS295 Orange Pi port, replaces the Arduino Wire platform). */
#ifndef _VL53L7CX_PLATFORM_H_
#define _VL53L7CX_PLATFORM_H_
#pragma once
#include <stdint.h>
#include <string.h>
#include "vl53l7cx_platform_config.h"

typedef struct {
  uint16_t address; /* 8-bit address as in the ULD (0x52 = 7-bit 0x29) */
  int fd;           /* open /dev/i2c-N */
} VL53L7CX_Platform;

#endif
