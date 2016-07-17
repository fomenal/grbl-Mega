/*
  grbl.h - main Grbl include file
  Part of Grbl

  Copyright (c) 2015 Sungeun K. Jeon

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef grbl_h
#define grbl_h

// Grbl versioning system
#define GRBL_VERSION "1.0d"
#define GRBL_VERSION_BUILD "20160510"

// Define standard libraries used by Grbl.
#ifdef AVRTARGET
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <inttypes.h>    
#include <stdbool.h>
#define PORTPINDEF uint8_t
#endif
#include <math.h>
#ifdef WIN32
#include <Windows.h>
typedef signed char  int8_t;
typedef signed short int16_t;
typedef signed int   int32_t;
typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int   uint32_t;
typedef signed long long   int64_t;
typedef unsigned long long uint64_t;
typedef int bool;
#define false 0
#define true 1
#define trunc(x) (int32_t)x
#define lround(x) (int32_t) (x + 0.5)
#define round(x) (int32_t) (x + 0.5)
#define PSTR(x) x
#define pgm_read_byte_near(x) *(x)
#define _delay_ms(x) Sleep(x)
#define M_PI (float)3.1415926
#define LOG(x,y)
#define PORTPINDEF uint8_t
#endif
#ifdef STM32F103C8
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#define PSTR(x) x
#define pgm_read_byte_near(x) *(x)
void _delay_ms(uint32_t x);
void _delay_us(uint32_t x);
#define false 0
#define true 1
#define PORTPINDEF uint16_t
typedef int bool;
#define NOEEPROMSUPPORT
#endif
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

// Define the Grbl system include files. NOTE: Do not alter organization.
#include "config.h"
#include "nuts_bolts.h"
#include "settings.h"
#include "system.h"
#include "defaults.h"
#include "cpu_map.h"
#include "coolant_control.h"
#include "eeprom.h"
#include "gcode.h"
#include "limits.h"
#include "motion_control.h"
#include "planner.h"
#include "print.h"
#include "probe.h"
#include "protocol.h"
#include "report.h"
#include "serial.h"
#include "spindle_control.h"
#include "stepper.h"
#include "sleep.h"
#endif
