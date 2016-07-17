/*
  coolant_control.c - coolant control methods
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon

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

#include "grbl.h"


void coolant_init()
{
#ifdef AVRTARGET
  COOLANT_FLOOD_DDR |= (1 << COOLANT_FLOOD_BIT);
  COOLANT_MIST_DDR |= (1 << COOLANT_MIST_BIT);
#endif
#ifdef STM32F103C8
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_COOLANT_FLOOD_PORT, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin   = 1 << COOLANT_FLOOD_BIT;
	GPIO_Init(COOLANT_FLOOD_PORT, &GPIO_InitStructure);

   	RCC_APB2PeriphClockCmd(RCC_COOLANT_MIST_PORT, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin   = 1 << COOLANT_MIST_BIT;
	GPIO_Init(COOLANT_FLOOD_PORT, &GPIO_InitStructure);
#endif
  coolant_stop();
}


uint8_t coolant_is_enabled()
{
#ifdef AVRTARGET
  #ifdef INVERT_COOLANT_FLOOD_PIN
    if (!(COOLANT_FLOOD_PORT & (1<<COOLANT_FLOOD_BIT))) { return(true); }
  #else
    if (COOLANT_FLOOD_PORT & (1<<COOLANT_FLOOD_BIT)) { return(true); }
  #endif
  #ifdef INVERT_COOLANT_MIST_PIN
    if (!(COOLANT_MIST_PORT & (1<<COOLANT_MIST_BIT))) { return(true); }
  #else
    if (COOLANT_MIST_PORT & (1<<COOLANT_MIST_BIT)) { return(true); }
  #endif
#endif
#ifdef STM32F103C8
  #ifdef INVERT_COOLANT_FLOOD_PIN
    if (!(GPIO_ReadOutputData(COOLANT_FLOOD_PORT) & (1<<COOLANT_FLOOD_BIT))) { return(true); }
  #else
    if (GPIO_ReadOutputData(COOLANT_FLOOD_PORT) & (1<<COOLANT_FLOOD_BIT)) { return(true); }
  #endif
  #ifdef INVERT_COOLANT_MIST_PIN
    if (!(GPIO_ReadOutputData(COOLANT_MIST_PORT) & (1<<COOLANT_MIST_BIT))) { return(true); }
  #else
    if (GPIO_ReadOutputData(COOLANT_MIST_PORT) & (1<<COOLANT_MIST_BIT)) { return(true); }
  #endif
#endif
  return(false); 
}


void coolant_stop()
{
#ifdef AVRTARGET
#ifdef INVERT_COOLANT_FLOOD_PIN
    COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT);
#else
    COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
#endif
  COOLANT_MIST_PORT &= ~(1 << COOLANT_MIST_BIT);
#endif
#ifdef STM32F103C8
#ifdef INVERT_COOLANT_FLOOD_PIN
  GPIO_SetBits(COOLANT_FLOOD_PORT,1 << COOLANT_FLOOD_BIT);
#else
  GPIO_ResetBits(COOLANT_FLOOD_PORT,1 << COOLANT_FLOOD_BIT);
#endif
  GPIO_ResetBits(COOLANT_MIST_PORT,1 << COOLANT_MIST_BIT);
#endif
}


void coolant_set_state(uint8_t mode)
{
  if (sys.abort) { return; } // Block during abort

  if (mode == COOLANT_FLOOD_ENABLE) {
#ifdef AVRTARGET
    #ifdef INVERT_COOLANT_FLOOD_PIN
      COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
    #else
      COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT);
    #endif
#endif
#ifdef STM32F103C8
  #ifdef INVERT_COOLANT_FLOOD_PIN
    GPIO_ResetBits(COOLANT_FLOOD_PORT,1 << COOLANT_FLOOD_BIT);
  #else
    GPIO_SetBits(COOLANT_FLOOD_PORT,1 << COOLANT_FLOOD_BIT);
  #endif
#endif

    } else if (mode == COOLANT_MIST_ENABLE) {
#ifdef AVRTARGET
    #ifdef INVERT_COOLANT_MIST_PIN
      COOLANT_MIST_PORT &= ~(1 << COOLANT_MIST_BIT);
    #else
      COOLANT_MIST_PORT |= (1 << COOLANT_MIST_BIT);
    #endif
#endif
#ifdef STM32F103C8
    #ifdef INVERT_COOLANT_MIST_PIN
      GPIO_ResetBits(COOLANT_MIST_PORT,~(1 << COOLANT_MIST_BIT));
    #else
      GPIO_SetBits(COOLANT_MIST_PORT,(1 << COOLANT_MIST_BIT));
    #endif
#endif
  } else {
    coolant_stop();
  }
}


void coolant_run(uint8_t mode)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Ensure coolant turns on when specified in program.  
  coolant_set_state(mode);
}
