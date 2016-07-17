/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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


void spindle_init()
{    
#ifdef AVRTARGET
  // Configure variable spindle PWM and enable pin, if required.
  SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.
  SPINDLE_TCCRA_REGISTER = SPINDLE_TCCRA_INIT_MASK; // Configure PWM output compare timer
  SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK;
  SPINDLE_OCRA_REGISTER = SPINDLE_OCRA_TOP_VALUE; // Set the top value for 16-bit fast PWM mode
  SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
  SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
#endif
#if defined (STM32F103C8)
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_SPINDLE_ENABLE_PORT, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP; 
#ifdef VARIABLE_SPINDLE
#else
	GPIO_InitStructure.GPIO_Pin   = 1 << SPINDLE_ENABLE_BIT;
    GPIO_Init(STEPPERS_DISABLE_PORT, &GPIO_InitStructure);
#endif
  #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
	GPIO_InitStructure.GPIO_Pin   = 1 << SPINDLE_DIRECTION_BIT;
    GPIO_Init(STEPPERS_DISABLE_PORT, &GPIO_InitStructure);
  #endif
#endif
  spindle_stop();
}


uint8_t spindle_is_enabled()
{
#ifdef AVRTARGET
  #ifdef INVERT_SPINDLE_ENABLE_PIN
    if ((SPINDLE_ENABLE_PORT) & (1<<SPINDLE_ENABLE_BIT)) { return(true); }
    else { return(false); }
  #else
    if ((SPINDLE_ENABLE_PORT) & (1<<SPINDLE_ENABLE_BIT)) { return(false); }
    else { return(true); }
  #endif
#endif
#if defined (STM32F103C8)
  #ifdef INVERT_SPINDLE_ENABLE_PIN
    if (GPIO_ReadOutputData(SPINDLE_ENABLE_PORT) & (1<<SPINDLE_ENABLE_BIT)) { return(true); }
    else { return(false); }
  #else
    if (GPIO_ReadOutputData(SPINDLE_ENABLE_PORT) & (1<<SPINDLE_ENABLE_BIT)) { return(false); }
    else { return(true); }
  #endif

#endif
#ifdef WIN32
   return false;
#endif
}


void spindle_stop()
{
#ifdef AVRTARGET
  SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.
  #ifdef INVERT_SPINDLE_ENABLE_PIN
    SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
  #else
    SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
  #endif
#endif
#if defined (STM32F103C8)
//    todo
    #ifdef INVERT_SPINDLE_ENABLE_PIN
      GPIO_WriteBit(SPINDLE_ENABLE_PORT,1<<SPINDLE_ENABLE_BIT,Bit_SET);
	#else
      GPIO_WriteBit(SPINDLE_ENABLE_PORT,1<<SPINDLE_ENABLE_BIT,Bit_RESET);
	#endif
#endif
}


void spindle_set_state(uint8_t state, float rpm)
{
  if (sys.abort) { return; } // Block during abort.

  // Halt or set spindle direction and rpm. 
  if (state == SPINDLE_DISABLE) {

    spindle_stop();

  } else {
#if defined(AVRTARGET) || defined (STM32F103C8)
    #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
      if (state == SPINDLE_ENABLE_CW) {
#if defined(AVRTARGET)
        SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
#else
        GPIO_WriteBit(SPINDLE_DIRECTION_PORT,1<<SPINDLE_DIRECTION_BIT,Bit_RESET);
#endif
      } else {
#if defined(AVRTARGET)
        SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
#else
        GPIO_WriteBit(SPINDLE_DIRECTION_PORT,1<<SPINDLE_DIRECTION_BIT,Bit_SET);
#endif
      }
    #endif

    #ifdef VARIABLE_SPINDLE
      // TODO: Install the optional capability for frequency-based output for servos.
      #ifdef CPU_MAP_ATMEGA2560
      	TCCRA_REGISTER = (1<<COMB_BIT) | (1<<WAVE1_REGISTER) | (1<<WAVE0_REGISTER);
        TCCRB_REGISTER = (TCCRB_REGISTER & 0b11111000) | 0x02 | (1<<WAVE2_REGISTER) | (1<<WAVE3_REGISTER); // set to 1/8 Prescaler
        OCR4A = 0xFFFF; // set the top 16bit value
        uint16_t current_pwm;
      #else
        TCCRA_REGISTER = (1<<COMB_BIT) | (1<<WAVE1_REGISTER) | (1<<WAVE0_REGISTER);
        TCCRB_REGISTER = (TCCRB_REGISTER & 0b11111000) | 0x02; // set to 1/8 Prescaler
        uint8_t current_pwm;
      #endif

      #define SPINDLE_RPM_RANGE (SPINDLE_MAX_RPM-SPINDLE_MIN_RPM)
      if ( rpm < SPINDLE_MIN_RPM ) { rpm = 0; } 
      else { 
        rpm -= SPINDLE_MIN_RPM; 
        if ( rpm > SPINDLE_RPM_RANGE ) { rpm = SPINDLE_RPM_RANGE; } // Prevent integer overflow
      }
      current_pwm = floor( rpm*(PWM_MAX_VALUE/SPINDLE_RPM_RANGE) + 0.5);
      #ifdef MINIMUM_SPINDLE_PWM
        if (current_pwm < MINIMUM_SPINDLE_PWM) { current_pwm = MINIMUM_SPINDLE_PWM; }
      #endif
      OCR_REGISTER = current_pwm; // Set PWM pin output
    
      // On the Uno, spindle enable and PWM are shared, unless otherwise specified.
      #if defined(CPU_MAP_ATMEGA2560) || defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) 
        #ifdef INVERT_SPINDLE_ENABLE_PIN
#if defined(AVRTARGET)
            SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
#else
            GPIO_WriteBit(SPINDLE_ENABLE_PORT,1<<SPINDLE_ENABLE_BIT,Bit_RESET);
#endif
        #else
#if defined(AVRTARGET)
            SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
#else
            GPIO_WriteBit(SPINDLE_ENABLE_PORT,1<<SPINDLE_ENABLE_BIT,Bit_SET);
#endif
        #endif
      #endif
      
    #else   
      #ifdef INVERT_SPINDLE_ENABLE_PIN
#if defined(AVRTARGET)
		SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
#else
        GPIO_WriteBit(SPINDLE_ENABLE_PORT,1<<SPINDLE_ENABLE_BIT,Bit_RESET);
#endif
	  #else
#if defined(AVRTARGET)
		SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
#else
        GPIO_WriteBit(SPINDLE_ENABLE_PORT,1<<SPINDLE_ENABLE_BIT,Bit_SET);
#endif
	  #endif
    #endif
#endif
  }
}


void spindle_run(uint8_t state, float rpm)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.  
  spindle_set_state(state, rpm);
}
