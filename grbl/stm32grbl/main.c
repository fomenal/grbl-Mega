#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include <string.h>
#include "serial.h"
#include "grbl.h"
#include "stm32eeprom.h"

system_t sys;

int main(void)
{
#ifdef LEDBLINK
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13 ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif
	Set_System();

	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();

#ifndef NOEEPROMSUPPORT
    FLASH_Unlock();
    eeprom_init();
#endif
    settings_init(); // Load Grbl settings from EEPROM
    stepper_init();  // Configure stepper pins and interrupt timers
    system_init();

    memset(&sys, 0, sizeof(system_t));  // Clear all system variables
    SysTick->CTRL&=0xfffffffb;



#ifdef AVRTARGET
  sei(); // Enable interrupts
#endif

  // Check for power-up and set system alarm if homing is enabled to force homing cycle
  // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
  // startup scripts, but allows access to settings and internal commands. Only a homing
  // cycle '$H' or kill alarm locks '$X' will disable the alarm.
  // NOTE: The startup script will run after successful completion of the homing cycle, but
  // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
  // things uncontrollably. Very bad.
  #ifdef HOMING_INIT_LOCK
    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
  #endif

  // Force Grbl into an ALARM state upon a power-cycle or hard reset.
  #ifdef FORCE_INITIALIZATION_ALARM
    sys.state = STATE_ALARM;
  #endif

  // Grbl initialization loop upon power-up or a system abort. For the latter, all processes
  // will return to this loop to be cleanly re-initialized.
  for(;;) {

    // TODO: Separate configure task that require interrupts to be disabled, especially upon
    // a system abort and ensuring any active interrupts are cleanly reset.

    // Reset Grbl primary systems.
    serial_reset_read_buffer(); // Clear serial read buffer
    gc_init(); // Set g-code parser to default state
    spindle_init();
    coolant_init();
    limits_init();
    probe_init();
    sleep_init();
    plan_reset(); // Clear block buffer and planner variables
    st_reset(); // Clear stepper subsystem variables.

    // Sync cleared gcode and planner positions to current system position.
    plan_sync_position();
    gc_sync_position();

    // Reset system variables.
    sys.abort = false;
    sys_rt_exec_state = 0;
    sys_rt_exec_alarm = 0;
    sys.suspend = false;
    sys.soft_limit = false;
//    EE_WriteVariable(0,0x5566);
//    EE_Flush();
    // Start Grbl main loop. Processes program inputs and executes them.
    protocol_main_loop();

  }
  return 0;   /* Never reached */

}
void _delay_ms(uint32_t x)
{
	u32 temp;
	SysTick->LOAD=(u32)72000000 / 8000;                     // Loading time
	SysTick->VAL =0x00;                                            // Empty the counter
	SysTick->CTRL=0x01 ;                                           // Start from bottom
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));                             // Wait time arrive
	SysTick->CTRL=0x00;                                            // Close the counter
	SysTick->VAL =0X00;                                            // Empty the counter
}

#ifdef LEDBLINK
void LedBlink(void)
{
	static BitAction nOnFlag = Bit_SET;
	GPIO_WriteBit(GPIOC,GPIO_Pin_13,nOnFlag);
	nOnFlag = (nOnFlag == Bit_SET) ? Bit_RESET : Bit_SET;
}
#endif
