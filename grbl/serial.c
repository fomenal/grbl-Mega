/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2011-2015 Sungeun K. Jeon
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
#ifdef WIN32
#include <stdio.h>
#include <process.h> 
#include <conio.h>
CRITICAL_SECTION CriticalSection; 

HANDLE hSerial = INVALID_HANDLE_VALUE;
void RecvthreadFunction( void *);
void SendthreadFunction( void *);
HANDLE g_hRecv;
HANDLE g_hSend;
HANDLE g_hSendEvent;

#endif
#ifdef STM32F103C8
#include "stm32f10x.h"
#include "core_cm3.h"
#endif

uint8_t serial_rx_buffer[RX_BUFFER_SIZE];
uint8_t serial_rx_buffer_head = 0;
volatile uint8_t serial_rx_buffer_tail = 0;

uint8_t serial_tx_buffer[TX_BUFFER_SIZE];
uint8_t serial_tx_buffer_head = 0;
volatile uint8_t serial_tx_buffer_tail = 0;


#ifdef ENABLE_XONXOFF
  volatile uint8_t flow_ctrl = XON_SENT; // Flow control state variable
#endif
  

// Returns the number of bytes used in the RX serial buffer.
uint8_t serial_get_rx_buffer_count()
{
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(serial_rx_buffer_head-rtail); }
  return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}


// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count()
{
  uint8_t ttail = serial_tx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_tx_buffer_head >= ttail) { return(serial_tx_buffer_head-ttail); }
  return (TX_BUFFER_SIZE - (ttail-serial_tx_buffer_head));
}


void serial_init()
{
#ifdef AVRTARGET
  // Set baud rate
  #if BAUD_RATE < 57600
    uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
    UCSR0A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
  #else
    uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
    UCSR0A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200
  #endif
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;
            
  // enable rx and tx
  UCSR0B |= 1<<RXEN0;
  UCSR0B |= 1<<TXEN0;
	
  // enable interrupt on complete reception of a byte
  UCSR0B |= 1<<RXCIE0;
	  
  // defaults to 8-bit, no parity, 1 stop bit
#endif
#ifdef WIN32
  InitializeCriticalSectionAndSpinCount(&CriticalSection,0x00000400);
#endif

}

#ifdef WIN32
#define MAX_DEVPATH_LENGTH 1024
void winserial_init(char *pPort)
{
    DCB dcb;
    BOOL fSuccess;
    TCHAR devicePath[MAX_DEVPATH_LENGTH];
    COMMTIMEOUTS commTimeout;

    if (pPort != NULL)
    {
        mbstowcs_s(NULL, devicePath, MAX_DEVPATH_LENGTH, pPort, strlen(pPort));
        hSerial = CreateFile(devicePath, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                    OPEN_EXISTING, 0, NULL);
    }
    if (hSerial != INVALID_HANDLE_VALUE)
    {
        //  Initialize the DCB structure.
        SecureZeroMemory(&dcb, sizeof(DCB));
        dcb.DCBlength = sizeof(DCB);
        fSuccess = GetCommState(hSerial, &dcb);
        if (!fSuccess) 
        {
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
            return;
        }

        GetCommState(hSerial, &dcb);
        dcb.BaudRate = CBR_115200;     //  baud rate
        dcb.ByteSize = 8;             //  data size, xmit and rcv
        dcb.Parity   = NOPARITY;      //  parity bit
        dcb.StopBits = ONESTOPBIT;    //  stop bit
        dcb.fBinary = TRUE;
        dcb.fParity = TRUE;

        fSuccess = SetCommState(hSerial, &dcb);
        if (!fSuccess) 
        {
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
            return;
        }

        GetCommTimeouts(hSerial, &commTimeout);
        commTimeout.ReadIntervalTimeout     = 1;
        commTimeout.ReadTotalTimeoutConstant     = 1;
        commTimeout.ReadTotalTimeoutMultiplier     = 1;
        commTimeout.WriteTotalTimeoutConstant     = 10;
        commTimeout.WriteTotalTimeoutMultiplier = 10;
        SetCommTimeouts(hSerial, &commTimeout);
    }
    g_hRecv = CreateMutex(NULL, false,NULL);
    g_hSend = CreateMutex(NULL, FALSE, NULL);
    g_hSendEvent = CreateEvent(NULL,FALSE,FALSE,NULL);
    _beginthread( RecvthreadFunction, 0, NULL );
    _beginthread( SendthreadFunction, 0, NULL );
}
#endif
// Writes one byte to the TX serial buffer. Called by main program.
// TODO: Check if we can speed this up for writing strings, rather than single bytes.
void serial_write(uint8_t data) {
#if defined(AVRTARGET) || defined (STM32F103C8)

  // Calculate next head
    uint8_t next_head = serial_tx_buffer_head + 1;
    if (next_head == TX_BUFFER_SIZE) { next_head = 0; }
  // Wait until there is space in the buffer
  while (next_head == serial_tx_buffer_tail) { 
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.    
    if (sys_rt_exec_state & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
  }

  // Store data and advance head
  serial_tx_buffer[serial_tx_buffer_head] = data;
  serial_tx_buffer_head = next_head;

#ifdef AVRTARGET
  UCSR0B |=  (1 << UDRIE0); 
#endif
#endif
#ifdef WIN32
    uint8_t next_head = serial_tx_buffer_head + 1;
    if (next_head == TX_BUFFER_SIZE) 
    { 
        next_head = 0; 
    }
    // Wait until there is space in the buffer
    while (next_head == serial_tx_buffer_tail) 
    { 
        Sleep(0);
    }
    // Store data and advance head
    WaitForSingleObject(g_hSend,INFINITE); 
    serial_tx_buffer[serial_tx_buffer_head] = data;
    serial_tx_buffer_head = next_head;
    ReleaseMutex(g_hSend);
    SetEvent(g_hSendEvent);
#endif
}
#ifdef AVRTARGET
// Data Register Empty Interrupt handler
ISR(SERIAL_UDRE)
{
  uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)
  
  #ifdef ENABLE_XONXOFF
    if (flow_ctrl == SEND_XOFF) { 
      UDR0 = XOFF_CHAR; 
      flow_ctrl = XOFF_SENT; 
    } else if (flow_ctrl == SEND_XON) { 
      UDR0 = XON_CHAR; 
      flow_ctrl = XON_SENT; 
    } else
  #endif
  { 
    // Send a byte from the buffer	
    UDR0 = serial_tx_buffer[tail];
  
    // Update tail position
    tail++;
    if (tail == TX_BUFFER_SIZE) { tail = 0; }
  
    serial_tx_buffer_tail = tail;
  }
  
  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (tail == serial_tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); }
}
#endif

#ifdef STM32F103C8
void OnUsbDataRx(uint8_t* dataIn, uint8_t length)
{
	//lcd_write_char(*dataIn);
	uint8_t next_head;
    uint8_t data;

	// Write data to buffer unless it is full.
	while (length != 0)
	{
        data = *dataIn ++;
        switch (data) 
        {
            case CMD_STATUS_REPORT: 
                system_set_exec_state_flag(EXEC_STATUS_REPORT); 
                break; // Set as true
            case CMD_CYCLE_START:   
                system_set_exec_state_flag(EXEC_CYCLE_START); 
                break; // Set as true
            case CMD_FEED_HOLD:     
                system_set_exec_state_flag(EXEC_FEED_HOLD); 
                break; // Set as true
            case CMD_SAFETY_DOOR:   
                system_set_exec_state_flag(EXEC_SAFETY_DOOR); 
                break; // Set as true
            case CMD_RESET:         
                mc_reset(); 
                break; // Call motion control reset routine.
            default:
                next_head = serial_rx_buffer_head + 1;
                if (next_head == RX_BUFFER_SIZE) 
                { 
                    next_head = 0; 
                }
    
                // Write data to buffer unless it is full.
                if (next_head != serial_rx_buffer_tail) 
                {
                    serial_rx_buffer[serial_rx_buffer_head] = data;
                    serial_rx_buffer_head = next_head;    
                }
                break;
        }
		length --;
	}
}

#endif
#ifdef WIN32
//#define WINLOG
void RecvthreadFunction(void *pVoid )
{
    DWORD  dwBytesRead;
    uint8_t data;
    uint8_t next_head;
    for (;;)
    {
        if (hSerial != INVALID_HANDLE_VALUE)
        {
            if (ReadFile(hSerial, &data, 1, &dwBytesRead, NULL) && dwBytesRead == 1)
            {
            }
            else
            {
                data = 0;
            }
        }
        else
        {
            while (_kbhit() == 0)
                ;
            data = _getch();
        }

        if (data != 0)
        {
            // Pick off realtime command characters directly from the serial stream. These characters are
            // not passed into the buffer, but these set system state flag bits for realtime execution.
            switch (data) 
            {
                case CMD_STATUS_REPORT: 
                    system_set_exec_state_flag(EXEC_STATUS_REPORT); 
                     break; // Set as true
                case CMD_CYCLE_START:   
                    system_set_exec_state_flag( EXEC_CYCLE_START); 
                    break; // Set as true
                case CMD_FEED_HOLD:     
                    system_set_exec_state_flag(EXEC_FEED_HOLD); 
                    break; // Set as true
                case CMD_SAFETY_DOOR:   
                    system_set_exec_state_flag(EXEC_SAFETY_DOOR); 
                    break; // Set as true
                case CMD_RESET:         
                    mc_reset(); 
                    break; // Call motion control reset routine.

                default: // Write character to buffer    
                    WaitForSingleObject(g_hRecv,INFINITE); 
                    next_head = serial_rx_buffer_head + 1;
                    if (next_head == RX_BUFFER_SIZE) 
                    { 
                        next_head = 0; 
                    }
    
                    // Write data to buffer unless it is full.
                    if (next_head != serial_rx_buffer_tail) 
                    {
                        serial_rx_buffer[serial_rx_buffer_head] = data;
                        serial_rx_buffer_head = next_head;    
                    }
                    ReleaseMutex(g_hRecv);
                    break;
            }
        }
    }
}
void SendthreadFunction( void *pVoid)
{
    unsigned char szBuf[TX_BUFFER_SIZE];
#if 1
    DWORD dwBytesWritten;
    for (;;)
    {
	    uint16_t USB_Tx_length;
        WaitForSingleObject(g_hSendEvent,INFINITE); 
        WaitForSingleObject(g_hSend,INFINITE); 

        if (serial_tx_buffer_head > serial_tx_buffer_tail)
		    USB_Tx_length = serial_tx_buffer_head - serial_tx_buffer_tail;
        else
        {
		    USB_Tx_length = TX_BUFFER_SIZE - serial_tx_buffer_tail;
            if (USB_Tx_length == 0)
            {
                USB_Tx_length = serial_tx_buffer_head;
                serial_tx_buffer_tail = 0;
            }
        }
    if (USB_Tx_length > 16)
        USB_Tx_length = 16;

        memcpy(szBuf,&serial_tx_buffer[serial_tx_buffer_tail],USB_Tx_length);
        serial_tx_buffer_tail += USB_Tx_length;
        if (serial_tx_buffer_tail == TX_BUFFER_SIZE)
            serial_tx_buffer_tail = 0;

        // copy data
        ReleaseMutex(g_hSend);
        if (serial_tx_buffer_head == serial_tx_buffer_tail)
            ResetEvent(g_hSendEvent);
        else
            SetEvent(g_hSendEvent);

        if (USB_Tx_length != 0)
        {
            if (hSerial != INVALID_HANDLE_VALUE)
                WriteFile(hSerial, szBuf,USB_Tx_length, &dwBytesWritten, NULL);
            else
                fwrite(szBuf,1,USB_Tx_length,stdout);
        }
    }

#else
    DWORD nTotalByte;
    DWORD dwBytesWritten;
    for (;;)
    {
        WaitForSingleObject(g_hSendEvent,INFINITE); 
        WaitForSingleObject(g_hSend,INFINITE); 
        nTotalByte = 0;
        while (serial_tx_buffer_tail != serial_tx_buffer_head)
        {
            szBuf[nTotalByte ++] = serial_tx_buffer[serial_tx_buffer_tail ++];
            if (serial_tx_buffer_tail == TX_BUFFER_SIZE)
                serial_tx_buffer_tail = 0;
        }
        // copy data
        ReleaseMutex(g_hSend);
        if (nTotalByte != 0)
        {
            if (hSerial != INVALID_HANDLE_VALUE)
                WriteFile(hSerial, szBuf,nTotalByte, &dwBytesWritten, NULL);
            else
                fwrite(szBuf,1,nTotalByte,stdout);
        }
    }
#endif
}
#endif
// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read()
{
#ifdef WIN32
    uint8_t tail = serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
    if (serial_rx_buffer_head == tail) 
    {
        return SERIAL_NO_DATA;
    } 
    else 
    {
        uint8_t data;
        WaitForSingleObject(g_hRecv,INFINITE); 
        data = serial_rx_buffer[tail];
        tail++;
        if (tail == RX_BUFFER_SIZE) 
        { 
            tail = 0; 
        }
        serial_rx_buffer_tail = tail;
        ReleaseMutex(g_hRecv);
        return data;
    }
#endif
#if defined(AVRTARGET) || defined (STM32F103C8)
  uint8_t tail = serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
  if (serial_rx_buffer_head == tail) {
    return SERIAL_NO_DATA;
  } else {
#if defined (STM32F103C8)
	  NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
#endif
    uint8_t data = serial_rx_buffer[tail];
    
    tail++;
    if (tail == RX_BUFFER_SIZE) { tail = 0; }
    serial_rx_buffer_tail = tail;

    #ifdef ENABLE_XONXOFF
      if ((serial_get_rx_buffer_count() < RX_BUFFER_LOW) && flow_ctrl == XOFF_SENT) { 
        flow_ctrl = SEND_XON;
        UCSR0B |=  (1 << UDRIE0); // Force TX
      }
    #endif
#if defined (STM32F103C8)
      NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
#endif
    
    return data;
  }
#endif
}

#ifdef AVRTARGET
ISR(SERIAL_RX)
{
  uint8_t data = UDR0;
  uint8_t next_head;
  
  // Pick off realtime command characters directly from the serial stream. These characters are
  // not passed into the buffer, but these set system state flag bits for realtime execution.
  switch (data) {
    case CMD_STATUS_REPORT: system_set_exec_state_flag(EXEC_STATUS_REPORT); break; // Set as true
    case CMD_CYCLE_START:   system_set_exec_state_flag(EXEC_CYCLE_START); break; // Set as true
    case CMD_FEED_HOLD:     system_set_exec_state_flag(EXEC_FEED_HOLD); break; // Set as true
    case CMD_SAFETY_DOOR:   system_set_exec_state_flag(EXEC_SAFETY_DOOR); break; // Set as true
    case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
    default: // Write character to buffer    
      next_head = serial_rx_buffer_head + 1;
      if (next_head == RX_BUFFER_SIZE) { next_head = 0; }
    
      // Write data to buffer unless it is full.
      if (next_head != serial_rx_buffer_tail) {
        serial_rx_buffer[serial_rx_buffer_head] = data;
        serial_rx_buffer_head = next_head;    
        
        #ifdef ENABLE_XONXOFF
          if ((serial_get_rx_buffer_count() >= RX_BUFFER_FULL) && flow_ctrl == XON_SENT) {
            flow_ctrl = SEND_XOFF;
            UCSR0B |=  (1 << UDRIE0); // Force TX
          } 
        #endif
        
      }
      //TODO: else alarm on overflow?
  }
}
#endif

void serial_reset_read_buffer() 
{
  serial_rx_buffer_tail = serial_rx_buffer_head;

  #ifdef ENABLE_XONXOFF
    flow_ctrl = XON_SENT;
  #endif
}
