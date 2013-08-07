/*
stellarap/delay.c
Copyright (c) 2013 Mark Roy

This file is part of the Stellarap 3D Printer firmware.
http://www.stellarcore.com
http://www.github.com/mroy/stellarap

Stellarap is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Stellarap is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Stellarap.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdlib.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "delay.h"

unsigned char delay_stat = DELAY_IDLE;
unsigned long delay_count = 0; 
void delay_isr()
{
  ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  
  if (delay_count > 50000)
  {
    delay_count -= 50000; 
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, 50000*80000);
    ROM_TimerEnable(TIMER1_BASE, TIMER_A);
  } else if (delay_count > 0)
  {
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, delay_count*80000);
    ROM_TimerEnable(TIMER1_BASE, TIMER_A);
    delay_count = 0;
  } else
  {
    delay_stat = DELAY_IDLE;
    ROM_TimerIntDisable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    ROM_TimerDisable(TIMER1_BASE, TIMER_A);
  }
}

void delay(unsigned int milli)
{
  delay_count = milli;
  if (delay_count > 50000) 
  {
    milli = 50000;
    delay_count -= milli; 
  } else
  {
    milli = delay_count;
    delay_count = 0;
  }

  // 80000000 ticks/s = 80000 ticks/ms7
  ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, milli*80000);
  delay_stat = DELAY_RUNNING;
  ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  ROM_TimerEnable(TIMER1_BASE, TIMER_A);
}

void delay_init()
{
  //configure timer1 for one shot intervals and assign interrupt routine
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
  ROM_TimerConfigure(TIMER1_BASE,TIMER_CFG_ONE_SHOT);
  ROM_TimerControlStall(TIMER1_BASE, TIMER_A, true);
  TimerIntRegister(TIMER1_BASE, TIMER_A, delay_isr);

}
