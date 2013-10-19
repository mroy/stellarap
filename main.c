/*
stellarap/main.c
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
#include <stdint.h>
#include <stdbool.h>


#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"

#include "driverlib/gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"

#include "delay.h"
#include "stepper_control.h"
#include "planner.h"
#include "interpreter.h"
#include "endstops.h"
#include "heaters.h"

int main(void)
{
  ROM_FPUStackingEnable();
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL  | SYSCTL_XTAL_16MHZ ); 

  endstops_init();
  motor_init();
  delay_init();
  interpreter_init();
  heaters_init();

  ROM_IntMasterEnable(); //enable interrupts

  while (true)
  {
    read_command();
  }
  
  return 0;
}
