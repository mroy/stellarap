/*
stellarap/endstops.c
Copyright 2013 Mark Roy

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

#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

#include "stepper_control.h"
#include "planner.h"
#include "endstops.h"


unsigned char endstop_triggered()
{
  if (ROM_GPIOPinRead(ENDSTOP_PORT, ENDSTOP_ALLPINS ) == (ENDSTOP_ALLPINS)) // endstops are active low.
    return 0;
  else 
    return 1;
}

unsigned char axis_endstop_triggered(unsigned char axis)
{
  unsigned char ret; 
  switch (axis)
  {
    case X:
      ret = (ROM_GPIOPinRead(ENDSTOP_PORT, XLIM_PIN) == 0);
      break;

    case Y:
      ret = (ROM_GPIOPinRead(ENDSTOP_PORT, YLIM_PIN) == 0);
      break;

    case Z:
      ret = (GPIOPinRead(ENDSTOP_PORT, ZLIM_PIN) == 0);
      break;
  }

  return ret;
}

/*
   When an endstop event is triggered,  debounce and then read all
   endstop bits.  If there is a block executing, check to see if 
   the block is being driven into the endstop or off of it.  If the 
   former, then abort the current block.  
   
   If the STOP pin is pulled low, the machine will stop
   regardless of the current block.  This could be used for
   a software emergency stop button.   

   Note that when a stop is hit, the machine will not decelerate
   smoothly, which could cause damage to electronics.
 */
void endstops_isr()
{
  ROM_IntMasterDisable();  
  unsigned char endstop_bits;
  unsigned char endstops[3];
  unsigned char abort = 0;
  int i;

  //debounce ~250uS
  for (i=0;i<100000;i++); 
  
  endstop_bits = ROM_GPIOPinRead(ENDSTOP_PORT, ENDSTOP_ALLPINS);
//  printf("endstop_bits: %x\r\n", endstop_bits);
  endstops[X] = ( (endstop_bits & XLIM_PIN)  == 0)?1:0;
  endstops[Y] = ( (endstop_bits & YLIM_PIN)  == 0)?1:0;
  endstops[Z] = ( (endstop_bits & ZLIM_PIN)  == 0)?1:0;
 

  if (cur_block != NULL && cur_block->status == BLOCK_RUNNING )
  {
    for (i=0;i<3;i++)
      if (cur_block->dir[i]<0 && cur_block->steps[i] > 0 && endstops[i] ) 
        abort=1;
  
    if (endstop_bits & STOP_PIN == 0 || abort)
    {
      puts("Endstop Triggered. Aborting Current Blocks\r\n");
      cur_block->status = BLOCK_ABORTED;
    }
  }

  ROM_GPIOPinIntClear( ENDSTOP_PORT, ENDSTOP_ALLPINS);
  ROM_IntMasterEnable();
}

void endstops_init()
{
  ROM_SysCtlPeripheralEnable(ENDSTOP_PERIPH);
  ROM_GPIOPinTypeGPIOInput(ENDSTOP_PORT, ENDSTOP_ALLPINS );
  ROM_GPIOPadConfigSet(ENDSTOP_PORT, ENDSTOP_ALLPINS, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  ROM_GPIOIntTypeSet(ENDSTOP_PORT, ENDSTOP_ALLPINS, GPIO_FALLING_EDGE );
  GPIOPortIntRegister( ENDSTOP_PORT, endstops_isr );
  ROM_GPIOPinIntEnable( ENDSTOP_PORT, ENDSTOP_ALLPINS );
}

