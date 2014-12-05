/*
stellarap/stepper_control.c
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

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "stepper_control.h"
#include "endstops.h"

unsigned int blk_queue_head = 0;
unsigned int blk_queue_tail = 0;
unsigned int num_blocks = 0 ;
block_t blk_queue[BLOCK_QUEUE_SIZE] ;

//calibration set
//thingiverse.com/thing:5573

//used by stepper control 
block_t *cur_block = NULL ;
long  acceleration_time = 0.0;
long step_rate=0;
long peak_rate=0;
unsigned long step_events_completed = 0;
unsigned long cur_steps[4]= {0,0,0,0}; // steps traveled in current block
unsigned char motors_enabled = false; 

unsigned long calculate_timer(long rate)
{
  if (rate > MAX_STEP_RATE) rate = MAX_STEP_RATE;
  if (rate < MIN_STEP_RATE) rate = MIN_STEP_RATE;

  return (unsigned long) lround( MCU_CLK / rate);
}

void motor_disable() 
{
  ROM_GPIOPinWrite( MOTOR_XENPORT, MOTOR_XEN, MOTOR_XEN );
  ROM_GPIOPinWrite( MOTOR_YENPORT, MOTOR_YEN, MOTOR_YEN );
  ROM_GPIOPinWrite( MOTOR_ZENPORT, MOTOR_ZEN, MOTOR_ZEN );
  ROM_GPIOPinWrite( MOTOR_EENPORT, MOTOR_EEN, MOTOR_EEN );
  motors_enabled = false;
}

void motor_enable()
{
  ROM_GPIOPinWrite( MOTOR_XENPORT, MOTOR_XEN, 0 );
  ROM_GPIOPinWrite( MOTOR_YENPORT, MOTOR_YEN, 0 );
  ROM_GPIOPinWrite( MOTOR_ZENPORT, MOTOR_ZEN, 0 );
  ROM_GPIOPinWrite( MOTOR_EENPORT, MOTOR_EEN, 0 );
  motors_enabled = true;
}

void motor_unstep() 
{
  ROM_GPIOPinWrite(LED_PORT, LED_R | LED_G | LED_B, 0 );
  ROM_GPIOPinWrite( MOTOR_XSTEPPORT, MOTOR_XPIN, 0);
  ROM_GPIOPinWrite( MOTOR_YSTEPPORT, MOTOR_YPIN, 0);
  ROM_GPIOPinWrite( MOTOR_ZSTEPPORT, MOTOR_ZPIN, 0);
  ROM_GPIOPinWrite( MOTOR_ESTEPPORT, MOTOR_EPIN, 0);

}
void motor_init()
{
  unsigned long timer;

  //configure timer0 for one shot intervals and assign interrupt routine
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  ROM_TimerConfigure(TIMER0_BASE,TIMER_CFG_ONE_SHOT);
  ROM_TimerControlStall(TIMER0_BASE, TIMER_A, true);
  TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0A_ISR);
  ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT); 

  //setup RGB led outputs. 
  ROM_SysCtlPeripheralEnable(LED_PERIPH);
  ROM_GPIOPinTypeGPIOOutput(LED_PORT, LED_R | LED_G | LED_B );

  //enable peripherals used for motor
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  
  //set motor pins to outputs
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, MOTOR_PORTA_PINS );
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, MOTOR_PORTB_PINS );
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, MOTOR_PORTD_PINS );
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, MOTOR_PORTE_PINS );


  //make sure motors are disabled and step pins are low
  motor_disable();
  motor_unstep();

  //start the timer.  the ISR will run at the minimum rate 
  //interval until there is a block to execute.
  timer=calculate_timer(MIN_STEP_RATE);
  ROM_TimerLoadSet(TIMER0_BASE,TIMER_A, timer);
  ROM_TimerEnable(TIMER0_BASE,TIMER_A);
}


void motor_step(unsigned char axis, signed char dir)
{
  int i=0;
  switch (axis)
  {
    case X: 
      ROM_GPIOPinWrite(LED_PORT, LED_R, LED_R);
      ROM_GPIOPinWrite( MOTOR_XDIRPORT, MOTOR_XDIR, ((dir==MOTOR_XINC_DIR)?MOTOR_XDIR:0));  
      for (i=0;i<5;i++);
      ROM_GPIOPinWrite( MOTOR_XSTEPPORT, MOTOR_XPIN, MOTOR_XPIN);
      break;
    case Y:
       ROM_GPIOPinWrite(LED_PORT, LED_G, LED_G);
       ROM_GPIOPinWrite( MOTOR_YDIRPORT,  MOTOR_YDIR, ((dir==MOTOR_YINC_DIR)?MOTOR_YDIR:0));  
       for (i=0;i<5;i++);
       ROM_GPIOPinWrite( MOTOR_YSTEPPORT, MOTOR_YPIN, MOTOR_YPIN);
       break;
    case Z:
       ROM_GPIOPinWrite(LED_PORT, LED_B, LED_B);
       ROM_GPIOPinWrite( MOTOR_ZDIRPORT,  MOTOR_ZDIR, ((dir==MOTOR_ZINC_DIR)?MOTOR_ZDIR:0));  
       for (i=0;i<5;i++)       
       ROM_GPIOPinWrite( MOTOR_ZSTEPPORT, MOTOR_ZPIN, MOTOR_ZPIN);
       break;
    case E:
       ROM_GPIOPinWrite( MOTOR_EDIRPORT,  MOTOR_EDIR, ((dir==MOTOR_EINC_DIR)?MOTOR_EDIR:0));  
       for (i=0;i<5;i++);
       ROM_GPIOPinWrite( MOTOR_ESTEPPORT, MOTOR_EPIN, MOTOR_EPIN);
      break;

  }
}



void  Timer0A_ISR(void)
{
  unsigned long timer;
  int i;
  unsigned char all_steps_completed; 
  float tmp; 
  unsigned char endstop_bits;
  unsigned char endstops[4];

  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); 
  ROM_IntMasterDisable();  

  if (cur_block == NULL)
  {
    if (num_blocks > 0 && blk_queue[blk_queue_head].status == BLOCK_QUEUED && blk_queue[blk_queue_head].recalculate == 0)
    {
      cur_block = &blk_queue[blk_queue_head];
      cur_block->status = BLOCK_RUNNING;
      step_rate = cur_block->initial_rate;
      acceleration_time=0;
      if (!motors_enabled)
        motor_enable();

      if (endstop_triggered())
      {
        endstop_bits = ROM_GPIOPinRead(ENDSTOP_PORT, ENDSTOP_ALLPINS);
        endstops[X] = ( (endstop_bits & XLIM_PIN)  == 0)?1:0;
        endstops[Y] = ( (endstop_bits & YLIM_PIN)  == 0)?1:0;
        endstops[Z] = ( (endstop_bits & ZLIM_PIN)  == 0)?1:0;
 

        for (i=0;i<3;i++)
          if (cur_block->dir[i]<0 && cur_block->steps[i] > 0 && endstops[i] ) 
            cur_block->status = BLOCK_ABORTED;
      }

    } else
    {
      cur_block = NULL;
      timer=calculate_timer(MIN_STEP_RATE);
      ROM_TimerLoadSet(TIMER0_BASE,TIMER_A, timer);
      ROM_TimerEnable(TIMER0_BASE, TIMER_A);
      ROM_IntMasterEnable();
      return ; 
    } 
  }

  //if an endstop is hit, the block is aborted.
  if (cur_block->status == BLOCK_ABORTED )
  {
      cur_block->status = BLOCK_ABORTED_RDY;
      cur_block = NULL;

      //dump the buffer
      num_blocks=0;
      blk_queue_head=0;
      blk_queue_tail=0;
      step_events_completed=0;
      for (i=0;i<4;i++)
        cur_steps[i]=0;

      timer=calculate_timer(MIN_STEP_RATE);
      ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, timer); 
      ROM_TimerEnable(TIMER0_BASE, TIMER_A); 
      ROM_IntMasterEnable(); 
      return; 
  }

  if (step_events_completed <= (cur_block->accelerate_until))
  {
    step_rate = (unsigned long)((float)acceleration_time/80000000 * (float)(cur_block->acceleration_rate)) + cur_block->initial_rate;
    
    if (step_rate >(cur_block->nominal_rate))
       step_rate=cur_block->nominal_rate;

//    printf("accel %d %d", acceleration_time, cur_block->acceleration_rate);
  } else if (step_events_completed > (cur_block->decelerate_after) && step_rate > cur_block->final_rate)
  {
      step_rate = peak_rate - (unsigned long)((float)acceleration_time/80000000 * (float)(cur_block->acceleration_rate));
      if (step_rate < cur_block->final_rate)
         step_rate = cur_block->final_rate;
  //   printf("decel %d %d ",deceleration_time, timer );
  }

  timer=calculate_timer(step_rate);
  if (step_events_completed == cur_block->decelerate_after)
  {
     acceleration_time = 0;
     peak_rate = step_rate;
  }

  acceleration_time += timer;

  step_events_completed++;
  all_steps_completed = 1;
  for (i=0;i<4;i++)
  {
  //  if (i==1)
   // printf(" %d %d > 0 && %d > %d\r\n", i, cur_block->steps[i], step_events_completed*(cur_block->steps[i])/(cur_block->step_event_count), cur_steps[i]);
    if (cur_block->steps[i] > 0 && lround(step_events_completed*(cur_block->steps[i])/(cur_block->step_event_count)) > cur_steps[i])
    {
      //printf("step %d ", i);
      motor_step(i,cur_block->dir[i]);
      cur_steps[i]++;
//      printf("cursteps %d\r\n", cur_steps[i]);
    }
    if (cur_steps[i] < (cur_block->steps[i]))
      all_steps_completed=0;
  } 


// printf("step_event_completd: %d blk_event_count: %d\r\n", step_events_completed, cur_block->step_event_count); 
  if (step_events_completed < cur_block->step_event_count || !all_steps_completed )
  {
    ROM_TimerConfigure(TIMER0_BASE,TIMER_CFG_ONE_SHOT);
    ROM_TimerLoadSet(TIMER0_BASE,TIMER_A, timer);
    ROM_TimerEnable(TIMER0_BASE,TIMER_A);
  } else
  {
    //ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
     cur_block->status=BLOCK_COMPLETED;
     cur_block = NULL;
     blk_queue_head++;

     if (blk_queue_head >= BLOCK_QUEUE_SIZE)
       blk_queue_head = 0;
      num_blocks--;

    step_events_completed =0;
    for (i=0;i<4;i++)
      cur_steps[i]=0;

    //printf("%d block completed.  Rescheduling timer for next block.\r\n",num_blocks);
    ROM_TimerConfigure(TIMER0_BASE,TIMER_CFG_ONE_SHOT);
    timer=calculate_timer(cur_block->final_rate);
    ROM_TimerLoadSet(TIMER0_BASE,TIMER_A, timer); 
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
    
  } 

  /*ROM_TimerConfigure(TIMER0_BASE,TIMER_CFG_ONE_SHOT);
  timer=calculate_timer(cur_block->final_rate);
  ROM_TimerLoadSet(TIMER0_BASE,TIMER_A, timer); 
  ROM_TimerEnable(TIMER0_BASE, TIMER_A);
*/

  motor_unstep();

//  printf("step_events: %d, step_rate: %d, timer: %d\r\n",step_events_completed, step_rate,timer);
  ROM_IntMasterEnable();

}


