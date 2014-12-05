/*
stellarap/planner.c
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

#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "stepper_control.h"

#include "planner.h"

config cfg = 
  {
    .axis_steps_per_mm = { 100.0, 100.0, 2267.72, 650 }, // aixs_steps_per_mm[]
    .axis_max_speed = { 300, 300, 4, 20},  // speed limit for each axis in mm/s 
  };
unsigned long lmin( unsigned long a, unsigned long b ) 
{ 
  return (a < b)? a : b; 
}

//used by planner

float feedrate = 1000.0;
float position[4] = {0,0,0,0};  // mm
float delta_mm[4];
float current_speed[4]  = {1.0,1.0,1.0,1.0};
float previous_speed[4] = {1.0,1.0,1.0,1.0};
unsigned char relative_positioning = 0; 
unsigned char relative_extruding = 0; 

unsigned char homing = 0; // when this is set, endstops are ignored. DANGER. DO NOT SET UNLESS HOMING

unsigned long estimate_accel_distance(unsigned long target_rate, unsigned long initial_rate, unsigned long accel)
{

  target_rate = (target_rate*target_rate - initial_rate*initial_rate)/(2*accel);
  if (target_rate == 0) 
    target_rate = 1;
  return target_rate;
}

unsigned long estimate_velocity(unsigned long initial_rate, unsigned long acceleration_rate, unsigned long distance )
{
 return  (unsigned long) lround( 0.5*(initial_rate + sqrt( pow(initial_rate,2) + 8*distance*acceleration_rate)));
}

// Used to find the point at which acceleration stops and deceleration begins in a block
// that is not able to reach the nominal rate.  
//
// For example, consider the block:
// initial rate:  Vi=17 steps/sec
// final rate:    Vf=2394 steps/sec
// acceleration:  a=16000 stps/sec^2 
// block length:  240 step events
// In this example, the maximum velocity achievable is ~2589.54 steps/sec.
// This is then converted to a distance using estimate_accel_distance:
// reversal point =  190.55.  
// solution to the quadratic:  
// Vm^2 - 0.5Vi^2 - 0.5Vf^2 - d*a = 0
unsigned long  estimate_max_velocity( unsigned int initial_rate, unsigned int final_rate, unsigned int acceleration_rate, unsigned int step_events  )
{
  return  (unsigned long) lround(sqrt ( (initial_rate*initial_rate + final_rate*final_rate)/2.0 + step_events * acceleration_rate ));
  //return estimate_accel_distance( vmax, initial_rate, acceleration_rate );
}

unsigned int next_block_index(unsigned int idx)
{
  idx++; 
  if (idx >= BLOCK_QUEUE_SIZE)
  {
    idx=0;
  }
  return idx;
}

void recalculate_block_entries_forward() 
{
  block_t *cur, *next;
  int next_index;
  unsigned int accel_steps;

  cur= &blk_queue[blk_queue_head];
  next_index=blk_queue_head;
  for (int i = 0; i < num_blocks-1; i++) // go up to, but not including last block.
  {
    next_index++;
    if (next_index >= BLOCK_QUEUE_SIZE)
      next_index=0;
    next = &blk_queue[next_index];
    

    if ( cur->recalculate && cur->final_rate > cur->initial_rate) // need to accelerate in this block
    {
      accel_steps = estimate_accel_distance( cur->final_rate, cur->initial_rate, cur->acceleration_rate );
      if ( accel_steps > cur->step_event_count ) // not enough time to reach the exit rate.
      {
        cur->final_rate = sqrt( pow(cur->final_rate,2) + 2*cur->acceleration_rate*cur->step_event_count ); 
        next->initial_rate = cur->final_rate;
        next->recalculate = 1;
      }
    }

    cur=next;

  }

}
void recalculate_block_entries_reverse()
{
  block_t *cur, *prev; 
  int prev_index; 
  unsigned int  decel_steps; 

  cur = &blk_queue[blk_queue_tail];
  prev_index=blk_queue_tail;
  // walk backwards to head, but dont touch head.
  for (int i=num_blocks; i>1; i--)
  {
   prev_index--; 
   if (prev_index < 0)
     prev_index= BLOCK_QUEUE_SIZE-1;
   prev = &blk_queue[prev_index];
  if (cur->status == BLOCK_QUEUED )
   {
    cur->initial_rate = cur->max_entry_rate; 
     if (cur->nominal_rate > cur->final_rate)
     {
       decel_steps = estimate_accel_distance( cur->nominal_rate, cur->final_rate, cur->acceleration_rate ); 
       if (decel_steps > cur->step_event_count )
       {
         cur->initial_rate = lmin( cur->max_entry_rate,  estimate_velocity(cur->final_rate, cur->acceleration_rate, cur->step_event_count ));
         cur->recalculate = 1;
       } 
     }
     if (prev->final_rate != cur->initial_rate)
     {
       prev->final_rate = cur->initial_rate; 
       prev->recalculate = 1; 
     } 
    }
    cur = prev;
  }
//  if (cur->nominal_rate > prev->nominal_rate) 
}

void recalculate_block_entries()
{
  block_t *cur, *prev;
  int prev_index = blk_queue_tail; 
  unsigned int accel_steps, decel_steps; 
  unsigned long vmax; 

  recalculate_block_entries_reverse();
  recalculate_block_entries_forward();

  cur = &blk_queue[blk_queue_tail];

  for (int i =num_blocks; i>0; i--)
  {
    prev_index--;
    if (prev_index < 0)
      prev_index= BLOCK_QUEUE_SIZE-1;
    prev = &blk_queue[prev_index];

    if (cur->recalculate != 0)
    {
      accel_steps = 0;    
      if (cur->initial_rate < cur->nominal_rate)
        accel_steps = estimate_accel_distance( cur->nominal_rate, cur->initial_rate, cur->acceleration_rate );
     
      decel_steps = 0;
      if (cur->final_rate < cur->nominal_rate)
        decel_steps = estimate_accel_distance( cur->nominal_rate, cur->final_rate, cur->acceleration_rate );

      if (accel_steps + decel_steps < cur->step_event_count)
      {
        // enough time to accelerate and decelerate from nominal rate
        cur->accelerate_until = accel_steps;
        cur->decelerate_after = cur->step_event_count - decel_steps; 

      } else if (cur->initial_rate > cur->final_rate)
      {
        //block finishes slower than it started 
        decel_steps = estimate_accel_distance( cur->initial_rate, cur->final_rate, cur->acceleration_rate );
        if (decel_steps >= cur->step_event_count)
        {
          //this block is pure deceleration. 
          cur->decelerate_after = 0; 
          cur->accelerate_until = 0;
        } else
        {
          //time to speed up then slow down to final rate. 
          vmax = estimate_max_velocity( cur->final_rate, cur->initial_rate, cur->acceleration_rate, cur->step_event_count );
          accel_steps = estimate_accel_distance( (unsigned long)vmax, cur->initial_rate, cur->acceleration_rate); 
          cur->accelerate_until = accel_steps;
          cur->decelerate_after = accel_steps; 
        }

      } else 
      {
        //block finishes faster than it started
        accel_steps = estimate_accel_distance( cur->final_rate, cur->initial_rate, cur->acceleration_rate ); 
        if (accel_steps >= cur->step_event_count)
        {
          //this block is pure acceleration.
          cur->accelerate_until = cur->step_event_count;
          cur->decelerate_after = cur->step_event_count;
        } else
        {
          //time to speed up then slow down to final rate.
          vmax = estimate_max_velocity( cur->initial_rate, cur->final_rate, cur->acceleration_rate, cur->step_event_count ); 
          accel_steps = estimate_accel_distance( (unsigned long)vmax, cur->initial_rate, cur->acceleration_rate);
          cur->accelerate_until = accel_steps;
          cur->decelerate_after = accel_steps; 
        }
      }

      cur->recalculate=0;
    }

    cur=prev;
  }
}

block_t* planner_line( float *dest,  float  feedrate)
{
  unsigned long accel_steps; 
  int i, last;
  unsigned char major_axis=0;
  float current_speed[4]={0,0,0,0};

  if (num_blocks > 0)
  {
    while (num_blocks == BLOCK_QUEUE_SIZE);   
  }

  last=blk_queue_tail-1;
  if (last < 0)
    last = BLOCK_QUEUE_SIZE-1;

  block_t *new_block = &blk_queue[blk_queue_tail];
  new_block->status = BLOCK_QUEUED;
  new_block->step_event_count = 0;
  new_block->distance=0;
  for (i=0;i<4;i++)
  {
//    dest[i]=dest[i]*cfg.axis_steps_per_mm[i];
    if ( (relative_positioning == true) || (relative_extruding == true && i >= 3))
    {
      new_block->steps[i] = labs( lround( dest[i]*cfg.axis_steps_per_mm[i]) ); 
      new_block->dir[i] = (dest[i] > 0)?DIR_CW:DIR_CCW;
    } else
    {
      new_block->steps[i] = labs(  lround( (dest[i]-position[i])*cfg.axis_steps_per_mm[i]) ) ;
      new_block->dir[i] = (dest[i]>position[i])?DIR_CW:DIR_CCW;
    }

    if (new_block->steps[i]> new_block->step_event_count)
    {
      new_block->step_event_count=new_block->steps[i];
      major_axis=i;
    } 
    delta_mm[i] = ((float)new_block->steps[i] / cfg.axis_steps_per_mm[i]);
    new_block->distance+= pow(delta_mm[i],2);
  }

  if (new_block->step_event_count < 5)
    return 0;

  new_block->distance = sqrt(new_block->distance);

  unsigned long nominal_rate = (unsigned long) feedrate*delta_mm[major_axis]*cfg.axis_steps_per_mm[major_axis]/(new_block->distance*60.0);  // steps/sec
  float jerk = 0.0;
  float ejerk = 0.0;
  float jerk_factor = 1.0;
  float speed_factor = 1.0;
  float tmp;  
  for (i=0;i<3;i++) // calculate xyz jerk only
  {
     current_speed[i] = (float)((int)new_block->dir[i]) * 
                          nominal_rate * new_block->steps[i]
                          /new_block->steps[major_axis] 
                          / (cfg.axis_steps_per_mm[i]);
     
     if (fabs(current_speed[i]) > cfg.axis_max_speed[i])
     {
       tmp = fabs(cfg.axis_max_speed[i]/current_speed[i]);
       if (tmp < speed_factor) 
         speed_factor = tmp;
     }

     jerk += pow(current_speed[i]-previous_speed[i],2);
  }
  ejerk = current_speed[3]-previous_speed[3];  
  jerk = sqrt(jerk);

  new_block->initial_rate = MIN_STEP_RATE;
  new_block->final_rate = MIN_STEP_RATE;
  nominal_rate = nominal_rate * speed_factor;
  new_block->nominal_rate = nominal_rate;
  new_block->acceleration_rate = ( new_block->steps[0] == 0 && new_block->steps[1] == 0)? MOTOR_ACCEL_RATE_ZE : MOTOR_ACCEL_RATE;
  new_block->max_entry_rate = nominal_rate;
  if (num_blocks > 0) 
  {

	if (blk_queue[last].status == BLOCK_RUNNING) 
		new_block->initial_rate = fmax(MIN_STEP_RATE, (&blk_queue[last])->final_rate); 
	else 
		new_block->initial_rate = (&blk_queue[last])->nominal_rate;

    if (jerk > MAX_XYZ_JERK)
      jerk_factor = MAX_XYZ_JERK/jerk;

    if (ejerk > MAX_E_JERK)
      jerk_factor = fmin( jerk_factor, MAX_E_JERK/ejerk );
      
    new_block->max_entry_rate = nominal_rate * fmin(jerk_factor,speed_factor);

    if (blk_queue[last].status != BLOCK_RUNNING) // dont touch running blocks
    {
       (&blk_queue[last])->final_rate = new_block->nominal_rate;
       (&blk_queue[last])->recalculate = 1;
    }
  } 

  accel_steps = estimate_accel_distance(nominal_rate, new_block->initial_rate, new_block->acceleration_rate);

  if (accel_steps == 0)
  {
    new_block->initial_rate = nominal_rate;

  } else if (accel_steps > (new_block->step_event_count/2)) // not enough time to reach nominal rate.
  {
    //  new_block->nominal_rate = (unsigned long)sqrt( (new_block->step_event_count) * (new_block->acceleration_rate));
      accel_steps = new_block->step_event_count/2; 
  }

  if (new_block->initial_rate < new_block->nominal_rate)
    new_block->accelerate_until = accel_steps; 
  else
    new_block->accelerate_until = 0;

  if (new_block->final_rate < new_block->nominal_rate)
    new_block->decelerate_after = new_block->step_event_count - accel_steps;
  else
     new_block->decelerate_after = new_block->step_event_count;

  new_block->recalculate = (num_blocks > 0);
  
  for (i=0;i<4;i++)
  {
//     if (relative_positioning == true)
      position[i]+= new_block->dir[i]*new_block->steps[i]/cfg.axis_steps_per_mm[i];
 //    else 
  //    position[i]= dest[i];

     previous_speed[i]=current_speed[i];
  }

  num_blocks++;

  blk_queue_tail=next_block_index(blk_queue_tail);
  recalculate_block_entries();

  //printf("step_event_count: %d, accel_steps: %d, nominal_rate: %d ", new_block->step_event_count, accel_steps, new_block->nominal_rate);
//  printf("initial_rate: %d, final_rate: %d ",new_block->initial_rate, new_block->final_rate);
//  printf("accel until: %d, decel after: %d\r\n ",new_block->accelerate_until, new_block->decelerate_after);
    return new_block;

}

void planner_home(unsigned char axis_map)
{

  float coords[4] = {0,0,0,0};
  block_t *crash; 
  float speed=1000.0;
  unsigned char old_positioning = relative_positioning;

  relative_positioning = 1;
  
  for (int i=0; i<3; i++)
  {
    if ((axis_map & (1<<i)) != 0)
    {
      coords[i] = -300.0;
       if (i==2)
       {
         speed=100.0;
         coords[i]=-10.0;
       }
      do
      {
        crash = planner_line(coords, speed);  // this will hit the endstop.
        while (crash->status != BLOCK_ABORTED_RDY && crash->status != BLOCK_COMPLETED);  // wait for the machine to reach endstop or finish block.
      } while (crash->status == BLOCK_COMPLETED); // if block finished, send it again.
          
      if (i==2) 
      {
        coords[i] = 0.5; 
      } else
        coords[i] = 1;
      
      planner_line(coords, speed ); // back off 1mm
      coords[i] = -10;
      while (num_blocks > 0);  // wait for finish

      crash = planner_line(coords, speed*0.1 ); // hit the endstop again, slowly this time
      while (crash->status != BLOCK_ABORTED_RDY);
 
      coords[i] = 0.1; 
      planner_line(coords, speed ); // back off 0.1mm
      while (num_blocks > 0); 
      position[i]=0;
      coords[i] = 0;

    }
  }
  
  relative_positioning = old_positioning;

}

