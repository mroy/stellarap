/*
stellarap/planner.h
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

#ifndef PLANNER_H
#define PLANNER_H

typedef struct
{
  float axis_steps_per_mm[4];  
  float axis_max_speed[4];
} config; 


extern config cfg;
extern float feedrate;
extern float position[4];  // mm
extern unsigned char relative_positioning;
extern unsigned char relative_extruding;
extern unsigned char homing;

unsigned long estimate_accel_distance(unsigned long target_rate, unsigned long initial_rate, unsigned long accel);
unsigned long estimate_velocity(unsigned long initial_rate, unsigned long acceleration_rate, unsigned long distance );
unsigned long  estimate_max_velocity( unsigned int initial_rate, unsigned int final_rate, unsigned int acceleration_rate, unsigned int step_events  );
unsigned int next_block_index(unsigned int idx);
void recalculate_block_entries_forward();
void recalculate_block_entries_reverse();
void recalculate_block_entries();
block_t* planner_line( float *dest,  float  feedrate);
void planner_home(unsigned char axis_map);

#endif

