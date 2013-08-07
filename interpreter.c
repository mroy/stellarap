/*
stellarap/interpreter.c
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

#include "inc/hw_types.h"
#include "inc/hw_ints.h" 
#include "inc/hw_sysctl.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "delay.h"
#include "stepper_control.h"
#include "planner.h"
#include "heaters.h"
#include "interpreter.h"

unsigned int current_line = 0;
unsigned char units_inch = 0;

void interpreter_init()
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
  ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  
  ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                          (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_WLEN_8));

}

void cmd_request_resend()
{
  printf("resend: %d\r\n", current_line+1);
}

int read_line(char *buf, int length)
{
  int i;
  for (i =0; i<length; i++)
  {
    buf[i]=ROM_UARTCharGet(UART0_BASE);
    if (buf[i] == '\n' || buf[i] == '\r' || buf[i] == ';')
    {
      buf[i]=0;
      break;
    }
//    ROM_UARTCharPut(UART0_BASE,buf[i]);
  }
  return i;
}

//NOTE strtok must have been 
//called previously for this function to work
//coords must be at least 4 floats long. 
unsigned char strtok_coords(char *token, float *coords)
{
  float coord;
  char axis;
  unsigned char axis_map = 0x00;

  if (!relative_positioning)
    memcpy((void*)coords,(void*)position, sizeof(float)*3);
  if (!relative_extruding)
    coords[3]=position[3]; 

  token=strtok(NULL," "); 
  while (token  != NULL)
  {  
      if (sscanf(token, "%[xXyYzZeEfF]%f",&axis,&coord))
      {   
        switch (axis)
        {
          case 'x':
          case 'X':
            axis_map |= 1;
            coords[0]=coord;
            if (units_inch)
             coords[0]*=MM_PER_INCH; 
            break;

          case 'y':
          case 'Y':
            axis_map |= 1<<1;
            coords[1]=coord;
            if (units_inch)
              coords[1]*=MM_PER_INCH;
            break;

          case 'z':
          case 'Z':
            axis_map |= 1<<2;
            coords[2]=coord;
            if (units_inch)
              coords[2]*=MM_PER_INCH;
            break;

          case 'e':
          case 'E':
            axis_map |= 1<<3;
            coords[3]=coord;
            if (units_inch)
              coords[3]*=MM_PER_INCH;
            break;

          case 'f':
          case 'F':
            axis_map |= 1<<4;
            feedrate=coord;
            if (units_inch)
              feedrate*=MM_PER_INCH;
            break;

        }
      }

      token=strtok(NULL, " ");   
   } 
  return axis_map;
}

void read_command()
{
    char cmd_buf[64];
    char* cmd = &cmd_buf[0];
    int gcode; 
    char* token ;
    
    char cmd_type; 
    unsigned char axis_map;
    int line = -1, i, checksum, checksum_received;
    float coord[4] = {0,0,0,0}; 

      read_line(cmd, 64);
      if (sscanf(cmd, "N%d", &line))
      {
        checksum=0;
        for (i=0;i<64 && cmd[i] != '*' && cmd[i] != 0 ;i++)
          checksum ^= cmd[i];
        if ( (sscanf( &(cmd[i]), "*%d", &checksum_received) == 0) || 
           (checksum_received != checksum)) // bad checksum. re-request command to be sent.
        {
          cmd_request_resend();
          return;
        }

        cmd_buf[i]=0; // truncate string to length of command.
        for (i=0;i<64;i++)
          if (cmd[i]=='G' || cmd[i]=='M' || cmd[i] == 'g' || cmd[i] == 'm')
          {
            cmd = &cmd[i]; // move string starting point to the beginning of the command.
            break;
          }
         
      }
      

      if ( (token = strtok( cmd, " ")) != NULL)
      {
        if (sscanf(token, "%[gGMm]%d",&cmd_type, &gcode))
        {

          if (line != -1)
          {
            if (line != current_line + 1 &&
                !((cmd_type == 'm' || cmd_type == 'M') && gcode == 110)) // line received out of order. re-request correct line.
            {
                cmd_request_resend(); 
                return;
            }
            current_line++; 
          }

          switch (cmd_type)
          {
            case 'g':
            case 'G':
              
              switch(gcode)
              {
               case 0:
               case 1: // basic move
                  strtok_coords(token,coord); 
                  planner_line(coord,feedrate);

                  if (feedrate >= 7800) 
                  {
                    gcode++;
                  }
                  puts("ok\r\n"); 
                 break;

               case 4:  // dwell
                  while (num_blocks > 0 ); // wait for buffer to finish.
                  token = strtok( NULL, " ");
                  if (token != NULL && sscanf(token, "%[pP]%d", &cmd_type, &gcode))
                  {
                      delay(gcode);
                      while (delay_stat == DELAY_RUNNING); // wait for delay to finish
                  }
                  puts("ok\r\n");
                  break;

               case 21: // set units to mm
                  units_inch = 0;
                  puts("ok\r\n");
                  break;

               case 22: // set units to inch
                  units_inch = 1; 
                  puts("ok\r\n");
                  break;

               case 28: // homing routine 
                 axis_map=strtok_coords(token,coord); 
                 if (axis_map == 0x00)
                    axis_map = 7;

                 planner_home(axis_map);
                 puts("ok\r\n");
                 break;

               case 90:  // switch to absolute positioning

                 relative_positioning = false;
                 relative_extruding = false;
                 puts("ok\r\n");

                 break;

               case 91:  // switch to relative positioning
                 relative_positioning = true;
                 relative_extruding = true; 
                 puts("ok\r\n"); 
                 break; 

               case 92: // set current position
                 strtok_coords(token,coord);
                 memcpy((void*)position,(void*)coord, sizeof(float)*4);
                 puts("ok\r\n");
                 break;
              }
              break;

            case  'm':
            case 'M':
              switch (gcode) 
              {
                case 0:  // shutdown machine
                case 1:  // sleep machine
                  while (num_blocks > 0);  // wait for buffer to empty
                  motor_disable(); 
                  //in future, also disable heaters.
                  puts("ok\r\n");
                  break;
                
                case 17:
                  motor_enable(); 
                  puts("ok\r\n");
                  break;

                case 18:
                case 84:
                  while (num_blocks > 0); // wait for buffer to empty
                  motor_disable();
                  puts("ok\r\n");
                  break;

                case 82: // switch to absolute extruding 
                  relative_extruding = false;
                  puts("ok\r\n");
                  break;

                case 83: // switch to relative extruding
                    relative_extruding = true;
                    puts("ok\r\n");
                    break;
                case 104: // set extruder temperature
                case 140: // set bed temperature
                    token = strtok( NULL, " ");
                    i = (gcode==104)?1:0;
                    if (token != NULL && sscanf(token, "%[sS]%d", &cmd_type, &gcode))
                    {
                     if (gcode ==0)
                       error_i[i] = 0;
                     setpoint[i]=gcode;
                    }
                    puts("ok\r\n");                 
                    break;
                case 105: // get temperatures 
                  printf("ok T:%f B:%f\r\n", cur_temp[1], cur_temp[0]);
                  break;

                case 110: // set current line number
                  token = strtok(NULL, " ");
                  if (sscanf(token, "%d", &line))
                  {
                    current_line = line;
                    puts ("ok\r\n");
                  }
                  break;

                case 112: // emergency stop
                  if (cur_block != NULL && num_blocks > 0)
                    cur_block->status = BLOCK_ABORTED;
                  puts("ok\r\n");
                  motor_disable();
                  while (true);  
                  break;

                 case 999: // heater test.  
                    for  (i=0; i<60;i++)
                    {
                      delay(500);
                      while (delay_stat == DELAY_RUNNING); // wait for delay to finish
                      printf("ok T:%f B:%f\r\n", cur_temp[1], cur_temp[0]);
                    }

                    setpoint[1] = 15000.0;

                    while (1)
                    {
                      delay(500);
                      while (delay_stat == DELAY_RUNNING); // wait for delay to finish
                      printf("ok T:%f B:%f\r\n", cur_temp[1], cur_temp[0]);
                    }


                    break;
              }
            break;
          }
        }
      }

}

