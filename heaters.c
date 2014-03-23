/*
stellarap/heaters.c
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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "heaters.h"


//table [0] is for heated build platform, table [1] is for hotend.
const unsigned long THERMO_TABLES[2][61] = { 
 
//table for US SENSOR 104JL1A NTC100k THERMISTOR
//values are rounded on 5C increments from 0C to 300C.
  {4038, 4022, 4001, 3977, 3947, 3912, 3870, 3821, 3764, 3698, 3623, 3539, 3445, 3342, 3230, 3109, 2980, 2846, 2707, 2564, 2420, 2276, 2133, 1993, 1856, 1725, 1599, 1480, 1366, 1260, 1162, 1069, 983, 903, 830, 763, 701, 644, 593, 545, 502, 463, 427, 394, 364, 336, 311, 289, 268, 249, 231, 215, 200, 187, 174, 163, 152, 142, 134, 125, 118 }, 

//table for Semitec 104GT-2 
// Made with MODIFIED createTemperatureLookup.py (http://svn.reprap.org/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py)
// ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=4700 --beta=4267 --max-adc=4096
// r0: 100000
// t0: 25
// r1: 0
// r2: 4700
// beta: 4267
// max adc: 4096
{ 4045, 4028, 4008, 3982, 3951, 3912, 3866, 3811, 3747, 3672, 3586, 3489, 3381, 3263, 3134, 2996, 2851, 2700, 2546, 2389, 2233, 2079, 1928, 1782, 1643, 1511, 1387, 1270, 1162, 1062, 970, 886, 808, 738, 674, 615, 562, 514, 471, 431, 396, 363, 334, 307, 283, 261, 241, 223, 206, 191, 177, 164, 153, 142, 133, 124, 115, 108, 101, 95, },


//table for EPCOS NTC100k B57540G0104F000 (table goes to 250C)
// Made with MODIFIED createTemperatureLookup.py (http://svn.reprap.org/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py)
// ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=4700 --beta=4036 --max-adc=4096
// r0: 100000
// t0: 25
// r1: 0
// r2: 4700
// beta: 4036
// max adc: 4096
//  {4041, 4025, 4004, 3979, 3949, 3912, 3869, 3818, 3758, 3690, 3612, 3525, 3428, 3322, 3207, 3083, 2953, 2817, 2677, 2534, 2390, 2246, 2104, 1965, 1830, 1701, 1577, 1460, 1350, 1246, 1150, 1060, 977, 900, 829, 764, 704, 649, 599, 553, 511, 472, 437, 404, 375, 348, 323, 300, 280, 260, 243, 227, 212, 198, 185, 174, 163, 153, 144, 135 },

  //table for McMaster Part No. 8209k11
//values are rounded on 5C increments from 0C to 300C
//  {4045, 4028, 4008, 3982, 3951, 3912, 3866, 3811, 3746, 3671, 3586, 3489, 3380, 3261, 3133, 2995, 2849, 2698, 2543, 2387, 2230, 2076, 1925, 1779, 1640, 1508, 1384, 1267, 1159, 1059, 967, 883, 806, 736, 672, 613, 561, 513, 469, 430, 394, 362, 332, 306, 282, 260, 240, 222, 205, 190, 176, 164, 152, 142, 132, 123, 115, 107, 101, 94},

};

float error_i[2]= { 0, 0}; 
float error_d[2] = {0, 0 };
float setpoint[2] = { 0.0, 0.0 };
unsigned long ctrl[2]; 

float cur_temp[2] = { 0, 0};

float p_gain[2] = { 2000.0, 1500.0 };
float i_gain[2] = { 10.0, 40.0 };

float adc_to_temp(long value, int table_index)
{
  float ret=0.0;
  int i;
  if (value < THERMO_TABLES[table_index][0])
  {
    for (i=0;THERMO_TABLES[table_index][i] > value && i < 61;i++);
    if (i == 62)
      ret=300.0;
    else
      ret = (i + 1.0 - (float)(value - THERMO_TABLES[table_index][i])/(THERMO_TABLES[table_index][i-1] - THERMO_TABLES[table_index][i]))*5.0; 
  } 
  return ret;
}

//control loop runs at 10Hz 
void heaters_isr()
{
  unsigned long adc_buf[2];
  float error[2] = {0 , 0};
  int tmp ;
  ADCIntClear(ADC0_BASE,0 );
  tmp = ADCSequenceDataGet(ADC0_BASE, 0, adc_buf);
  if (tmp == 2)
  {
    
    for (int i=0; i<2; i++)
    {
      cur_temp[i] = adc_to_temp(adc_buf[i], i);
      if (setpoint[i] > 0.0)
      {
        error[i] = setpoint[i] - cur_temp[i];
        if ( i==0 || (i == 1 && labs(error[i]) < 50.0)) 
            error_i[i] += error[i]*0.1; 
     
        ctrl[i]= (unsigned long)(p_gain[i]*error[i] + i_gain[i]*error_i[i]);
        if (ctrl[i] > 0xFFF0)
         ctrl[i]=0xFFF0;
      } else 
      {
        ctrl[i] = 0;
      }

//      if (i==0)
 //     {
//        printf("%f %d\r\n",cur_temp[i],ctrl[i]);
//                printf("%d %f %f %f %d\r\n",i, cur_temp[i], error[i], error_i[i], (unsigned long)ctrl[i]); 
        ROM_TimerMatchSet(TIMER2_BASE, (i==1)?TIMER_A:TIMER_B, (unsigned long)(ctrl[i]));
  //    }

    }


  }

}

void heaters_init()
{
  //setup GPIO_B0 and GPIO_B1 as PWM outputs
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  ROM_GPIOPinConfigure(GPIO_PB0_T2CCP0);
  ROM_GPIOPinConfigure(GPIO_PB1_T2CCP1); 
  ROM_GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  //timer3 trigger for ADC sequence
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
  ROM_TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
  ROM_TimerLoadSet(TIMER3_BASE, TIMER_A, 100*80000); // 100ms
  ROM_TimerControlTrigger(TIMER3_BASE, TIMER_A, true);
  
  //setup pins E2, E3 as ADC inputs
  //1MSPS speed
  //64 times oversampling
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2 | GPIO_PIN_3 );
  //SysCtlADCSpeedSet(SYSCTL_ADCSPEED_1MSPS);
  ADCHardwareOversampleConfigure(ADC0_BASE, 64);

  //setup ADC sequence to read CH0 and CH1 on the timer3 trigger
  ADCSequenceDisable(ADC0_BASE, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1 | ADC_CTL_END | ADC_CTL_IE );
  ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0 );
  ADCSequenceConfigure(ADC0_BASE, 0 , ADC_TRIGGER_TIMER, 0);
  ADCSequenceEnable(ADC0_BASE, 0);

  //Assign ISR for sequence 0 and enable the interrupt and 
  //start the timer3 trigger.
  ADCIntRegister(ADC0_BASE, 0, heaters_isr);
  ADCIntEnable(ADC0_BASE, 0);
  ROM_TimerEnable(TIMER3_BASE, TIMER_A); 

  //setup timer2 as a split timer for generating PWM outputs
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
  ROM_TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM );
  ROM_TimerControlLevel(TIMER2_BASE, TIMER_BOTH, true);
//  ROM_TimerPrescaleSet(TIMER2_BASE, TIMER_BOTH, 0);
  ROM_TimerLoadSet(TIMER2_BASE, TIMER_BOTH, 0xFFFF ); // 1.22khz
  ROM_TimerMatchSet(TIMER2_BASE, TIMER_A, 0);
  ROM_TimerMatchSet(TIMER2_BASE, TIMER_B, 0);
  ROM_TimerEnable(TIMER2_BASE, TIMER_BOTH);

}
