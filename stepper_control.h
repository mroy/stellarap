/*
stellarap/stepper_control.h
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

#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#define MCU_CLK 80000000
#define MOTOR_PERIPH SYSCTL_PERIPH_GPIOB
#define BLOCK_QUEUE_SIZE 16 

#define MAX_STEP_RATE 20000
#define MIN_STEP_RATE 100 
#define MOTOR_ACCEL_RATE_ZE 30000
#define MOTOR_ACCEL_RATE 20000 // steps / second^2 
#define MAX_XYZ_JERK 20
#define MAX_E_JERK 28
enum DIRECTION { DIR_CW = 1, DIR_CCW = -1 };

// defines the direction to spin each motor 
// in order to increase the coordinate.  
// ie, spin x motor counter clockwise to increase 
// the x axis position.  Used to invert direction bits
enum MOTOR_XDIR {
    MOTOR_XINC_DIR = DIR_CW,
    MOTOR_YINC_DIR = DIR_CCW,
    MOTOR_ZINC_DIR = DIR_CCW,
    MOTOR_EINC_DIR = DIR_CW
};


enum BLK_STATUS { BLOCK_QUEUED, BLOCK_RUNNING, BLOCK_COMPLETED, BLOCK_ABORTED, BLOCK_ABORTED_RDY };
enum AXIS { X, Y, Z, E};

#define LED_PERIPH SYSCTL_PERIPH_GPIOF
#define LED_PORT GPIO_PORTF_BASE 

enum LED_PINS {

    LED_R = GPIO_PIN_1, 
    LED_G = GPIO_PIN_3, 
    LED_B = GPIO_PIN_2  
};


enum MOTOR_PORTS {
    MOTOR_YSTEPPORT = GPIO_PORTB_BASE,
    MOTOR_YDIRPORT  = GPIO_PORTB_BASE,
    MOTOR_YENPORT   = GPIO_PORTE_BASE,

    MOTOR_XSTEPPORT = GPIO_PORTB_BASE,
    MOTOR_XDIRPORT  = GPIO_PORTB_BASE, 
    MOTOR_XENPORT   = GPIO_PORTA_BASE,

    MOTOR_ZSTEPPORT = GPIO_PORTE_BASE, 
    MOTOR_ZDIRPORT  = GPIO_PORTB_BASE,
    MOTOR_ZENPORT   = GPIO_PORTD_BASE,

    MOTOR_ESTEPPORT = GPIO_PORTE_BASE,
    MOTOR_EDIRPORT  = GPIO_PORTB_BASE,
    MOTOR_EENPORT   = GPIO_PORTA_BASE,
};

enum MOTOR_PINS { 
    MOTOR_YPIN = GPIO_PIN_3,  // port B
    MOTOR_XPIN = GPIO_PIN_7,  // port B
    MOTOR_ZPIN = GPIO_PIN_4,  // port E
    MOTOR_EPIN = GPIO_PIN_5   // port E
};

enum MOTOR_DIR_PINS { 
    MOTOR_YDIR = GPIO_PIN_2,  // port B
    MOTOR_XDIR = GPIO_PIN_6,  // port B
    MOTOR_ZDIR = GPIO_PIN_5,  // port B
    MOTOR_EDIR = GPIO_PIN_4   // port B
};

enum MOTOR_EN_PINS {
    MOTOR_YEN = GPIO_PIN_0,   // port E
    MOTOR_XEN = GPIO_PIN_3,   // port A
    MOTOR_ZEN = GPIO_PIN_6,   // port D
    MOTOR_EEN = GPIO_PIN_7    // port A
};

#define MOTOR_PORTA_PINS MOTOR_EEN | MOTOR_XEN 
#define MOTOR_PORTB_PINS MOTOR_XPIN | MOTOR_YPIN | MOTOR_ZDIR | MOTOR_XDIR | MOTOR_YDIR | MOTOR_EDIR 
#define MOTOR_PORTD_PINS MOTOR_ZEN
#define MOTOR_PORTE_PINS MOTOR_ZPIN | MOTOR_EPIN | MOTOR_YEN

typedef struct 
{
  enum BLK_STATUS status;
  long steps[4];
  signed char dir[4]; 
  unsigned char recalculate;
  float distance;

  long step_event_count; 
  long accelerate_until;  
  long decelerate_after;

  long max_entry_rate;  
  long nominal_rate;       // steps/min
  long initial_rate;       // steps/min
  long final_rate;         // steps/min
  long acceleration_rate; // steps/sec^
  
} block_t; 


extern unsigned int blk_queue_head;
extern unsigned int blk_queue_tail;
extern unsigned int num_blocks;
extern block_t blk_queue[BLOCK_QUEUE_SIZE];
extern block_t *cur_block;
extern unsigned char motors_enabled;

void motor_enable();
void motor_disable();
void motor_init();
unsigned long calculate_timer(long rate);
void motor_unstep();
void motor_step(unsigned char axis, signed char dir);
void  Timer0A_ISR(void);

#endif

