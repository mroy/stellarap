/*
stellarap/endstops.h
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

#ifndef ENDSTOPS_H
#define ENDSTOPS_H

#define ENDSTOP_PERIPH SYSCTL_PERIPH_GPIOC
#define ENDSTOP_PORT GPIO_PORTC_BASE 
enum ENDSTOP_PINS { XLIM_PIN = GPIO_PIN_5, YLIM_PIN = GPIO_PIN_4, ZLIM_PIN = GPIO_PIN_6, STOP_PIN = GPIO_PIN_7 };
#define ENDSTOP_ALLPINS XLIM_PIN | YLIM_PIN | ZLIM_PIN | STOP_PIN 

unsigned char endstop_triggered();
unsigned char axis_endstop_triggered(unsigned char axis);
void endstops_isr();
void endstops_init();


#endif
