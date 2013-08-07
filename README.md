Stellarap
http://www.stellarcore.com/
This is a firmware for a stellaris launchpad based controller for a 3D printer.

License:
The Stellarap code is licensed under the GNU GENERAL PUBLIC LICENSE, Version 3, 29 June 2007.
The license is available here http://www.gnu.org/licenses/gpl-3.0.txt and is also provided in the accompanying LICENSE file.
This License MUST be included in all distributed versions of this code.
All of this software is provided AS-IS with no implied warranty or liability under sections 15, and 16 of the GPL V3.
If your printer burns down your house, it's not my fault. 

15. Disclaimer of Warranty.
THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED BY APPLICABLE LAW. EXCEPT WHEN OTHERWISE STATED IN WRITING THE 
COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, 
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE ENTIRE 
RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU. SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF 
ALL NECESSARY SERVICING, REPAIR OR CORRECTION.

16. Limitation of Liability.
IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN WRITING WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO 
MODIFIES AND/OR CONVEYS THE PROGRAM AS PERMITTED ABOVE, BE LIABLE TO YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL, INCIDENTAL 
OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OR INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO LOSS OF DATA OR 
DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY OTHER 
PROGRAMS), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.

Pre-requisites: 

ARM Cortex-M4 toolchain such as:
	GCC Arm Embedded https://launchpad.net/gcc-arm-embedded
  CodeSourcery Lite GCC EABI for ARM http://www.mentor.com/embedded-software/sourcery-tools/sourcery-codebench/editions/lite

Stellarisware Software Package (since renamed to TivaWare): 
This software library includes functions and libraries for many of the on-board peripherals used in the chip.  
The library has also been compiled and optimized in the ROM that is available on the chip itself, meaning that 
library calls to TivaWare functions cost no program memory.  It also includes some special initialization stuff that is 
needed for the particular ARM core in the chip.
http://www.ti.com/tool/sw-tm4c

C Libraries 
I have used my own compiled version of Newlib to provide libc.a and libm.a which provides some standard C functions as 
well as math functions.  It is likely that you will be able to find a pre-compiled version of newlib or even use the version
that is included with the toolchain (above).
http://sourceware.org/newlib
http://eehusky.wordpress.com/2012/12/17/using-gcc-with-the-ti-stellaris-launchpad-newlib/

Recommended Software:
OpenOCD compiled with support for the TI Link 
This is a great software that I use to debug with the Stellaris Launchpad.  It allows you to use the built in TI Link that is on
the development board with the GDB included in the GCC toolchain.  Instructions for building/using the Stellaris Launchpad with OpenOCD are here:
http://processors.wiki.ti.com/index.php/Stellaris_Launchpad_with_OpenOCD_and_Linux

Build Instructions:
	Set your path to include your compiler toolchain.

	$make clean
	$make 

Flash/Debug Instructions:
	Start OpenOCD with the stellaris connected through the development USB connector. 
	$./openocd &
	$arm-none-eabi-gdb main.axf 
		target extended-remote :3333
		monitor reset halt
		load 
		monitor reset init 
		run 


Acknowledgements: 
Great thanks to the folks in  irc.freenode.net/#reprap for the help to understand 3D printing and for helping me to understand what the firmware *should* be doing.
Special thanks to Kliment for his help explaining the Marlin firmware, which my firmware may bare a striking resemblance to.


