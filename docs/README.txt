Tumanako_QP
-----------

Tumanako_QP is my attempt at pulling all the Tumanako code "bits" together, it  has been combining with a small preemptive operation
system (QP) to improve real-time performance, and to make code manageable/readable and easy to maintain.

QP is an operating system based on the concept of "Active Objects" (AO), active objects are really event driven state machines
so all communication between AO's are performed with events, ..which may or may not contain data. An event queue is provided for each 
AO, and any data is passed efficiently in re-usable "pools"

QP is a run-to-completion system .. there are no "while" loops in a QP application. All code is a simple reaction to an event, does
some stuff then exits ..

All data in QP is statically defined. This is done for safety/efficiency reasons, static memory is much more 
deterministic than dynamic memory and has inherent safety as all allocations are known at "compile" time, not "run" time 
   .... who wants a memory leak causing a software crash causing a "real" crash .... anyone? 
   
QP uses ASSERT's throughout the code, these can be used in the application as well. This gives added safety because if an unexpected
software error occurs, the assert is fired and the assert handler shuts down the car ..

Have a read of main.cpp and bsp.cpp to get an idea how it all fits together.
The main code is in inverter.cpp ..

You can find out more about QP at: www.state-machine.com

QP complies against the Motor Industry Software Reliability Association (MISRA) "Guidelines for the Use of the  C Language in Vehicle Based Software", 
April 1998, ISBN 0-9524156-9-0 (see also http://www.misra.org.uk). 
A separate Application Note "QEP/C++ MISRA Compliance Matrix" contains detailed account how QEP/C++ compiles with the 127 MISRA rules 
(See http://www.state-machine.com/doc/AN_QP_Cpp_MISRA.pdf).

----------------------------------------------------------------------------------------------------------------------

Build instructions
-------------------
Tumanako_QP is built using either Eclipse or from the command line ... here are some instructions:

0. Install some arm cross compiler tools ... I use Sourcery G++: Lite Edition in particular the arm-2010q1 version as it produces 
   smaller binaries. But any compiler toolset will likely do .. as the Makefile shows, I tried a few ..
1. Install the QP source and libraries close to the root of your drive .. i.e C:\qpcpp or /home/user/qpcpp on Linux.
2. Install Tumanako_QP in a place of your choice ..
3. Install the libopencm3 directory at the same directory level as Tumanako_QP
4. Build a fresh set of QP libs by navigating to ../qpcpp/ports/arm-cortex/qk/gnu/ .. you will find Windoze DOS scripts
   or a Linux bash script that you can run to build the libs for either debug or release versions. By default debug is built, 
   if you want to build a release version just type for example "make_cortex-m3.sh rel"
5. Navigate to the libopencm3 directory and type "make" to build the library. Note: The libopencm3 lib has been modified to support
   QP (interrupt vector name changes) and the linker script and startup/vector code changed to support C++ static initializations which
   where not supported.
6. Build the main Tumanako_QP binary by navigating to the top level of the source and typing "make" " make clean", or  "CONF=rel make" etc
   See the Makefile for details.
7. For steps 5 and 6 please modify the hard code paths in the Makefiles to the  location to your arm build tools and the location 
   of any linked libs .. (the libopencm3 makefile is in libopencm3/lib/stm32/f1)
8. The resulting binaries are in the "rel" and "dbg" directories. See the Makefile on how to produce a binary type (hex,bin etc).
9. To build under eclipse, just import the eclipse project. I use the Turtilizer open hardware JTAG module and openocd.
   Openocd is started as a separate "tool" under eclipse and I have setup a gdb script so that eclipse works with the hardware debugger plugin.
   A config file for openocd for the STM32/Turlilizer is included (stm32.cfg) .. you could hack the header of that file for other JTAG adapters.
   I can email details if anyone is interested.... or maybe add a HOWTO. Building and programming the board is just a couple of mouse clicks.
   
   .. but you guys will work it out ... ;-)  

----------------------------------------------------------------------------------------------------------------------
The project as it stands should be usable to drive a car using slip control. You may need to alter defines in bsp.h and serial.hpp
as machine parameters have not been implemented just yet. A TODO list of what will happen next is below:
(eg. a key parameter to define would be NUM_IMPULSE_PER_REV for your optical encoder ..)

TODO:
1. Implement IFOC. I have included the relevant files in the IFOC directory. I will add code to the ifoc.cpp file 
   to implement an AO that will tie all the code together into the rest of the machine.
   As a stepping stone to implement the above, the A2D code will be re-written more efficiently. 
   (Required for phase current measurement )
2. Add machine controls ... This will be a serial comms protocol that will allow a console device (touch-screen, laptop, tablet, etc )
   to input machine control parameters that will get stored in on-board flash on the STM32.
3. Implement a watchdog.

BUGS:
1. Static version of Filter class is not working at present ... crashes code. ...not enabled for now ..

----------------------------------------------------------------------------------------------------------------------
NOTES:
I have made some design decisions in building this project which hopefully won't offend too many of you ... ;-)

1. Removed all "car" specific code to make this code as generic as possible ( .. I personally hate #ifdef's everywhere, as this 
   makes the code unmanageable and less readable .. and besides the forthcoming machine controls will remove the need ..)
2. For similar reasons I have made this code specific to the KiwiAC hardware. This is firmware for that board only.   
3. I have tried to strip out as much unused code/defines that I came across to reduce confusion.
   ..if and when they are used, they can be added back in ..
4. I have split the code up into manageable/logic modules. I have used an AO where it made sense to do so.
   for example, the pre-charge sequence is an AO, just fire it a message to start and it will respond with another message when
   complete. (that is what is done in inverter.cpp)
5. I have changed the way serial uart is used ... it provides a "printf" that is not tied to any clib library. 
6. I don't use a "Start" button, there is a #define you can un-comment in bsp.h if you require one ..
7. If you want to contribute to this code, I don't mind being the maintainer of this branch, so please forward patches to me.
   I would like to keep the structure of the code consistent throughout ..
   
   Please email me with comments/feedback  ... (positive or negative) ... I will ignore both equally :-) 


Enjoy! ... Bernard Mentink. (bmentink@gmail.com)

