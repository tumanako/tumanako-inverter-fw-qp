
//------------------------------------------------------------------------------
//   Tumanako_QP - Electric Vehicle and Motor control software
//   Copyright (C) 2012 Bernard Mentink <bmentink@gmail.com>
//
//   This file is part of Tumanako_QP.
//
//   Tumanako_QP is free software: you can redistribute it and/or modify
//   it under the terms of the GNU Lesser General Public License as published
//   by the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
//   Tumanako_QP is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU Lesser General Public License for more details.
//
//   You should have received a copy of the GNU Lesser General Public License
//   along with Tumanako_QP.  If not, see <http://www.gnu.org/licenses/>.
//------------------------------------------------------------------------------
#include "bsp.h"


void sysTickInit(void);


// --------------------------------------------------------------------------------------------------------------------------------
void BSP_init(void) {
	O_sine->Init();		// Initialize the sine object. Does the main clock initialization also.
	O_stm32->Init();	// Initialize the interface object.
	O_serial->Init();	//                serial object.
}

// --------------------------------------------------------------------------------------------------------------------------------
void QF::onStartup(void) {
    // set the system tick rate ..
	sysTickInit();

	// Should enable and set priorities for all interrupts in the system here. TODO
	// nvic_set_priority(u8 irqn, u8 priority) .... used IRQ number from nvic_enable_irq()
	// at the moment done in void SineController::nvic_setup(void) for pwm irq.
	nvic_enable_irq(TUMANAKO_NVIC_PWM_IRQ);
	nvic_set_priority(TUMANAKO_NVIC_PWM_IRQ, 0);
}
// --------------------------------------------------------------------------------------------------------------------------------
void QF::onCleanup(void) {
}
// --------------------------------------------------------------------------------------------------------------------------------
void QK::onIdle(void) {
    QF_INT_DISABLE();
    	// Can put the processor to sleep here, wake on int or timer
    QF_INT_ENABLE();
}

// --------------------------------------------------------------------------------------------------------------------------------
void Q_onAssert(char const Q_ROM * const Q_ROM_VAR file, int line) {
    (void)file;                                      // avoid compiler warning
    (void)line;                                      // avoid compiler warning
    QF_INT_DISABLE();            // make sure that all interrupts are disabled
    O_stm32->shutdownPower();	// Make the machine safe.
    for (;;) {
    }
}
// --------------------------------------------------------------------------------------------------------------------------------


// C Interface functions --- SysTick interrupt handler and init function ----------------------------------------------------------


extern "C" void sys_tick_handler(void) __attribute__((__interrupt__));
extern "C" void sys_tick_handler() {
	QK_ISR_ENTRY();                          // inform QK-nano about ISR entry
		QF::TICK(&l_SysTick_Handler);             // process all armed time events
	QK_ISR_EXIT();
}


// --------------------------------------------------------------------------------------------------------------------------------
void sysTickInit(void) {
	//72MHz = 72,000,000 counts per second
	systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB); 	//Processor clock, AHB clock(HCLK)
	//Interrupt every at rate set by BSP_TICKS_PER_SEC
	systick_set_reload(36000*(2000/BSP_TICKS_PER_SEC));

	systick_counter_enable();

	//PRI 2, SUB_PRI 0
	SCB_SHPR3 &= (u32)0xFF0000; //clear PRI_15 (SysTcik)
	SCB_SHPR3 |= 0x08; //PRI = 2 and SUB PRI =0

	//start counting
	systick_interrupt_enable();
}
