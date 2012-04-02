
//------------------------------------------------------------------------------
//   Tumanako_QP - Electric Vehicle and Motor control software
//
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

Q_DEFINE_THIS_FILE

#define NUMBER_LEDS 4

enum InternalSignals {                 	// Some timer signals
    PCB_RED_TIMEOUT_SIG = MAX_SIG,
	PCB_GREEN_TIMEOUT_SIG,
	DASH_RED_TIMEOUT_SIG,
	DASH_GREEN_TIMEOUT_SIG,
};

// Active object class ------------------------------------------------------------------------------------------------------------
class Led : public QActive {

private:
	QTimeEvt 		m_PCB_RED_timeEvt;
	QTimeEvt 		m_PCB_GREEN_timeEvt;
	QTimeEvt 		m_DASH_RED_timeEvt;
	QTimeEvt 		m_DASH_GREEN_timeEvt;
    LedStateTag 	m_ledState[NUMBER_LEDS];
	int				m_ledFlashRate[NUMBER_LEDS];
	bool			m_ledToggle[NUMBER_LEDS];

public:
    Led();

private:
    static QState initial(Led *me, QEvent const *e);
    static QState idle(Led *me, QEvent const *e);
};

// Local objects ------------------------------------------------------------------------------------------------------------------
static Led l_led;                                    // local Table object

// Public-scope objects -----------------------------------------------------------------------------------------------------------
QActive * const AO_Led = &l_led;                    // "opaque" AO pointer


// --------------------------------------------------------------------------------------------------------------------------------
Led::Led() : QActive((QStateHandler)&Led::initial),
		m_PCB_RED_timeEvt(PCB_RED_TIMEOUT_SIG),
		m_PCB_GREEN_timeEvt(PCB_GREEN_TIMEOUT_SIG),
		m_DASH_RED_timeEvt(DASH_RED_TIMEOUT_SIG),
		m_DASH_GREEN_timeEvt(DASH_GREEN_TIMEOUT_SIG)
{
	int i;
	
	for(i=0;i<NUMBER_LEDS;++i) {
		m_ledState[i] = OFF;
		m_ledToggle[i] = false;
		m_ledFlashRate[i] = 100;	// 200ms default.
	}
}

// --------------------------------------------------------------------------------------------------------------------------------
QState Led::initial(Led *me, QEvent const *) {
	// TODO Set up any GPIO here ..
    return Q_TRAN(&Led::idle);
}

// --------------------------------------------------------------------------------------------------------------------------------
// This is the normal idle state for the led.
QState Led::idle(Led *me, QEvent const *e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {

            return Q_HANDLED();
		}
		// RED LED on PCB board 
		case PCB_RED_SIG: {
			me->m_ledState[0] = ((LedEvt const *)e)->ledState;
			switch (me->m_ledState[0]) {
				case OFF: 
					me->m_PCB_RED_timeEvt.disarm();
					O_stm32->setPCBErrorLED(false);
					break;
				case STEADY: 
					me->m_PCB_RED_timeEvt.disarm();
					O_stm32->setPCBErrorLED(true);
					break;
				case FLASH:
					int flashRate;

					flashRate =  ((LedEvt const *)e)->ledFlashRate;
					if(flashRate) me->m_ledFlashRate[0] =  flashRate;
					me->m_PCB_RED_timeEvt.postEvery(me, me->m_ledFlashRate[0]/BSP_MS);
					break;
			}
			return Q_HANDLED();
		}
		
		//GREEN LED on PCB board
		case PCB_GREEN_SIG: {
			me->m_ledState[1] = ((LedEvt const *)e)->ledState;
			switch (me->m_ledState[1]) {
				case OFF: 
					me->m_PCB_GREEN_timeEvt.disarm();
					O_stm32->setPCBRunLED(false);
					break;
				case STEADY: 
					me->m_PCB_GREEN_timeEvt.disarm();
					O_stm32->setPCBRunLED(true);
					break;
				case FLASH:
					int flashRate;

					flashRate =  ((LedEvt const *)e)->ledFlashRate;
					if(flashRate) me->m_ledFlashRate[1] =  flashRate;
					me->m_PCB_GREEN_timeEvt.postEvery(me, me->m_ledFlashRate[1]/BSP_MS);
					break;
			}
			return Q_HANDLED();
		}
		
		// RED led on Dashboard.
		case DASH_RED_SIG: {
			me->m_ledState[2] = ((LedEvt const *)e)->ledState;
			switch (me->m_ledState[2]) {
				case OFF: 
					me->m_DASH_RED_timeEvt.disarm();
					O_stm32->setErrorLED(false);	// turn LED off
					break;
				case STEADY: 
					me->m_DASH_RED_timeEvt.disarm();
					O_stm32->setErrorLED(true);	// turn LED on
					break;
				case FLASH:
					int flashRate;

					flashRate =  ((LedEvt const *)e)->ledFlashRate;
					if(flashRate) me->m_ledFlashRate[2] =  flashRate;
					me->m_DASH_RED_timeEvt.postEvery(me, me->m_ledFlashRate[2]/BSP_MS);
					break;
			}
			return Q_HANDLED();
		}
		
		// GREEN led on Dashboard.
		case DASH_GREEN_SIG: {
			me->m_ledState[3] = ((LedEvt const *)e)->ledState;
			switch (me->m_ledState[3]) {
				case OFF: 
					me->m_DASH_GREEN_timeEvt.disarm();
					O_stm32->setRunLED(false);	// turn LED off
					break;
				case STEADY: 
					me->m_DASH_GREEN_timeEvt.disarm();
					O_stm32->setRunLED(true);	// turn LED on
					break;
				case FLASH:
					int flashRate;

					flashRate =  ((LedEvt const *)e)->ledFlashRate;
					if(flashRate) me->m_ledFlashRate[3] =  flashRate;
					me->m_DASH_GREEN_timeEvt.postEvery(me, me->m_ledFlashRate[3]/BSP_MS);
					break;
			}
			return Q_HANDLED();
		}

		case BOTH_LEDS_OFF_SIG: {
			me->m_DASH_GREEN_timeEvt.disarm();
			me->m_DASH_RED_timeEvt.disarm();
			O_stm32->setRunLED(false);	// turn LED off
			O_stm32->setErrorLED(false);
			return Q_HANDLED();
		}
      
        // Handle all the toggling of the LED's
        case PCB_RED_TIMEOUT_SIG: {
			me->m_ledToggle[0] = me->m_ledToggle[0]?false:true;
			O_stm32->setPCBErrorLED(me->m_ledToggle[0]);
        	return Q_HANDLED();
		}	
        case PCB_GREEN_TIMEOUT_SIG: {
			me->m_ledToggle[1] = me->m_ledToggle[1]?false:true;
			O_stm32->setPCBRunLED(me->m_ledToggle[1]);
        	return Q_HANDLED();	
		}	
        case DASH_RED_TIMEOUT_SIG: {
			me->m_ledToggle[2] = me->m_ledToggle[2]?false:true;
			O_stm32->setErrorLED(me->m_ledToggle[2]);
        	return Q_HANDLED();       	
		}	
        case DASH_GREEN_TIMEOUT_SIG: {
			me->m_ledToggle[3] = me->m_ledToggle[3]?false:true;
        	O_stm32->setRunLED(me->m_ledToggle[3]);
        	return Q_HANDLED(); 
		}
        return Q_IGNORED();
    }
    return Q_SUPER(&QHsm::top);
}
