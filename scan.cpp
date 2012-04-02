
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

#define SCAN_TIME	20		//ms

// Bit field struct keeps track of local gpio state
typedef struct _gpio_state {
	unsigned 	bIgnition : 1;
	unsigned 	bStartButton : 1;
	unsigned 	bFWD : 1;
	unsigned 	bREV : 1;
	unsigned 	bBrake : 1;
	unsigned	bIGBT : 1;
}gpio_state_t;

enum InternalSignals {                                     // internal signals
    TIMEOUT_SIG = MAX_SIG
};

// Active object class ------------------------------------------------------------------------------------------------------------
class Scan : public QActive {

private:
	QTimeEvt 		m_timeEvt;  
    gpio_state_t	gpio;

public:
    Scan();

private:
    static QState initial(Scan *me, QEvent const *e);
    static QState idle(Scan *me, QEvent const *e);
};

// Local objects ----------------------------------------------------------------------------------------------------------------
static Scan l_scan;                                    // local Table object

// Public-scope objects -------------------------------------------------------------------------------------------------------
QActive * const AO_Scan = &l_scan;                    // "opaque" AO pointer


// --------------------------------------------------------------------------------------------------------------------------------
Scan::Scan() : QActive((QStateHandler)&Scan::initial),
	m_timeEvt(TIMEOUT_SIG)
 {
	gpio.bIgnition = false;
	gpio.bStartButton = false;
	gpio.bFWD = false;
	gpio.bREV = false;
	gpio.bBrake = false;
}

// --------------------------------------------------------------------------------------------------------------------------------
QState Scan::initial(Scan *me, QEvent const *) {


    return Q_TRAN(&Scan::idle);
}

// --------------------------------------------------------------------------------------------------------------------------------
// This is the normal idle state for the scanner.
// TODO .. maybe this whole module could be replaced by an interrupt handler
QState Scan::idle(Scan *me, QEvent const *e) {

    switch (e->sig) {
        case Q_ENTRY_SIG: {
			me->m_timeEvt.postEvery(me, SCAN_TIME/BSP_MS);
            return Q_HANDLED();
        }
      
        // Scan all of the input signals for edge transitions and analog inputs for limits.
        case TIMEOUT_SIG: {
        	// Start button pushed?
        	if(O_stm32->getStart() && me->gpio.bStartButton == false ){
				me->gpio.bStartButton = true;
				static QEvent const signal = { START_BUTTON_SIG, 0 };
				AO_Inverter->POST(&signal, me);		// TODO who wants this signal? do we need to publish?
        	}
			else if (!(O_stm32->getStart()) && me->gpio.bStartButton == true ){
				me->gpio.bStartButton = false;
			}
        	// Ignition switch?
        	if(O_stm32->getIGN() && me->gpio.bIgnition == false ){
				me->gpio.bIgnition = true;
        		static QEvent const signal = { IGNITION_ON_SIG, 0 };
        		AO_Inverter->POST(&signal, me);
        	}
        	else if (!(O_stm32->getIGN()) && me->gpio.bIgnition == true ){
				me->gpio.bIgnition = false;
				static QEvent const signal = { IGNITION_OFF_SIG, 0 };
				AO_Inverter->POST(&signal, me);
			}	
			// FWD direction
			if(O_stm32->getFWD() && me->gpio.bFWD == false ){
				me->gpio.bFWD = true;
        		static QEvent const signal = { FORWARD_SIG, 0 };
        		AO_Inverter->POST(&signal, me);
			}
			else if (!(O_stm32->getFWD()) && me->gpio.bFWD == true ){
				me->gpio.bFWD = false;
				static QEvent const signal = { NEUTRAL_SIG, 0 };
				AO_Inverter->POST(&signal, me);
			}
			// REV direction
			if(O_stm32->getREV() && me->gpio.bREV == false ){
				me->gpio.bREV = true; 
        		static QEvent const signal = { REVERSE_SIG, 0 };
        		AO_Inverter->POST(&signal, me);
			}
			else if (!(O_stm32->getREV()) && me->gpio.bREV == true ){
				me->gpio.bREV = false;
				static QEvent const signal = { NEUTRAL_SIG, 0 };
				AO_Inverter->POST(&signal, me);
			}
			// IGBT fault
			if(O_stm32->getIGBTFault() && me->gpio.bIGBT == false ){
				me->gpio.bIGBT = true;
				static QEvent const signal = { IGBT_FAULT_SIG, 0 };
				AO_Inverter->POST(&signal, me);
			}
			else if (!(O_stm32->getIGBTFault()) && me->gpio.bIGBT == true ){
				me->gpio.bIGBT = false;
				static QEvent const signal = { IGBT_FAULT_CLEARED_SIG, 0 };
				AO_Inverter->POST(&signal, me);
			}
        	return Q_HANDLED();
		}
        return Q_IGNORED();
    }
    return Q_SUPER(&QHsm::top);
}
