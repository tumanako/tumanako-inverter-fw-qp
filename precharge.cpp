
//------------------------------------------------------------------------------
//   Tumanako_QP - Electric Vehicle and Motor control software
//
//   Copyright (C) 2011 Philip Court <philip@greenstage.co.nz>
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


//--------PreCharge Active Object -----------------------------------------------------------------------------
class PreCharge : public QActive {                    // extend the QActive class

private:
    QTimeEvt 		m_timeEvt;                        // time event
    unsigned short 	m_previousBusVoltage;
    unsigned short 	m_busVoltage;
    int 			m_time_accum;

public:
    PreCharge(void);

private: // HSM
    static QState initial         (PreCharge *me, QEvent const *e);
    static QState start           (PreCharge *me, QEvent const *e);
    static QState wait_voltage    (PreCharge *me, QEvent const *e);
    static QState check_voltage   (PreCharge *me, QEvent const *e);
    static QState finished   	  (PreCharge *me, QEvent const *e);
    static QState error           (PreCharge *me, QEvent const *e);

private: // Helper functions

};
//--------------------------------------------------------------------------------------------------------------


enum InternalSignals {                                     // internal signals
    TIMEOUT_SIG = MAX_SIG
};

static PreCharge l_PreCharge;       // the sole instance of the PreCharge active object

//---------Global Objects --------------------------------------------------------------------------------------
QActive * const AO_PreCharge = &l_PreCharge;             // opaque pointer to PreCharge

//---------Local Objects ---------------------------------------------------------------------------------------


//---------Constructor------------------------------------------------------------------------------------------
PreCharge::PreCharge()
    : QActive((QStateHandler)&PreCharge::initial),
      m_timeEvt(TIMEOUT_SIG),
      m_previousBusVoltage(0),
      m_time_accum(0)
{}

//--------------------------------------------------------------------------------------------------------------
QState PreCharge::initial(PreCharge *me, QEvent const *) {

    return Q_TRAN(&PreCharge::start);
}

//--------------------------------------------------------------------------------------------------------------
QState PreCharge::start(PreCharge *me, QEvent const *e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {

			return Q_HANDLED();
        }

        case PRECHARGE_START_SIG: {			// We have been asked to start a pre-charge cycle.
        	if (!((O_stm32->getK1() == false) && (O_stm32->getK2() == false) && (O_stm32->getK3() == false))) {
				O_serial->printf("\nCONTACTOR_FEEDBACK_ERROR - ', At precharge start, not all contactors are off! (not really a precharge error though)'");
				return Q_TRAN(&PreCharge::error);
			}
			//start precharge
			O_stm32->setK2(false);  			//to be sure K2 is off
			O_stm32->setK1(true);
			O_stm32->setK3(true);
			me->m_timeEvt.postIn(me,  TUMANAKO_PRECHARGE_FEEDBACK_TIME/BSP_MS);  // Time to allow contactor to close.
			return Q_HANDLED();
        }

        case TIMEOUT_SIG: {
        	 if (!((O_stm32->getK2() == false) && (O_stm32->getK3() == true))) {
        	    O_serial->printf("\nCONTACTOR_FEEDBACK_ERROR - ', Initial contactor setup failed.'");
        	    return Q_TRAN(&PreCharge::error);
        	  }
        	return Q_TRAN(&PreCharge::wait_voltage);
        }
    }
    return Q_SUPER(&QHsm::top);
}

//--------------------------------------------------------------------------------------------------------------
QState PreCharge::wait_voltage(PreCharge *me, QEvent const *e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
			me->m_timeEvt.postEvery(me,  TUMANAKO_PRECHARGE_ITERATION_WAIT/BSP_MS);		// Check voltage at this rate
			return Q_HANDLED();
        }

        case TIMEOUT_SIG: {
        	 me->m_time_accum += TUMANAKO_PRECHARGE_ITERATION_WAIT;
			 me->m_busVoltage = O_stm32->getRawScaledBusVolt();
			 O_serial->printf("\r\n PreCharge Phase 1   - BusVolts (V): %d  (digital): %d",me->m_busVoltage,O_stm32->getRawBusVolt());
			 if ((me->m_busVoltage > me->m_previousBusVoltage) || (me->m_busVoltage > TUMANAKO_PRECHARGE_V)) {
			   //Voltage is still raising or we have reached target voltage
			   me->m_previousBusVoltage = me->m_busVoltage;
			 } else { //Error: Voltage is not changing and we have not reached target voltage!
			   O_serial->printf("\n\n\r  ERROR - 'Voltage is not changing and we have not reached target voltage!'");
			   return Q_TRAN(&PreCharge::error);
			 }
			 if(me->m_time_accum>=TUMANAKO_MIN_PRECHARGE_TIME) {
				 return Q_TRAN(&PreCharge::check_voltage);
			 }
        	 return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
        	me->m_timeEvt.disarm();	// Stop timer
        	return Q_HANDLED();
        }
    }
    return Q_SUPER(&QHsm::top);
}

//--------------------------------------------------------------------------------------------------------------
QState PreCharge::check_voltage(PreCharge *me, QEvent const *e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
			me->m_timeEvt.postEvery(me,  TUMANAKO_PRECHARGE_ITERATION_WAIT/BSP_MS);		// Check voltage at this rate
			return Q_HANDLED();
        }
        case TIMEOUT_SIG: {
        	me->m_time_accum += TUMANAKO_PRECHARGE_ITERATION_WAIT;
        	me->m_busVoltage = O_stm32->getRawScaledBusVolt();

        	if(me->m_busVoltage >= TUMANAKO_PRECHARGE_V) {
        		return Q_TRAN(&PreCharge::finished);		// We have reached target voltage.
        	}
        	O_serial->printf("\r\n PreCharge Phase 2   - BusVolts (V): %d  (digital): %d",me->m_busVoltage,O_stm32->getRawBusVolt());
        	if (me->m_time_accum >= TUMANAKO_MAX_PRECHARGE_TIME) {  //MAX_PRECHARGE_TIME
												//Error: MAX_PRECHARGE_TIME has elapsed and precharge not finished!
			  O_serial->printf("\n\n\r  ERROR - 'MAX_PRECHARGE_TIME has elapsed and precharge not finished!'");
			  return Q_TRAN(&PreCharge::error);		// Pre-charge failed.
			}
        	return Q_HANDLED();
        }
        case Q_EXIT_SIG: {
			me->m_timeEvt.disarm();	// Stop timer
			return Q_HANDLED();
		}
    }
	return Q_SUPER(&QHsm::top);
}

//--------------------------------------------------------------------------------------------------------------
QState PreCharge::finished(PreCharge *me, QEvent const *e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
        	//precharge finished, full operating current begins
        	O_stm32->setK2(true);   //main circuit on
        	O_stm32->setK1(false);  //precharge circuit off
        	me->m_timeEvt.postIn(me,  TUMANAKO_PRECHARGE_FEEDBACK_TIME/BSP_MS);  // Time to allow contactor to close.
			return Q_HANDLED();
        }
        case TIMEOUT_SIG: {
        	 O_serial->printf("\n\n\rK1= %d K2= %d K3= %d\n\r",O_stm32->getK1(),O_stm32->getK2(),O_stm32->getK3());
        	  //Test contactor feedback
        	  if ( ! O_stm32->getContactorsInRunStateConfiguration() ) {
        	    O_serial->printf("\nCONTACTOR_FEEDBACK_ERROR - ',Precharge complete, but final contactor change failed.'");
        	    return Q_TRAN(&PreCharge::error);
        	  }
        	  O_serial->printf("\r\nPrecharge COMPLETE");
        	  // Post message that pre-charge completed ok.
        	  PreChargeCompleteEvt *pe = Q_NEW(PreChargeCompleteEvt, PRECHARGE_COMPLETE_SIG);
        	  QF::PUBLISH(pe, me);
        	  //return Q_TRAN(&PreCharge::start);		// go back to start
        	  return Q_HANDLED();	// Don't allow another start TODO is this correct?
        }
    }
	return Q_SUPER(&QHsm::top);
}

//--------------------------------------------------------------------------------------------------------------
QState PreCharge::error(PreCharge *me, QEvent const *e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
			O_stm32->shutdownPower(); 	//contactors off, timers off, everything off!
        	//precharge error, flash RED led.
        	LedEvt *pe = Q_NEW(LedEvt, DASH_RED_SIG);
			pe->ledState = FLASH;
			AO_Led->POST(pe, me);
			// Turn off GREEN LED
			pe = Q_NEW(LedEvt, DASH_GREEN_SIG);
			pe->ledState = OFF;
			AO_Led->POST(pe, me);
			return Q_HANDLED();
        }
    }
	return Q_SUPER(&QHsm::top);
}
