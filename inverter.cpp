
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

//--------Inverter Active Object -----------------------------------------------------------------------------
class Inverter : public QActive {           	// extend the QActive class

private:
    QTimeEvt 		m_timeEvt;                  // time event
    QTimeEvt 		m_throttletimeEvt;          // time event
    int 			m_time_accum;
	Direction_T 	m_direction;
	signed short 	m_MotorRPM;
	unsigned short 	m_Flux;  					// Rotor flux set point
	short 			m_AcceleratorRef;  			// actually this represents +ve and -ve torque
	unsigned short 	m_RawAcceleratorRef;  		// from ADC
	short 			m_PrevAccRef; 				// used to detect violent direction change
	short 			m_CountMinThrottleError;
	short 			m_CountMaxThrottleError;
	unsigned short 	m_MotorStallError;
	unsigned short 	m_MotorDirectionError;
	short			m_MotorTemperature;
	short			m_IGBTFaultCount;

public:
    Inverter(void);

private: // HSM
    static QState initial     	(Inverter *me, QEvent const *e);
    static QState start        	(Inverter *me, QEvent const *e);
    static QState active       	(Inverter *me, QEvent const *e);	// Base state for HSM
    static QState idle        	(Inverter *me, QEvent const *e);
    static QState ready        	(Inverter *me, QEvent const *e);
    static QState run        	(Inverter *me, QEvent const *e);
    static QState error        	(Inverter *me, QEvent const *e);

private: // Helper functions
    bool 		calcThrottle(void);
    Direction_T getDirection(void);

};
//--------------------------------------------------------------------------------------------------------------


enum InternalSignals {                                     // internal signals
    TIMEOUT_SIG = MAX_SIG,
    THROTTLE_TIMEOUT_SIG
};

static Inverter l_Inverter;       // the sole instance of the Inverter active object

//---------Global Objects --------------------------------------------------------------------------------------
QActive * const AO_Inverter = &l_Inverter;             // opaque pointer to Inverter

//---------Local Objects ---------------------------------------------------------------------------------------


//---------Constructor------------------------------------------------------------------------------------------
Inverter::Inverter()
    : QActive((QStateHandler)&Inverter::initial),
      m_timeEvt(TIMEOUT_SIG),
      m_throttletimeEvt(THROTTLE_TIMEOUT_SIG),
      m_time_accum(0),
      m_direction(NET),
	  m_MotorRPM(0),
	  m_Flux(DEFAULT_FLUX),
	  m_AcceleratorRef(0),
	  m_PrevAccRef(0),
	  m_CountMinThrottleError(0),
	  m_CountMaxThrottleError(0),
	  m_MotorStallError(0),
	  m_MotorDirectionError(0),
	  m_MotorTemperature(0)
{}

//--------------------------------------------------------------------------------------------------------------
QState Inverter::initial(Inverter *me, QEvent const *) {

	me->subscribe(PRECHARGE_COMPLETE_SIG);	// We are interested in these two published signals.
	me->subscribe(MOTOR_TEMP_SIG);
    return Q_TRAN(&Inverter::start);
}

//--------------------------------------------------------------------------------------------------------------
QState Inverter::start(Inverter *me, QEvent const *e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
        	me->m_timeEvt.postIn(me,  200/BSP_MS); // Allow 200ms for Skiip to initialize.
			return Q_HANDLED();
        }

        case TIMEOUT_SIG: {
        	return Q_TRAN(&Inverter::idle);
        }
    }
    return Q_SUPER(&QHsm::top);
}

//--------------------------------------------------------------------------------------------------------------
// This "SUPER" state catches all the common signals for the following 3 states.
QState Inverter::active(Inverter *me, QEvent const *e) {
    switch (e->sig) {

        case Q_ENTRY_SIG: {
			return Q_HANDLED();
        }
        case IGNITION_OFF_SIG: {
        	return Q_TRAN(&Inverter::idle);	// Ignition off, back to idle state.
        }
        case THROTTLE_TIMEOUT_SIG: {
        	if(me->calcThrottle() == false) {		// Calculate throttle and flux settings, this is common to the three
        		return Q_TRAN(&Inverter::error);	// .. sub states below.
        	}

        	//Test motor stator temp (max 125 deg C)
        	if (me->m_MotorTemperature > TUMANAKO_MAX_MOTOR_TEMP) {
			  O_serial->printf("\n\n\r  ERROR - 'MAX MOTOR TEMPERATURE'");
			  return Q_TRAN(&Inverter::error);
        	}

        	// TODO Need to also  test for IGBT over temperature

        	return Q_HANDLED();
        }
        case MOTOR_TEMP_SIG: {
        	me->m_MotorTemperature = ((motorTempEvt const *)e)->motorTemp;
        	return Q_HANDLED();
        }
    }
    return Q_SUPER(&QHsm::top);
}

//--------------------------------------------------------------------------------------------------------------
QState Inverter::idle(Inverter *me, QEvent const *e) {
    switch (e->sig) {
		case Q_INIT_SIG: {
			me->m_throttletimeEvt.disarm();
        	me->m_throttletimeEvt.postEvery(me,  20/BSP_MS); // Execute stuff in background every 20ms
        	return Q_HANDLED();						 // .. taken care of by the "active" super state.
		}
        case Q_ENTRY_SIG: {
        	O_stm32->shutdownPower();	// Make sure we are off in this state.
        	// Indicate both LED's off
        	LedEvt *pe = Q_NEW(LedEvt, BOTH_LEDS_OFF_SIG);
        	AO_Led->POST(pe, me);
			return Q_HANDLED();
        }
        case IGNITION_ON_SIG: {
        	// Turn on green LED
        	LedEvt *pe = Q_NEW(LedEvt, DASH_GREEN_SIG);
			pe->ledState = FLASH;
			AO_Led->POST(pe, me);
        	// Start a precharge sequence
        	static QEvent const pre_charge = { PRECHARGE_START_SIG, 0};
        	AO_PreCharge->POST(&pre_charge, me);
        	return Q_HANDLED();
        }
        case PRECHARGE_COMPLETE_SIG: {
        	O_stm32->motorInit();
#ifdef USE_START_BUTTON
        	//indicate Contactors engaged (just Run LED flashing)
        	LedEvt *pe = Q_NEW(LedEvt, DASH_GREEN_SIG);
			pe->ledState = STEADY;
			AO_Led->POST(pe, me);
			return Q_TRAN(&Inverter::ready);
#else
        	return Q_TRAN(&Inverter::run);
#endif
        }

    }
    return Q_SUPER(&Inverter::active);
}

//--------------------------------------------------------------------------------------------------------------
QState Inverter::ready(Inverter *me, QEvent const *e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {

			return Q_HANDLED();
        }
        case START_BUTTON_SIG: {
        	if (!O_stm32->getNET()) {			// Check for neutral.
				O_serial->printf("\n\n\r  ERROR - 'Must be in netural to start'");
				return Q_TRAN(&Inverter::error);
        	}
        	if (me->m_AcceleratorRef > ZERO_THROTTLE_TORQUE) { //throttle must not be engaged when start button pushed
				O_serial->printf("\n\n\r  ERROR - 'throttle must be zero when start button pushed'.");
				return Q_TRAN(&Inverter::error);
			}
        	return Q_TRAN(&Inverter::run);		// All ok, go to run state.
        }
    }
	return Q_SUPER(&Inverter::active);
}

//--------------------------------------------------------------------------------------------------------------
QState Inverter::run(Inverter *me, QEvent const *e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
        	// Indicate motor ready to go ... GREEN LED steady ...
        	LedEvt *pe = Q_NEW(LedEvt, DASH_GREEN_SIG);
			pe->ledState = STEADY;
			AO_Led->POST(pe, me);
        	me->m_timeEvt.postEvery(me,  100/BSP_MS);	// Check for various motor conditions every 100ms
			return Q_HANDLED();
        }
        case REVERSE_SIG:
        case FORWARD_SIG: {					// Start the motor now .. push throttle and drive buddy!
        	if(me->m_MotorRPM < 50) {		// Make sure rpm is < 50 to change direction.
        		O_stm32->motorStart();
        	} else {
        		O_stm32->motorStop();
        	}
        	return Q_HANDLED();
        }
        case NEUTRAL_SIG: {
        	O_stm32->motorStop();
        	return Q_HANDLED();
        }
        case TIMEOUT_SIG: {
        	// Check for motor stall condition during run.
        	if ((O_stm32->getTorque() > 400) && (me->m_MotorRPM == 0)) {
			  me->m_MotorStallError++;
			  if (me->m_MotorStallError >= 6000) {
				O_serial->printf("\n\n\r  ERROR - 'MOTOR_STALL! Check vehicle.'");
				return Q_TRAN(&Inverter::error);
			  }
			} else {
			  me->m_MotorStallError=0;
			}
        	// Check for bus voltage error
        	if (!O_stm32->busVoltageOK()) {
				//Go to error state and Log error to serial
				O_serial->printf("\n\n\r  ERROR - 'TUMANAKO_MAX_REGEN_POWER exceeded!'");  //print stuff to RS232
				return Q_TRAN(&Inverter::error);
			}
        	//Check contactor feedback
			if ( !O_stm32->getContactorsInRunStateConfiguration() ) {
				//Go to error state and Log error to serial
				O_serial->printf("\n\n\rK1= %d K2= %d K3= %d\n\r",O_stm32->getK1(),O_stm32->getK2(),O_stm32->getK3());
				O_serial->printf("\nCONTACTOR_FEEDBACK_ERROR - ',Unexpected Contactor change reported during RunState_RUN.'");
				return Q_TRAN(&Inverter::error);
			}
        	return Q_HANDLED();
        }
        case IGBT_FAULT_SIG: {
        	return Q_TRAN(&Inverter::error);
        }
        case Q_EXIT_SIG: {
        	me->m_timeEvt.disarm();	// Stop timer on exit.
       		return Q_HANDLED();
        }
    }
	return Q_SUPER(&Inverter::active);
}

//--------------------------------------------------------------------------------------------------------------
QState Inverter::error(Inverter *me, QEvent const *e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
        	LedEvt *pe;

        	me->m_AcceleratorRef = 0;
        	O_stm32->shutdownPower(); 	// Shut the motor down. Stay in this state
        	// Indicate RED error LED
        	pe = Q_NEW(LedEvt, DASH_RED_SIG);
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

//------------Private Helper functions--------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------------------
bool Inverter::calcThrottle(void) {
	//Read throttle POT position and calc torque ref
	m_RawAcceleratorRef = O_stm32->readADC(ACCEL_AI);
	m_AcceleratorRef = (short)((TUMANAKO_A * (float)m_RawAcceleratorRef) + (float)TUMANAKO_B);

	// Fetch current RPM
	m_MotorRPM = O_stm32->getRPM();

	//Check throttle limits (indicates wiring fault)s
	if (m_RawAcceleratorRef > THROTTLE_POT_MAX) {
	  m_CountMaxThrottleError++;
	  if (m_CountMaxThrottleError > THROTTLE_ERROR_COUNT) {
		O_serial->printf("\n\n\r  ERROR - 'POT_MAX exceeded! Check Throttle wires.'");
		return(false);	// Error!!
	  }
	} else m_CountMaxThrottleError = 0; //reset error count
	if (m_RawAcceleratorRef < THROTTLE_POT_MIN) {
	  m_CountMinThrottleError++;
	  if (m_CountMinThrottleError > THROTTLE_ERROR_COUNT) {
		O_serial->printf("\n\n\r  ERROR - 'POT_MIN exceeded! Check Throttle wires.'");
		return(false);	// Error!!
	  }
	} else  { m_CountMinThrottleError=0; } 	//reset error count

	//Regenerative braking logic
	if ((m_AcceleratorRef < 0) && (m_MotorRPM < GLIDE_RPM)) {
		m_AcceleratorRef = ZERO_THROTTLE_TORQUE;
	}

	//provide dead spot in pot curve
	if ((m_AcceleratorRef > -5) && (m_AcceleratorRef < 5)) {
	  m_AcceleratorRef = 0;
	}

	//TODO add speed limit to this logic
	m_direction = getDirection();
	if (m_direction == REV)
	  m_AcceleratorRef = -m_AcceleratorRef;

	 //TODO simple traction control logic

#if 0
	//Field Weakening (Flux control)
	if (m_MotorRPM < END_REGEN_RAMP_RPM) {
	  m_Flux = FLUX_MIN;
	} else if (m_MotorRPM < FIELD_STREGTHEN_RPM_END) {
	  //New code to smooth flux bump
	  m_Flux = (unsigned short)((TUMANAKO_FLUX_A * m_MotorRPM) + TUMANAKO_FLUX_B);
	  if ((m_Flux < FLUX_MIN) || (m_Flux > FLUX_MAX)) m_Flux = FLUX_MIN + 73;  //This (FLUX_MIN+73) indicates an error in the calc above
	} else if (m_MotorRPM > FIELD_WEAKENING_RPM_START) {
	  m_Flux = FLUX_MAX - TK_ABS(m_MotorRPM);
	} else if (m_MotorRPM > FIELD_WEAKENING_RPM_END) {
	  m_Flux = FLUX_MIN;
	} else {
	  m_Flux = FLUX_MAX;  //default flux
	}
	O_stm32->setFlux(m_Flux);


	//Wrong direction detect (current IFOC motor control can flip into reverse direction after a stall, hence the need for this code)
	if (((O_stm32->getTorque() > 0) && (m_MotorRPM < 0)) ||
		((O_stm32->getTorque() < 0) && (m_MotorRPM > 0)) ) {
	  m_MotorDirectionError++;
	  if (m_MotorDirectionError >= 10) {
		//if (mState != RunState_ERROR) O_serial->printf("\n\n\r  ERROR - 'MOTOR_DIRECTION_REVERSED! Check vehicle.'");
		//mState = RunState_ERROR;

		//Zero accelerator and flux to regain sanity at motor control level (TODO understand why it behaves this way)
		m_AcceleratorRef = 0;
		m_Flux = 0;
	  }
	} else {
	  if (m_MotorDirectionError > 0) { //direction is OK, hence reduce accumulated error if it is non zero
		m_MotorDirectionError--;
	  }
	  else {
		m_Flux = DEFAULT_FLUX;
	  }
	}
#endif

	 //reduce motor power if temperature is high (115 deg C)
	if (m_MotorTemperature > TUMANAKO_HALF_POWER_MOTOR_TEMP) m_AcceleratorRef = m_AcceleratorRef/2;

	// Save the current torque setting, this is our accelerator interface to the slip/IFOC module etc ...
	O_stm32->setTorque(m_AcceleratorRef);

	m_PrevAccRef = m_AcceleratorRef;
	return(true);
}

//--------------------------------------------------------------------------------------------------------------
Direction_T Inverter::getDirection(void) {
  if (O_stm32->getFWD() && !O_stm32->getREV()) return(FWD);
  else if (O_stm32->getREV() && !O_stm32->getFWD()) return(REV);
  else if (O_stm32->getNET()) return(NET);
  else {
    O_stm32->setErrorLED(true);
    O_serial->printf("\nWARNING - 'FWD and REV inputs inconsistant'.\r\n");
    return(NET);  //Netural
  }
}
