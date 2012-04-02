
//------------------------------------------------------------------------------
//   Tumanako_QP - Electric Vehicle and Motor control software
//
//   Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
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


//--------SlipControl Active Object -----------------------------------------------------------------------------
class SlipControl : public QActive {                  // extend the QActive class

    QTimeEvt 		m_timeEvt;                        // time event

public:
	SlipControl(void);

private: // HSM
    static QState initial     	(SlipControl *me, QEvent const *e);
    static QState run       	(SlipControl *me, QEvent const *e);

private:
    s16 	LastCtr;
    s16 	CtrVal;
   	float 	slip_spnt;
   	float 	frq;
   	float 	slip;
   	bool 	led_toggle;
   	PID 	pid;

private: // Helper functions

};

//--------------------------------------------------------------------------------------------------------------
enum InternalSignals {                       // internal signals
    TIMEOUT_SIG = MAX_SIG
};

//---------Object Declaration-----------------------------------------------------------------------------------
static SlipControl l_Slip;      			 // the sole instance of this object

QActive * const AO_Slip = &l_Slip;           // opaque pointer to the Slip object.
//--------------------------------------------------------------------------------------------------------------

//---------Constructor------------------------------------------------------------------------------------------
SlipControl::SlipControl()
    : QActive((QStateHandler)&SlipControl::initial),
      m_timeEvt(TIMEOUT_SIG),
      LastCtr(0),
      led_toggle(false)
{}

#define LIMIT				1450.0		// Speed Limit

//--------------------------------------------------------------------------------------------------------------
QState SlipControl::initial(SlipControl *me, QEvent const *) {
	 me->pid.InitPid(9.5,	// Kp		Setup PID parameters.
			 	 	 0.0,	// Ki
			 	 	 1.0,	// Kd
			 	 	 1.0,	// Ko
			 	 	LIMIT);	// MaxOutput
    return Q_TRAN(&SlipControl::run);
}

//--------------------------------------------------------------------------------------------------------------
QState SlipControl::run(SlipControl *me, QEvent const *e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
        	me->m_timeEvt.postEvery(me,  SAMPLE_INTERVAL/BSP_MS); 		// The control works at this rate. (100ms)
			return Q_HANDLED();											// RPM uses this sample rate
        }

        case TIMEOUT_SIG: {
        	volatile float  f_rotor,  newfrq;
        	volatile short  rpm;

			me->slip_spnt = O_stm32->getTorque(); 				// slip_spnt varies from -30 .. 100 slip frequency.

			LedEvt *pe = Q_NEW(LedEvt, PCB_RED_SIG);			// Flash PCB red LED to show lifesign ..
			if(me->led_toggle==false) { pe->ledState = STEADY; me->led_toggle=true; }
			else { pe->ledState = OFF; me->led_toggle=false; }
			AO_Led->POST(pe, me);

			me->CtrVal = TUMANAKO_ROT_TIM_CNT - me->LastCtr;
			me->LastCtr = TUMANAKO_ROT_TIM_CNT;

			f_rotor = (float)me->CtrVal * N_FACTOR;

			newfrq = f_rotor + me->slip_spnt;

			rpm = (120.0 * f_rotor) / POLE_PAIRS;				//rpm = 120f/p
			O_stm32->setRPM(rpm);								// Save RPM value

			// Calculate new frequency rate with PID
			me->frq = me->pid.DoPid(newfrq, f_rotor);

			O_sine->SetFrqSpnt((int)me->frq);					// Pass frequency request to sine object.
			return Q_HANDLED();
        }
    }
    return Q_SUPER(&QHsm::top);
}


