
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

Q_DEFINE_THIS_FILE


enum InternalSignals {                 	// Some timer signals
    TIMEOUT_SIG = MAX_SIG,

};

// Active object class ------------------------------------------------------------------------------------------------------------
class DashBoard : public QActive {

private:
	QTimeEvt 		m_timeEvt;
	s16				m_MotorTemperature;

public:
    DashBoard();

private:
    static QState initial(DashBoard *me, QEvent const *e);
    static QState run(DashBoard *me, QEvent const *e);

    void displayData(void);
    short convertToDegrees(short digitalAngle);
};

// Local objects ------------------------------------------------------------------------------------------------------------------
static DashBoard l_DashBoard;                                    // local Table object

// Public-scope objects -----------------------------------------------------------------------------------------------------------
QActive * const AO_DashBoard = &l_DashBoard;                    // "opaque" AO pointer


// --------------------------------------------------------------------------------------------------------------------------------
DashBoard::DashBoard() : QActive((QStateHandler)&DashBoard::initial),
		m_timeEvt(TIMEOUT_SIG),
		m_MotorTemperature(0)
{}

// --------------------------------------------------------------------------------------------------------------------------------
QState DashBoard::initial(DashBoard *me, QEvent const *) {
	// TODO Set up any GPIO here ..
	me->subscribe(MOTOR_TEMP_SIG);
    return Q_TRAN(&DashBoard::run);
}

// --------------------------------------------------------------------------------------------------------------------------------
// This is the normal run state for the DashBoard.
QState DashBoard::run(DashBoard *me, QEvent const *e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
        	me->m_timeEvt.postEvery(me,  250/BSP_MS); // Update screen every 250ms
            return Q_HANDLED();
		}
        case TIMEOUT_SIG: {
        	me->displayData();
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
// Write dashboard type data to RS232 port for now .. eventually probably should be CAN or USB to vehicle display module.
void DashBoard::displayData(void) {

    //signed short phaseC = getPhaseC(O_stm32->getPhase1(), O_stm32->getPhase2());
    s16 phaseC = O_stm32->getPhase3();
    float current = TK_ABS(O_stm32->getPhase1()) + TK_ABS(O_stm32->getPhase2()) + TK_ABS(phaseC);

	O_serial->usartWriteDisclaimer();
	O_serial->printf("       Bus (V): %d ",O_stm32->busVoltage());
	O_serial->printf("pwrStgTemp (C): %d \r\n",O_stm32->powerStageTemperature());
	O_serial->printf("     Motor Frq: %d ",O_stm32->getFrq());
	O_serial->printf(" motorTemp (C): %d \r\n",m_MotorTemperature);
	O_serial->printf("           RPM: %d ",O_stm32->getRPM());
	O_serial->printf("          Flux: %d \r\n",O_stm32->getFlux());
	O_serial->printf("   RawAccelPOT: %d ",O_stm32->readADC(ACCEL_AI));
	O_serial->printf("    Torque Out: %d \r\n",O_stm32->getTorque());
	O_serial->printf("   Phase 1 (A): %d ",O_stm32->getPhase1());
	O_serial->printf("   Phase 2 (A): %d \r\n",O_stm32->getPhase2());
	O_serial->printf(" PhAOffset (A): %d ",O_stm32->getPhaseAOffset());
	O_serial->printf(" PhBOffset (A): %d \r\n",O_stm32->getPhaseBOffset());
	O_serial->printf("    phase3 (A): %d ",phaseC);
	O_serial->printf("     Total (A): %d.%d \r\n",(int)current,((int)(current*100.0))%100);
	O_serial->printf("RotorTimeConst: %d ",O_stm32->getRotorTimeConstant());
	O_serial->printf("      SlipFreq: %d \r\n",O_sine->GetCurFrq());
	O_serial->printf("          Slip: %d ",O_stm32->getSlip());
	O_serial->printf("     FluxAngle: %d \r\n",convertToDegrees(O_stm32->getFluxAngle()));
}

//--------------------------------------------------------------------------------------------------------------
short DashBoard::convertToDegrees(short digitalAngle) {
  return (short)(((long)digitalAngle * 180)/32768);  //degrees
}

