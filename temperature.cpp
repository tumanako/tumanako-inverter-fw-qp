
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

//--------Temperature Active Object -----------------------------------------------------------------------------
class Temperature : public QActive {            // extend the QActive class

    QTimeEvt 	m_timeEvt;                      // time event
    bool 		m_IsFirstMeasurement;		    //variables for Motor  calculation
    u16 		m_PreviousCount;

public:
    Temperature(void);

private: // HSM
    static QState initial     	(Temperature *me, QEvent const *e);
    static QState run        	(Temperature *me, QEvent const *e);


private: // Helper functions
    s16 	calcMotorTemperature(void);
    void 	initMotorTemperature(void);

};
//--------------------------------------------------------------------------------------------------------------


enum InternalSignals {                   // internal signals
    TIMEOUT_SIG = MAX_SIG

};

static Temperature l_Temperature;       // the sole instance of the Temperature active object

//---------Global Objects --------------------------------------------------------------------------------------
QActive * const AO_Temperature = &l_Temperature;             // opaque pointer to Temperature

//---------Local Objects ---------------------------------------------------------------------------------------


//---------Constructor------------------------------------------------------------------------------------------
Temperature::Temperature()
    : QActive((QStateHandler)&Temperature::initial),
      m_timeEvt(TIMEOUT_SIG),
      m_IsFirstMeasurement(true),
      m_PreviousCount(0)
{}

//--------------------------------------------------------------------------------------------------------------
QState Temperature::initial(Temperature *me, QEvent const *) {

	me->initMotorTemperature();
    return Q_TRAN(&Temperature::run);
}

//--------------------------------------------------------------------------------------------------------------
QState Temperature::run(Temperature *me, QEvent const *e) {
    switch (e->sig) {
        case Q_ENTRY_SIG: {
        	me->m_timeEvt.postEvery(me,  1000/BSP_MS); 	// Do temperature measurement every second
			return Q_HANDLED();
        }

        case TIMEOUT_SIG: {
        	// Broadcast temperature result.
        	motorTempEvt *pe = Q_NEW(motorTempEvt, MOTOR_TEMP_SIG);
        	pe->motorTemp =  me->calcMotorTemperature();
        	QF::PUBLISH(pe, me);

        	// TODO Need to also measure IGBT temperature
        	return Q_HANDLED();
        }
    }
    return Q_SUPER(&QHsm::top);
}

//------------Private Helper functions--------------------------------------------------------------------------
//Setup hardware to read temperature data (input as a freq on PC8)
void Temperature::initMotorTemperature(void) {

	//ENABLE TIM3 full remap (PC8 on chnl 3)
	AFIO_MAPR |= AFIO_MAPR_TIM3_REMAP_FULL_REMAP;

	//TIM3 clock source enable
	RCC_APB1ENR |= RCC_APB1ENR_TIM3EN;

	//Enable GPIOC clock
	RCC_APB2ENR |= RCC_APB2ENR_IOPCEN;

	//Set GPIOC Pin 8 as input floating (motor temperature - freq maps to temperature)
	gpio_set_mode(
	  GPIOC,
	  GPIO_MODE_INPUT,
	  GPIO_CNF_INPUT_FLOAT,
	  GPIO8);

	//TIM3 Prescaler = 0  i.e. No prescaling
	TIM3_PSC = 0x0;
	//TIM3 Counter Mode Up;
	timer_set_alignment(TIM3, TIM_CR1_CMS_EDGE);
	//TIM3 Period = 65535
	TIM3_ARR = 65535;
	//TIM3 clock division 1
	timer_set_clock_division(TIM3, TIM_CR1_CKD_CK_INT);

	//(CC1 input, IC1 mapped onto TI1) and OC1Ref is cleared when High detected on ETRF input
	TIM3_CCMR1 |= 0x04 | 0x01;
	TIM3_CR2 |= 0x0080;  //Set TI1S (CH1, CH2 and CH3 are connected to the TI1 input (XOR)
	TIM3_SMCR = 0x50 | 0x07; //TI1FR1 (b101) and SMS = external clock /mode 1
	TIM3_SR = ~((u16)0x0001); //Clear Update interupt flag (UIF)
	TIM3_DIER |= 0x0001; //IT update (UIE - update interupt enable)
	TIM3_CNT = 0;   //Clear counter
	TIM3_CR1 |= TIM_CR1_CEN;  //Finally, enable it!
}

//--------------------------------------------------------------------------------------------------------------
/*******************************************************************************
* Function Name  : calcMotorTemp (PCC)
* Description    : Compute return latest motor temp. This method is called every 200ms
*
* Input          : None
* Output         : s16
* Return         : Return motor temp in 1 deg C resolution.
*******************************************************************************/
s16 Temperature::calcMotorTemperature(void)
{
	s32 freq_count;  //count pulses from sensor since last read
	s32 CurrentCount;
	s16 degC = 0;

	// TIM3 is Motor Temp timer counter (it cycles to MAX_UINT)
	CurrentCount = TIM3_CNT;

	if (!m_IsFirstMeasurement)
	{
	  freq_count = (CurrentCount - m_PreviousCount -1 );  //Subtract extra 1 to remove LED flash triggers

	// INLINE TESTING for KTY84 constants (uncomment to use)
	//freq_count = 410 - 65536;  //expect 60 deg C - Result PASS
	//freq_count = 410;  //expect 60 deg C - Result PASS
	//freq_count = 364;  //expect 100 dec C - Result PASS
	//freq_count = 307;  //expect 150 dec C - Result 149 deg C (PASS with acceptable linear approximation error)
	//freq_count = 446;  //expect 30 dec C - Result 29.9 deg C (PASS with acceptable linear approximation error)

	  //convert to  (degres celcius)
	  degC = ((freq_count*TUMANAKO_MT_A) + TUMANAKO_MT_B)/100;

	} //is first measurement, discard it
	else
	{
	  m_IsFirstMeasurement = false;
	  degC = 0;
	}

	m_PreviousCount = CurrentCount;

	return( degC );
}
//--------------------------------------------------------------------------------------------------------------
