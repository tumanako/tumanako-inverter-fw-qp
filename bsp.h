
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
#ifndef BSP_H_
#define BSP_H_

#define STM32F1

extern "C"
{
// C Includes
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f1/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/nvic.h>
#include <libopencm3/stm32/f1/scb.h>
#include <libopencm3/stm32/systick.h>
#include <libopencm3/cm3/common.h>  //u8 etc
#include <stdio.h>
#include <stdarg.h>  //variable args
#include <string.h>  //c string

}

// Includes ----------------------------------------------------------------------------------------------------------
#include "qp_port.h"
#include "filter/filter.hpp"
#include "sine.hpp"
#include "sine_lookup_table.h"
#include "STM32_interface.hpp"
#include "serial.hpp"
#include "pid.hpp"
#include "SlipAngle.hpp"
#include "ClarkeParkTransforms.hpp"

// All signals used ---------------------------------------------------------------------------------------------------
enum TumanokoSignals {
    PRECHARGE_COMPLETE_SIG  = Q_USER_SIG,
    MOTOR_TEMP_SIG,
    // insert other published signals here ...
    MAX_PUB_SIG,                                  // the last published signal

    PRECHARGE_START_SIG,

    IGNITION_ON_SIG,
    IGNITION_OFF_SIG,

    START_BUTTON_SIG,

    FORWARD_SIG,
    REVERSE_SIG,
    NEUTRAL_SIG,

    BRAKE_ON_SIG,
    BRAKE_OFF_SIG,

    IGBT_FAULT_SIG,
    IGBT_FAULT_CLEARED_SIG,

    PCB_RED_SIG,
    PCB_GREEN_SIG,
    DASH_RED_SIG,
    DASH_GREEN_SIG,
    BOTH_LEDS_OFF_SIG,

    MAX_SIG                              // the last signal (keep always last)
};

//Active objects ------------------------------------------------------------------------------------------------------
extern QActive * const AO_Inverter;
extern QActive * const AO_Led;
extern QActive * const AO_Scan;
extern QActive * const AO_PreCharge;
extern QActive * const AO_Temperature;
extern QActive * const AO_Slip;
extern QActive * const AO_DashBoard;

// Normal objects -----------------------------------------------------------------------------------------------------
extern STM32Interface * const O_stm32;
extern SineController * const O_sine;
extern Serial * const O_serial;

// Typedef's, enum's --------------------------------------------------------------------------------------------------
typedef enum {FWD, REV, NET} Direction_T;

enum LedStateTag {
	FLASH = 0,
	STEADY,
	OFF
};


// Events -------------------------------------------------------------------------------------------------------------
struct motorTempEvt : public QEvent {       // derive from the QEvent class
	s16 motorTemp;							// send the data.
};

struct PreChargeCompleteEvt : public QEvent {
	// stuff can go here
};

struct LedEvt : public QEvent {		// Derive from the QEvent class.
	LedStateTag	ledState;
	int			ledFlashRate;
};

// Pre-declarations ---------------------------------------------------------------------------------------------------

void BSP_init(void);

// Defines ------------------------------------------------------------------------------------------------------------


#define max(a,b,c) (a>b && a>c)?a:(b>a && b>c)?b:c
#define min(a,b,c) (a<b && a<c)?a:(b<a && b<c)?b:c
#define minmax(a,b,c) (a<b)?b:(a>c)?c:a

//PCC
//KTY84 linear approximation (degC=A.freq+B) - These constants are x100 to retain accuracy + integer maths performance
#define TUMANAKO_MT_A 			-87
#define TUMANAKO_MT_B 			41758  // = 41708 + 50 (for rounding to nearest 1 deg C)

// System clock tick rate [Hz]
#define BSP_TICKS_PER_SEC 		200
#define BSP_MS					(1000/BSP_TICKS_PER_SEC)


#define TUMANKAO_VERSION "1.0.0 open" //'open' denotes a version with 100% open source code

#define TUMANAKO_PRECHARGE_V 	20 					//voltage precharge is required to reach before main contactors engage
#define TUMANAKO_MAX_BUX_V 		26 					//Max bus voltage we ever expect to see
#define TUMANAKO_MIN_PRECHARGE_TIME 		2000 	//  No matter what happens a successful precharge will take at least this long (ms)
#define TUMANAKO_MAX_PRECHARGE_TIME 		3000 	//  If a precharge takes longer than this, precharge is considered a failure.
#define TUMANAKO_PRECHARGE_FEEDBACK_TIME 	45 		//25 millisec (gigavac), 45 (kilovac) - Allow time for contactors to change
//#define TUMANAKO_USE_FILTER //Turns the software filters for digital signals on
#define TUMANAKO_HALF_POWER_MOTOR_TEMP 		115 //115 deg C - Half system power
#define TUMANAKO_MAX_MOTOR_TEMP 			125 //125 deg C - Shutdown system

#define TUMANAKO_PRECHARGE_ITERATION_WAIT 	100  //number of millisec to wait between precharge iterative tests

#define TK_ABS(x) 				( x < 0 ? - x :  x)

#define TUMANAKO_MAX_REGEN_POWER 			40000000

#define ACCEL_AI 				14

// Un-comment this if you want to use a start button.
//#define USE_START_BUTTON

#define THROTTLE_POT_MAX 		2570	// My POT max/min
#define THROTTLE_POT_MIN 		5

#define TORQUE_MAX 				200		// This gives max slip frequency of 20 Hz
#define TORQUE_MIN 				-30		// braking slip of 3Hz
#define ZERO_THROTTLE_TORQUE 	0

#define START_REGEN_RAMP_RPM 	750
#define END_REGEN_RAMP_RPM 		950
#define GLIDE_RPM 				100

// Default flux values.
#define DEFAULT_FLUX 			30

//Constants for linear equation to map throttle input to torque request
#define TUMANAKO_A 				(((float)TORQUE_MAX - TORQUE_MIN) / (THROTTLE_POT_MAX - THROTTLE_POT_MIN))
#define TUMANAKO_B 				(TORQUE_MIN - TUMANAKO_A * THROTTLE_POT_MIN)

#define FLUX_MIN 				2500
#define FLUX_MAX 				DEFAULT_FLUX
#define FIELD_WEAKENING_RPM_START 3000
#define FIELD_WEAKENING_RPM_END	 6000  //range between START and END must be less than range between FLUX_MAX and FLUX_MIN

//Constants for linear equation to map RPM to flux request (strengthen from low point during idle)
#define FIELD_STREGTHEN_RPM_START 1000 		//Must be greater than END_REGEN_RAMP_RPM
#define FIELD_STREGTHEN_RPM_END 1500  		//Must be less than FIELD_WEAKENING_RPM_END

//Constants for linear equation to map RPM to flux request
#define TUMANAKO_FLUX_A 		(((float)FLUX_MAX - FLUX_MIN) / (FIELD_STREGTHEN_RPM_END - FIELD_STREGTHEN_RPM_START))
#define TUMANAKO_FLUX_B 		(FLUX_MIN - TUMANAKO_FLUX_A * FIELD_STREGTHEN_RPM_START)

#define THROTTLE_ERROR_COUNT 	6


#endif /* BSP_H_ */
