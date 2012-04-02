
//------------------------------------------------------------------------------
//   Tumanako_QP - Electric Vehicle and Motor control software
//
//	 Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
//   Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
//	 Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
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

#ifndef STM32_SINE_H_INCLUDED
#define STM32_SINE_H_INCLUDED

#include <stdint.h>

/** @todo calculate this instead of using a constant */
#define PERIPH_CLK      ((u32)36000000)

#define SINTAB_ARGDIGITS 8
#define SINTAB_ENTRIES  (1 << SINTAB_ARGDIGITS)
/* Value range of sine lookup table */
#define SINTAB_DIGITS    16
#define SINTAB_MAX      (1 << SINTAB_DIGITS)
/* Domain of lookup function */
#define SINLU_ARGDIGITS  16
#define SINLU_ONEREV    (1 << SINLU_ARGDIGITS)
#define PWM_DIGITS       12
#define PWM_MAX         (1 << PWM_DIGITS)
#define PHASE_SHIFT120  ((u32)(     SINLU_ONEREV / 3))
#define PHASE_SHIFT240  ((u32)(2 * (SINLU_ONEREV / 3)))

#define PWM_FREQ           (PERIPH_CLK / (u32)PWM_MAX)
#define HZ_PER_DIGIT_8    ((256 * PWM_FREQ) / SINLU_ONEREV)
#define DIGIT_TO_N_8       (HZ_PER_DIGIT_8 / POLE_PAIRS)
#define NUM_IMPULSE_PER_REV 200	//BRM
#define POLE_PAIRS           2
#define SAMPLE_INTERVAL    100
#define SAMPLES_PER_SEC 	(1000/SAMPLE_INTERVAL)
#define N_FACTOR 			((float)SAMPLES_PER_SEC/(float)NUM_IMPULSE_PER_REV)

#define RPM_FACT           (6000/NUM_IMPULSE_PER_REV * 1000 / SAMPLE_INTERVAL)
#define HZ_FACT            (POLE_PAIRS * RPM_FACT/6)

#define DIGIT_TO_FRQ(Digit)		((Digit * PWM_FREQ) / 65536)
#define FRQ_TO_DIGIT(frq)		(( frq * 65536) / PWM_FREQ)

#define TUMANAKO_PWM_TIM_CCR1 TIM1_CCR1
#define TUMANAKO_PWM_TIM_CCR2 TIM1_CCR2
#define TUMANAKO_PWM_TIM_CCR3 TIM1_CCR3

#define TUMANAKO_TIMX_SR TIM1_SR

#define	TUMANAKO_PWM_RCC_APBXENR RCC_APB2ENR
#define	TUMANAKO_PWM_RCC_APBXENR_TIMXEN (RCC_APB2ENR_TIM1EN | RCC_APB2ENR_AFIOEN)

#define TUMANAKO_PWM_TIM_PORT GPIOE
//#define TUMANAKO_PWM_TIM_CHANNELS (GPIO_TIM1_CH1_REMAP | GPIO_TIM1_CH2_REMAP | GPIO_TIM1_CH3_REMAP | GPIO_TIM1_CH1N_REMAP | GPIO_TIM1_CH2N_REMAP | GPIO_TIM1_CH3N_REMAP)
#define TUMANAKO_PWM_TIM_CHANNELS (GPIO9 | GPIO11 | GPIO13 | GPIO8 | GPIO10 | GPIO12)

#define TUMANAKO_NVIC_PWM_IRQ NVIC_TIM1_UP_IRQ

#define TUMANAKO_PWM_TIM_EGR TIM1_EGR
#define TUMANAKO_PWM_TIM_DIER TIM1_DIER

#define TUMANAKO_PWM_CR1 TIM1_CR1
#define TUMANAKO_PWM_CCMR1 TIM1_CCMR1
#define TUMANAKO_PWM_CCMR2 TIM1_CCMR2
#define TUMANAKO_PWM_CCER TIM1_CCER

#define TUMANAKO_PWM_PSC TIM1_PSC
#define TUMANAKO_PWM_ARR TIM1_ARR

#define TUMANAKO_ROT_TIM_SCMR         TIM2_SMCR //TODO test new KiwiAC speed sensor timer setup
#define TUMANAKO_ROT_TIM_CR1          TIM2_CR1
#define TUMANAKO_ROT_TIM_CNT          TIM2_CNT


typedef struct slip_params_tag {
	int frq_setpt;
	int boost_frq;
	int boost_amp;
	int field_weakening_frq;
	int max_amp;
	int   slew_rate;

    int v_frq;
    int v_amp;
}slip_params;


class SineController {
   public:
      SineController(void);
      void SetFrqSpnt(int frqspnt);
      int  GetCurFrq();
      void TimerInterrupt(void);
      void Init(void);

   private:
      void clock_setup(void);
      void gpio_setup(void);
      void tim_setup(void);
      s16 CalcSVPWMOffset(u16 a, u16 b, u16 c);
      void RampFrq(void);
      void CalcUf(void);
      u16 MultiplyAmplitude(u16  Amplitude, u16  Baseval);
      u16 SineLookup(u16  Arg);
      slip_params params;

      int slewCtr;
      u16 DutyCycle[3];
      u16 CtrVal;
      u8  mode;
      s16 frq;
};


#endif // STM32_SINE_H_INCLUDED
