
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

#include "bsp.h"

Q_DEFINE_THIS_FILE


static const unsigned short SinTab[] = { SINTAB };/* sine LUT */

//---------Object Declaration-----------------------------------------------------------------------------------
static SineController l_SineController;       		// the sole instance of this module

SineController * const O_sine = &l_SineController;   // opaque pointer to Sine object.
//--------------------------------------------------------------------------------------------------------------

//---------Constructor------------------------------------------------------------------------------------------
SineController::SineController()
{
	frq = 1;	// Default frequency
	slewCtr = 0;

	params.boost_amp = 2000;
	params.boost_frq = FRQ_TO_DIGIT(1);
	params.field_weakening_frq = FRQ_TO_DIGIT(100);
	params.frq_setpt =  FRQ_TO_DIGIT(50);	// ...Just initially
	params.max_amp = 37813;
	params.slew_rate = 1;

	params.v_frq = 0;
	params.v_amp = 0;
}
//--------------------------------------------------------------------------------------------------------------
// Interface to set/get frq
void SineController::SetFrqSpnt(int frqspnt) // freqency in digit form
{
   params.frq_setpt = frqspnt;
}

//--------------------------------------------------------------------------------------------------------------
int SineController::GetCurFrq()
{
   return frq;
}

//--------------------------------------------------------------------------------------------------------------
/* Performs a lookup in the sine table */
/* 0 = 0, 2Pi = 65535 */
u16 SineController::SineLookup(u16 Arg)
{
    /* No interpolation for now */
    /* We divide arg by 2^(SINTAB_ARGDIGITS) */
    /* No we can directly address the lookup table */
    Arg >>= SINLU_ARGDIGITS - SINTAB_ARGDIGITS;
    return SinTab[Arg];
}

//--------------------------------------------------------------------------------------------------------------
void SineController::RampFrq()
{
   if (frq < params.frq_setpt)
      //frq++;		// inc by factor/digit?
   	   frq +=  FRQ_TO_DIGIT(1);
   else if (frq > params.frq_setpt)
      //frq--;
	   frq -=  FRQ_TO_DIGIT(1);
}

//--------------------------------------------------------------------------------------------------------------
void SineController::CalcUf()
{
   params.v_frq = TK_ABS(params.v_frq);	// Just in case direction is reverse.
   if (params.v_frq == 0)
   {
      params.v_amp = 0;
   }
   else if (params.v_frq < params.boost_frq)
   {
      params.v_amp = params.boost_amp;
   }
   else if (params.v_frq < params.field_weakening_frq)
   {
      int m = (params.max_amp - params.boost_amp)/(params.field_weakening_frq - params.boost_frq);
      int b = params.max_amp - (m * params.field_weakening_frq);
      params.v_amp = (m * params.v_frq) + b;
   }
   else
   {
      params.v_amp = params.max_amp;
   }
}
//--------------------------------------------------------------------------------------------------------------
extern "C" void tim1_brk_isr(void) __attribute__((__interrupt__));
extern "C" void tim1_brk_isr(void)
{
   // TODO WHat's this for?
}

//--------------------------------------------------------------------------------------------------------------
/* Calculate dutycycles */
extern "C" void tim1_up_isr(void) __attribute__((__interrupt__));
extern "C" void tim1_up_isr(void)
{
	O_sine->TimerInterrupt();
}

//--------------------------------------------------------------------------------------------------------------
/* 0 = 0, 1 = 32767 */
u16 SineController::MultiplyAmplitude(u16 Amplitude, u16 Baseval)
{
    u32 Temp;
    Temp = (u32)Amplitude * (u32)Baseval;
    /* Divide by 32768 */
    /* -> Allow overmodulation, for SVPWM or FTPWM */
    Temp >>= (SINTAB_DIGITS - 1);
    /* Match to PWM resolution */
    Temp >>= (SINTAB_DIGITS - PWM_DIGITS);
    return Temp;
}

//--------------------------------------------------------------------------------------------------------------
s16 SineController::CalcSVPWMOffset(u16 a, u16 b, u16 c)
{
    /* Formular for svpwm:
       Offset = 1/2 * (min{a,b,c} + max{a,b,c}) */
    /* this is valid only for a,b,c in [-32768,32767] */
    /* we calculate from [0, 65535], thus we need to subtract 32768 to be symmetric */
    s16 acor = a - (SINTAB_MAX >> 1);
    s16 bcor = b - (SINTAB_MAX >> 1);
    s16 ccor = c - (SINTAB_MAX >> 1);

    s16 Minimum = min(acor, bcor, ccor);
    s16 Maximum = max(acor, bcor, ccor);
    s16 Offset = Minimum + Maximum;

    return Offset >> 1;
}


//--------------------------------------------------------------------------------------------------------------
void SineController::TimerInterrupt(void)
{
    static   u8  IntCnt = 0;
    static   u16 Arg = 0;
             s16 Ofs;
             u8  Idx;
    volatile u32 *pTimCcr = &TUMANAKO_PWM_TIM_CCR1;
    		 u16 DutyCycle[3];

    if (slewCtr++ >= params.slew_rate)
    {
	  RampFrq();
	  CalcUf();
	  params.v_frq = frq;
	  slewCtr= 0;
    }

    IntCnt++;

    /* Clear interrupt pending flag - do this at the start of the routine */
    TUMANAKO_TIMX_SR &= ~TIM_SR_UIF;

    /* In center aligned mode, this ISR is called twice:
       - in the middle of the period
       - at the end of the period
       we only want to calculate in the middle */
    if (IntCnt & 1)
    {
        /* 1. Calculate sine */
        DutyCycle[0] = SineLookup(Arg);
        DutyCycle[1] = SineLookup((u16)(((u32)Arg + PHASE_SHIFT120) & 0xFFFF));
        DutyCycle[2] = SineLookup((u16)(((u32)Arg + PHASE_SHIFT240) & 0xFFFF));

        /* 2. Calculate the offset of SVPWM */
        Ofs = CalcSVPWMOffset(DutyCycle[0], DutyCycle[1], DutyCycle[2]);

        for (Idx = 0; Idx < 3; Idx++, pTimCcr++)
        {
            /* 3. subtract it from all 3 phases -> no difference in phase-to-phase voltage */
            DutyCycle[Idx] -= Ofs;
            /* 4. Set desired amplitude and match to PWM resolution */
            DutyCycle[Idx] = MultiplyAmplitude(params.v_amp, DutyCycle[Idx]);
            /* 5. Write to compare registers (set the PWM Duty Cycle!)*/
            *pTimCcr = DutyCycle[Idx];
        }

        /* Increase sine arg */
        /* values below 0 makes us run through the sine lookup backwards
           -> motor spins the other direction */
        Arg += frq;
    } /* end if */
} /* end isr */

//--------------------------------------------------------------------------------------------------------------
void SineController::clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable all present GPIOx clocks. (whats with GPIO F and G?)*/
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPDEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPEEN);

	/* Enable clock for USART1. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);

	/* Enable TIM1 clock */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_TIM1EN);

	/* Enable TIM2 clock */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);

	/* Enable TIM3 clock */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM3EN);

	/* Enable PWM TIM clock (TIM1 or TIM4 depending on KiwiAC or not defined above) */
	rcc_peripheral_enable_clock(&TUMANAKO_PWM_RCC_APBXENR, TUMANAKO_PWM_RCC_APBXENR_TIMXEN);
}


//--------------------------------------------------------------------------------------------------------------
void SineController::tim_setup(void)
{
    /* Center aligned PWM - control register 1 */
    TUMANAKO_PWM_CR1 = TIM_CR1_CMS_CENTER_1 | TIM_CR1_ARPE;
    /* PWM mode 1 and preload enable - capture compare mode register 1 and 2 */
    TUMANAKO_PWM_CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
    TUMANAKO_PWM_CCMR2 = TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE;
    /* Output enable */

    //clear CC1P (capture compare enable register, active high)
    TIM1_CCER &= (uint16_t)~TIM_CCER_CC1P;

    TUMANAKO_PWM_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;
    //Deadtime = 28 ~ 800ns
    TIM1_BDTR = (TIM_BDTR_OSSR | TIM_BDTR_OSSI | 28 ) & ~TIM_BDTR_BKP & ~TIM_BDTR_AOE;

    TIM1_CR2 &= (uint16_t)~(TIM_CR2_OIS1 || TIM_CR2_OIS1N || TIM_CR2_OIS2 || TIM_CR2_OIS2N || TIM_CR2_OIS3 || TIM_CR2_OIS3N);

    /* Enable update generation */
    TUMANAKO_PWM_TIM_EGR = TIM_EGR_UG;
    /* Enable update event interrupt */
    TUMANAKO_PWM_TIM_DIER = TIM_DIER_UIE;
    /* Prescaler */
    TUMANAKO_PWM_PSC = 0x1;
    /* PWM frequency */
    TUMANAKO_PWM_ARR = PWM_MAX; //4096 

    TIM1_RCR = 1;  //set rep_rate

    /* start out with 50:50 duty cycle (does NOT handle rolling starts!!!)*/
    TUMANAKO_PWM_TIM_CCR1 = PWM_MAX / 2;
    TUMANAKO_PWM_TIM_CCR2 = PWM_MAX / 2;
    TUMANAKO_PWM_TIM_CCR3 = PWM_MAX / 2;
    /* Enable timer */
    TUMANAKO_PWM_CR1 |= TIM_CR1_CEN;

    //TODO I suspect this encoder setup needs to be vastly different for the KiwiAC
    /* setup capture timer. Strong filtering for EMI "robustness" */
    TUMANAKO_ROT_TIM_SCMR = TIM_SMCR_ECE | TIM_SMCR_ETP | TIM_SMCR_ETF_DTS_DIV_16_N_8;
    /* setup capture timer */
    TUMANAKO_ROT_TIM_CR1  = TIM_CR1_CEN;
}

//--------------------------------------------------------------------------------------------------------------
void SineController::gpio_setup(void)
{

	AFIO_MAPR |= AFIO_MAPR_TIM1_REMAP_FULL_REMAP;

    /* Timer Ch GPIO */
    gpio_set_mode(TUMANAKO_PWM_TIM_PORT, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, TUMANAKO_PWM_TIM_CHANNELS);
}

//--------------------------------------------------------------------------------------------------------------
/* Used to initailise the stm32_sine module*/
void SineController::Init(void)
{
	clock_setup();
	gpio_setup();
	tim_setup();
}
//--------------------------------------------------------------------------------------------------------------


