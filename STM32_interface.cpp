
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

#define STM32F1  //applicable to the STM32F1 series of devices

//--------------------------------------------------------------------------------------------------------------
#include "bsp.h"

#ifdef TUMANAKO_USE_FILTER
#include "filter/filter.hpp"
#endif


//---------Object Declaration-----------------------------------------------------------------------------------
static STM32Interface l_STM32Interface;       // the sole instance of this module

STM32Interface * const O_stm32 = &l_STM32Interface;             // opaque pointer to Interface object.
//--------------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------------------
STM32Interface::STM32Interface()
#ifdef TUMANAKO_USE_FILTER
    : startFilter(), //used to filter out noise (to prevent false starting)
  	crawlFilter(),
    ignFilter(),
	fwdFilter(),
	revFilter()

#endif
{
	torque_ = 0;
	rpm_ = 0;
}

//--------------------------------------------------------------------------------------------------------------
//returns the scaled bus voltage direct from a single digital conversion
unsigned short STM32Interface::getRawScaledBusVolt() {
  //PCC
  //SKAI 1200 0V in = 0V DC Bus, 9V in = 900V DC Bus (10V = 1000V)
  //SKAI 600 0V in = 0V DC Bus, 9V in = 450V DC Bus

  //KiwiAC max = 10 * 3.3/10.1 (99% of full 3.3V scale)
  //(10 * 3.3/10.1) * 32768 = 32444

  //TODO create SI unit object.  All data exposed via these objects
  // si.eu (scaled enginerring units - fixed point and float versions)
  // si.digital (raw digital value)

  //return ((u16)(( (u32)getRawBusVolt() * 1000)/32444));
	return (24);
}

//--------------------------------------------------------------------------------------------------------------
//returns the digital bus voltage A/D conversion without SI unit scaling
unsigned short STM32Interface::getRawBusVolt() {
  adc_on(ADC2); // If the ADC_CR2_ON bit is already set -> setting it another time starts the conversion
  while (!(ADC_SR(ADC2) & ADC_SR_EOC)) {};  // Waiting for end of conversion
  return (ADC_DR(ADC2) & 0xFFF);   // read adc data (needs scalling!!!)
}


//--------------------------------------------------------------------------------------------------------------
// ==== Provide access to various motor control variables ====

unsigned short STM32Interface::getPhaseAOffset() {
  //TODO - part of IFOC
  return 1024;
}
unsigned short STM32Interface::getPhaseBOffset() {
  //TODO - part of IFOC
  return 1024;
}
short STM32Interface::getPhase1() {
  //TODO - part of IFOC
  return 1024;
}
short STM32Interface::getPhase2() {
  //TODO - part of IFOC
  return 1024;
}
short STM32Interface::getPhase3() {
  //TODO - part of IFOC
  return 1024;
}
unsigned long STM32Interface::getRotorTimeConstant() {
  //TODO - part of IFOC
  return 1024;
}
void STM32Interface::setRotorTimeConstant(unsigned long) {
  //TODO - part of IFOC
}
long STM32Interface::getFrq() {
  //TODO - part of IFOC
  return 1024;
}
short STM32Interface::getFluxAngle() {
  //TODO - part of IFOC
  return 1024;
}
short STM32Interface::getElectricalAngle() {
  //TODO - part of IFOC
  return 1024;
}

//--------------------------------------------------------------------------------------------------------------
// ==== IO from the vehicle loom ====

// ==== Physical Inputs ====

bool STM32Interface::getIGN(void) { //Ignition switch on or off?
	bool sw = gpio_get(TK_IGN_PORT, TK_IGN_PIN)?false:true;
#ifdef TUMANAKO_USE_FILTER
	ignFilter.store(sw);
	return ignFilter.result();
#else
  return sw;  //return raw value
#endif
}

bool STM32Interface::getStart(void) { //Start button engaged?
	bool sw = gpio_get(TK_START_PORT, TK_START_PIN)?false:true;
#ifdef TUMANAKO_USE_FILTER
  startFilter.store(sw);
  return startFilter.result();
#else
  return sw;  //return raw value
#endif
}

bool STM32Interface::getCrawl(void) { //Crawl switch on or off?
	bool sw = gpio_get(TK_CRAWL_PORT, TK_CRAWL_PIN)?false:true;
#ifdef TUMANAKO_USE_FILTER
	crawlFilter.store(sw);
	return crawlFilter.result();
#else
  return sw;  //return raw value
#endif
}

bool STM32Interface::getFWD(void) { //Forward selected?
	bool sw = gpio_get(TK_FWD_PORT, TK_FWD_PIN)?false:true;
#ifdef TUMANAKO_USE_FILTER
	fwdFilter.store(sw);
	return fwdFilter.result();
#else
	return sw;  //return raw value
#endif
}

bool STM32Interface::getREV(void) { //Reverse selected?
	bool sw = gpio_get(TK_REV_PORT, TK_REV_PIN)?false:true;
#ifdef TUMANAKO_USE_FILTER
	revFilter.store(sw);
	return revFilter.result();
#else
	return sw;  //return raw value
#endif
}

bool STM32Interface::getIGBTFault(void) { //Fault detected?
	bool sw = gpio_get(TK_IGBT_FAULT_PORT, TK_IGBT_FAULT_PIN)?true:false;
	return sw;
}

//--------------------------------------------------------------------------------------------------------------
// ==== Physical Outputs ====

void STM32Interface::setErrorLED(bool value) { //Show red error light
  if (value == true) gpio_set(TK_ERROR_LED_PORT, TK_ERROR_LED_PIN);
  else gpio_clear(TK_ERROR_LED_PORT, TK_ERROR_LED_PIN);
}

void STM32Interface::setRunLED(bool value) { //Show green run light
  if (value == true) gpio_set(TK_RUN_LED_PORT, TK_RUN_LED_PIN);
  else gpio_clear(TK_RUN_LED_PORT, TK_RUN_LED_PIN);
}

void STM32Interface::setPCBErrorLED(bool value) {	// Red LED on PCB
  if (value == true) gpio_set(TK_PCB_ERROR_LED_PORT, TK_PCB_ERROR_LED_PIN);
  else gpio_clear(TK_PCB_ERROR_LED_PORT, TK_PCB_ERROR_LED_PIN);
}

void STM32Interface::setPCBRunLED(bool value) {		// Green LED on PCB
  if (value == true) gpio_set(TK_PCB_RUN_LED_PORT, TK_PCB_RUN_LED_PIN);
  else gpio_clear(TK_PCB_RUN_LED_PORT, TK_PCB_RUN_LED_PIN);
}

//--------------------------------------------------------------------------------------------------------------
// Reads the value of the specified ADC channel
unsigned short STM32Interface::readADC(unsigned char channel) { //:TODO: needs to be encapsulated into +ve torque and -ve torque etc (channel param not being used!)
  adc_on(ADC1); // If the ADC_CR2_ON bit is already set -> setting it another time starts the conversion
  while (!(ADC_SR(ADC1) & ADC_SR_EOC)) {};  // Waiting for end of conversion 
  return (ADC_DR(ADC1) & 0xFFF);   /* read adc data */
}

//--------------------------------------------------------------------------------------------------------------
void STM32Interface::gpioInit(void) {

  AFIO_MAPR |= AFIO_MAPR_TIM1_REMAP_FULL_REMAP;

  // Set dashboard Run and Error LEDs 'output push-pull'
  gpio_set_mode(TK_ERROR_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, TK_ERROR_LED_PIN);
  gpio_set_mode(TK_RUN_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, TK_RUN_LED_PIN);

  /* Set PCB LEDs (in GPIO port C) to 'output push-pull'. */
  gpio_set_mode(TK_PCB_ERROR_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
			    GPIO_CNF_OUTPUT_PUSHPULL, TK_PCB_ERROR_LED_PIN);
  gpio_set_mode(TK_PCB_RUN_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
			    GPIO_CNF_OUTPUT_PUSHPULL, TK_PCB_RUN_LED_PIN);

  //Contactor output
  gpio_set_mode(TK_K_OUT_PORT, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, TK_K1_OUT_PIN | TK_K2_OUT_PIN | TK_K3_OUT_PIN);

  //Contactor feedback
  gpio_set_mode(TK_K_IN_PORT, GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN, TK_K1_IN_PIN | TK_K2_IN_PIN | TK_K3_IN_PIN);
  gpio_set(TK_K_IN_PORT, TK_K1_IN_PIN | TK_K2_IN_PIN | TK_K3_IN_PIN);  //make it pull up (reset to make pull down)

  //FWD
  gpio_set_mode(TK_FWD_PORT, GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN, TK_FWD_PIN);
  gpio_set(TK_FWD_PORT, TK_FWD_PIN);  //make it pull up (reset to make pull down)

  //REV
  gpio_set_mode(TK_REV_PORT, GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN, TK_REV_PIN);
  gpio_set(TK_REV_PORT, TK_REV_PIN);  //make it pull up (reset to make pull down)

  //Crawl
  gpio_set_mode(TK_CRAWL_PORT, GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN, TK_CRAWL_PIN);
  gpio_set(TK_CRAWL_PORT, TK_CRAWL_PIN);  //make it pull up (reset to make pull down)

  //Start
  gpio_set_mode(TK_START_PORT, GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN, TK_START_PIN);
  gpio_set(TK_START_PORT, TK_START_PIN);  //make it pull up (reset to make pull down)

  //IGBT Fault Input
  gpio_set_mode(TK_IGBT_FAULT_PORT, GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_PULL_UPDOWN, TK_IGBT_FAULT_PIN);
  gpio_set(TK_IGBT_FAULT_PORT, TK_IGBT_FAULT_PIN);  //make it pull up (reset to make pull down)

}

//--------------------------------------------------------------------------------------------------------------
void STM32Interface::adc_setup(u32 adc_port) {
  switch (adc_port) {
     case ADC1:  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
        break;
     case ADC2:  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC2EN);
        break;
     case ADC3:  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC3EN);
        break;
     default: shutdownPower();  while(1) {};  //TODO log something (should never happen)
        break;
  }

  /* make sure it didnt run during config */
  adc_off(adc_port);

  /* we configure everything for one single conversion */
  adc_disable_scan_mode(adc_port);
  adc_set_single_conversion_mode(adc_port);
  adc_enable_discontinous_mode_regular(adc_port);
  adc_disable_external_trigger_regular(adc_port);
  adc_set_right_aligned(adc_port);
  /* we want read out the temperature sensor so we have to enable it */
  adc_enable_temperature_sensor(adc_port);
  adc_set_conversion_time_on_all_channels(adc_port, ADC_SMPR_SMP_28DOT5CYC);

  adc_on(adc_port);
  /* wait for adc starting up*/
  for (volatile int i = 0; i < 80000; i++) {}; // wait (volitile removes need for -O0 CFLAGS). */

  adc_reset_calibration(adc_port);
  adc_calibration(adc_port);
}

//--------------------------------------------------------------------------------------------------------------
//TODO this has been copied and pasted from sine code (tidy up!)
u8 STM32Interface::adcchfromport(int command_port, int command_bit) {
  /*
   PA0 ADC12_IN0
   PA1 ADC12_IN1
   PA2 ADC12_IN2
   PA3 ADC12_IN3 - Bus Voltage
   PA4 ADC12_IN4 - Brake POT input.
   PA5 ADC12_IN5
   PA6 ADC12_IN6
   PA7 ADC12_IN7
   PB0 ADC12_IN8
   PB1 ADC12_IN9
   PC0 ADC12_IN10 - IGBT Temperature
   PC1 ADC12_IN11 - U Current
   PC2 ADC12_IN12 - V Current
   PC3 ADC12_IN13 - W Current
   PC4 ADC12_IN14 - Accellerator POT input
   PC5 ADC12_IN15
   temp ADC12_IN16
   */
  switch (command_port) {
  case 0: /* port A */
    if (command_bit<8) return command_bit;
    break;
  case 1: /* port B */
    if (command_bit<2) return command_bit+8;
    break;
  case 2: /* port C */
    if (command_bit<6) return command_bit+10;
    break;
  }
  return 16;
}

//--------------------------------------------------------------------------------------------------------------
/** Used to init the Motor Control libraries*/
int STM32Interface::Init(void) {

  gpioInit();

  //analogue init
  static u8 channel_array[16] = {16,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};
  static u8 channel_array_volt[16] = {16,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};

  //clock_setup(); this is done in the main
  adc_setup(ADC1);

  gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO4);
  channel_array[0] = adcchfromport(2,4); //2=C
  adc_set_regular_sequence(ADC1, 1, channel_array);

  //Setup DC Bus volt analogue (on ADC2) - TODO will need to utilise the ADC more effectively with IFOC
  adc_setup(ADC2);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO3);
  channel_array_volt[0] = adcchfromport(0,3); //0=A
  adc_set_regular_sequence(ADC2, 1, channel_array_volt);

  return 0;
}

//--------------------------------------------------------------------------------------------------------------
// Getter/Setters for the current torque setting (Iq)
signed short STM32Interface::getTorque(void) { //read current value of Torque
	return torque_;
}

void STM32Interface::setTorque(signed short torque) { //Set the target Torque value (used in torque control algorithm)
	torque_ = torque;
}
// Slip
signed short STM32Interface::getSlip(void) { //read current value of Torque
	return slip_;
}

void STM32Interface::setSlip(signed short slip) { //Set the target Torque value (used in torque control algorithm)
	slip_ = slip;
}

// Getter/Setters for the current torque setting (Iq)
signed short STM32Interface::getSpeed(void) { //read current value of Torque
  //TODO
  return 1024;
}
void STM32Interface::setSpeed(signed short) { //Set the target Torque value (used in torque control algorithm)
  //TODO
}

// Getter/Setters for the current flux setting (Id)
signed short STM32Interface::getFlux(void) { //read current value of rotor Flux
  //TODO
  return 1024;
}
void STM32Interface::setFlux(signed short) { //Set the target rotor Flux value (used in torque control algorithm)
  //TODO
}

// Getter/Setters for the current speed (RPM) setting
signed short STM32Interface::getRPM(void) { //read current value of motor RPM
  return (rpm_);
}
void STM32Interface::setRPM(signed short rpm) { //Set the target RPM (speed control loop only)
	rpm_ = rpm;
}

//Is the bus voltage OK? (TODO - more work needed here.  Must expose limits etc...)
bool STM32Interface::busVoltageOK(void) {
  unsigned short busVolt = getRawScaledBusVolt();

  //  check bus over voltage
  if ((busVolt >= TUMANAKO_PRECHARGE_V) && (busVolt <= TUMANAKO_MAX_BUX_V ))
    return true;
  else
    return false;
}

//Get current bus voltage (via a historical 16 reading average)
short STM32Interface::busVoltage(void) {
	//return getRawScaledBusVolt();
	return ((u16)(( (u32)getRawBusVolt() * 1000)/32444));
}

//Get current temperature from power stage
short STM32Interface::powerStageTemperature(void) {
  //TODO
  return 1024;
}

//--------------------------------------------------------------------------------------------------------------
//Contactor controls (See diagram here: http://liionbms.com/php/precharge.php)
bool STM32Interface::getK1() {
  return (gpio_get(TK_K_IN_PORT, TK_K1_IN_PIN) == 0);
}

bool STM32Interface::getK2() {
  return (gpio_get(TK_K_IN_PORT, TK_K2_IN_PIN) == 0);
}

bool STM32Interface::getK3() {
  return (gpio_get(TK_K_IN_PORT, TK_K3_IN_PIN) == 0);
}

void STM32Interface::setK1(bool status) {
  if (status == true) gpio_set(TK_K_OUT_PORT, TK_K1_OUT_PIN);
  else gpio_clear(TK_K_OUT_PORT, TK_K1_OUT_PIN);
}

void STM32Interface::setK2(bool status) {
  if (status == true) gpio_set(TK_K_OUT_PORT, TK_K2_OUT_PIN);
  else gpio_clear(TK_K_OUT_PORT, TK_K2_OUT_PIN);
}

void STM32Interface::setK3(bool status) {
  if (status == true) gpio_set(TK_K_OUT_PORT, TK_K3_OUT_PIN);
  else gpio_clear(TK_K_OUT_PORT, TK_K3_OUT_PIN);
}


//--------------------------------------------------------------------------------------------------------------
//Sanity checks (TODO paramatise this)
void STM32Interface::checkPowerStageLimits() {
  //TODO
}

//Prep motor for start (TODO document details)
void STM32Interface::motorInit() {
  //TODO
}

//Start motor
void STM32Interface::motorStart() {
  TIM1_BDTR |= TIM_BDTR_MOE; //enable TIM1 main outputs
  }

//Start motor
void STM32Interface::motorStop() {
	TIM1_BDTR &= (uint16_t)~(TIM_BDTR_MOE);
  }

//test to be executed in the main loop when motor is running (TODO document details)
void STM32Interface::motorTestForSpeedError() {
  //TODO see motorStart logic
}

//Shutdown motor control and power stage
void STM32Interface::shutdownPower() {
  //Disable the PWM main outputs
  TIM1_BDTR &= (uint16_t)~(TIM_BDTR_MOE); //main output enable OFF

  //Explicitly disconnect contactors
  setK1(false);
  setK2(false);
  setK3(false);
}
//--------------------------------------------------------------------------------------------------------------


