
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

#ifndef STM32_INTERFACE_HPP_
#define STM32_INTERFACE_HPP_


//DIGITAL INPUTS
#define TK_IGN_PIN          GPIO7  //Triggers engagement of contactors (Ignition key on)
#define TK_IGN_PORT         GPIOE
#define TK_START_PIN        GPIO10  //Triggers transition to run mode (Start/Run button pushed)
#define TK_START_PORT       GPIOD
#define TK_CRAWL_PIN        GPIO7  //Triggers motor speed to be held slow
#define TK_CRAWL_PORT       GPIOA
#define TK_FWD_PIN          GPIO11  //Triggers motor to rotate in the forward direction
#define TK_FWD_PORT         GPIOD
#define TK_REV_PIN          GPIO8  //Triggers motor to rotate in reverse
#define TK_REV_PORT         GPIOA
#define TK_IGBT_FAULT_PIN   GPIO15  //Combined IGBT fault input
#define TK_IGBT_FAULT_PORT  GPIOE

//DIGITAL OUTPUTS
#define TK_ERROR_LED_PIN    GPIO12  //This output controls the Error LED (Red) on the dashboard
#define TK_ERROR_LED_PORT   GPIOA
#define TK_RUN_LED_PIN      GPIO11  //This output controls the Run LED (Green) on the dashboard
#define TK_RUN_LED_PORT     GPIOA

#define TK_PCB_ERROR_LED_PIN    GPIO6  //This output controls the Error LED (Red) on the PCB
#define TK_PCB_ERROR_LED_PORT   GPIOC
#define TK_PCB_RUN_LED_PIN      GPIO7  //This output controls the Run LED (Green) on the PCB
#define TK_PCB_RUN_LED_PORT     GPIOC

//Outputs used to control contactor
#define TK_K_OUT_PORT GPIOB
#define TK_K1_OUT_PIN GPIO8 //precharge (red)
#define TK_K2_OUT_PIN GPIO9 //+ve 750V (yellow)
#define TK_K3_OUT_PIN GPIO10 //-ve 750V (blue)
//brown = ground

//Contactor state feedback
#define TK_K_IN_PORT GPIOC
#define TK_K1_IN_PIN GPIO10 //precharge
#define TK_K2_IN_PIN GPIO11 //+ve 750V
#define TK_K3_IN_PIN GPIO12 //-ve 750V

// Class definition --------------------------------------------------------------------------------------------
class STM32Interface {
public:
  STM32Interface();

  void sysTickInit();

  //returns the scaled bus voltage direct from a single digital conversion
  unsigned short 	getRawScaledBusVolt();

  //returns the digital bus voltage A/D conversion without SI unit scaling
  unsigned short 	getRawBusVolt();

  //Provide access to various motor control variables
  unsigned short 	getPhaseAOffset();
  unsigned short 	getPhaseBOffset();
  short 			getPhase1();
  short 			getPhase2();
  short 			getPhase3();
  unsigned long 	getRotorTimeConstant();
  void 				setRotorTimeConstant(unsigned long);
  long 				getFrq();
  short 			getFluxAngle();
  short 			getElectricalAngle();

  filter startFilter;
  filter crawlFilter;
  filter ignFilter;
  filter fwdFilter;
  filter revFilter;

  /**IO from the vehcile loom*/

  //Physical INPUT
  bool getIGN(void);  		//Ignition switch on or off?
  bool getRAWStart(void);  	//Raw one off read of Start button digital line (is it engaged or noise though?)
  bool getStart(void);  	//Start button engaged? (same as above, except through a software noise filter)
  bool getCrawl(void);  	//Crawl switch on or off?
  bool getFWD(void);   		//Forward selected?
  bool getREV(void);  		//Reverse selected?
  bool getIGBTFault(void);	// IGBT fault?

  bool getNET(void)  		//Netural (neither FWD or REV selected)?
  {
    return (!getREV() && !getFWD());
  }

  //Physical OUTPUT
  void setErrorLED(bool value); 	// Show red error light on Dash
  void setRunLED(bool value); 		// Show green run light on Dash

  void setPCBErrorLED(bool value);	// Show red error light on PCB
  void setPCBRunLED(bool value);	// Show green run light on PCB

  // Reads the value of the specified ADC channel
  unsigned short readADC(unsigned char channel);  //:TODO: needs to be encapsulated into +ve torque and -ve torque etc

  /** Used to initialise the Motor Control libraries*/
  int Init(void);

  // Getter/Setters for the current torque setting (Iq)
  signed short getTorque(void); //read current value of Torque
  void setTorque(signed short); //Set the target Torque value (used in torque control algorithm)

  // Getters/Setters for the current slip.
  signed short getSlip(void);
  void setSlip(signed short slip);

  // Getter/Setters for the current speed
  signed short getSpeed(void); //read current value of Speed
  void setSpeed(signed short); //Set the target Speed value (not implemented yet)

  // Getter/Setters for the current flux setting (Id)
  signed short getFlux(void); //read current value of rotor Flux
  void setFlux(signed short); //Set the target rotor Flux value (used in torque control algorithm)

  // Getter/Setters for the current speed (RPM) setting
  signed short getRPM(void);  //read current value of motor RPM
  void setRPM(signed short);  //Set the target RPM (speed control loop only - not currently supported by Tumanako)

  //Is the bus voltage OK? (TODO - more work needed here.  Must expose limits etc...)
  bool busVoltageOK(void);

  //Get current bus voltage (via a historical 16 reading average)
  short busVoltage(void);

  //Get current temperature from power stage (via a running average)
  short powerStageTemperature(void);

  //Contactor controls (See diagram here: http://liionbms.com/php/precharge.php)
  bool getK1();  //reads K1 physical contactor feedback
  bool getK2();  //reads K2 physical contactor feedback
  bool getK3();  //reads K3 physical contactor feedback
  void setK1(bool status);
  void setK2(bool status);
  void setK3(bool status);

  /**
  * Returns true if the contactors are safely engaged (according to digital feedback)
  */
  bool getContactorsInRunStateConfiguration()
  {
    if ( (getK2() == true) && (getK3() == true))
      return true;
    else
      return false;
  }

  //Sanity checks, except it takes action internally without interaction with vehicle control layer. Updates motor control internal state machine if fault detected)
  void checkPowerStageLimits();

  //Prep motor for start (init PID loop, encoder buffer, IFOC algorithm variables, Enable PWM outputs and calibrate 3 phase current sensors)
  void motorInit();

  //Start motor (startup checks and switch motor control state machine to GO!)
  void motorStart();

  void motorStop();

  //test to be executed in the main loop when motor is running (checks against motor hardware limits - Max RPM)
  void motorTestForSpeedError();

  //Shutdown motor control and power stage (disable PWM, disconnect contactors, zero IFOC outputs and shutdown motor controller state machine)
  void shutdownPower();

  private:
  	  //Prepare Timer 3 to measure motor temp (KiwiAC STM32MCU board specific - converts KTY84 resistance to freq)
  	  void motorTempSensorInit();

  	  //TODO tidy these methods away
  	  void adc_setup(u32 adc_port);
  	  u8 adcchfromport(int command_port, int command_bit);

  	  void gpioInit(void);

  private:
  	  signed short 	torque_;
  	  signed short 	rpm_;
  	  signed short	slip_;

};

#endif /* STM32_INTERFACE_HPP_ */
