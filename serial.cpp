
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
//
// DESCRIPTION:
//   This module setups up the serial port for diognostic output and it 
//   provides a few helper functions.
//
//------------------------------------------------------------------------------


#include "bsp.h"

Q_DEFINE_THIS_FILE

using namespace std;

//---------Object Declaration-----------------------------------------------------------------------------------
static Serial l_Serial;       			// the sole instance of this module

Serial * const O_serial = &l_Serial;   	// opaque pointer to Serial object.
//--------------------------------------------------------------------------------------------------------------

// Private variables
char TxBuffer[] = \
                  " \f\x1b\x5b\x48\x1b\x5b\x32\x4a\r\n";

//--------------------------------------------------------------------------------------------------------------
Serial::Serial()

{ }

//--------------------------------------------------------------------------------------------------------------
void Serial::Init(void) {
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

  /* Setup UART parameters. */
  usart_set_baudrate(USART1, 115200);
  //usart_set_baudrate(USART1, 38400);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX_RX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

  /* Finally enable the USART. */
  usart_enable(USART1);
}

//--------------------------------------------------------------------------------------------------------------
void Serial::usartWriteDisclaimer(void) {
  printf("%s",TxBuffer);
  printf( "Version: %s SKiip KiwiAC\n\r",TUMANKAO_VERSION);
}


//--------------------------------------------------------------------------------------------------------------
void Serial::printf (const char *fmt,...)
{
	va_list ap;
	char s[120];
	char *p_char = s;

	va_start(ap, fmt);
	vsprintf(s, fmt,ap);
	va_end(ap);
	while(0 != *p_char)
	{
		usart_send_blocking(USART1, *p_char++);
	}
}


