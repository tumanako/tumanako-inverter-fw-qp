
//------------------------------------------------------------------------------
//   Tumanako_QP - Electric Vehicle and Motor control software
//
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

#ifndef SERIAL_HPP_
#define SERIAL_HPP_

// Private defines
#define TK_USART_PORT		USART1
#define TK_TxBufferSize   	(countof(TxBuffer) - 1)

// Private macros
#define countof(a)   (sizeof(a) / sizeof(*(a)))


class Serial {
   public:
      Serial(void);
      void Init(void);
      void printFormat( const char* Format, ... );
      void usartWriteDisclaimer(void);
      void usartWriteChars(char const * chars);
      void printf (const char *fmt,...);

   private:
      void usartWrite(long a, long b, long c, long d, long e);
};



#endif /* SERIAL_HPP_ */
