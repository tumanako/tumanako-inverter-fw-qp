
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
#ifndef PID_HPP_
#define PID_HPP_

class PID
{
   public:
      PID();
      int 	DoPid(float Sp, float Fb);
      void	InitPid(float Kp, float Ki, float Kd, float Ko, float MaxOutput);

   private:
      float Setpoint;
      float Feedback; 	// Current position
      float	Kp;
      float	Ki;
      float	Kd;
      float	Ko; 		// Output Scale
      float	MaxOutput;	// For example = 0x7f for 8-bit output, usually 1/2 fullscale
      float	PrevErr;
      float	Ierror;
};

#endif /* PID_HPP_ */
