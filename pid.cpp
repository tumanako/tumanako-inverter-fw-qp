
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
#include "bsp.h"


//---------Constructor------------------------------------------------------------------------------------------
PID::PID() :
		PrevErr(0),
		Ierror(0)
{}

// ------------------------------------------ PID loop functions ----------------------------------------------

void PID::InitPid(float p, float i, float d, float o, float MaxOut) {
	Kp = p;
	Ki = i;
	Kd = d;
	Ko = o;
	MaxOutput = MaxOut;
}

//--------------------------------------------------------------------------------------------------------------
int PID::DoPid(float Sp, float Fb)
{
    float Perror, output;

    Setpoint = Sp; Feedback = Fb;

    // Calculate proportional error
    Perror = Setpoint - Feedback;

    // Derivative error is the delta Perror over Td (rate at which this function executes)
    output = (Kp*Perror + Kd*(Perror - PrevErr) + Ki*Ierror)/Ko;
    PrevErr = Perror;

    // Accumulate Integral error *or* Limit output.
    // Stop accumulating when output saturates
    if (output >= MaxOutput)
        output = MaxOutput;
    else if (output <= -MaxOutput)
        output = -MaxOutput;
    else
        Ierror += Perror;
    //Offset output to positive only, MaxOuput usually offset to half way .. 0x7f for 8 bit out.
    //return (output+(MaxOutput)  );
    return (output);
}
