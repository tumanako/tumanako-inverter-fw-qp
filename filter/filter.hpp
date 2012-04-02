
//------------------------------------------------------------------------------
//   Tumanako_QP - Electric Vehicle and Motor control software
//   Copyright (C) 2010 Philip Court <philip@greenstage.co.nz>
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
//
// DESCRIPTION:
//   Filters digital data by storing historical data in a buffer. Implements
//   a simple only ON if all ON type logic. 
//
// HISTORY:
//   Philip Court 15/Nov/2010 - First Cut
//   Bernard Mentink 10/Mar/2012 - changed memory allocation to static.
//------------------------------------------------------------------------------

#ifndef __FILTER_H
#define __FILTER_H

//CONSTANTS
#define TK_FILTER_SIZE 5 			//increase to make digital signal software more tolerant of noise

// Used to filter noisy digital lines.  Will only
// return true if line has been continuously on for
// the given number of values above
class filter {

private:
  bool 				*buffer;
  unsigned short 	index;

public:

  //Constructor, specifies the size of the filter
  filter (void) {
    index=0;
    // NOT using dynamic allocation now , only static, much more deterministic in embedded systems!! ..
    // Hey! this is a CAR ... lives are at stake! ....
    static bool buffer[TK_FILTER_SIZE];	// TODO does this need to be static?

    //initialise
    for (unsigned short i=0; i<TK_FILTER_SIZE; i++)
      buffer[i] = false;
  }

  //Destructor not implemented (this class does not expect to be destructed, i.e. it will be used until the power is turned off!)

  //Store data in the filters buffer
  void store(bool data) {
    buffer[index] = data;
    if (++index == TK_FILTER_SIZE) index = 0; //increment and loop back around
  }

  //Get the result (true if all data in the buffer is true, otherwise false)
  bool result() {
    for (unsigned short i=0; i<TK_FILTER_SIZE; i++) {
      if (buffer[i] == false) return false;
    }
    return true;
  }

  //Returns the percentage of values in the buffer that don't match the actual reported result (i.e. noise)
  unsigned short percentageNoise() {
    unsigned short count = 0;
    bool actualResult = result();
    for (unsigned short i=0; i<TK_FILTER_SIZE; i++) {
      if (buffer[i] != actualResult) count++;
    }
    return (100*count/TK_FILTER_SIZE);
  }
};

#endif  //__FILTER_H
