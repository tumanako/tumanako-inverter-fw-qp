//------------------------------------------------------------------------------
//  Tumanako_QP - Electric Vehicle and Motor control software
//
//  Copyright (C) 2010 Graeme Bell <graemeb@users.sourceforge.net>
//  Copyright (C) 2012 Bernard Mentink <bmentink@gmail.com>
//
//  This file is part of Tumanako_QP.
//
// 	This software may be distributed and modified under the terms of the GNU
// 	General Public License version 2 (GPL) as published by the Free Software
// 	Foundation and appearing in the file GPL.TXT included in the packaging of
// 	this file. Please note that GPL Section 2[b] requires that all works based
// 	on this software must also be made publicly available under the terms of
// 	the GPL ("Copyleft").
//
//   TumanakoVC is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//
// DESCRIPTION:
//   This file provides standalone functions for implementing Clarke and Park
//   transforms. A highlevel implementation suitable for initial use & testing,
//   but may require subsequent optimisiation for the final hardware.
//
// HISTORY:
//   Graeme Bell 1/May/2001 - First Cut
//   Bernard Mentink 28/3/2012 - Changed to C++ class, added to Tumanako_QP project
//------------------------------------------------------------------------------

#ifndef MATHS_CLARKEPARKTRANSFORMS_H
#define MATHS_CLARKEPARKTRANSFORMS_H

#include "bsp.h"

    
namespace Maths {

//---------Constructor------------------------------------------------------------------------------------------
	Transform::Transform()
	{}

	//-------------------------------------------------------------------------
	// Combined Clarke & Park Transformations

	inline tTwoPhaseDQ Transform::ClarkePark( tReal angle, const tThreePhase& in ) {
		return Park( angle, Clarke( in ) );
	}
	inline tThreePhase Transform::InvClarkePark( tReal angle, const tTwoPhaseDQ& in ) {
		return InvClarke( InvPark( angle, in ) );
	}

    //-------------------------------------------------------------------------
    // Park Transformations
    
    inline tTwoPhaseDQ Transform::Park( const tCosSin& angle, const tTwoPhase& in ) {
        return tTwoPhaseDQ( 
             in.alpha*angle.cos + in.beta*angle.sin, 
            -in.alpha*angle.sin + in.beta*angle.cos );
    }
    inline tTwoPhaseDQ Transform::Park( tReal angle, const tTwoPhase& in ) {
        return Park( tCosSin(angle), in );
    }
    
    inline tTwoPhase Transform::InvPark( const tCosSin& angle, const tTwoPhaseDQ& in ) {
        return tTwoPhase(
             in.d*angle.cos - in.q*angle.sin, 
             in.d*angle.sin - in.q*angle.cos );
    }
    inline tTwoPhase Transform::InvPark( tReal angle, const tTwoPhaseDQ& in ) {
        return InvPark( tCosSin(angle), in );
    }

    //-------------------------------------------------------------------------
    // Clarke Transformations
    
    inline tTwoPhase Transform::Clarke( const tThreePhase& in ) {
        // note: this assumes in.a+in.b+in.c == 0
        return tTwoPhase( in.a, in.a*CONST_1_SQRT3 + in.b*CONST_2_SQRT3 );
    }
    inline tThreePhase Transform::InvClarke( const tTwoPhase& in ) {
        tReal r1 = -in.alpha/2;
        tReal r2 = in.beta*CONST_SQRT3_2;
        return tThreePhase( in.alpha, r1 + r2, r1 - r2 );
    }

//-----------------------------------------------------------------------------
    
}  // namespace Maths

#endif  // inclusion guard
