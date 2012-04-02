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

#ifndef CLARKEPARKTRANSFORMS_HPP_
#define CLARKEPARKTRANSFORMS_HPP_

//--- Includes -----------------------------------------------------------------
#include <cmath>

//--- Defines ------------------------------------------------------------------
#define CONST_SQRT3_  1.732050807568877293527446315059
#define CONST_1_SQRT3 tReal(1/CONST_SQRT3_)
#define CONST_2_SQRT3 tReal(2/CONST_SQRT3_)
#define CONST_SQRT3_2 tReal(CONST_SQRT3_/2)


namespace Maths
{
	//-------------------------------------------------------------------------
	//

	// typedef fixed< int32_t, 16 > tReal
	typedef float tReal;

	/**
	 * Simple structure for representing values in a three-phase system.
	 */
	struct tThreePhase {
		tThreePhase( tReal ra, tReal rb, tReal rc ) : a(ra), b(rb), c(rc) {}

		tReal  a, b, c;
	};

	/**
	 * Simple structure for representing values in a two-phase orthogonal
	 * system.
	 */
	struct tTwoPhase {
		tTwoPhase( tReal ralpha, tReal rbeta ) : alpha(ralpha), beta(rbeta) {}

		tReal  alpha, beta;
	};

	/**
	 * Simple structure for representing values in a two-phase system with a
	 * rotating frame of reference.
	 */
	struct tTwoPhaseDQ {
		tTwoPhaseDQ( tReal rd, tReal rq ) : d(rd), q(rq) {}

		tReal  d, q;
	};

	/**
	 * Simple structure for holding the sin & cos of an angle. This is to allow
	 * the values to be calculated just once for the current rotor angle and
	 * then be used in multiple transforms.
	 */
	struct tCosSin {
		tCosSin( tReal radians ) : cos( std::cos(radians) ), sin( std::sin(radians) ) {}

		tReal  cos;
		tReal  sin;
	};

	class Transform
	{
		public:
		  Transform(void);
		  inline tTwoPhaseDQ ClarkePark( tReal angle, const tThreePhase& in );
		  inline tThreePhase InvClarkePark( tReal angle, const tTwoPhaseDQ& in );

		private:
		  inline tTwoPhaseDQ Park( const tCosSin& angle, const tTwoPhase& in );
		  inline tTwoPhaseDQ Park( tReal angle, const tTwoPhase& in );
		  inline tTwoPhase InvPark( const tCosSin& angle, const tTwoPhaseDQ& in );
		  inline tTwoPhase InvPark( tReal angle, const tTwoPhaseDQ& in );

		  inline tTwoPhase Clarke( const tThreePhase& in );
		  inline tThreePhase InvClarke( const tTwoPhase& in );
	};
} /* end  namespace */



#endif /* CLARKEPARKTRANSFORMS_HPP_ */
