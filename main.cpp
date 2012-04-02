
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

static QEvent const * l_prechargeQueueSto[5];
static QEvent const * l_temperatureQueueSto[5];
static QEvent const * l_inverterQueueSto[5];
static QEvent const * l_ledQueueSto[5];
static QEvent const * l_scanQueueSto[5];
static QEvent const * l_slipQueueSto[5];
static QEvent const * l_dashboardQueueSto[2];

static QSubscrList   l_subscrSto[MAX_PUB_SIG];

// Event pools
static motorTempEvt 	l_smlPoolSto[10];    	/* storage for the small event pool */
static LedEvt 			l_medPoolSto[10];       /* storage for the medium event pool */

int main()
{
	BSP_init();       // initialize the Board Support Package

	QF::init();       // initialize the framework and the underlying RT kernel

	QF::psInit(l_subscrSto, Q_DIM(l_subscrSto));     // init publish-subscribe

    // initialize the event pools...
	QF::poolInit(l_smlPoolSto, sizeof(l_smlPoolSto), sizeof(l_smlPoolSto[0]));
	QF::poolInit(l_medPoolSto, sizeof(l_medPoolSto), sizeof(l_medPoolSto[0]));

	// Start up all the active objects.
	AO_Inverter->start(6,                                           // priority
					l_inverterQueueSto, Q_DIM(l_inverterQueueSto),// evt queue
					(void *)0, 0,                 // no per-thread stack
					(QEvent *)0);                 // no initialization event */
	AO_PreCharge->start(2,                                           // priority
					l_prechargeQueueSto, Q_DIM(l_prechargeQueueSto),      // evt queue
					(void *)0, 0,                 // no per-thread stack
					(QEvent *)0);                 // no initialization event */
	AO_Temperature->start(3,                                           // priority
					l_temperatureQueueSto, Q_DIM(l_temperatureQueueSto),  // evt queue
					(void *)0, 0,                 // no per-thread stack
					(QEvent *)0);                 // no initialization event */
	AO_Scan->start(4,                                           // priority
					l_scanQueueSto, Q_DIM(l_scanQueueSto),  // evt queue
					(void *)0, 0,                 // no per-thread stack
					(QEvent *)0);                 // no initialization event */
	AO_Led->start(5,                                           // priority
					l_ledQueueSto, Q_DIM(l_ledQueueSto),  // evt queue
					(void *)0, 0,                 // no per-thread stack
					(QEvent *)0);                 // no initialization event
	AO_Slip->start(1,                                           // priority
					l_slipQueueSto, Q_DIM(l_slipQueueSto),  // evt queue
					(void *)0, 0,                 // no per-thread stack
					(QEvent *)0);                 // no initialization event
	AO_DashBoard->start(7,                                           // priority
					l_dashboardQueueSto, Q_DIM(l_dashboardQueueSto),  // evt queue
						(void *)0, 0,                 // no per-thread stack
						(QEvent *)0);                 // no initialization event

	QF::run();	// go run the OS!!

  return 0;
}
  
