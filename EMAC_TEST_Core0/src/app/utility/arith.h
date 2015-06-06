/*
 * arith.h
 *
 *  Created on: 2014-7-30
 *      Author: wu jm
 */

#ifndef ARITH_H_
#define ARITH_H_

#include "dri_ptp_engine.h"

/**
* \brief Time structure to handle  time information
 */


#pragma inline
void normalizeTime ( TimeInternal *r );


void addTime ( TimeInternal *r, const TimeInternal *x, const TimeInternal *y );

void subTime ( TimeInternal *r, const TimeInternal *x, const TimeInternal *y );


int Filter(int input);

#endif /* ARITH_H_ */
