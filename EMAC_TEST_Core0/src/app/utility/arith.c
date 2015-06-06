/*
 * arith.c
 *
 *  Created on: 2014-12-1
 *      Author: Wu jm
 */

#include "arith.h"

#pragma inline
void
normalizeTime ( TimeInternal *r )
{
	r->seconds += r->nanoseconds / 1000000000;
	r->nanoseconds -= ( r->nanoseconds / 1000000000 ) * 1000000000;

	if ( r->seconds > 0 && r->nanoseconds < 0 )
	{
		r->seconds -= 1;
		r->nanoseconds += 1000000000;
	}

	else if ( r->seconds < 0 && r->nanoseconds > 0 )
	{
		r->seconds += 1;
		r->nanoseconds -= 1000000000;
	}
}

void addTime ( TimeInternal *r, const TimeInternal *x, const TimeInternal *y )
{
	r->seconds = x->seconds + y->seconds;
	r->nanoseconds = x->nanoseconds + y->nanoseconds;

	normalizeTime ( r );
}

void subTime ( TimeInternal *r, const TimeInternal *x, const TimeInternal *y )
{
	r->seconds = x->seconds - y->seconds;
	r->nanoseconds = x->nanoseconds - y->nanoseconds;

	normalizeTime ( r );
}



#define FILTER_LENGTH 4

typedef struct Average_FILTER
{
	int Counter;
	int PreValue;
	int y;
}Average_FILTER;

Average_FILTER ofm_filter={0};



int Filter(int input)
{
	int  deltaVal,AbsDeltaVal,AbsPreVal;
	int  ret;
	static int s_Counter = 0;

	if(ofm_filter.Counter == 0)
	{
		ofm_filter.PreValue = input;
		ret = input;
		ofm_filter.Counter++;
	}
	else
	{
		deltaVal = ofm_filter.PreValue - input;
		AbsDeltaVal = (deltaVal > 0)?deltaVal:-deltaVal;

		AbsPreVal = ofm_filter.PreValue>0 ? ofm_filter.PreValue:-ofm_filter.PreValue;

		if(AbsPreVal < 100 && AbsDeltaVal > 200)
		{
			ret = ofm_filter.PreValue ;
			s_Counter++;
			if(s_Counter > 2)
			{
				ofm_filter.Counter = 0;
				s_Counter = 0;
			}
		}
		else
		{
			ofm_filter.PreValue = input;
			ret = input;
			s_Counter = 0;
		}

	}

	return ret;
}
