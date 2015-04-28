/*
 * servo.c
 *
 *  Created on: 2015-3-11
 *      Author: Administrator
 */


#include "servo.h"

//PIservo PI={20000,0, 0,0,1.99915,2, 0,0,1 };
//PIservo PI={20000,0, 0,0,0.99915,0.03, 0,0,1 };//for EZ-KIT version
//PIservo PI={20000,0, 0,0,0.99915,0.03, 0,0,1 }; //2014-12-06
PIservo PI={20000,0, 0, 0, 0.9995, 0.55, 0, 1 };
offset_from_master_filter  ofm_filt = {0,0};


double
runPIservo ( PIservo *servo, const int input )
{

	if ( servo->dt <= 0.0 )
		servo->dt = 1.0;

	servo->input = input;

	if ( servo->kP < 0.000001 )
		servo->kP = 0.000001;

	if ( servo->kI < 0.0000001 )
		servo->kI = 0;

	servo->observedDrift += servo->dt * ( ( input + 0.0 ) * servo->kI );

//	if ( servo->observedDrift >= servo->maxOutput )
//	{
//		servo->observedDrift = servo->maxOutput;
//		servo->runningMaxOutput = 1;
//	}
//
//	else if ( servo->observedDrift <= -servo->maxOutput )
//	{
//		servo->observedDrift = -servo->maxOutput;
//		servo->runningMaxOutput = 1;
//	}
//
//	else
//	{
//		servo->runningMaxOutput = 0;
//	}

	servo->output = ( servo->kP * ( input + 0.0 ) ) + servo->observedDrift;


	return -servo->output;
}

int
calcAdjAddend ( unsigned int addend, double adj, double dT )
{

	int  deltaAddend;
	double dAddend;

	dAddend= adj*addend/(dT*1000000000);

	deltaAddend  = (int) (dAddend+0.5);

	return deltaAddend;
}
