/*
 * servo.h
 *
 *  Created on: 2014-7-30
 *      Author: Administrator
 */

#ifndef SERVO_H_
#define SERVO_H_
typedef struct
{
	int maxOutput;
	int input;
	double output;
	double observedDrift;
	double kP;//比例因子
	double kI;//积分因子
	bool runningMaxOutput;
	double dt;

} PIservo;

extern PIservo PI;

/**
* \brief Struct used to average the offset from master
*
* The FIR filtering of the offset from master input is a simple, two-sample average
 */
typedef struct
{
	int  nsec_prev; //前一次
	int  y;          //当前输出
} offset_from_master_filter;

extern offset_from_master_filter  ofm_filt;


double runPIservo ( PIservo *servo, const int input );

int calcAdjAddend ( unsigned int addend, double adj, double dT );
#endif /* SERVO_H_ */
