/*
 * phaseouts.h
 *
 *  Created on: Apr 22, 2020
 *      Author: Alka
 */

#ifndef INC_PHASEOUTS_H_
#define INC_PHASEOUTS_H_

#include "main.h"

void allOff(void);
void comStep (int newStep);
void fullBrake(void);
void allpwm(void);
void proportionalBrake(void);
void twoChannelForward(void);
void twoChannelReverse(void);

void phaseAPWM(void);
void phaseBPWM(void);
void phaseCPWM(void);
#endif /* INC_PHASEOUTS_H_ */
