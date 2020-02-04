/*
 * FOC.h
 *
 *  Created on: Oct 1, 2019
 *      Author: LoganRosenmayer
 */

#ifndef FOC_H_
#define FOC_H_

#include "stm32h7xx_hal.h"


int rawdata_to_angle(int rawdata);
void parkclark(int Ia,int Ib,int Ic,int theta,float *Iq,float *Id );
void inv_parkclark(int *Va,int *Vb,int *Vc,int theta,int Vq,int Vd );
float sin_lut(int angle);
float cos_lut(int angle);

#endif /* FOC_H_ */
