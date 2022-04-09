/*
 * STM_registers.h
 *
 *  Created on: Apr 9, 2022
 *      Author: norbe
 */

#ifndef STM_REGISTERS_H_
#define STM_REGISTERS_H_

#include "ov2640.h"

#define RES_STM160x120 (160 * 120 * 3)
#define RES_STM320x240 (320 * 240 * 3)
#define RES_STM480x272 (480 * 272 * 3)
#define RES_STM640x480 (640 * 480 * 3)

void STM_OV2640_ResolutionConfiguration(unsigned opt);
#endif /* STM_REGISTERS_H_ */
