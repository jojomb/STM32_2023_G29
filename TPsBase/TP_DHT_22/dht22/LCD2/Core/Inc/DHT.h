/*
 * timer.h
 *
 *  Created on: Feb 13, 2021
 *      Author: JUANB
 */

#include "stm32l4xx_hal.h"

#ifndef INC_DHT_H_
#define INC_DHT_H_

void delay (uint16_t delay);
void Data_Output (GPIO_TypeDef *PORT, uint16_t PIN);
void Data_Input (GPIO_TypeDef *PORT, uint16_t PIN);
void Read_data (uint8_t *data);

#endif /* INC_DHT_H_ */
