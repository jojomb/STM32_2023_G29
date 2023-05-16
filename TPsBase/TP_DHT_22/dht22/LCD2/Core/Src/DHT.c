/*
 * timer.c
 *
 *  Created on: Feb 13, 2021
 *      Author: JUANB
 */

#include "DHT.h"
//#include "timer.h"
#include "stm32l4xx_hal.h"
#include "tim.h"

void delay (uint16_t delay)//Mise en place d'un timer pour la milliseconde
{
	__HAL_TIM_SET_COUNTER (&htim3, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim3)) < delay);

}

void Data_Output (GPIO_TypeDef *PORT, uint16_t PIN) //direction vers le capteur
  {
  	GPIO_InitTypeDef GPIO_InitStruct = {0};

  	GPIO_InitStruct.Pin = PIN;
  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; //pushpull
  	HAL_GPIO_Init(PORT, &GPIO_InitStruct);
  }

void Data_Input (GPIO_TypeDef *PORT, uint16_t PIN) //direction vers le microcontroleur
  {
  	GPIO_InitTypeDef GPIO_InitStruct = {0};

  	GPIO_InitStruct.Pin = PIN;
  	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  	HAL_GPIO_Init(PORT, &GPIO_InitStruct);
  }

void Read_data (uint8_t *data)
  {
  	int i, k;
  	for (i=0;i<8;i++)
  	{
  		if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET)//
  		{
  			(*data)&= ~(1<<(7-i)); //data bit is 0
  			while(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)));//on attend que le pin passe à haut
  			delay(40);//on attend 40us
  		}
  		else                       //data bit is 1
  		{
  			(*data)|= (1<<(7-i));
  			for (k=0;k<1000;k++)
  			{
  				if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET)
  				  {
  				  	break;
  				  }
  			}
  			while(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)));
  			delay(40);
  		}
  	 }
  }
