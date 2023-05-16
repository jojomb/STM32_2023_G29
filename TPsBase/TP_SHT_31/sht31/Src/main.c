/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h" //include de fachier .h
#include "i2c.h"//include de fachier .h de I2C
#include "usart.h"//include de fachier .h
#include "gpio.h"//include de fachier .h de GPIO

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lib_lcd.h" //include le fichier lib_lcd pour l'affichage
#define _OPEN_SYS_ITOA_EXT
//#include <stdlib.h>
//#include <string.h>
//#include <stdio.h>



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static const uint8_t CAPTEUR_ADRS = 0x44 << 1;//adresse de capteur
static const uint8_t CAPTEUR_CMD_MSB = 0x2C;//commande  avec horloge enable et haute Répétabilité pour le capteur
static const uint8_t CAPTEUR_CMD_LSB = 0x06;// paramètres des bits de poids faibles de capteur
static rgb_lcd lcdData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	HAL_StatusTypeDef ret;
		uint8_t buf[12];//tableau dans lequel on met les octets de capteurs
		volatile uint16_t value;//variable contient la valeur de capteur

		volatile float temp;//variable contient la valeur de la température
		float umid;//variable contient la valeur de la température
		float Decimal_part;// variable contient la valeur dicimale
		float Entier_part;//variable contient la valeur entier

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------
   * ------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();//initialisation des périphériques

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();//initialisation de I2C
  /* USER CODE BEGIN 2 */

    lcd_init(&hi2c1,&lcdData);
    lcd_position(&hi2c1,0,0);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* USER CODE END WHILE */
	  	  buf[0] = CAPTEUR_CMD_MSB;//on met les paramètres de capteur pour le mode de fonctionnement
	  	  		buf[1] = CAPTEUR_CMD_LSB;//on met la valeur de l'octet de paramètres de capteur
	  	  		ret = HAL_I2C_Master_Transmit( &hi2c1, CAPTEUR_ADRS, buf, 2, HAL_MAX_DELAY);//envoie des octets via I2C
	  	  		if ( ret != HAL_OK)
	  	  		{
	  	  			strcpy((char*)buf, "erreur_T!!\r\n");
	  	  		}
	  	  			else
	  	  		{

	  	  		ret = HAL_I2C_Master_Receive( &hi2c1, CAPTEUR_ADRS, buf, 6, HAL_MAX_DELAY);//réception des octets
	  	  					if ( ret != HAL_OK)
	  	  				{
	  	  					strcpy((char*)buf, "erreur_R!!\r\n");
	  	  				}
	  	  		else
	  	  		{

	  	  			value  =   buf[1] | buf[0] << 8;// le variable value contient la valeur des octets des MSB et LSB de capteur avec un décalge vers la gauche


	  	  			temp = -45 + 175 * ( (float)value / 65535);// on calcule la valeur de la température

	  	  			Entier_part = (int) temp;// récuperation de la partie dicimale de la température
	  	  			Decimal_part = temp;// récuperation de la partie dicimale de la température
	  	  			Decimal_part *= 100;
	  	  			Decimal_part = Decimal_part - (Entier_part * 100);// calcule de la partie dicimale de la température
	  	  			value = buf[4] | buf[3] << 8;

	  	  			umid = -49 + 315 *( (float)value / 65535);
	  	  			sprintf( (char*)buf, "%u.%u C ; %u D", (unsigned int) Entier_part,(unsigned int) Decimal_part,(unsigned int) umid  );
	  	  			lcd_position(&hi2c1,0,0);
	  	  			lcd_print(&hi2c1,"T =  ");//affichage de la température
	  	  			lcd_position(&hi2c1,7,0);
	  	  			lcd_print(&hi2c1,buf);//affichage de la valeur de la  température
	  	  			lcd_position(&hi2c1,0,1);

	  	  			lcd_print(&hi2c1,"H = ");//affichage de l'humidité
	  	  			lcd_position(&hi2c1,7,1);
	  	  			lcd_print(&hi2c1,&buf[10]);//affichage de la valeur de l'humidité

	  	  		}
	  	  		}
	  	  		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);//transmission des octets
	  	  		HAL_Delay(1000);




	      /* USER CODE BEGIN 3 */
	    }
	    /* USER CODE END 3 */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
