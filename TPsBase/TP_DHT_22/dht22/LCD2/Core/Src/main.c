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
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "DHT.h"

#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lib_lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static rgb_lcd lcdData;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

//DHT22_TypeDef dht22;
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*void delay (uint16_t delay)
{
	__HAL_TIM_SET_COUNTER (&htim3, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim3)) < delay);

}*/
/*
void Display_Temp (float Temp)
{
	char str[20] = {0};
	lcd_put_cur(0,0);

	sprintf(str,"TEMP:- %.2f ", Temp);
	lcd_send_string(str);
	lcd_send_data('C');
}*/

/*uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH = 0, TEMP = 0;


//DHT_DataTypedef DHT22_Data;
float Temperature, Humidity;
uint8_t Presence = 0;*/

float Temperature = 0, Humidite = 0;
uint16_t RH = 0, TEMP = 0;
uint8_t dataH1;
uint8_t dataH2;
uint8_t dataT1;
uint8_t dataT2;
uint8_t SUM;
char bufRH[20]; //pour stocker une valeur et l'afficher sur le LCD
char bufT[20];  //pour stocker une valeur et l'afficher sur le LCD

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
int k = 0; //variable de boucle

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
lcd_init(&hi2c1, &lcdData);
lcd_position(&hi2c1,0,0);
lcd_print(&hi2c1, "Bienvenue");//on affiche une chaîne de caractère à l'allumage
HAL_Delay(2000);//attente
clearlcd(); //on efface

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//initialisation

	  	  Data_Output(GPIOA, GPIO_PIN_8); //on définit la broche comme sortie
	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); //On tire la broche vers le bas
	  	  delay(1200); //on attend environ minimum 1 ms
	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); //On tire la broche vers le haut
	  	  delay(30); //on attend 30 us
	  	  Data_Input(GPIOA, GPIO_PIN_8); //on définit la broche comme entrée pour recevoir les données 

	  	  /*commence la reception de donnees*/

	  	  while(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)));

	  	  for (k=0;k<1000;k++)
	  	  {
	  		  if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET)//si le pin est en mode haut
	  		  {
	  	  	  	break;
	  	  	  }
	  	  }

	  	  while(!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8))); //on attend que le pin passe en mode haut
	  	  delay(40);//on attend 40 us

	  	  Read_data(&dataH1); //on récupère les adresses stockées
	  	  Read_data(&dataH2);
	  	  Read_data(&dataT1);
	  	  Read_data(&dataT2);
	  	  Read_data(&SUM);

//Affichage
	  	  if (SUM==((dataH1+dataH2+dataT1+dataT2) & 0x00FF))
	  	  {
	  	 Humidite = (((dataH1<<8) | dataH2)*0.1f);
	  	 Temperature = (((dataT1<<8) | dataT2)*0.1f);
	  	  }



	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); //on repasse la broche à haut pour la prochaine lecture

	  	  /*commence transmission vers LCD*/
	  	  clearlcd();
	  	  sprintf(bufRH,"Humidite: %.1f %", Humidite);
	  	  sprintf(bufT, "Temp.: %.1f C", Temperature);
	  	  lcd_position(&hi2c1,0,0);
	  	  lcd_print(&hi2c1,bufRH);
	  	  lcd_position(&hi2c1,0,1);
	  	  lcd_print(&hi2c1,bufT);
	  	  reglagecouleur(0,0,255);

	
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
