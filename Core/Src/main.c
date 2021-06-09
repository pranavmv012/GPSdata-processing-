/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * FILE           : main.c
  * PROJECT        : vasudevan_assignment5
  * PROGRAMMER     : Pranav Manakkulamparambil Vasudevan
  * FIRST VERSION  : 2020-04-22
  * DESCRIPTION    : Code for implementing a GPS data retrieval and processing
                     system. Which tokenizes a given gps data and display time,
                     latitude,longitude and altitude.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"//library for printf and scanf
#include "debounce.h"//library for debounce
#include "string.h"//library for string functions
#include "stdlib.h"//standard library for atoi and atof

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char stringBuffer[16]= {0}; // string value for sprintf function for the lcd
//the integers to store the time values
int16_t hour=0;
int16_t minute=0;
float second=0;//float variable to store second
//integers to store the latitude values
int16_t latDeg=0;
int16_t latMin=0;
int16_t latDec=0;
//integers to store the longitude values
int16_t longDeg=0;
int16_t longMin=0;
int16_t longDec=0;
float alti=0;//float variable to store altitude
//sub-strings to store the utc time
 char utcTimehour[10];
 char utcTimeminute[10];
 char utcTimesecond[10];
 //sub-strings to store the latitude
 char latDegree[10];
 char latMinutes[10];
 char latDecimal[10];
 //sub-strings to store the longitude
 char longDegree[10];
 char longMinutes[10];
 char longDecimal[10];

 /*FUNCTION   :void printRetrieve()
  * DESCRIPTION: function for printing the retrieve message on lcd and putty
  * PARAMETERS : Nothing
  * RETURNS    : Nothing
  */
void printRetrieve()
{
	HD44780_Init(); // initializing lcd
	HD44780_PutStr("Retrieving Data..");
	printf("Retrieving Data\r\n");
}
/*FUNCTION   :void printProcess()
  * DESCRIPTION: function for printing the process message on lcd and putty
  * PARAMETERS : Nothing
  * RETURNS    : Nothing
  */
void printProcess()
{
	HD44780_ClrScr(); //clear lcd
	HD44780_PutStr("Processing Data..");
	printf("Processing Data...\r\n");
	HAL_Delay(5000);
}
/*FUNCTION   :void printTask()
  * DESCRIPTION: function for printing the task message on lcd and putty
  * PARAMETERS : Nothing
  * RETURNS    : Nothing
  */
void printTask()
{
	HD44780_ClrScr(); //clear lcd
	HD44780_PutStr("1:Time");
	HD44780_GotoXY(7, 0);
	HD44780_PutStr("2:Latitu");
	HD44780_GotoXY(0, 1);
	HD44780_PutStr("3:Longi");
	HD44780_GotoXY(8, 1);
	HD44780_PutStr("4:Altitu");
	printf("1)Time\t2)Latitu\r\n");
	printf("3)Longi\t4)Altitu\r\n");
}
/*FUNCTION   :void printTask()
  * DESCRIPTION: function for printing the Time is hours,minute
                 and second format on lcd and Putty.
  * PARAMETERS : stringBuffer[16],hour,minute,second
  * RETURNS    : Nothing
  */
void printTime(char stringBuffer[16], int16_t hour, int16_t minute,
		float second)
{
	HD44780_ClrScr(); //clear lcd
	HD44780_PutStr("Time:");
	HD44780_GotoXY(0, 1);
	sprintf(stringBuffer, "%02dh", hour);
	HD44780_PutStr(stringBuffer); // string buffer value
	HD44780_GotoXY(4, 1);
	sprintf(stringBuffer, "%02dm", minute);
	HD44780_PutStr(stringBuffer); // string buffer value
	HD44780_GotoXY(8, 1);
	sprintf(stringBuffer, "%.2fs", second);
	HD44780_PutStr(stringBuffer); // string buffer value
	printf("Time:\r\n%02dh %02dm %.2fs\r\n", hour, minute, second);
}
/*FUNCTION   :void printLatitude()
  * DESCRIPTION: function for printing the latitude in degree,minute and
                 decimal degrees on lcd and putty
  * PARAMETERS : stringBuffer[16],latDeg,latMin,latDec
  * RETURNS    : Nothing
  */
void printLatitude(char stringBuffer[16], int16_t latDeg, int16_t latMin,
		int16_t latDec)
{
	HD44780_ClrScr(); //clear lcd
	HD44780_PutStr("Latitude:");
	HD44780_GotoXY(0, 1);
	sprintf(stringBuffer, "%02dd", latDeg);
	HD44780_PutStr(stringBuffer); // string buffer value
	HD44780_GotoXY(4, 1);
	sprintf(stringBuffer, "%02dm", latMin);
	HD44780_PutStr(stringBuffer); // string buffer value
	HD44780_GotoXY(8, 1);
	sprintf(stringBuffer, "%04ddd", latDec);
	HD44780_PutStr(stringBuffer); // string buffer value
	printf("Latitude:\r\n%02dd %02dm %04ddd\r\n", latDeg, latMin, latDec);
}

/*FUNCTION   :void printLongitude()
  * DESCRIPTION: function for printing the longitude in degree,minute and
                 decimal degrees on lcd and putty
  * PARAMETERS : stringBuffer,longDeg,longMin,longDec
  * RETURNS    : Nothing
  */
void printLongitude(char stringBuffer[16], int16_t longDeg, int16_t longMin,
		int16_t longDec)
{
	HD44780_ClrScr(); //clear lcd
	HD44780_PutStr("Longitude:");
	HD44780_GotoXY(0, 1);
	sprintf(stringBuffer, "%03dd", longDeg);
	HD44780_PutStr(stringBuffer); // string buffer value
	HD44780_GotoXY(5, 1);
	sprintf(stringBuffer, "%02dm", longMin);
	HD44780_PutStr(stringBuffer); // string buffer value
	HD44780_GotoXY(9, 1);
	sprintf(stringBuffer, "%04ddd", longDec);
	HD44780_PutStr(stringBuffer); // string buffer value
	printf("Longitude:\r\n%03dd %02dm %04ddd\r\n", longDeg, longMin, longDec);
}
/*FUNCTION   :void printLongitude()
  * DESCRIPTION: function for printing the altitude in degree,minute and
                 decimal degrees on lcd and putty
  * PARAMETERS : stringBuffer,alti
  * RETURNS    : Nothing
  */
void printAltitude(char stringBuffer[16], float alti)
{
	HD44780_ClrScr(); //clear lcd
	HD44780_PutStr("Altitude:");
	HD44780_GotoXY(0, 1);
	sprintf(stringBuffer, "%.3fmeter", alti);
	HD44780_PutStr(stringBuffer); // string buffer value
	printf("Altitude:\r\n%.3fmeter\r\n", alti);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//setting up GPIO pins
		uint16_t pin= GPIO_PIN_3;//button for 1
		uint16_t pin2= GPIO_PIN_4;//button for 2
		uint16_t pin3= GPIO_PIN_7;//button for 3
		uint16_t pin4= GPIO_PIN_8;//button for 4

		char port= 'A';//setting port
		int8_t mode = 0;//setting mode
		//initializing ports
		int8_t pA3=1;
		int8_t pA4=1;
		int8_t pA7=1;
		int8_t pA8=1;
		//initializing the debounce
		deBounceInit( pin, port, mode);
		deBounceInit2( pin2, port, mode);
		deBounceInit3( pin3, port, mode);
		deBounceInit4( pin4, port, mode);
		//the flag variables to remain in the state
		int16_t flag1=0;
		int16_t flag2=0;
		int16_t flag3=0;
		int16_t flag4=0;

		char stringGps[256];//string for gps data
        char * strData=NULL;//pointer whihch points to the stringGps
	    char *delim=",";//delimeter
	    //strings for tokenization of gps data.
	    char *sentenceId=NULL;
        char *utcTime=NULL;
        char *latitude=NULL;
    	  char *latMeasured=NULL;
    	  char *longitude=NULL;
    	  char *longMeasured=NULL;
    	  char *posFix=NULL;
    	  char *satUsed=NULL;
    	  char *hdop =NULL;
    	  char *altitude=NULL;
    	  char *altUnit=NULL;
    	  char *geoId=NULL;
    	  char *geoUnint=NULL;
    	  char *dgpsAge=NULL;
    	  char *checkSum=NULL;

	int16_t state=0;//argument for switch
	int16_t count=1;//counter for assigning the tokenized string into corresponding string

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(flag1==0)//checking the flag value to run the following code only once
	  {
	  printRetrieve();//print retrieve data
	  scanf( "%s",&stringGps);//scaning gps data
	  printProcess();//print process data
	  //tokenising all possible tokens in the dps data where ',' is the delimeter
	  for (strData = strtok(stringGps, delim); strData; strData = strtok(NULL, delim))
	  {
		  if(count==2)//storing time value
		  {
		    utcTime=strData;
		  }
		  else if (count==3)//storing latitude
		  {
			latitude=strData;
		  }
		  else if (count==5)//storing longitude
		  {
		   longitude=strData;
		  }
		  else if (count==10)//storing altitude
		  {
		   altitude=strData;
		  }
	      count=count+1;//increment count
	  }
	  state=1;//changing state
	  flag1=1;//changing flag value to skip the above code next time

	  }

//swithc with state as argument to get the input from the button and
//show corresponding output
	  switch(state)
	  {

	  case 1:
			printTask();//function call to print task to selected
			state=2;
	  case 2:
    //reading input from all 4 buttons continously until anyone of these is pressed
	pA3 = deBounceReadPin(pin, port, mode);
	pA4 = deBounceReadPin2(pin2, port, mode);
	pA7 = deBounceReadPin3(pin3, port, mode);
	pA8 = deBounceReadPin4(pin4, port, mode);
	if(pA3==0)//when button 1 pressed execute following code
	{
		  printf("utc time is %s\r\n",utcTime);//printing the tokenised string for time
		  //making substrings of individual data using string copying function
		  strncpy(utcTimehour,utcTime,2);//
		  utcTimehour[2]='\0';//setting null character
		  strncpy(utcTimeminute,utcTime+2,2);
		  utcTimeminute[2]='\0';//setting null character
		  strncpy(utcTimesecond,utcTime+4,5);
          utcTimesecond[5]='\0';//setting null character
          hour=atoi(utcTimehour);//to conver to integer
          minute=atoi(utcTimeminute);//to conver to integer
          second=atof(utcTimesecond);//convert the string into float
		  printTime(stringBuffer, hour, minute, second);//function call to print time data
          state=1;//state chage
      	  HAL_Delay(5000);
          break;
	}

	else if(pA4==0)//when button 2 pressed execute following code
	{
		  printf("latitude is %s\r\n",latitude);
		  //making substrings of individual data using string copying function
		  strncpy(latDegree,latitude,2);
		  latDegree[2]='\0';
		  strncpy(latMinutes,latitude+2,2);
		  latMinutes[2]='\0';
		  strncpy(latDecimal,latitude+5,4);
		  latDecimal[4]='\0';
          latDeg=atoi(latDegree);//to convert to integer
          latMin=atoi(latMinutes);//to convert to integer
          latDec=atoi(latDecimal);//to convert to integer
		  printLatitude(stringBuffer, latDeg, latMin, latDec);//function call to print latitude data
	      state=1;//state change
      	  HAL_Delay(5000);
	      break;
	}
	else if(pA7==0)//when button 3 pressed execute following code
	{
	 printf("longitude is %s\r\n",longitude);
	 //making substrings of individual data using string copying function
	 strncpy(longDegree,longitude,3);
	 longDegree[3]='\0';
	 strncpy(longMinutes,longitude+3,2);
	 longMinutes[2]='\0';
	 strncpy(longDecimal,longitude+6,4);
	 longDecimal[4]='\0';
	 longDeg=atoi(longDegree);//to convert to integer
	 longMin=atoi(longMinutes);//to convert to integer
     longDec=atoi(longDecimal);//to convert to integer
	 printLongitude(stringBuffer, longDeg, longMin, longDec);//function call to print longitude data
	 state=1;//state chage
	 HAL_Delay(5000);
	 break;
	}
	else if(pA8==0)//when button 4 pressed execute following code
	{
	 printf("altitude is %s\r\n",altitude);
	 alti=atof(altitude);//convert the string into float
	 printAltitude(stringBuffer, alti);
	 state=1;//state chage
	 HAL_Delay(5000);
	 break;
	}
	state=2;//calling the same state
	break;
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|LD3_Pin|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA4 PA7 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 LD3_Pin PB4 
                           PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|LD3_Pin|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
