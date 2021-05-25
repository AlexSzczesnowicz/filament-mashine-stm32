/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ILI9341_GFX.h"
#include "fonts.h"
#include "img.h"
#include "xpt2046_touch.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void startup(void);
_Bool touch_work(void);
void main_cicle(void);
char buf[64] = {0,};
void draw_rect_tol (uint16_t kx1,uint16_t ky1,uint16_t kx2,uint16_t ky2,uint16_t fon,uint16_t tol);
uint8_t button_click(void);
void ADC_out(void);
void ADC_SetActiveChannel(ADC_HandleTypeDef *hadc, uint32_t AdcChannel);
void ADC_Select_CH0 (void);
void ADC_Select_CH1 (void);
uint32_t adcBuffer[2];
uint8_t flag_press = 1;
uint32_t time_press = 0;
int licznik = 0;
int adc_sum[2];
int adc_buf[2];
void adc_show(void);
uint8_t flag_hold = 1;
uint32_t timme_hold = 0;
void button_draw_icons(void);
void curent_state_raport(void);
void pneumo_start(void);
uint16_t t_installed[2] = {60,55 };
uint16_t t_readed[2] = {0,0};
_Bool menu_position[10] = {0,0,0,0,0,0,0,0,0,0};
void heaters_start(void);
void heaters_stop(void);
void perekluchatel(int passed);
int plus_minus_addon(int passed);
uint16_t x = 0;
uint16_t y = 0;
uint16_t buttons_coord[10][4]={
		{20,20,75,50},
		{95,20,150,50},
		{170,20,225,50},
		{245,20,300,50},
		{200,80,300,120},
		{200,140,300,180},


		{10,90,60,120},
		{70,90,120,120},
		{10,130,60,160},
		{70,130,120,160}		//test buttons

};


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
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
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  startup();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  main_cicle();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TFT_RST_Pin|TOUCH_CS_Pin|TFT_DC_Pin|TFT_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Relay_2_Pin|Relay_1_Pin|Tranz_2_Pin|Tranz_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IRQ_Pin hall2_Pin hall1_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin|hall2_Pin|hall1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_RST_Pin TOUCH_CS_Pin TFT_DC_Pin TFT_CS_Pin */
  GPIO_InitStruct.Pin = TFT_RST_Pin|TOUCH_CS_Pin|TFT_DC_Pin|TFT_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : led_Pin */
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay_2_Pin Relay_1_Pin Tranz_2_Pin Tranz_1_Pin */
  GPIO_InitStruct.Pin = Relay_2_Pin|Relay_1_Pin|Tranz_2_Pin|Tranz_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void startup(void){
	__HAL_SPI_ENABLE(DISP_SPI_PTR); // включаем SPI
	 // HAL_TIM_Base_Start(&htim2);
	 // HAL_ADC_Start_DMA(&hadc1, adcBuffer, 2);
	 HAL_ADC_Start(&hadc1);
	  DISP_CS_UNSELECT;
	  TOUCH_CS_UNSELECT; // это нужно только если есть тач

	  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  ILI9341_Init(); // инициализация дисплея

	  /////////////////////////// далее демонстрируются различные пользовательские функции ////////////////////////////
	  ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2); // установка ориентации экрана (варианты в файле ILI9341_GFX.h)

	  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  ILI9341_Fill_Screen(MYFON); // заливка всего экрана цветом (цвета в файле ILI9341_GFX.h)
	  //ILI9341_WriteString(10, 10, "Hello World", Font_7x10, WHITE, MYFON);

	  button_draw();
	  button_draw_icons();
}

_Bool touch_work(void)
{

	_Bool what_to_send = 0;
  if(HAL_GPIO_ReadPin(IRQ_GPIO_Port, IRQ_Pin) == GPIO_PIN_RESET && flag_press) // если нажат тачскрин
  {

	  	  x = 0;
          y = 0;

          TOUCH_CS_UNSELECT;
          DISP_CS_UNSELECT;

          HAL_SPI_DeInit(DISP_SPI_PTR);
          hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
          HAL_SPI_Init(DISP_SPI_PTR);

          if(ILI9341_TouchGetCoordinates(&x, &y))
          {
                  flag_press = 0;
                  what_to_send = 1;

                  //////// вывод координат в уарт для отладки ////////
//                  snprintf(buf, 64, "X = %d, Y = %d\n", x, y);
//                  //HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
//                  buf[strlen(buf) - 1] = '\0';
                  ////////////////////////////////////////////////////
          }

          HAL_SPI_DeInit(DISP_SPI_PTR);
          hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
          HAL_SPI_Init(DISP_SPI_PTR);

          __HAL_SPI_ENABLE(DISP_SPI_PTR);
          DISP_CS_SELECT;

          time_press = HAL_GetTick();
  }

  if(!flag_press && (HAL_GetTick() - time_press) > 200) // задержка до следующего нажатия
  {
          flag_press = 1;
  }


  //////////////////////////// удержание кнопки //////////////////////////////
  if(!flag_hold && HAL_GPIO_ReadPin(IRQ_GPIO_Port, IRQ_Pin) != GPIO_PIN_RESET)
  {
          flag_hold = 1;
  }

  if(!flag_hold && (HAL_GetTick() - timme_hold) > 2000) // 2 sek удержание кнопки
  {
          if(HAL_GPIO_ReadPin(IRQ_GPIO_Port, IRQ_Pin) == GPIO_PIN_RESET)
          {
//                  if(x > 5 && x < 90 && y > 160 && y < 230) // первая кнопка
//                  {
//                          //HAL_UART_Transmit(&huart1, (uint8_t*)"LONG PRESS_1\n", 13, 100); // отладка
//                          ILI9341_WriteString(10, 150, "LONG PRESS_1", Font_11x18, WHITE, MYFON); // отладка
//                          // что-то делаем
//                  }
//                  else if(x > 100 && x < 200 && y > 160 && y < 230) // вторая кнопка
//                  {
//                         // HAL_UART_Transmit(&huart1, (uint8_t*)"LONG PRESS_2\n", 13, 100); // отладка
//                          ILI9341_WriteString(10, 150, "LONG PRESS_1", Font_11x18, WHITE, MYFON); // отладка
//                          // что-то делаем
//                  }
          }

          flag_hold = 1;
  }
   return what_to_send;
}




void main_cicle(void){
int ff;
	if(touch_work()){
		ff = button_click();
		//if()
		//perekluchatel(plus_minus_addon(ff));
//		snprintf(buf, 64, "X = %d, Y = %d\n", x, y);
//		ILI9341_WriteString(10, 120, buf, Font_7x10, WHITE, MYFON);
//		snprintf(buf, 64, "button = %d", ff);
//		ILI9341_WriteString(10, 170, buf, Font_7x10, WHITE, MYFON);
		//perekluchatel();
	}
	ADC_out();
	adc_show();
//	for(int i=0;i<6;i++){
//		snprintf(buf, 64, "%d", menu_position[i] );
//		ILI9341_WriteString(10+10*i, 120, buf, Font_7x10, WHITE, MYFON);
//	}


	perekluchatel(ff);




curent_state_raport();
}

void adc_show(void){
		snprintf(buf, 64, "T GOL = %d", adc_buf[0]);
		ILI9341_WriteString(10, 230, buf, Font_7x10, WHITE, MYFON);
		snprintf(buf, 64, "T TRU = %d", adc_buf[1]);
		ILI9341_WriteString(100, 230, buf, Font_7x10, WHITE, MYFON);


		snprintf(buf, 64, "T GOL instal = %d", t_installed[0]);
		ILI9341_WriteString(10, 210, buf, Font_7x10, WHITE, MYFON);
		snprintf(buf, 64, "T TRUB instal = %d", t_installed[1]);
		ILI9341_WriteString(160, 210, buf, Font_7x10, WHITE, MYFON);

}

void curent_state_raport(void){
	for(int i=0;i<10;i++){
			snprintf(buf, 64, "%d", menu_position[i] );
			ILI9341_WriteString(10+10*i, 70, buf, Font_7x10, WHITE, MYFON);
		}
}




void button_draw(void){
	for(int i = 0; i <10; i++){
		draw_rect_tol(buttons_coord[i][0],buttons_coord[i][1],buttons_coord[i][2],buttons_coord[i][3],RED,3);
		//ILI9341_WriteString(buttons_coord[i][0]+5, buttons_coord[i][1]+5,but_icons[i], Font_7x10, RED, MYFON);
	}
}
void button_draw_icons(void){
	char *but_icons[10] ={
	"golovka",
	"truba",
	"start",
	"pneumo",
	"plus",
	"minus",
	"R 1",
	"R 2",
	"T 1",
	"T 2"
	};
	for(int i = 0; i <10; i++){
			//draw_rect_tol(buttons_coord[i][0],buttons_coord[i][1],buttons_coord[i][2],buttons_coord[i][3],RED,3);
			ILI9341_WriteString(buttons_coord[i][0]+5, buttons_coord[i][1]+5,but_icons[i], Font_7x10, RED, MYFON);
		}
}


void draw_rect_tol (uint16_t kx1,uint16_t ky1,uint16_t kx2,uint16_t ky2,uint16_t fon,uint16_t tol){
	for(int i=0;i<tol;i++){
		ILI9341_Draw_Hollow_Rectangle_Coord(kx1-i, ky1-i, kx2+i, ky2+i, fon);
	}
}

uint8_t button_click(void){
	uint8_t what_to_return;
	_Bool is_the_flag = 0;
	for(int i = 0; i <10; i++){
		if(x > buttons_coord[i][0] && x < buttons_coord[i][2] && y > buttons_coord[i][1] && y < buttons_coord[i][3]){

			draw_rect_tol(buttons_coord[i][0],buttons_coord[i][1],buttons_coord[i][2],buttons_coord[i][3],GREEN,3);
			HAL_Delay(500);
			draw_rect_tol(buttons_coord[i][0],buttons_coord[i][1],buttons_coord[i][2],buttons_coord[i][3],RED,3);

			what_to_return = i;
			is_the_flag = 1;
			menu_position[i] = !menu_position[i];

			break;
		}

	}
	if(is_the_flag == 0){
		what_to_return = 10;
	}
	return what_to_return;
}

void ADC_SetActiveChannel(ADC_HandleTypeDef *hadc, uint32_t AdcChannel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = AdcChannel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
   Error_Handler();
  }
}

void ADC_out(void){
 int preskal = 100;

	if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	    {
		adcBuffer[0] = HAL_ADC_GetValue(&hadc1); // Get X value
	      ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_0);
	      HAL_ADC_Start(&hadc1);
	    }

	    if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	    {
	    	adcBuffer[1] = HAL_ADC_GetValue(&hadc1); // Get Y value
	      ADC_SetActiveChannel(&hadc1, ADC_CHANNEL_1);//PB 0
	      HAL_ADC_Start(&hadc1);
	    }


	    adc_sum[0]+=adcBuffer[0];
	    adc_sum[1]+=adcBuffer[1];
	    licznik+=1;
	    if(licznik >= preskal){
	    	licznik = 0;
	    	 adc_buf[0] =  (adc_sum[0]/preskal)/25;
	    	 adc_buf[1] =  (adc_sum[1]/preskal)/25;
	    	 adc_sum[0]=0;//возожно где то здесь гимор
	    	 adc_sum[1]=0;
//	snprintf(buf, 64, "ADC 0 = %d", adc_sum[0]/25);
//	ILI9341_WriteString(10, 200, buf, Font_7x10, WHITE, MYFON);
//	snprintf(buf, 64, "ADC 1 = %d", adc_sum[1]/25);
//	ILI9341_WriteString(10, 220, buf, Font_7x10, WHITE, MYFON);
	    }


}
//char *but_icons[6] ={
//	"t pre",
//	"t heat",
//	"start",
//	"stop",
//	"plus",
//	"minus"
//	};

void perekluchatel(int ff){

	if(menu_position[0] == 1 && menu_position[4] == 1){
				ff=10;
				t_installed[0] = t_installed[0]+1;
				menu_position[4] = 0;
				}else if(menu_position[0] == 1 && menu_position[5] == 1){
				ff = 10;
				t_installed[0] = t_installed[0]-1;
				menu_position[5] = 0;
				}
		if(menu_position[1] == 1 && menu_position[4] == 1){
					ff=10;
					t_installed[1] = t_installed[1]+1;
					menu_position[4] = 0;
					}else if(menu_position[1] == 1 && menu_position[5] == 1){
					ff = 10;
					menu_position[5] = 0;
					t_installed[1] = t_installed[1]-1;
				}
		if(menu_position[2] == 1){
			heaters_start();
		} else if(menu_position[2] == 0){
			heaters_stop();
		}
		if(menu_position[3] == 1){
			menu_position[3] = 0;
				pneumo_start();
		}


		if(menu_position[6] == 1){
			HAL_GPIO_WritePin(GPIOA, Relay_1_Pin, GPIO_PIN_SET);
			} else if(menu_position[6] == 0){
				HAL_GPIO_WritePin(GPIOA, Relay_1_Pin, GPIO_PIN_RESET);
				}

		if(menu_position[7] == 1){
					HAL_GPIO_WritePin(GPIOA, Relay_2_Pin, GPIO_PIN_SET);
					} else if(menu_position[7] == 0){
						HAL_GPIO_WritePin(GPIOA, Relay_2_Pin, GPIO_PIN_RESET);
						}
		if(menu_position[8] == 1){
					HAL_GPIO_WritePin(GPIOA, Tranz_1_Pin, GPIO_PIN_SET);
					} else if(menu_position[8] == 0){
						HAL_GPIO_WritePin(GPIOA, Tranz_1_Pin, GPIO_PIN_RESET);
						}
		if(menu_position[9] == 1){
							HAL_GPIO_WritePin(GPIOA, Tranz_2_Pin, GPIO_PIN_SET);
							} else if(menu_position[9] == 0){
								HAL_GPIO_WritePin(GPIOA, Tranz_2_Pin, GPIO_PIN_RESET);
								}


}

int plus_minus_addon(int passed){
	int i = 0;
	if(passed==4){
		i=i+1;
	}else if(passed==5){
		i=i-1;
	}
	return i;
}

void heaters_start(void){
	if( adc_buf[0]<t_installed[0]){
		 HAL_GPIO_WritePin(GPIOA, Relay_1_Pin, GPIO_PIN_SET);
		 ILI9341_WriteString(20, 180,"heater GL start", Font_7x10, WHITE, MYFON);
	} else if( adc_buf[0]>=t_installed[0]){
		 HAL_GPIO_WritePin(GPIOA, Relay_1_Pin, GPIO_PIN_RESET);
		 ILI9341_WriteString(20, 180,"heater GL stop ", Font_7x10, WHITE, MYFON);
	}
	if( adc_buf[1]<t_installed[1]){
			 HAL_GPIO_WritePin(GPIOA, Relay_2_Pin, GPIO_PIN_SET);
			 ILI9341_WriteString(20, 190,"heater TR start", Font_7x10, WHITE, MYFON);
		} else if( adc_buf[1]>=t_installed[1]){
			 HAL_GPIO_WritePin(GPIOA, Relay_2_Pin, GPIO_PIN_RESET);
			 ILI9341_WriteString(20, 190,"heater TR stop", Font_7x10, WHITE, MYFON);
		}
}
void heaters_stop(void){
	 HAL_GPIO_WritePin(GPIOA, Relay_2_Pin|Relay_1_Pin,GPIO_PIN_RESET);
	 ILI9341_WriteString(20, 190,"heater GL STOP ", Font_7x10, WHITE, MYFON);
	 ILI9341_WriteString(20, 180,"heater TR STOP ", Font_7x10, WHITE, MYFON);
}

void pneumo_start(void){
	ILI9341_WriteString(20, 140,"Pneumo start ", Font_7x10, WHITE, MYFON);
	ILI9341_WriteString(20, 160,"pistone pressed  ", Font_7x10, WHITE, MYFON);
	 HAL_GPIO_WritePin(GPIOA, Tranz_1_Pin,GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOA, Tranz_2_Pin,GPIO_PIN_RESET);
	 HAL_Delay(3000);
	 ILI9341_WriteString(20, 160,"pistone released", Font_7x10, WHITE, MYFON);
	 HAL_GPIO_WritePin(GPIOA, Tranz_1_Pin,GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOA, Tranz_2_Pin,GPIO_PIN_SET);
	 HAL_Delay(2000);
	 HAL_GPIO_WritePin(GPIOA, Tranz_1_Pin,GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOA, Tranz_2_Pin,GPIO_PIN_RESET);
	 ILI9341_WriteString(20, 140,"Pneumo STOP   ", Font_7x10, WHITE, MYFON);
	 ILI9341_WriteString(20, 160,"xxxxxxx xxxxxxxx", Font_7x10, WHITE, MYFON);
}





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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
