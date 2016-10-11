/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "i2c.h"
#include "ltdc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* USER CODE BEGIN Includes */
#include "st_logo1.h"
#include "st_logo2.h"
#include "lcd.h"
#include "voice.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
LTDC_HandleTypeDef LtdcHandle;

__IO uint32_t ReloadFlag = 0;
uint32_t Xpos1 = 0;
uint32_t Xpos2 = 0;
uint32_t Ypos1 = 0;
uint32_t Ypos2 = 160;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void LCD_Config(void)
{  
  LTDC_LayerCfgTypeDef pLayerCfg;
  LTDC_LayerCfgTypeDef pLayerCfg1;

  /* Initialization of ILI9341 component*/
  ST7789V_init();
  
/* LTDC Initialization -------------------------------------------------------*/
  
  /* Polarity configuration */
  /* Initialize the horizontal synchronization polarity as active low */
  LtdcHandle.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  /* Initialize the vertical synchronization polarity as active low */ 
  LtdcHandle.Init.VSPolarity = LTDC_VSPOLARITY_AL; 
  /* Initialize the data enable polarity as active low */ 
  LtdcHandle.Init.DEPolarity = LTDC_DEPOLARITY_AL; 
  /* Initialize the pixel clock polarity as input pixel clock */  
  LtdcHandle.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  
  /* Timing configuration  (Typical configuration from ILI9341 datasheet)
      HSYNC=10 (9+1)
      HBP=20 (29-10+1)
      ActiveW=240 (269-20-10+1)
      HFP=10 (279-240-20-10+1)

      VSYNC=2 (1+1)
      VBP=2 (3-2+1)
      ActiveH=320 (323-2-2+1)
      VFP=4 (327-320-2-2+1)
  */  

  /* Timing configuration */
  /* Horizontal synchronization width = Hsync - 1 */  
  LtdcHandle.Init.HorizontalSync = 9;
  /* Vertical synchronization height = Vsync - 1 */
  LtdcHandle.Init.VerticalSync = 3;
  /* Accumulated horizontal back porch = Hsync + HBP - 1 */
  LtdcHandle.Init.AccumulatedHBP = 19;
  /* Accumulated vertical back porch = Vsync + VBP - 1 */
  LtdcHandle.Init.AccumulatedVBP = 7; 
  /* Accumulated active width = Hsync + HBP + Active Width - 1 */ 
  LtdcHandle.Init.AccumulatedActiveH = 327;
  /* Accumulated active height = Vsync + VBP + Active Heigh - 1 */
  LtdcHandle.Init.AccumulatedActiveW = 259;
  /* Total height = Vsync + VBP + Active Heigh + VFP - 1 */
  LtdcHandle.Init.TotalHeigh = 335;
  /* Total width = Hsync + HBP + Active Width + HFP - 1 */
  LtdcHandle.Init.TotalWidth = 297;
  
  /* Configure R,G,B component values for LCD background color */
  LtdcHandle.Init.Backcolor.Blue = 0xff;
  LtdcHandle.Init.Backcolor.Green = 0;
  LtdcHandle.Init.Backcolor.Red = 0;

  LtdcHandle.Instance = LTDC;
  
/* Layer1 Configuration ------------------------------------------------------*/
  
  /* Windowing configuration */ 
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 240;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 160;
  
  /* Pixel Format configuration*/ 
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  
  /* Start Address configuration : frame buffer is located at FLASH memory */
  pLayerCfg.FBStartAdress = (uint32_t)&ST_LOGO_1;
//  pLayerCfg.FBStartAdress = 0;
  /* Alpha constant (255 totally opaque) */
  pLayerCfg.Alpha = 255;
  
  /* Default Color configuration (configure A,R,G,B component values) */
  pLayerCfg.Alpha0 = 1;
  pLayerCfg.Backcolor.Blue = 0xff;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  
  /* Configure blending factors */
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  
  /* Configure the number of lines and number of pixels per line */
  pLayerCfg.ImageWidth = 240;
  pLayerCfg.ImageHeight = 160;

/* Layer2 Configuration ------------------------------------------------------*/
  
  /* Windowing configuration */
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 240;
  pLayerCfg1.WindowY0 = 160;
  pLayerCfg1.WindowY1 = 320;
  
  /* Pixel Format configuration*/ 
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  
  /* Start Address configuration : frame buffer is located at FLASH memory */
  pLayerCfg1.FBStartAdress = (uint32_t)&ST_LOGO_2;
//  pLayerCfg1.FBStartAdress = 0;
  /* Alpha constant (255 totally opaque) */
  pLayerCfg1.Alpha = 200;
  
  /* Default Color configuration (configure A,R,G,B component values) */
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.Backcolor.Blue = 0xFF;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  
  /* Configure blending factors */
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  
  /* Configure the number of lines and number of pixels per line */
  pLayerCfg1.ImageWidth = 240;
  pLayerCfg1.ImageHeight = 160;  
   
  /* Configure the LTDC */  
  if(HAL_LTDC_Init(&LtdcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }

  /* Configure the Background Layer*/
  if(HAL_LTDC_ConfigLayer(&LtdcHandle, &pLayerCfg, 0) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
  
  /* Configure the Foreground Layer*/
  if(HAL_LTDC_ConfigLayer(&LtdcHandle, &pLayerCfg1, 1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }  
}  
static void PicturesPosition(uint32_t* x1, uint32_t* y1, uint32_t* x2, uint32_t* y2, uint32_t index)
{
  /* picture1 position */
  *x1 = 0;
  *y1 = index*4; 
  
  /* picture2 position */
  *x2 = 0;
  *y2 = 160 - index*4;
}
void HAL_LTDC_ReloadEventCallback(LTDC_HandleTypeDef *hltdc)
{
  ReloadFlag = 1;
}

///////////////////////////////////////////////////////////////////////////
//void DAC_Ch1_EscalatorConfig(void)
//{
//  /*##-1- Initialize the DAC peripheral ######################################*/
//  if(HAL_DAC_Init(&hdac) != HAL_OK)
//  {
//    /* Initialization Error */
//    Error_Handler();
//  }
//  
//  /*##-1- DAC channel1 Configuration #########################################*/
//  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
//  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;  

//  if(HAL_DAC_ConfigChannel(&DacHandle, &sConfig, DACx_CHANNEL1) != HAL_OK)
//  {
//    /* Channel configuration Error */
//    Error_Handler();
//  }
//  
//  /*##-2- Enable DAC Channel1 and associated DMA #############################*/
//  if(HAL_DAC_Start_DMA(&DacHandle, DACx_CHANNEL1, (uint32_t*)aEscalator8bit, 6, DAC_ALIGN_8B_R) != HAL_OK)
//  {
//    /* Start DMA Error */
//    Error_Handler();
//  }
//}
///////////////////////////////////////////////////////////////////////////


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint32_t index = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_USART6_UART_Init();
  MX_DAC_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */
	LCD_ENABLE();
	LCD_BL_ON();
//	LCD_Config();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
//		for (index = 0; index < 40; index++)
//    {
//      /* calculate new picture position */
//      PicturesPosition(&Xpos1, &Ypos1, &Xpos2, &Ypos2, (index+1));
//      
//      /* reconfigure the layer1 position  without Reloading*/
//      HAL_LTDC_SetWindowPosition_NoReload(&LtdcHandle, Xpos1, Ypos1, 0);
//      /* reconfigure the layer2 position  without Reloading*/
//      HAL_LTDC_SetWindowPosition_NoReload(&LtdcHandle, Xpos2, Ypos2, 1);
//      /* Ask for LTDC reload within next vertical blanking*/
//      ReloadFlag = 0;
//      HAL_LTDC_Relaod(&LtdcHandle,LTDC_SRCR_VBR);
//      
//      while(ReloadFlag == 0)
//      {
//        /* wait till reload takes effect (in the next vertical blanking period) */
//      }
//    }
//    HAL_Delay(500);
//    
//    for (index = 0; index < 40; index++)
//    {
//      /* calculate new picture position */
//      PicturesPosition(&Xpos2, &Ypos2, &Xpos1, &Ypos1, (index+1));
//      
//      /* reconfigure the layer1 position  without Reloading*/
//      HAL_LTDC_SetWindowPosition_NoReload(&LtdcHandle, Xpos1, Ypos1, 0);
//      /* reconfigure the layer2 position  without Reloading*/
//      HAL_LTDC_SetWindowPosition_NoReload(&LtdcHandle, Xpos2, Ypos2, 1);
//      /* Ask for LTDC reload within next vertical blanking*/
//      ReloadFlag = 0;
//      HAL_LTDC_Relaod(&LtdcHandle,LTDC_SRCR_VBR);
//      
//      while(ReloadFlag == 0)
//      {
//        /* wait till reload takes effect (in the next vertical blanking period) */
//      }
//    }
    HAL_Delay(500);
		MX_DAC_Init();
		HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t*)aEscalator8bit, 67726, DAC_ALIGN_8B_R);
  }

  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 224;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 4;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
