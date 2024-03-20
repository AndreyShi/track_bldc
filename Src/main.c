/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "main.h"
#include "stm32l4xx_hal.h"
#include "bldc.h"
#include "defs.h"
#include <string.h>
/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Private function prototypes -----------------------------------------------*/
var_str var_prev;
volatile var_str var_cur;
var_str var_test = {
	.ver         = 1,
	.r1          = 2,
	.on          = 3,
	.Ilimit      = 4,
	.PWM_max     = 5,
	.Arise       = 6,
	.Afall       = 7,
	.A           = 8,
	.KpRise      = 9,
	.KpFall      = 10,
	.Kp          = 11,			
	.TicksDown   = 12,					
	.Vcontinuous = 13,
	.Vsteps      = 14,
	.Acur        = 15,
	.KpCur       = 16,
	.TicksStep   = 17,
	.CurTicks    = 18,
	.Imotor      = 19,
	.Ilimit_flag = 20,
	.Steps       = 21,
	.Vin         = 22,
	.HallTicks   = 23,
	.pwmLimitFlag= 24,
	.HallState   = 25,
	.PWMA        = 26,
	.Vrot        = 27,
	.Vrot_next   = 28,
	.direction   = 29,
	};

static uint32_t lock_nesting_count = 0;
static uint32_t stopTime = 0;

void __critical_enter(void) {
	__disable_irq();
	++lock_nesting_count;
}
void __critical_exit(void) {
	/* Unlock interrupts only when we are exiting the outermost nested call. */
	--lock_nesting_count;
	if (lock_nesting_count == 0) {
		__enable_irq();
	}
}
#include "mb.h"
#include "mbport.h"

#define REG_HOLDING_START    1
#define REG_HOLDING_NREGS    (sizeof(var_str)>>1)

uint16_t usRegHoldingStart = REG_HOLDING_START;
//uint16_t* usRegHoldingBuf;

#include <stdbool.h>

eMBErrorCode eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress,
		USHORT usNRegs) {
	stopTime = HAL_GetTick() + 2000;
	return MB_ENOREG;
}

eMBErrorCode eMBRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress,
		USHORT usNRegs, eMBRegisterMode eMode) {
	eMBErrorCode eStatus = MB_ENOERR;
	int iRegIndex;
	stopTime = HAL_GetTick() + 2000;
	if ((usAddress >= REG_HOLDING_START) && (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS)) {
		iRegIndex = (int) (usAddress - usRegHoldingStart);
		uint16_t *usRegHoldingBufPtr;
		usRegHoldingBufPtr = (uint16_t*) &var_cur;
		switch (eMode) {
		case MB_REG_READ:
		    //usRegHoldingBufPtr = (uint16_t*) &var_test;
			memcpy(((uint8_t*) &pucRegBuffer[0]),(uint8_t*) &usRegHoldingBufPtr[iRegIndex], usNRegs << 1);
			usNRegs = 0;
			break;
		case MB_REG_WRITE:
			memcpy((uint8_t*) &usRegHoldingBufPtr[iRegIndex],(uint8_t*) &(pucRegBuffer[0]), usNRegs << 1);
			usNRegs = 0;
			break;
		}
	} else {
		eStatus = MB_ENOREG;
	}
	return eStatus;
}

eMBErrorCode eMBRegCoilsCB(UCHAR * pucRegBuffer, USHORT usAddress,
		USHORT usNCoils, eMBRegisterMode eMode) {
	stopTime = HAL_GetTick() + 2000;
	return MB_ENOREG;
}

eMBErrorCode eMBRegDiscreteCB(UCHAR * pucRegBuffer, USHORT usAddress,
		USHORT usNDiscrete) {
	stopTime = HAL_GetTick() + 2000;
	return MB_ENOREG;
}

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	/* Configure the system clock */
	SystemClock_Config();
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_TIM6_Init();
	MX_USART1_UART_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	HAL_TIM_Base_Start_IT(&htim2);
	BLDC_Init();

	//usRegHoldingBuf=(uint16_t*)&var_cur;
	eMBErrorCode eStatus;

	eStatus = eMBInit(MB_RTU, TRACK_ADR, 0, 115200, MB_PAR_NONE);
	eStatus = eMBEnable();
	/*	if (0xFF != *(uint8_t *) 0x0801F800UL)
	 for (uint8_t i = 0; i < sizeof(usRegHoldingBuf); i++)
	 *(((uint8_t *) usRegHoldingBuf) + i) = *(uint8_t *) (0x0801F800UL
	 + i);*/
	var_cur.ver = 101;
	//var_cur.work = 0;
	var_cur.KpRise = 5;
	var_cur.KpFall = 5;
	var_cur.KpCur = 5;
	var_cur.Kp = 2;
	var_cur.Afall = 500;
	var_cur.A = 20;
	var_cur.Arise = 150;
	var_cur.Vsteps = 300;
	var_cur.Vcontinuous = 100;
	var_cur.Ilimit = 500;
	var_cur.TicksDown = 50;
	var_cur.TicksStep = 300;
	var_cur.HallTicks = 0;
	static bool adc_flag = false;
	//uint32_t paused = 0;
	//uint32_t stopCurrentTime = 0;
	while (1) {
		if (stopTime < HAL_GetTick()) {
			var_cur.on = 0;
		}
		(void) eMBPoll();

		if (var_prev.on == var_cur.on) {

		} else {
			BLDC_On(var_cur.on);
			var_cur.Ilimit_flag = false;
			var_cur.pwmLimitFlag = false;
		}

/*		if (var_cur.Ilimit_flag == true) {
			if (stopCurrentTime < HAL_GetTick()) {
				BLDC_On(var_cur.on);
				var_cur.Ilimit_flag = false;
			}
		}*/

		BLDC_Poll();

		static ADC_ChannelConfTypeDef sConfig;
		if (adc_flag == false) {
			if (sConfig.Channel == ADC_CHANNEL_7)
				sConfig.Channel = ADC_CHANNEL_8;
			else
				sConfig.Channel = ADC_CHANNEL_7;
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
			sConfig.SingleDiff = ADC_SINGLE_ENDED;
			sConfig.OffsetNumber = ADC_OFFSET_NONE;
			sConfig.Offset = 0;
			if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
				_Error_Handler(__FILE__, __LINE__);
			}
			HAL_ADC_Start(&hadc1);
			adc_flag = true;
		}
		if (adc_flag)
			if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC)) {
				if (sConfig.Channel == ADC_CHANNEL_7) {
					var_cur.Imotor = 2048 - HAL_ADC_GetValue(&hadc1);
				} else if (sConfig.Channel == ADC_CHANNEL_8) {
					var_cur.Vin = HAL_ADC_GetValue(&hadc1);
					BLDC_SetPWMMax(var_cur.Vin);
				}
				adc_flag = false;
			}
		if (var_cur.Imotor > var_cur.Ilimit) {
			//��������� ����� �� ����
			BLDC_On(0);
			var_cur.Ilimit_flag = true;
			//stopCurrentTime = HAL_GetTick()+1000;
		}

		if (var_prev.Vrot_next == var_cur.Vrot_next) {
		} else {
			BLDC_SetVrot(var_cur.Vrot_next);
		}
		if (var_prev.Ilimit == var_cur.Ilimit
				&& var_prev.TicksStep == var_cur.TicksStep
				&& var_prev.Vcontinuous == var_cur.Vcontinuous
				&& var_prev.Vsteps == var_cur.Vsteps && var_prev.A == var_cur.A
				&& var_prev.Afall == var_cur.Afall
				&& var_prev.Arise == var_cur.Arise && var_prev.Kp == var_cur.Kp
				&& var_prev.KpRise == var_cur.KpRise
				&& var_prev.KpFall == var_cur.KpFall
				&& var_prev.TicksDown == var_cur.TicksDown) {
		}
		var_cur.Vrot = BLDC_GetVrot();

		if (var_prev.direction == var_cur.direction) {
		} else {
			BLDC_SetDirection(var_cur.direction);
			//paused = 100000;
		}
		var_prev = var_cur;
	}

}


/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 24;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_ADC;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.AdcClockSelection=RCC_ADCCLKSOURCE_SYSCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void) {


	ADC_ChannelConfTypeDef sConfig;

	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the ADC multi-mode
	 */
#ifdef ADC_MULTIMODE_SUPPORT
	ADC_MultiModeTypeDef multimode;
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
#endif
	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM1 init function */
static void MX_TIM1_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 3;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 239;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM6 init function */
static void MX_TIM6_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;

	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 23;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 49;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART1 init function */
static void MX_USART1_UART_Init(void) {

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit =
	UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
	huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 PA2   ------> SharedAnalog_PA2
 PA3   ------> SharedAnalog_PA3
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA2 PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB10 PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PA12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
