/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

#include <stdint.h>
//typedef __packed volatile struct
typedef volatile struct {
	uint16_t ver :16;
	uint16_t r1 :16;

	uint16_t on;//0-выкл 1/2-постоянное вращение пер/назад
				//3/4- Вращение на заданное количество тиков CurTicks(26/27 адрес)пер/назад
	uint16_t Ilimit;//Ток ограничения в дискретах. 2000 это максимум. Минимальные значения где-то 0
	uint16_t PWM_max :16;//Значение ШИМ для 24 вольт. Текущее значение ограничено этой ячейкой.
	uint16_t Arise :16;
	uint16_t Afall :16;
	uint16_t A :16;
	uint16_t KpRise :16;
	uint16_t KpFall :16;
	uint16_t Kp;			//ADR=10
	uint16_t TicksDown;		//Количество тиков после которого BLDC переходит
							//в режим выключения и падения оборотов для on=3, on=4
	uint16_t Vcontinuous;//Заданная скорость вращения для режима непрерывного вращения on=1, on=2
	uint16_t Vsteps;//Заданная скорость вращения для режима  	вращения по шагам on=3, on=4
	uint16_t Acur;
	uint16_t KpCur;
	int32_t TicksStep;//R тиков Холла при движении на заданное кол-во тиков. on=3, on=4
	int32_t CurTicks;	//обратный счётчик тиков Холла при 	движении на заданное кол-во тиков
	int16_t Imotor;//Ток текущий. ADR=20
	uint16_t Ilimit_flag;//Флаг срадатывания ограничения.
	uint16_t Steps;		//Количество выполненых заданий движения по шагам.
	uint16_t Vin;		//Входное напряжение в дискретах АЦП
	int32_t HallTicks;//Счётчик тиков Холла	ADR=24
	uint16_t pwmLimitFlag; // флаг срабатывания if (pwm>=pwmMax) флаг очищается при каждом старте (так-же как флаг тока)
	uint16_t HallState;//Положение датчика Холла.
	uint16_t PWMA :16;	//Текущее значение ШИМ. Всего 999.
	uint16_t Vrot;		//Скорость вращения в тиках датчика 	Холла в секунду
	uint16_t Vrot_next;	//Стабилизируемая скорость вращения двигателя в тиках датчика hall. Не изменяется.
	uint16_t direction;//Текущее направление вращения
} var_str;
//extern volatile uint16_t usRegHoldingBuf[];
//#define var_cur (*((var_str *)usRegHoldingBuf))


extern var_str var_cur;
extern var_str var_prev;

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* ########################## Assert Selection ############################## */
/**
 * @brief Uncomment the line below to expanse the "assert_param" macro in the
 *        HAL drivers code
 */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#include <stdint.h>
#include <stdbool.h>
extern volatile bool tim2_flag;

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
