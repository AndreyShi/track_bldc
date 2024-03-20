#include "bldc.h"
//#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "main.h"
#include "stm32l4xx_hal.h"
#include "defs.h"

extern TIM_HandleTypeDef htim1;

#define UH	0
#define UL	1
#define VH	2
#define VL	3
#define WH	4
#define WL	5

static volatile uint16_t BLDC_PWM_A = 0;
static volatile float BLDC_PWM_Af = 0;
static volatile bool BLDC_direction = false;
static volatile bool BLDC_on = false;
static volatile uint8_t BLDC_position = 0;
static volatile bool BLDC_Forward = false;
static volatile uint16_t BLDC_PWM_Max = 0;
static volatile uint16_t BLDC_Vrot[4];
static volatile uint8_t BLDC_Vrot_p = 0;
static volatile uint16_t BLDC_Vrot_next = 0;


static volatile float floatCurTicks;
uint16_t BLDC_GetVrot() {
	uint16_t temp[4];
	__expression_atomic(*(uint64_t * )temp = *(uint64_t * )BLDC_Vrot);
	uint16_t out = temp[0] + temp[1] + temp[2] + temp[3];
	return (out << 1) + (out >> 1); //*2.5
}


//#define DIV_FACTOR	1.0F
#define DIV_FACTOR	0.67F		//коэффициент пересчета тиков датчика холла относительно нормальных двигателей


// BLDC motor steps tables
//static const uint8_t BLDC_BRIDGE_STATE_FORWARD[8][6] =   // Motor steps
//{
////  UH,UL              VH,VL           WH,WL
//   { 0,0        ,       0,0     ,       0,0 },  // 0 //000
//   { 0,1        ,       0,1     ,       1,0 },
//   { 1,0        ,       0,1     ,       0,1 },
//   { 1,0        ,       0,1     ,       1,0 },
//   { 0,1        ,       1,0     ,       0,1 },
//   { 0,1        ,       1,0     ,       1,0 },
//   { 1,0        ,       1,0     ,       0,1 },
//   { 0,0        ,       0,0     ,       0,0 },  // 0 //111
//};
//Original BLDC order
static const uint8_t BLDC_BRIDGE_STATE_FORWARD[8][6] =   // Motor steps
			{
//  UH,UL              VH,VL           WH,WL
						{ 0, 0, 0, 0, 0, 0 },  // 0 //000
						{ 0, 1, 0, 0, 1, 0 },
						{ 1, 0, 0, 1, 0, 0 },
						{ 0, 0, 0, 1, 1, 0 },
						{ 0, 0, 1, 0, 0, 1 },
						{ 0, 1, 1, 0, 0, 0 },
						{ 1, 0, 0, 0, 0, 1 },
						{ 0, 0, 0, 0, 0, 0 },  // 0 //111
			};
/* ������� ������ �����
 static const uint8_t BLDC_BRIDGE_STATE_FORWARD[8][6] =   // Motor steps
 {
 //	UH,UL		VH,VL	WH,WL
 { 0,0	,	0,0	,	0,0 },  // 0 //000
 { 0,1	,	1,0	,	0,0 },
 { 0,0	,	0,1	,	1,0 },
 { 0,1	,	0,0	,	1,0 },
 { 1,0	,	0,0	,	0,1 },
 { 0,0	,	1,0	,	0,1 },
 { 1,0	,	0,1	,	0,0 },
 { 0,0	,	0,0	,	0,0 },  // 0 //111
 };
 */

uint8_t BLDC_MotorSpin = 0;
//uint8_t BLDC_STATE[6] = {0,0,0,0,0,0};
const uint8_t *BLDC_STATE = BLDC_BRIDGE_STATE_FORWARD[7];
//uint8_t BLDC_STATE_PREV[6] = {0,0,0,0,0,0};
//const uint8_t *BLDC_STATE_PREV=BLDC_BRIDGE_STATE_FORWARD[7];

static const uint8_t BLDC_BRIDGE_STATE_BACKWARD[8][6] =   // Motor steps
			{
//	UH,UL		VH,VL	WH,WL
						{ 0, 0, 0, 0, 0, 0 },  //  //000
						{ 1, 0, 0, 0, 0, 1 },
						{ 0, 1, 1, 0, 0, 0 },
						{ 0, 0, 1, 0, 0, 1 },
						{ 0, 0, 0, 1, 1, 0 },
						{ 1, 0, 0, 1, 0, 0 },
						{ 0, 1, 0, 0, 1, 0 },
						{ 0, 0, 0, 0, 0, 0 },  //  //111
			};

//#define BLDC_OUTPUT_DISABLE() (htim1.Instance->CCER &= ~(TIM_CCER_CC1E|TIM_CCER_CC1NE|TIM_CCER_CC2E|TIM_CCER_CC2NE|TIM_CCER_CC3E|TIM_CCER_CC3NE))
#define BLDC_OUTPUT_DISABLE(__np) (htim1.Instance->CCER &= ~TIM_CCER_CC##__np##E)
#define BLDC_OUTPUT_ENABLE(__np) (htim1.Instance->CCER |= TIM_CCER_CC##__np##E)

void BLDC_WritePWM(uint8_t __HallState) {
	if (BLDC_direction)
		BLDC_STATE = BLDC_BRIDGE_STATE_BACKWARD[__HallState];
	else
		BLDC_STATE = BLDC_BRIDGE_STATE_FORWARD[__HallState];

	// Enable if need. If previous state is Enabled then not enable again. Else output do flip-flop.
	if (BLDC_STATE[UL])  // & !BLDC_STATE[UH] & !BLDC_STATE_PREV[UL])
	{
		htim1.Instance->CCR1 = 0;
		BLDC_OUTPUT_ENABLE(1);
		BLDC_OUTPUT_ENABLE(1N);
	}
	if (BLDC_STATE[VL])  // & !BLDC_STATE[VH] & !BLDC_STATE_PREV[VL])
	{
		htim1.Instance->CCR2 = 0;
		BLDC_OUTPUT_ENABLE(2);
		BLDC_OUTPUT_ENABLE(2N);
	}
	if (BLDC_STATE[WL])  // & !BLDC_STATE[WH] & !BLDC_STATE_PREV[WL])
	{
		htim1.Instance->CCR3 = 0;
		BLDC_OUTPUT_ENABLE(3);
		BLDC_OUTPUT_ENABLE(3N);
	}
	if (BLDC_STATE[UH])  // & !BLDC_STATE[UL] & !BLDC_STATE_PREV[UH])
	{
		htim1.Instance->CCR1 = BLDC_on ? BLDC_PWM_A : 0;
		BLDC_OUTPUT_ENABLE(1);
		BLDC_OUTPUT_ENABLE(1N);
	}
	if (BLDC_STATE[VH])  // & !BLDC_STATE[VL] & !BLDC_STATE_PREV[VH])
	{
		htim1.Instance->CCR2 = BLDC_on ? BLDC_PWM_A : 0;
		BLDC_OUTPUT_ENABLE(2);
		BLDC_OUTPUT_ENABLE(2N);
	}
	if (BLDC_STATE[WH])  // & !BLDC_STATE[WL] & !BLDC_STATE_PREV[WH])
	{
		htim1.Instance->CCR3 = BLDC_on ? BLDC_PWM_A : 0;
		BLDC_OUTPUT_ENABLE(3);
		BLDC_OUTPUT_ENABLE(3N);
	}

	// Disable if need
	if (!BLDC_STATE[UH] && !BLDC_STATE[UL]) {
		BLDC_OUTPUT_DISABLE(1);BLDC_OUTPUT_DISABLE(1N);
	}
	if (!BLDC_STATE[VH] && !BLDC_STATE[VL]) {
		BLDC_OUTPUT_DISABLE(2);BLDC_OUTPUT_DISABLE(2N);
	}
	if (!BLDC_STATE[WH] && !BLDC_STATE[WL]) {
		BLDC_OUTPUT_DISABLE(3);BLDC_OUTPUT_DISABLE(3N);
	}

}

uint8_t BLDC_HallSensorsGetPosition(void) {
	return (uint8_t) ((GPIOB->IDR & (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10))
			>> 8);
}



void BLDC_SetPWMA(uint16_t __pwm_a) {
	//if(__pwm_a)
	if (__pwm_a < 50)
		__pwm_a = 0;
	__expression_atomic(BLDC_PWM_A = __pwm_a);
	BLDC_Forward = true;
}

void BLDC_SetPWM(uint16_t PWM) {
	htim1.Instance->CCR1 = PWM;
	htim1.Instance->CCR2 = PWM;
	htim1.Instance->CCR3 = PWM;
}

uint8_t BLDC_ZeroPosition;



void BLDC_Init() {
	var_cur.HallState = (BLDC_ZeroPosition = BLDC_position =
			BLDC_HallSensorsGetPosition());
	BLDC_SetPWM(0);        // 0..99 PWM Control
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_3);
	//BLDC_ALL_OUTPUT_DISABLE();
	//BLDC_SetPWMA(500);
	BLDC_Forward = true;
}

volatile bool BLDC_ChangeDirection_flag = false;
volatile bool BLDC_NextDirection;

void BLDC_SetDirection(bool __direction) {
	if (BLDC_on) {
		if (__direction != BLDC_direction) {
			BLDC_NextDirection = __direction;
			BLDC_ChangeDirection_flag = true;
		}
	} else {
		BLDC_direction = __direction;
	}
	//BLDC_Forward=true;
}

volatile bool BLDC_Off_flag = false;
volatile bool BLDC_Down_flag = false;
static int lastScanDirection=0;
void BLDC_On(bool __on) {
	if (__on) {
		if (var_cur.on == 1 || var_cur.on == 2) {
			lastScanDirection=var_cur.on;
			var_cur.CurTicks=0;
			floatCurTicks=0;
			if (var_cur.Vcontinuous){

			}else {
				var_cur.on = var_prev.on;
				return;
			}
			__expression_atomic(var_cur.Vrot_next = var_cur.Vcontinuous);
			BLDC_Down_flag = false;
			BLDC_Off_flag = false;
		} else {
			if (var_cur.Vsteps){

			}else {
				var_cur.on = var_prev.on;
				return;
			}
			if (lastScanDirection!=var_cur.on){
				if ((lastScanDirection==3)||(lastScanDirection==4)){
					var_cur.CurTicks=-var_cur.CurTicks;
				}
				lastScanDirection=var_cur.on;
			}
			if (var_cur.TicksStep > (int32_t) var_cur.TicksDown) {
				__expression_atomic(var_cur.Vrot_next = var_cur.Vsteps);
				int c_t=var_cur.TicksStep+var_cur.CurTicks;
				if (c_t<0)
					c_t=0;
				__expression_atomic(floatCurTicks = c_t);
				__expression_atomic(var_cur.CurTicks = c_t);
				BLDC_Down_flag = true;
				BLDC_Off_flag = false;
			} else {
				var_cur.on = var_prev.on;
				return;
			}
		}
		var_cur.direction = (var_cur.on == 2 || var_cur.on >= 4) ? 1 : 0;
		__expression_atomic(BLDC_on=true);
		BLDC_Forward = true;
	} else {
		BLDC_Off_flag = true;
	}
}

#define BLDC_1S_TIM1 24000
static volatile uint32_t BLDC_timer_1s = BLDC_1S_TIM1;
static volatile uint16_t BLDC_HallCounter = 0;
static volatile float BLDC_floatHallCounter = 0;
#define BLDC_0_1S_TIM1 2400
static volatile uint32_t BLDC_timer_0_1s = BLDC_0_1S_TIM1;
volatile bool BLDC_t01_flag = false;

//#include "main.h"

//#define Kusrise (0.03*7.0/(float)var_cur.Trise)
//#define Kusfall (0.03*7.0/(float)var_cur.Tfall)
#define SZBLDC_tmp 5000
float BLDC_tmp[SZBLDC_tmp];
int BLDC_tmp_inc;
void BLDC_Poll() {
	if (BLDC_t01_flag) {
		BLDC_t01_flag = false;
		//if(!BLDC_on) return;
		float Vn = BLDC_GetVrot();
		float V0 = BLDC_Vrot_next;
		float V1;


		static bool on_prev = 0;
		static float A = 0, Kp = 0;
		static uint8_t state = 0;
		if (BLDC_on && !on_prev)
			state = 1;
		if (BLDC_Off_flag)
			state = 2;
		switch (state) {
		default:
			state = 0;
		case 0:
			if (var_cur.on == 1 || var_cur.on == 2) {
				__expression_atomic(var_cur.Vrot_next = var_cur.Vcontinuous);
			} else {
				__expression_atomic(var_cur.Vrot_next = var_cur.Vsteps);
			}
			__expression_atomic(A = var_cur.A)
			;
			__expression_atomic(Kp = var_cur.Kp)
			;
			break;
		case 1:
			__expression_atomic(A = var_cur.Arise)
			;
			__expression_atomic(Kp = var_cur.KpRise)
			;
			if (Vn >= V0) {
				state = 0;
				__expression_atomic(A = var_cur.A);
				__expression_atomic(Kp = var_cur.Kp);
			}
			break;
		case 2:
			__expression_atomic(A = var_cur.Afall)
			;
			__expression_atomic(Kp = var_cur.KpFall)
			;
			if (Vn == 0) {
				state = 0;
				__expression_atomic(A = var_cur.A);
				__expression_atomic(Kp = var_cur.Kp);
			}
		}
		on_prev = BLDC_on;

		__expression_atomic(var_cur.Acur = (uint16_t )A);
		__expression_atomic(var_cur.KpCur = (uint16_t )Kp);
		if (!BLDC_on)
			return;
		if (BLDC_ChangeDirection_flag || BLDC_Off_flag) {
			if (BLDC_GetVrot() < 60) {
				if (BLDC_Off_flag) {
					BLDC_on = false;
					BLDC_Off_flag = false;
					BLDC_PWM_Af = 0;
					BLDC_PWM_A = 0;
					BLDC_SetPWMA(0);
				} else {
					BLDC_direction = BLDC_NextDirection;
					BLDC_ChangeDirection_flag = false;
					on_prev = false;
				}
			}
			__expression_atomic(A = var_cur.Afall);
			__expression_atomic(Kp = var_cur.KpFall);
			__expression_atomic(var_cur.Acur = (uint16_t )A);
			__expression_atomic(var_cur.KpCur = (uint16_t )Kp);
			V0 = 0;
		}
//                else
//                {
		if (Vn < V0) {
			if ((V0 - Vn) > A) {        //1 var
				V1 = Vn + A;
			} else {        //3 var
				V1 = V0;
			}
		} else {
			if ((Vn - V0) > A) {        //2 var
				V1 = Vn - A;
			} else {        //3 var
				V1 = V0;
			}
		}
		float temp = V1 - Vn;
		BLDC_PWM_Af += (Kp / 10) * temp;
//                }


		if (BLDC_PWM_Af > BLDC_PWM_Max)
			BLDC_PWM_Af = BLDC_PWM_Max;
		if (BLDC_PWM_Af < 0)
			BLDC_PWM_Af = 0;
			
		var_cur.PWMA = (uint16_t) (BLDC_PWM_Af + .5);

		if(BLDC_tmp_inc < SZBLDC_tmp)
            {BLDC_tmp[BLDC_tmp_inc++] = var_cur.PWMA;}
		else
		    {__asm("nop");}
		BLDC_SetPWMA(var_cur.PWMA);
	}
}

int hall_table[8]={
		-1,//000
		1,//001
		3,//010
		2,//011
		5,//100
		6,//101
		4,//110
		-1,//111
};

//1 3 2 6 4 5 1

void BLDC_TIM1_Callback() {
	uint8_t position = BLDC_HallSensorsGetPosition();
	int cur_direction=0;
	if (position == BLDC_position) {
		if (BLDC_Forward) {
			BLDC_Forward = false;
			BLDC_WritePWM(BLDC_HallSensorsGetPosition());
		}
	} else {
		switch (position){
		case 1:
			if(BLDC_position==5){
				var_cur.HallTicks++;
				cur_direction=1;
			}else{
				var_cur.HallTicks--;
				cur_direction=-1;
			}
			break;
		case 2:
			if(BLDC_position==3){
				var_cur.HallTicks++;
				cur_direction=1;
			}else{
				var_cur.HallTicks--;
				cur_direction=-1;
			}
			break;
		case 3:
			if(BLDC_position==1){
				var_cur.HallTicks++;
				cur_direction=1;
			}else{
				var_cur.HallTicks--;
				cur_direction=-1;
			}
			break;
		case 4:
			if(BLDC_position==6){
				var_cur.HallTicks++;
				cur_direction=1;
			}else{
				var_cur.HallTicks--;
				cur_direction=-1;
			}
			break;
		case 5:
			if(BLDC_position==4){
				var_cur.HallTicks++;
				cur_direction=1;
			}else{
				var_cur.HallTicks--;
				cur_direction=-1;
			}
			break;
		case 6:
			if(BLDC_position==2){
				var_cur.HallTicks++;
				cur_direction=1;
			}else{
				var_cur.HallTicks--;
				cur_direction=-1;
			}
			break;
		default:
			break;
		}
		BLDC_position = position;
		var_cur.HallState = BLDC_position;
		BLDC_WritePWM(BLDC_HallSensorsGetPosition());

		BLDC_floatHallCounter=BLDC_floatHallCounter+DIV_FACTOR;
		BLDC_HallCounter=BLDC_floatHallCounter+0.5F;

		if (BLDC_Down_flag) {
			floatCurTicks=floatCurTicks-DIV_FACTOR;
			var_cur.CurTicks=floatCurTicks+0.5F;
			if (var_cur.CurTicks <= (int32_t) var_cur.TicksDown) {
				BLDC_Off_flag = true;
			}
		}else{
			if (var_cur.on==0){
				if (lastScanDirection==3){
					if (cur_direction==1){
						floatCurTicks=floatCurTicks+DIV_FACTOR;
						var_cur.CurTicks=floatCurTicks+0.5F;
					}else{
						floatCurTicks=floatCurTicks-DIV_FACTOR;
						var_cur.CurTicks=floatCurTicks+0.5F;
					}
				}else
				if (lastScanDirection==4){
					if (cur_direction==-1){
						floatCurTicks=floatCurTicks+DIV_FACTOR;
						var_cur.CurTicks=floatCurTicks+0.5F;
					}else{
						floatCurTicks=floatCurTicks-DIV_FACTOR;
						var_cur.CurTicks=floatCurTicks+0.5F;
					}
				}
			}
		}

	}
	if (BLDC_Down_flag){
		if (var_cur.CurTicks <= (int32_t) var_cur.TicksDown){
			if (BLDC_GetVrot() == 0) {
				BLDC_Down_flag = false;
				var_cur.on = 0;
				var_cur.Steps++;
			}
		}
	}

	if (BLDC_timer_1s--) {
	} else {
		BLDC_timer_1s = BLDC_1S_TIM1;
	}
	if (BLDC_timer_0_1s--) {
	} else {
		BLDC_timer_0_1s = BLDC_0_1S_TIM1;
		BLDC_t01_flag = true;

		BLDC_Vrot[BLDC_Vrot_p] = BLDC_HallCounter;
		if (BLDC_Vrot_p--){

		}else{
			BLDC_Vrot_p = 3;
		}
		BLDC_HallCounter = 0;
		BLDC_floatHallCounter=0;
	}
}

//Resistance of divider
#define BLDC_RH ((float)75e3)
#define BLDC_RL ((float)2e3)
#define BLDC_VIN_MAX ((float)24)
#define BLDC_Vadc ((float)3.3)
volatile float pdwd;
void BLDC_SetPWMMax(uint16_t __vin) {
	float PWM_Max = BLDC_VIN_MAX / (__vin / 4096.0 * BLDC_Vadc * (BLDC_RH + BLDC_RL)) * BLDC_RL * 999.0;
	if (PWM_Max > 998.0)
		{PWM_Max = 998.0;}
		pdwd = PWM_Max;
	__expression_atomic(var_cur.PWM_max = BLDC_PWM_Max = (uint16_t )PWM_Max);
}


void BLDC_SetVrot(uint16_t __vrot) {
	BLDC_Vrot_next = __vrot;
}
