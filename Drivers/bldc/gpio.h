    
#ifndef __GPIO_H
#define __GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
/* begin code*/

#include <stdint.h>
#include <stdbool.h>

#include "stm32f103xb.h"
	 
void GPIO_Init(void);

typedef volatile struct
{
	bool PORT0:1;
	bool PORT1:1;
	bool PORT2:1;
	bool PORT3:1;
	bool PORT4:1;
	bool PORT5:1;
	bool PORT6:1;
	bool PORT7:1;
	bool PORT8:1;
	bool PORT9:1;
	bool PORT10:1;
	bool PORT11:1;
	bool PORT12:1;
	bool PORT13:1;
	bool PORT14:1;
	bool PORT15:1;
}GPIO_PORT;

#define GPIO_ODR(__port,__num) ((*((GPIO_PORT *)&GPIO##__port->ODR)).PORT##__num)
#define LED GPIO_ODR(C,13)
#define NSS GPIO_ODR(B,12)
#define xNSS(__x) GPIO_BSBR(B,12,__x)
//#define SS GPIO_ODR(B,12)
#define GPIO_BSBR(__port,__num,__val) ((__val)?(GPIO##__port->BSRR=GPIO_BSRR_BS##__num):(GPIO##__port->BRR=GPIO_BRR_BR##__num))
#define xLED(__x) GPIO_BSBR(C,13,__x)
		//set
		//*((uint32_t *)0x48000014)|=1<<0;
		//*((uint32_t *)0x48000018)=(1<<0);
		//LED=1;
		//SET_LED();
		//xLED(true);
		//reset
		//*((uint32_t *)0x48000014)&=~(1<<0);
		//*((uint32_t *)0x48000028)=(1<<0);
		//*((uint32_t *)0x48000018)=(1<<16);
		//LED=0;
		//RESET_LED();
		//xLED(false);

/* end code*/
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __GPIO_H */
