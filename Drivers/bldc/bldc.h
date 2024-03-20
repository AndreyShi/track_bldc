#ifndef __BLDC_H
#define __BLDC_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
/* begin code*/
        #include <stdint.h>
        #include <stdbool.h>

        #define UH	0
        #define UL	1
        #define VH	2
        #define VL	3
        #define WH	4
        #define WL	5

        void BLDC_Init(void);
        void BLDC_SetDirection(bool __direction);
        void BLDC_On(bool __on);
        void BLDC_SetVrot(uint16_t __vrot);
        void BLDC_TIM1_Callback(void);
        void BLDC_SetPWMMax(uint16_t __vin);
        uint16_t BLDC_GetVrot(void);
        void BLDC_Poll(void);
   
/* end code*/
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __BLDC_H */
