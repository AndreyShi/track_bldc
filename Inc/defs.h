    
#ifndef __DEFS_H
#define __DEFS_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
/* begin code*/

#include <stdint.h>
#include <stdbool.h>

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define __expression_atomic(__expression)\
	{\
		uint32_t OldState=__get_PRIMASK();\
		__disable_irq();\
		__expression;\
		__set_PRIMASK(OldState);\
	}

#define __expression_atomic_start()\
                uint32_t OldState=__get_PRIMASK();\
                __disable_irq()

#define __critical_start() __expression_atomic_start()

#define __expression_atomic_stop()\
                __set_PRIMASK(OldState)

#define __critical_stop() __expression_atomic_stop()

#define __critical __expression_atomic

#define __expression_repeat_dec(__expression,n_count)\
        {\
                uint8_t i=(n_count);\
                while(i--){__expression;}\
        }


/* end code*/
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __DEFS_H */
