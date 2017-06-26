

#ifndef _COMMON_H_
#define _COMMON_H_


typedef unsigned char	uint8;  /*  8 bits */
typedef unsigned short int	uint16; /* 16 bits */
typedef unsigned long int	uint32; /* 32 bits */

typedef signed char		int8;   /*  8 bits */
typedef short int	        int16;  /* 16 bits */
typedef int		        int32;  /* 32 bits */

typedef volatile int8		vint8;  /*  8 bits */
typedef volatile int16	vint16; /* 16 bits */
typedef volatile int32	vint32; /* 32 bits */

typedef volatile uint8	vuint8;  /*  8 bits */
typedef volatile uint16	vuint16; /* 16 bits */
typedef volatile uint32	vuint32; /* 32 bits */

typedef float   		float32; /*  32 bits */
typedef double   		float64; /*  64 bits */

typedef unsigned char   boolean;      /* 8-bit*/
typedef unsigned char   bool;
#define false 0
#define true  1

#ifdef	FALSE
#undef	FALSE
#endif
#define FALSE	0

#ifdef	TRUE
#undef	TRUE
#endif
#define	TRUE	1

#ifndef NULL
#define NULL    0
#endif

/********************************************************************/


#include "k60_card.h"

#include "MK60DZ10.h"

#include "relocate.h"

#if (defined(__IAR_SYSTEMS_ICC__))
	#include "intrinsics.h"
#endif

#include "LPLD_Drivers.h"


/***********************************************************************/
/* 
 * 中断相关函数定义
 */
//使能全局中断宏定义
#define EnableInterrupts __enable_irq()
//禁用全局中断宏定义
#define DisableInterrupts  __disable_irq()
//使能指定中断向量号的中断
#define enable_irq(IRQn)    NVIC_EnableIRQ(IRQn)
//禁用指定中断向量号的中断
#define disable_irq(IRQn)    NVIC_DisableIRQ(IRQn)
//更改中断向量表地址，写VTOR寄存器
#define write_vtor(vector_addr) SCB->VTOR = (uint32_t)vector_addr;
/***********************************************************************/

void main(void);


/********************************************************************/

#endif /* _COMMON_H_ */
