/**
 * @file system_MK60DZ10.c
 * @version 1.2.1[By LPLD]
 * @date 2013-06-18
 * @brief MK60DZ10系列单片机系统配置文件
 *
 * 更改建议:禁止修改
 *
 * 该代码提供系统配置函数以及一个储存系统主频的全局变量。
 * 配置函数主要负责建立系统各模块的时钟。
 * 代码还实现常见的系统中断函数。
 *
 * 版权所有:北京拉普兰德电子技术有限公司
 * http://www.lpld.cn
 * mail:support@lpld.cn
 *
 * @par
 * 本代码由拉普兰德[LPLD]开发并维护，并向所有使用者开放源代码。
 * 开发者可以随意修使用或改源代码。但本段及以上注释应予以保留。
 * 不得更改或删除原版权所有者姓名，二次开发者可以加注二次版权所有者。
 * 但应在遵守此协议的基础上，开放源代码、不得出售代码本身。
 * 拉普兰德不负责由于使用本代码所带来的任何事故、法律责任或相关不良影响。
 * 拉普兰德无义务解释、说明本代码的具体原理、功能、实现方法。
 * 除非拉普兰德[LPLD]授权，开发者不得将本代码用于商业产品。
 *
 *  Modified by Qian Qiyang(KisaragiAyanoo@twitter)
 *  Date : 2015/12/01
 */

#include <stdint.h>
#include "common.h"

#include "OLED.h"
#include "Setting.h"

/*----------------------------------------------------------------------------
  定义时钟相关值
 *----------------------------------------------------------------------------*/
#define CPU_XTAL_CLK_HZ                 50000000u       //外部有源晶振频率，单位Hz
#define CPU_XTAL32k_CLK_HZ              32768u          //外部32k时钟晶振频率，单位Hz    
#define CPU_INT_SLOW_CLK_HZ             32768u          //慢速内部振荡器的值，单位Hz
#define CPU_INT_FAST_CLK_HZ             4000000u        //快速内部振荡器的值，单位Hz
#define DEFAULT_SYSTEM_CLOCK            100000000u      //默认系统主频，单位Hz

/**
 * @brief 系统主频（单位Hz）
 */
uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;



void SystemInit (void) {
  
  SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK
              | SIM_SCGC5_PORTB_MASK
              | SIM_SCGC5_PORTC_MASK
              | SIM_SCGC5_PORTD_MASK
              | SIM_SCGC5_PORTE_MASK );
  
  WDOG->UNLOCK = (uint16_t)0xC520u;
  WDOG->UNLOCK  = (uint16_t)0xD928u; 
  /* WDOG_STCTRLH: ??=0,DISTESTWDOG=0,BYTESEL=0,TESTSEL=0,TESTWDOG=0,??=0,STNDBYEN=1,WAITEN=1,STOPEN=1,DBGEN=0,ALLOWUPDATE=1,WINEN=0,IRQRSTEN=0,CLKSRC=1,WDOGEN=0 */
  WDOG->STCTRLH = (uint16_t)0x01D2u;
  
  common_relocate();
  
  LPLD_PLL_Setup(CORE_CLK_MHZ);
  
  SystemCoreClockUpdate();
  
  g_core_clock = SystemCoreClock;
  g_bus_clock = g_core_clock / ((uint32_t)((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK) >> SIM_CLKDIV1_OUTDIV2_SHIFT)+ 1u);
  g_flexbus_clock =  g_core_clock / ((uint32_t)((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV3_MASK) >> SIM_CLKDIV1_OUTDIV3_SHIFT)+ 1u);
  g_flash_clock =  g_core_clock / ((uint32_t)((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV4_MASK) >> SIM_CLKDIV1_OUTDIV4_SHIFT)+ 1u);
  
  // ==== Init Oled ===
  
  Oled_Init();
  Oled_Putstr(1,3,"<< Clock Init >>");
  Oled_Putstr(2,1,"Bus Clk");
  Oled_Putnum(2,9,g_bus_clock/1000000);
  Oled_Putstr(2,18,"MHz");
  
  
  NVIC_SetPriorityGrouping(NVIC_GROUP);
}

void SystemCoreClockUpdate (void) {
  uint32_t temp;
  temp =  CPU_XTAL_CLK_HZ *((uint32_t)(MCG->C6 & MCG_C6_VDIV_MASK) + 24u );
  temp = (uint32_t)(temp/((uint32_t)(MCG->C5 & MCG_C5_PRDIV_MASK) +1u ));
  SystemCoreClock = temp;
}




