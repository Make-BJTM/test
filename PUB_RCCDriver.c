/*******************************************************************************
 深圳市汇川技术有限公司 版权所有（C）All rights reserved.
 文件名:    PUB_RCCDriver.c
 创建人：匡两传                创建日期：12.10.09
 修改人：
 描述： 
     1.systimconfig函数文件
     2.This file configures the system clock as follows:
  *=============================================================================
  *=============================================================================
  *        Supported STM32F2xx device revision    | Rev B and Y
  *-----------------------------------------------------------------------------
  *        System Clock source                    | PLL (HSE)
  *-----------------------------------------------------------------------------
  *        SYSCLK(Hz)                             | 120000000
  *-----------------------------------------------------------------------------
  *        HCLK(Hz)                               | 120000000
  *-----------------------------------------------------------------------------
  *        AHB Prescaler                          | 1
  *-----------------------------------------------------------------------------
  *        APB1 Prescaler                         | 4
  *-----------------------------------------------------------------------------
  *        APB2 Prescaler                         | 2
  *-----------------------------------------------------------------------------
  *        HSE Frequency(Hz)                      | 25000000
  *-----------------------------------------------------------------------------
  *        PLL_M                                  | 25
  *-----------------------------------------------------------------------------
  *        PLL_N                                  | 240
  *-----------------------------------------------------------------------------
  *        PLL_P                                  | 2
  *-----------------------------------------------------------------------------
  *        PLL_Q                                  | 5
  *-----------------------------------------------------------------------------
  *        PLLI2S_N                               | NA
  *-----------------------------------------------------------------------------
  *        PLLI2S_R                               | NA
  *-----------------------------------------------------------------------------
  *        I2S input clock                        | NA
  *-----------------------------------------------------------------------------
  *        VDD(V)                                 | 3.3
  *-----------------------------------------------------------------------------
  *        Flash Latency(WS)                      | 3
  *-----------------------------------------------------------------------------
  *        Prefetch Buffer                        | ON
  *-----------------------------------------------------------------------------
  *        Instruction cache                      | ON
  *-----------------------------------------------------------------------------
  *        Data cache                             | ON
  *-----------------------------------------------------------------------------
  *        Require 48MHz for USB OTG FS,          | Enabled
  *        SDIO and RNG clock                     |
  *-----------------------------------------------------------------------------

 修改记录：  
     1.xx.xx.xx      XX  
       变更内容： xxxxxxxxxxx
     2.xx.xx.xx      XX
       变更内容： xxxxxxxxxxx

********************************************************************************/
#include "stm32f4xx.h"
#include "PUB_RCCDriver.h"

/* Private_Macros ------------------------------------------------------------*/
/* 带参数的宏定义 */


#define VECT_TAB_OFFSET  0xc200 /*!< Vector Table base offset field. 
                                   This value must be a multiple of 0x200. */



// PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N 
/* Caution:
 * The software has to set PLL_M bits correctly to ensure that the VCO input frequency
 * ranges from 1 to 2 MHz. It is recommended to select a frequency of 2 MHz to limit
 * PLL jitter.
 * The software has to set PLL_N bits correctly to ensure that the VCO output
 * frequency is between 192 and 432 MHz.
*/
#define PLL_M      8
#define PLL_N      336

// SYSCLK = PLL_VCO / PLL_P 
//Caution: The software has to set PLL_P bits correctly not to exceed 120 MHz on this domain.
#define PLL_P      2

// USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ
/* Caution: 
 * The USB OTG FS requires a 48 MHz clock to work correctly. The SDIO and the
 * random number generator need a frequency lower than or equal to 48 MHz to work
 * correctly.
 */
#define PLL_Q      7

/* Private_Variables ---------------------------------------------------------*/
/* 文件内变量定义 */

__I uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};


/* Private_Functions ---------------------------------------------------------*/
/* 该文件内部调用的函数的声明 */

static void SetSysClock(void);
void PUB_PeripheralClockConfig(void);
/*******************************************************************************
  函数名:  PUB_SystemInit
  输入:    None
  输出:    None
  子函数:         
  描述:  Setup the microcontroller system
  *         Initialize the Embedded Flash Interface, the PLL and update the 
  *         SystemFrequency variable.
********************************************************************************/
void PUB_SystemInit(void)
{
  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;

  /* Reset CFGR register */
  RCC->CFGR = 0x00000000;

  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  /* Reset PLLCFGR register */
  RCC->PLLCFGR = 0x24003010;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Disable all interrupts */
  RCC->CIR = 0x00000000;

         
  /* Configure the System clock source, PLL Multiplier and Divider factors, 
     AHB/APBx prescalers and Flash settings ----------------------------------*/
  SetSysClock();

  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif
}

/*******************************************************************************
  函数名:  SetSysClock
  输入:    None
  输出:    None
  子函数:         
  描述:  Configures the System clock source, PLL Multiplier and Divider factors, 
  *         AHB/APBx prescalers and Flash settings
  Note   This function should be called only once the RCC clock configuration  
  *         is reset to the default reset state (done in SystemInit() function).
********************************************************************************/

static void SetSysClock(void)
{
/******************************************************************************/
/*            PLL (clocked by HSE) used as System clock source                */
/******************************************************************************/
	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;

	/* Enable HSE */
	RCC->CR |= ((uint32_t)RCC_CR_HSEON);

	/* Wait till HSE is ready and if Time out is reached exit */
	do
	{
		HSEStatus = RCC->CR & RCC_CR_HSERDY;
		StartUpCounter++;
	} while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

	if ((RCC->CR & RCC_CR_HSERDY) != RESET)
	{
		HSEStatus = (uint32_t)0x01;
	}
	else
	{
		HSEStatus = (uint32_t)0x00;
	}

	if (HSEStatus == (uint32_t)0x01)
	{
		/*AHB HCLK = SYSCLK / 1    168MHz*/
		RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
		  
		/*APB2 PCLK2 = HCLK / 2  84MHz*/
		RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

		/*APB1 PCLK1 = HCLK / 4  42MHz*/
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

		/* Configure the main PLL */
		RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
		               (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

		/* Enable the main PLL */
		RCC->CR |= RCC_CR_PLLON;

		/* Wait till the main PLL is ready */
		while((RCC->CR & RCC_CR_PLLRDY) == 0)
		{
		}

		/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
		FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

		/* Select the main PLL as system clock source */
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= RCC_CFGR_SW_PLL;

		/* Wait till the main PLL is used as system clock source */
		while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL)
		{
		}
	}
	else
	{ /* If HSE fails to start-up, the application will have wrong clock
	     configuration. User can add here some code to deal with this error */
	}
    /*  主频测试   */
	//RCC->CFGR |= RCC_CFGR_MCO1PRE;
	//RCC->CFGR |= RCC_CFGR_MCO1 ;
    //RCC->CFGR |= RCC_CFGR_MCO2PRE;
	//RCC->CFGR |= RCC_CFGR_MCO2 ;
}


/*******************************************************************************
  函数名:  PUB_PeripheralClockConfig
  输入:    None
  输出:    None
  子函数:         
  描述:  完成外围时钟配置.
********************************************************************************/

void PUB_PeripheralClockConfig(void)
{
    /*PUB模块调用的外设，其相应时钟的开启*/
	//开启时钟安全监测，如果外部晶振出现问题，进入NMI中断
	//TIM1接口时钟使能，中断外设都属于PUB模块中
	//TIM3时钟使能
	//TIM6时钟使能
	//TIM7时钟使能
	//TIM8时钟使能
	//电源接口时钟使能
	//备份接口时钟使能
	//窗口看们狗时钟使能

    /*FUNC和MTR 两模块都调用的外设，其相应时钟的开启*/
	//GPIOI接口时钟使能
	//GPIOH接口时钟使能
	//GPIOG接口时钟使能
	//GPIOF接口时钟使能
	//GPIOE接口时钟使能
	//GPIOD接口时钟使能
	//GPIOC接口时钟使能
	//GPIOB接口时钟使能
	//GPIOA接口时钟使能
    //DMA1时钟使能，DMA1在MTR模块中的AD采样，以及在FUNC模块中I2C和SPI中进行调用

    /*FUNC 模块调用的外设，其相应时钟的开启*/
	//I2C1时钟使能，FUNC模块中的Eeprom操作的I2C操作使用      
	//SPI2时钟使能，FUNC模块中的控制板面及DIDO传输数据时使用
	//AFIO接口时钟使能，I2C的操作中使用

    /*MTR 模块调用的外设，其相应时钟的开启*/
	//ADC1接口时钟使能，MTR 中的AD采样
	//ADC2接口时钟使能，MTR 中的AD采样
	// PUB_RCC->RCC_APB2ENR.bit.ADC3EN = PUB_ENABLE; //ADC3接口时钟使能
	//配置ADC时钟为HCLK/8,也就是12MHz
	//使能FSMC时钟, MTR 中的FPGA中使用
	//SRAM时钟使能

    /*COMM 模块调用的外设，其相应时钟的开启*/
    //USART1接口时钟使能，COMM中的RS485中使用
    //CAN时钟使能

	RCC->CR        |= 0x00080000;  /* Enable the CSS(CR:bit19) interface clock */

	//AHB最大频率168MHz
	RCC->AHB3ENR    = 0x00000001;

	RCC->AHB1ENR   |= 0x00040000; /* Enable the SRAM (AHB1ENR:bit18)interface clock */
	RCC->AHB1ENR   |= 0x00200000; /* Enable the DMA1 (AHB1ENR:bit21)interface clock */
	RCC->AHB1ENR   |= 0x00400000; /* Enable the DMA2 (AHB1ENR:bit22)interface clock */
	RCC->AHB1ENR   |= 0x00001000; /* Enable the CRC (AHB1ENR:bit12)interface clock */
	RCC->AHB1ENR   |= 0x00000001; /* Enable the GPIOA (AHB1ENR:bit0)interface clock */
	RCC->AHB1ENR   |= 0x00000002; /* Enable the GPIOB (AHB1ENR:bit1)interface clock */
	RCC->AHB1ENR   |= 0x00000004; /* Enable the GPIOC (AHB1ENR:bit2)interface clock */
	RCC->AHB1ENR   |= 0x00000008; /* Enable the GPIOD (AHB1ENR:bit3)interface clock */
	RCC->AHB1ENR   |= 0x00000010; /* Enable the GPIOE (AHB1ENR:bit4)interface clock */
	RCC->AHB1ENR   |= 0x00000020; /* Enable the GPIOF (AHB1ENR:bit5)interface clock */
	RCC->AHB1ENR   |= 0x00000040; /* Enable the GPIOG (AHB1ENR:bit6)interface clock */
	RCC->AHB1ENR   |= 0x00000080; /* Enable the GPIOH (AHB1ENR:bit7)interface clock */
	RCC->AHB1ENR   |= 0x00000100; /* Enable the GPIOI (AHB1ENR:bit8)interface clock */
	//APB1最大频率42MHz
	RCC->APB1ENR   |= 0x00000001; /* Enable the TIM2 (APB1ENR:bit0)interface clock */
	RCC->APB1ENR   |= 0x00000002; /* Enable the TIM3 (APB1ENR:bit1)interface clock */
	RCC->APB1ENR   |= 0x00000004; /* Enable the TIM4 (APB1ENR:bit2)interface clock */
	RCC->APB1ENR   |= 0x00000008; /* Enable the TIM5 (APB1ENR:bit3)interface clock */
	RCC->APB1ENR   |= 0x00000020; /* Enable the TIM7 (APB1ENR:bit5)interface clock */
	RCC->APB1ENR   |= 0x00000800; /* Enable the WWDG (APB1ENR:bit11)interface clock */
	RCC->APB1ENR   |= 0x00004000; /* Enable the SPI2 (APB1ENR:bit14)interface clock */
	RCC->APB1ENR   |= 0x00080000; /* Enable the USART4 (APB1ENR:bit19)interface clock */
	RCC->APB1ENR   |= 0x00200000; /* Enable the I2C1 (APB1ENR:bit21)interface clock */
	RCC->APB1ENR   |= 0x02000000; /* Enable the CAN1 (APB1ENR:bit25)interface clock */
	RCC->APB1ENR   |= 0x10000000; /* Enable the POWER (APB1ENR:bit28)interface clock */
	RCC->APB1ENR   |= 0x20000000; /* Enable the DAC (APB1ENR:bit29)interface clock */
	//APB2最大频率84MHz
	RCC->APB2ENR   |= 0x00000010; /* Enable the USART1 (APB2ENR:bit4)interface clock */
	RCC->APB2ENR   |= 0x00000100; /* Enable the ADC1 (APB2ENR:bit8)interface clock */
	RCC->APB2ENR   |= 0x00000200; /* Enable the ADC2 (APB2ENR:bit9)interface clock */
	RCC->APB2ENR   |= 0x00000400; /* Enable the ADC3 (APB2ENR:bit10)interface clock */
	RCC->APB2ENR   |= 0x00001000; /* Enable the SPI1 (APB2ENR:bit12)interface clock */
	RCC->APB2ENR   |= 0x00004000; /* Enable the SYSCFG (APB2ENR:bit14)interface clock */
	RCC->APB2ENR   |= 0x00010000; /* Enable the TIM9  (APB2ENR:bit16)interface clock */
	RCC->APB2ENR   |= 0x00020000; /* Enable the TIM10  (APB2ENR:bit17)interface clock */
    RCC->APB2ENR   |= 0x00040000; /* Enable the TIM11 (APB2ENR:bit1)interface clock */
	
}

