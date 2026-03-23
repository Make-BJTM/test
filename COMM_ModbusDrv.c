/*******************************************************************************
//深圳市汇川技术有限公司 版权所有（C）All rights reserved.
// 文件名: COMM_ModbusDrv.c
// 创建人: 韦水平            创建日期：2011年10月 [V.001]
// 描述: SCI底层驱动C文件  
// 修改历史:
********************************************************************************/ 

/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */
#include "PUB_Main.h"

#include "FUNC_FunCode.h"
#include "FUNC_COMMInterface.h"

#include "COMM_System.h"
#include "COMM_ModbusDrv.h"
#include "COMM_Modbus.h"

#include "stm32f4xx.h"

#include "FUNC_OperEeprom.h"
#include "FUNC_ErrorCode.h" 

/* Private_Constants ---------------------------------------------------------*/
/* 不带参数的宏定义 */
#define USART1_DR_Address       0x40011004
#define USART_TX_DMA_STREAM     DMA2_Stream7

/* Private_Macros ------------------------------------------------------------*/
/* 带参数的宏定义 */
//#define GPIO_WriteRTS485(A)    A?(GPIOD->BSRRL = GPIO_Pin_6):(GPIOD->BSRRH = GPIO_Pin_6)


/* Private_TypesDefinitions --------------------------------------------------*/ 
/* 结构体变量定义 枚举变量定义 */
DMA_InitTypeDef     STR_DMA_InitStructure_USART_TX; 

/* Private_Variables ---------------------------------------------------------*/
/* 变量定义 */
const Uint32 gastrBaudRegData[] =  
{
    2400,            // 0, 2400bps
    4800,            // 1, 4800bps
    9600,            // 2, 9600bps
    19200,           // 3, 19200bps
    38400,           // 4, 38400bps
    57600,           // 5, 57600bps
	115200,          // 6, 115200bps
};

const Uint16 cFRAMEJIANGETIME[7]=     //帧之间间隔，3.5个字节时间
{
    11600,   //11.6ms 2400
    5830,    //5.83ms 4800
    2910,    //2.91ms 9600
    1450,    //1.45ms 19200
    720,     //0.72ms 38400
    480,     //0.48ms 57600
    240,     //0.24ms 115200
};
const Uint16 cFRAMESTOPTIME[7]=     //帧中断时间，1.5个字节时间
{
    5000,  //5ms 2400
    2500,  //2.5ms 4800
    1250,  //1.25ms 9600
    620,   //0.62ms 19200
    310,   //0.31ms 38400
    210,   //0.21ms 57600
    100,   //0.10ms 115200
};


Uint16 wFrame3P5Time;                 //3.5字节对应PWM触发次数         OK
Uint16 wFrame1P5Time;                 //1.5字节对应PWM触发次数         OK

Uint16 gFrame3P5CntStartFlag;         //3.5字节计时启动标志            OK
Uint16 gFrame1P5CntStartFlag;         //1.5字节计时启动标志

Uint16 gFrame3P5Arrive;               //3.5字节计时到达标志            OK
Uint16 gFrame1P5Arrive;               //1.5字节计时到达标志            OK

static volatile Uint8  RcvDealSel;   //Modbus接收处理 0 使能接收中断 1电流环中断查询    OK

/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */
void SciaReadEna(Uint16 EnaFlag);
void SciaSendEna(Uint16 EnaFlag);
//void ActivateSciSend(void);
void CleanUpErrFlag(void);
Uint16 GetSciStatus(void);

void COMM_UpdateSciSet(void);
void COMM_SciInit(void);

void UART_TX_DMA_LowLevelConfig(Uint32 Buffer, Uint32 BufferSize);

void USART1_RcvDealToqInt(void);  //转矩中断中查询接收到的数据
/* Private_Functions ---------------------------------------------------------*/
/* 该文件内部调用的函数的声明 */ 
Static_Inline void Comm_Rcv_Para_Init(void);
Static_Inline void InitSciaGpio(void);
Static_Inline void InitSciaIrt(void);
Static_Inline void SetSciRegs(void);
Static_Inline void RcvIsr(void);

/***********************************************************
  函数名:         // SciaReadEna
  输入:           // EnaFlag:接收允许标志位
  输出:           // 
  子函数:         // 
  描述:           // 根据接收标志位允许或禁止SCIA接收操作
***********************************************************/
void SciaReadEna(Uint16 EnaFlag)
{
    if(EnaFlag)
    {
//        GPIO_WriteRTS485(0);      //打开接收

        USART1->CR1 |=0x00000004; //使能接收
        if(0 == RcvDealSel) USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //使能USART接收完成中断
    }
    else
    {
//        GPIO_WriteRTS485(1);      //关闭接收

        USART1->CR1 &=0xfffffffb; //禁止接收
        USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); //禁止USART接收完成中断
    }
}

/***********************************************************
  函数名:         // SciaSendEna
  输入:           // EnaFlag:发送允许标志位
  输出:           // 
  子函数:         // 
  描述:           // 根据接收标志位允许或禁止SCIA接收操作
***********************************************************/
void SciaSendEna(Uint16 EnaFlag)
{
    if(EnaFlag)
    {
//        GPIO_WriteRTS485(1);      //打开发送

        USART1->CR1 |=0x00000008; //使能发送
    }
    else
    {
//        GPIO_WriteRTS485(0);      //关闭发送

        USART1->CR1 &=0xfffffff7; //禁止发送
        USART_ITConfig(USART1, USART_IT_TXE, DISABLE); //禁止USART发送完成中断 
    }
}

/***********************************************************
  函数名:         // CleanUpErrFlag
  输入:           // 
  输出:           // 
  子函数:         // 
  描述:           // 清除SCIA错误标志
***********************************************************/
void CleanUpErrFlag(void)
{
    volatile Uint16 ReadUsart1DRDummy;

    if(USART1->SR & 0x0000001F)
	{
        DINT;
        ReadUsart1DRDummy = USART1->SR; //低八位为数据
        ReadUsart1DRDummy = USART1->DR; //低八位为数据
        ReadUsart1DRDummy = ReadUsart1DRDummy;
        EINT;

        USART_ClearITPendingBit(USART1, USART_IT_CTS | USART_IT_LBD | USART_IT_TC | USART_IT_RXNE);
	}
}

/***********************************************************
  函数名:         // ActivateSciSend
  输入:           // 
  输出:           // 
  子函数:         // 
  描述:           // 启动SCI传输
***********************************************************/
//void ActivateSciSend(void)
//{
//	USART_SendData(USART1, gaCommSendBuff[0]);//启动发送
//}


/***********************************************************
  函数名:         // GetSciStatus
  输入:           // 
  输出:           // 
  子函数:         // 
  描述:           // 返回SCI状态寄存器值
***********************************************************/
Uint16 GetSciStatus(void)
{
	return USART1->SR;
}


/***********************************************************
  函数名:         // COMM_UpdateSciSet
  输入:           // 
  输出:           // 
  子函数:         // 
  描述:           // 根据功能码设定重新初始化SCIA寄存器
***********************************************************/
void COMM_UpdateSciSet(void)
{
    static Uint16 oldBaud = 5;              //5默认波特率57600
    static Uint16 oldParity = 0;  

	u32 tmpreg = 0x00, apbclock = 0x00;
    u32 integerdivider = 0x00;
    u32 fractionaldivider = 0x00;

	RCC_ClocksTypeDef RCC_ClocksStatus;          //0默认无校验

	Uint16 Baud;
	Uint16 Parity;
	Uint16 DataBuf[2] = {0};

    DataBuf[0] = FunCodeUnion.code.CM_BodeRate;
    DataBuf[1] = FunCodeUnion.code.CM_Parity;

	Baud = DataBuf[0];
	Parity = DataBuf[1];

    if((0 == RcvDealSel) && (6 == Baud) && (1 == FunCodeUnion.code.ModbusRcvDeal))
    {
        //H0C31恢复成0
        FunCodeUnion.code.ModbusRcvDeal = 0;
        SaveToEepromOne(GetCodeIndex(FunCodeUnion.code.ModbusRcvDeal)); 
    } 

    if((1 == RcvDealSel) && (6 == Baud))
    {
        //禁止通讯，必须重新上电        
        Comm_Rcv_Para_Init();
        PostErrMsg(PCHGDWARN);
        if(1 == FunCodeUnion.code.ModbusRcvDeal)
        {
            FunCodeUnion.code.ModbusRcvDeal = 0;
            SaveToEepromOne(GetCodeIndex(FunCodeUnion.code.ModbusRcvDeal));
        }
    }

    if (oldBaud != Baud)
    {		
        Comm_Rcv_Para_Init(); 

/*---------------------------- USART BRR Configuration -----------------------*/
        /* Configure the USART Baud Rate */
        RCC_GetClocksFreq(&RCC_ClocksStatus);

        apbclock = RCC_ClocksStatus.PCLK2_Frequency;
  
        /* Determine the integer part */
        if ((USART1->CR1 & USART_CR1_OVER8) != 0)
        {
            /* Integer part computing in case Oversampling mode is 8 Samples */
            integerdivider = ((25 * apbclock) / (2 * (gastrBaudRegData[Baud])));    
        }
        else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
        {
            /* Integer part computing in case Oversampling mode is 16 Samples */
            integerdivider = ((25 * apbclock) / (4 * (gastrBaudRegData[Baud])));    
        }
        tmpreg = (integerdivider / 100) << 4;

        /* Determine the fractional part */
        fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

        /* Implement the fractional part in the register */
        if ((USART1->CR1 & USART_CR1_OVER8) != 0)
        {
            tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
        }
        else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
        {
            tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
        }
  
        /* Write to USART BRR register */
        USART1->BRR = (uint16_t)tmpreg;
    }
    if (oldParity != Parity)
    {
		Comm_Rcv_Para_Init();

        if(Parity== 0)  //无校验
        {
            tmpreg = USART1->CR2;
			tmpreg &= ((u16)0xCFFF); //清除Bits 13:12  STOP bits
			tmpreg |= (u32)USART_StopBits_2;  //2位停止位
			USART1->CR2 = (u16)tmpreg;

			tmpreg = 0x00;
            tmpreg = USART1->CR1;
			tmpreg &= ((u16)0xEBFF);  //清除 Bit 10 PCE: Parity control enable,Length is 8bits
			tmpreg |= (u32)USART_Parity_No;
            USART_Cmd(USART1, DISABLE); //禁止USART1模块
			USART1->CR1 = (u16)tmpreg;
            USART_Cmd(USART1, ENABLE); //使能USART1模块
        }
        else if(Parity== 1)  //偶校验
        {
	        tmpreg = USART1->CR2;
			tmpreg &= ((u16)0xCFFF); //清除Bits 13:12  STOP bits
			tmpreg |= (u32)USART_StopBits_1;  //1位停止位
			USART1->CR2 = (u16)tmpreg;

			tmpreg = 0x00;
            tmpreg = USART1->CR1;
			tmpreg &= ((u16)0xF9FF);  //清除 Bit 10 PCE: Parity control enable
			tmpreg |= (u32)USART_Parity_Even;
			tmpreg |= (u32)0x1000;
			USART1->CR1 = (u16)tmpreg;
        }
        else if(Parity== 2)  //奇校验
        {
	        tmpreg = USART1->CR2;
			tmpreg &= ((u16)0xCFFF); //清除Bits 13:12  STOP bits
			tmpreg |= (u32)USART_StopBits_1;  //1位停止位
			USART1->CR2 = (u16)tmpreg;

			tmpreg = 0x00;
            tmpreg = USART1->CR1;
			tmpreg &= ((u16)0xF9FF);  //清除 Bit 10 PCE: Parity control enable
			tmpreg |= (u32)USART_Parity_Odd;  
			tmpreg |= (u32)0x1000;
			USART1->CR1 = (u16)tmpreg;
        }
        else
        {
            tmpreg = USART1->CR2;
			tmpreg &= ((u16)0xCFFF); //清除Bits 13:12  STOP bits
			tmpreg |= (u32)USART_StopBits_1;  //1位停止位
			USART1->CR2 = (u16)tmpreg;

			tmpreg = 0x00;
            tmpreg = USART1->CR1;
            tmpreg &= ((u16)0xEBFF);  //清除 Bit 10 PCE: Parity control enable,Length is 8bits
			tmpreg |= (u32)USART_Parity_No;
            USART_Cmd(USART1, DISABLE); //禁止USART1模块
			USART1->CR1 = (u16)tmpreg;
            USART_Cmd(USART1, ENABLE); //使能USART1模块
        }
		Comm_Rcv_Para_Init();
    }

    oldBaud = Baud;
    oldParity = Parity;    
}


/***********************************************************
  函数名:         // COMM_SciInit
  输入:           // 
  输出:           // 
  子函数:         // 
  描述:           // SCIA初始化函数,在Pub初始化中主循环之前调用一次
***********************************************************/
void COMM_SciInit(void)
{
    FunCodeUnion.code.CM_ErrorType = 0;

    Comm_Rcv_Para_Init();

    InitSciaIrt(); //SCIA中断设定
  
    InitSciaGpio(); //SCIA GPIO设定   

	SetSciRegs();

    SciaReadEna(ENABLE); //打开接收
    SciaSendEna(DISABLE); //禁止发送

    CleanUpErrFlag(); //清除错误标志
}

/**************************************************************
  函数名:         // Comm_Rcv_Para_Init
  输入:           //
  输出:           //
  子函数:         //
  描述:           //modbus通讯参数初始化
**************************************************************/  
Static_Inline void Comm_Rcv_Para_Init(void)
{
    wFrame3P5Time = cFRAMEJIANGETIME[FunCodeUnion.code.CM_BodeRate];
    wFrame1P5Time = cFRAMESTOPTIME[FunCodeUnion.code.CM_BodeRate]; 

    DINT;
    gCommRcvLenth = 0;
    EINT;

    gFrame3P5CntStartFlag = c3P5END;
    gFrame1P5CntStartFlag = c1P5END;

    gFrame3P5Arrive = c3P5NOARRIVE;
    gFrame1P5Arrive = c1P5NOARRIVE;

    RcvProcessRdy = cRCVPROCESSNORDY;
    SendProcessRdy = cSENDPROCESSNORDY;
}

/*************************************************
  函数名:         // InitSciaIrt
  输入:           // 
  输出:           // 
  子函数:         // 
  描述:           // SCIA中断初始化
*************************************************/
Static_Inline void InitSciaIrt(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    if(6 == FunCodeUnion.code.CM_BodeRate)
    {
        RcvDealSel = 0;
    }
    else
    {
        RcvDealSel = FunCodeUnion.code.ModbusRcvDeal;    
    }

    if(1 == RcvDealSel) return;

    /*Enable the USART1 Interrupt*/
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART1_IRQ_PreemptionPriority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = USART1_IRQ_SubPriority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);   
}

/*************************************************
  函数名:         // InitSciaGpio
  输入:           // 
  输出:           // 
  子函数:         // 
  描述:           // 初始化GPIO用于SCI通信
*************************************************/
Static_Inline void InitSciaGpio(void)
{	
    GPIO_InitTypeDef GPIO_InitStructure;

    //跟陈云辉沟通，电路板已经有+5V上拉，不进行上下拉配置
    //设置USART1 Tx（PA9)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //PA9配置为AF7 UART1_TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); 

    //设置USART1 Rx（PA10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //PA9配置为AF7 UART1_RX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
}

/***********************************************************
  函数名:         // SetSciRegs
  输入:           // 
  输出:           // 
  子函数:         // 
  描述:           // SCIA寄存器初始化函数
***********************************************************/
Static_Inline void SetSciRegs(void)
{
	Uint16 Baud;
	Uint16 Parity;
	Uint16 DataBuf[2] = {0};

	USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef   USART_ClkInitStructure;  

    DataBuf[0] = FunCodeUnion.code.CM_BodeRate;
    DataBuf[1] = FunCodeUnion.code.CM_Parity;

	Baud = DataBuf[0];
	Parity = DataBuf[1];

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    //USART1->CR1 |= 0x00008000;  //OVER8=0;
	USART_InitStructure.USART_BaudRate = gastrBaudRegData[Baud] ;
    if(Parity== 0)  //无校验
    {
        USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_StopBits = USART_StopBits_2;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    }
    else if(Parity== 1)  //偶校验
    {
		USART_InitStructure.USART_Parity = USART_Parity_Even;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	}
    else if(Parity== 2)  //奇校验
    {
		USART_InitStructure.USART_Parity = USART_Parity_Odd;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    }
    else
    {
        USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_StopBits = USART_StopBits_1; 
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    }
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* Configure the USART1 */
    USART_Init(USART1, &USART_InitStructure);	//初始化USART

    USART_ClkInitStructure.USART_Clock = USART_Clock_Disable;
    USART_ClkInitStructure.USART_CPOL = USART_CPOL_Low;
    USART_ClkInitStructure.USART_CPHA = USART_CPHA_2Edge;
    USART_ClkInitStructure.USART_LastBit = USART_LastBit_Disable;
    USART_ClockInit(USART1, &USART_ClkInitStructure); //初始化时钟  

	USART_Cmd(USART1, ENABLE); //使能USART1模块

    USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
    DMA_DeInit((DMA_Stream_TypeDef*) DMA2_Stream7_BASE);

    DMA_Cmd(USART_TX_DMA_STREAM, DISABLE);
    //配置USART1  DMA2 STREAM7 channel4 配置为TX
    STR_DMA_InitStructure_USART_TX.DMA_Channel = DMA_Channel_4; 
	STR_DMA_InitStructure_USART_TX.DMA_PeripheralBaseAddr = USART1_DR_Address;
	STR_DMA_InitStructure_USART_TX.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	STR_DMA_InitStructure_USART_TX.DMA_MemoryInc = DMA_MemoryInc_Enable;
	STR_DMA_InitStructure_USART_TX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	STR_DMA_InitStructure_USART_TX.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	STR_DMA_InitStructure_USART_TX.DMA_Mode = DMA_Mode_Normal; 
	STR_DMA_InitStructure_USART_TX.DMA_Priority = DMA_Priority_Low;
	STR_DMA_InitStructure_USART_TX.DMA_FIFOMode = DMA_FIFOMode_Disable;        
	STR_DMA_InitStructure_USART_TX.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	STR_DMA_InitStructure_USART_TX.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(USART_TX_DMA_STREAM, &STR_DMA_InitStructure_USART_TX);
}

/*******************************************************************************
  函数名: 
  输入:    
  输出:    
  子函数:         
  描述: 
********************************************************************************/
void USART1_IRQHandler(void)
{
	// by huangxin201804 _3	  导入黄金飞修改拔网线死机bug
	volatile Uint16 tmp = 0;

	if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)  
   	{   
     	tmp = USART1->SR;								   	
		tmp = USART1->DR;
   	}    
	// by huangxin201804 _3	  导入黄金飞修改拔网线死机bug

	//接收中断
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		RcvIsr();	
	}
	else
	{
		Comm_Rcv_Para_Init();
	}

    USART_ClearITPendingBit(USART1, USART_IT_CTS | USART_IT_LBD | USART_IT_TC | USART_IT_RXNE);
}

/*******************************************************************************
  函数名: 
  输入:    
  输出:    
  子函数:         
  描述: 
********************************************************************************/
void USART1_RcvDealToqInt(void)
{
    volatile Uint16 ReadUsart1DRDummy;

    G_COMM_ModbusSchedual();

    if(0 == RcvDealSel) return;

    if(USART1->SR & 0x00000020)
    {
        RcvIsr();
         
        //清除SCIA错误标志
        DINT;
        ReadUsart1DRDummy = USART1->SR; //低八位为数据
        ReadUsart1DRDummy = USART1->DR; //低八位为数据
        ReadUsart1DRDummy = ReadUsart1DRDummy; 
        EINT; 
    }
    else if(USART1->SR & 0x0000001F)
	{
        //清除SCIA错误标志
        DINT;
        ReadUsart1DRDummy = USART1->SR; //低八位为数据
        ReadUsart1DRDummy = USART1->DR; //低八位为数据
        ReadUsart1DRDummy = ReadUsart1DRDummy;
        EINT;

        USART_ClearITPendingBit(USART1, USART_IT_CTS | USART_IT_LBD | USART_IT_TC | USART_IT_RXNE);
	}
}

/*************************************************
  函数名:         // RcvIsr
  输入:           // 
  输出:           // 
  子函数:         // 
  描述:           // SCIA接收中断处理函数
*************************************************/
Static_Inline void RcvIsr(void)
{
	if((gFrame3P5Arrive==c3P5ARRIVE) || (gFrame1P5Arrive==c1P5ARRIVE))
	{
        gCommRcvLenth=0;
	}

    AuxFunCodeUnion.code.OS_UartStatus |= 2;    //接收中 BIT1 = 1

	if(gCommRcvLenth >= 259)
	{
		gCommRcvLenth = 259;
	}

    gaCommRcvBuff[gCommRcvLenth] = USART_ReceiveData(USART1) & 0x00FF; //低八位为数据

	gFrame3P5CntStartFlag= c3P5START;
	gFrame1P5CntStartFlag= c1P5START;
	gFrame3P5Arrive=c3P5NOARRIVE;
    gFrame1P5Arrive=c1P5NOARRIVE;

	gCommRcvLenth++;
}


/*******************************************************************************
  函数名: 
  输入:    
  输出:    
  子函数:         
  描述: 
********************************************************************************/
void UART_TX_DMA_LowLevelConfig(Uint32 Buffer, Uint32 BufferSize)
{
    DMA_Cmd(USART_TX_DMA_STREAM, DISABLE);   /* Enable the DMA Tx Channel */
    STR_DMA_InitStructure_USART_TX.DMA_Memory0BaseAddr = (Uint32)Buffer;
    STR_DMA_InitStructure_USART_TX.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    STR_DMA_InitStructure_USART_TX.DMA_BufferSize = (Uint32)BufferSize;
    DMA_Init(USART_TX_DMA_STREAM, &STR_DMA_InitStructure_USART_TX);
    DMA_ClearITPendingBit(USART_TX_DMA_STREAM,DMA_IT_FEIF7 | DMA_IT_DMEIF7 | DMA_IT_TEIF7 | DMA_IT_HTIF7\
                           | DMA_IT_TCIF7);

    DMA_Cmd(USART_TX_DMA_STREAM, ENABLE);   /* Enable the DMA Tx Channel */
}

/********************************* END OF FILE *********************************/
