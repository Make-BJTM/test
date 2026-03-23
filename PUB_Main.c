/*******************************************************************************
 深圳市汇川技术有限公司 版权所有（C）All rights reserved.
 文件名:    PUB_Main.c
 创建人：童文邹                创建日期：10.07.01
 修改人：李浩,王治国,朱祥华    修改日期：11.10.08
 描述： main函数文件

 与FPGA直连IO管脚说明：V0001.5 
 1. FPGA发送中断信号    PB0   ->  FPGA管脚c3     电路图网络INT1
 2. 20位编码器原点信号  PB11  ->  FPGA管脚B14    电路图网络IO3 
 3. DI8输入             PD3   ->  FPGA管脚K15    电路图网络IO9
 4. DI9输入             PD2   ->  FPGA管脚J16    电路图网络IO8
 6. 直线编码器A信号     PA0   ->  FPGA管脚C14    电路图网络IO4   
 7. 直线编码器B信号     PA1   ->  FPGA管脚F13    电路图网络IO5   
 7. 直线编码器Z信号     PB6   ->  FPGA管脚N15    电路图网络IO1 


空闲IO3 IO6 IO7 INT2,在FUNC_GPIODriver.c文件中配置为输入模式，下拉
如果以上四个管脚 需要重新配置寄存器
PA8   ->  FPGA管脚F16    电路图网络IO2   该网络不能使用
PC8   ->  FPGA管脚F15    电路图网络INT2
PB10  ->  FPGA管脚A15    电路图网络IO6
PC13  ->  FPGA管脚P15    电路图网络IO7 
********************************************************************************/

/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */
#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dbgmcu.h"
#include "stm32f4xx_syscfg.h"
#include "PUB_RccDriver.h"

#include "PUB_Main.h"
#include "FUNC_System.h"
#include "MTR_System.h" 
#include "CANopen_Home.h"
#include "ECT_Probe.h"
#include "CANopen_DeviceCtrl.h"
#include "FUNC_GlobalVariable.h"
#include "ECT_CSP.h"
#include "FUNC_FunCode.h"

#include "FUNC_GPIODriver.h"

/* Private_Constants ---------------------------------------------------------*/
/* 不带带参数的宏定义 */
//#define PUB_SYSCLK_FREQ_72MHz     72000000
#define PUB_ENABLE                1
#define PUB_DISABLE               0

/* Private_Macros ------------------------------------------------------------*/
/* 带参数的宏定义 */

/* Private_TypesDefinitions --------------------------------------------------*/ 
/* 结构体变量定义 枚举变量定义 */
STR_PUB_GLOBALVARIABLE  STR_PUB_Gvar = STR_PUB_GLOBALVARIABLE_DEFAULT;   //PUB模块全局变量结构体

/* Private_Variables ---------------------------------------------------------*/
/* 文件内变量定义 */

/* Private_Variables ---------------------------------------------------------*/
/* 可供其它文件调用的变量定义 */

//指针变量初始化地址对应的变量
Uint32 PUB_Null_PointVar32Init = 0;
Uint16 PUB_Null_PointVar16Init = 0;

Uint16 * TIM4_CNT = (Uint16 *)(0x40000000 + 0x0800 + 0x24);
#if CODE_RUN_TIME_TEST
    Uint16 * TIM11_CNT = (Uint16 *)(0x40010000 + 0x4800 + 0x24);
#endif
/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */


/* Private_Functions ---------------------------------------------------------*/
/* 该文件内部调用的函数的声明 */ 
Static_Inline void Pub_PeripheralConfig_RST(void);
Static_Inline void Init_TIM2_UnRealTimer(void);
Static_Inline void Init_TIM4AndTIM11_ProgramTimeTest(void);
Static_Inline void Init_PeripheralClocks(void);
Static_Inline void PUB_Interrupt_RST(void);
Static_Inline void ZFalling_Handler(void);

/*******************************************************************************
  函数名:  
  输入:    
  输出:    
  子函数:         
  描述:  
********************************************************************************/
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// @ 进main()之前的函数 ，在start.s中调用 tongwenzou20100727
//功能:1 初始化系统时钟，包含HSE的启动PLL FLASH 调用SystemInit (void)
// @brief  Setup the microcontroller system
//         Initialize the Embedded Flash Interface, the PLL and update the
//         SystemCoreClock variable.
//2.读取省线编码器UVW信息
//++++++++++原来的清零的功能在这里不必要了++++++++++++++++++++++++++++++

void startB4main(void)
{
    volatile int32 * DataAddr = (volatile int32 *) 0x20000000;
    int32 i = 0;

    //关闭所有的外设            
    RCC->AHB1RSTR |= 0xFFFFFFFF;  //RCC AHB1 peripheral reset register (RCC_AHB1RSTR)
    RCC->AHB2RSTR |= 0xFFFFFFFF;  //RCC AHB2 peripheral reset register (RCC_AHB2RSTR)
    RCC->AHB3RSTR |= 0xFFFFFFFF;  //RCC AHB3 peripheral reset register (RCC_AHB3RSTR)
    RCC->APB1RSTR |= 0xFFFFFFFF;  //RCC APB1 peripheral reset register (RCC_APB1RSTR)
    RCC->APB2RSTR |= 0xFFFFFFFF;  //RCC APB2 peripheral reset register (RCC_APB2RSTR)
    RCC->AHB1RSTR &= 0;  //RCC AHB1 peripheral reset register (RCC_AHB1RSTR)
    RCC->AHB2RSTR &= 0;  //RCC AHB2 peripheral reset register (RCC_AHB2RSTR)
    RCC->AHB3RSTR &= 0;  //RCC AHB3 peripheral reset register (RCC_AHB3RSTR)
    RCC->APB1RSTR &= 0;  //RCC APB1 peripheral reset register (RCC_APB1RSTR)
    RCC->APB2RSTR &= 0;  //RCC APB2 peripheral reset register (RCC_APB2RSTR)

    SysTick->CTRL = 0;

    EXTI->IMR = 0;
    EXTI->EMR = 0;

    NVIC->ICER[0] = 0xFFFFFFFF;
    NVIC->ICER[1] = 0xFFFFFFFF;
    NVIC->ICER[2] = 0xFFFFFFFF;
    NVIC->ICER[3] = 0xFFFFFFFF;
    NVIC->ICER[4] = 0xFFFFFFFF;
    NVIC->ICER[5] = 0xFFFFFFFF;
    NVIC->ICER[6] = 0xFFFFFFFF;
    NVIC->ICER[7] = 0xFFFFFFFF;  

    //Boot区使用Ram清零
    for(i=0;i<0x5000;i+=4)
    {
        *DataAddr = 0;
        DataAddr += 4;
    }

    PUB_SystemInit();

    DBGMCU_APB1PeriphConfig(DBGMCU_TIM2_STOP | DBGMCU_TIM3_STOP | DBGMCU_TIM4_STOP, ENABLE);
    DBGMCU_APB1PeriphConfig(DBGMCU_WWDG_STOP | DBGMCU_I2C1_SMBUS_TIMEOUT , ENABLE);
    DBGMCU_APB2PeriphConfig(DBGMCU_TIM11_STOP, ENABLE);
    DBGMCU_Config(DBGMCU_STOP, ENABLE);
}

/*******************************************************************************
  函数名:  
  输入:    
  输出:    
  子函数:         
  描述:  
********************************************************************************/
int32 main(void)
{
    static Uint16 Schedule_EnFlag = 0;
    static Uint16 Schedule_OckTime = 0;
    static Uint16 TimeLatch = 0;
    Uint16 SystemTime = 0;

    STR_PUB_Gvar.AllInitDone = 0;

    //PUB模块外设初始化
    Pub_PeripheralConfig_RST();

    //FUNC模块外设初始化
    FUNC_PeripheralConfig_RST();

    //MTR模块外设初始化
    MTR_PeripheralConfig_RST();

    //FUNC模块参数初始化1 主要是功能码部分
    FUNC_Parameter_Frist_RST();

    //MTR模块参数初始化1 
    MTR_Parameter_Frist_RST();

    //FUNC模块参数初始化2
    FUNC_Parameter_Second_RST();

    //MTR模块参数初始化2
    MTR_Parameter_Second_RST();

    //FUNC模块中断函数初始化
    FUNC_Interrupt_RST();

    //MTR模块中断函数初始化
    MTR_Interrupt_RST();

    //PUB模块中断函数初始化
    PUB_Interrupt_RST();
                                    
    //看门狗初始化
    FUNC_InitAndEnableWatchDog();

    STR_PUB_Gvar.AllInitDone = 1;

    SystemTime = GetSysTime_1MHzClk();
    Schedule_OckTime = SystemTime - 1001;

    while(1)
    {
        //主循环调度处理  
        SystemTime = GetSysTime_1MHzClk();
        if( ((Uint16)(SystemTime - Schedule_OckTime) > 1000) && 
            ((Uint16)(SystemTime - Schedule_OckTime) < 60000) )
        {
            Schedule_OckTime += 1000;
            Schedule_EnFlag = 1;
        }
        else if((Uint16)(SystemTime - Schedule_OckTime) > 60000)  //实际调度过慢 复位
        {
            Schedule_OckTime = SystemTime;
            Schedule_EnFlag = 1;
        }

        //主循环函数处理
        if(Schedule_EnFlag == 1)
        {
            Schedule_EnFlag = 0;

            // 主循环程序调度时间测试(每隔多长时间调度一次)
            SystemTime = GetSysTime_1MHzClk();
            STR_PUB_Gvar.MainLoop_PSTime = SystemTime - TimeLatch;
            STR_PUB_Gvar.MainLoop_PSTime = STR_PUB_Gvar.MainLoop_PSTime & 0xFFFF;
            TimeLatch = SystemTime;

            MTR_MainLoop();                 //-----------主循环MTR执行程序 

            FUNC_MainLoop();                //-----------主循环FUNC执行程序 

            /* 主循环程序执行时间测试 */
            STR_PUB_Gvar.MainLoop_PRTime = GetSysTime_1MHzClk() - TimeLatch;
            STR_PUB_Gvar.MainLoop_PRTime = STR_PUB_Gvar.MainLoop_PRTime & 0xFFFF;
        }
    }
}

/*******************************************************************************
  函数名:  
  输入:    
  输出:    
  子函数:         
  描述:  //用于三环调度的中断
********************************************************************************/
void EXTI0_IRQHandler()   // FPGA中断  EXTI0_IRQHandler
{
    static Uint16 ToqIntScheduleTime = 0;         //转矩中断调度时间测试变量

    //中断开始时刻测试
    STR_PUB_Gvar.ToqIntStartTime = GetSysTime_1MHzClk();  

    //转矩(FPGA)中断执行程序
    //if(EXTI_GetITStatus(EXTI_Line0) != RESET)     //确定是来自0管脚的中断
    if (((EXTI->PR & EXTI_Line0) != 0) && ((EXTI->IMR & EXTI_Line0) != 0))
    {
        //清中断标志位
        //EXTI_ClearITPendingBit(EXTI_Line0);
        EXTI->PR = EXTI_Line0;

        if(STR_PUB_Gvar.AllInitDone == 0) return;

        FUNC_AdcStart_ToqInterrupt();

        //中断开始到MTR_GetPara_ToqInterrupt()函数运行完共用时间 720/120 us
        MTR_GetPara_ToqInterrupt(); 

        //速度模式：启用斜坡函数时1228/120us， 没启用斜坡函数时1156/120us
        //转矩模式：996/120us
        FUNC_CmdProcess_ToqInterrupt(); 

        //速度模式：运行到FPGA转矩指令赋值时间2228/120us
        //转矩模式：运行到FPGA转矩指令赋值时间726/120us
        //速度模式：MTR_ReguControl_ToqInterrupt()运行时间2770/120us
        //转矩模式：MTR_ReguControl_ToqInterrupt()运行时间1246/120us 
        MTR_ReguControl_ToqInterrupt();

        //示波器采样、自适应滤波器启动时，运行时间234/120us
        FUNC_AuxFunc_ToqInterrupt();

        ZFalling_Handler();

        if(STR_PUB_Gvar.SoftInterruptEn == 1)
        {
            STR_PUB_Gvar.SoftInterruptEn = 0;
            EXTI->SWIER |= EXTI_Line1;
        }
    }

    /*转矩(FPGA)中断时间测试*/
    STR_PUB_Gvar.ToqInterrupt_PSTime = STR_PUB_Gvar.ToqIntStartTime - ToqIntScheduleTime;   //转矩中断调度时间测试
    STR_PUB_Gvar.ToqInterrupt_PSTime = STR_PUB_Gvar.ToqInterrupt_PSTime & 0xFFFF;
    ToqIntScheduleTime = STR_PUB_Gvar.ToqIntStartTime;

    /*转矩(FPGA)中断时间测试*/
    STR_PUB_Gvar.ToqInterrupt_PRTime = GetSysTime_1MHzClk() - STR_PUB_Gvar.ToqIntStartTime;  //转矩中断运行时间测试
    STR_PUB_Gvar.ToqInterrupt_PRTime = STR_PUB_Gvar.ToqInterrupt_PRTime & 0xFFFF;
}

/*******************************************************************************
  函数名:  
  输入:    
  输出:    
  子函数:         
  描述:  
********************************************************************************/
void EXTI1_IRQHandler(void)
{
    static Uint16 PosSoftIntScheduleTime = 0;     //位置环软中断调度时间测试变量
    Uint16 PosIntTimeTest = 0;                    //位置环软中断运行时间测试变量
    
    /*位置环软中断时间测试*/
    PosIntTimeTest = GetSysTime_1MHzClk();        //位置环软中断运行时间测试
    STR_PUB_Gvar.PosInterrupt_PSTime = PosIntTimeTest - PosSoftIntScheduleTime;
    STR_PUB_Gvar.PosInterrupt_PSTime = STR_PUB_Gvar.PosInterrupt_PSTime & 0xFFFF;
    PosSoftIntScheduleTime = PosIntTimeTest;

    /*位置环软中断执行程序*/    
    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        GPIO_Disable_POS();
        EXTI_ClearITPendingBit(EXTI_Line1);
        
        FUNC_PostionControl_PosInterrupt();
        MTR_PostionControl_PosInterrupt();
		GPIO_Enable_POS();
    }

    /*位置环软中断时间测试*/
    STR_PUB_Gvar.PosInterrupt_PRTime = GetSysTime_1MHzClk() - PosIntTimeTest;   //位置环软中断运行时间测试
    STR_PUB_Gvar.PosInterrupt_PRTime = STR_PUB_Gvar.PosInterrupt_PRTime & 0xFFFF;
}

/*******************************************************************************
  函数名:  
  输入:    
  输出:    
  子函数:         
  描述:  //非实时处理部分计数中断，和三环中断分离是为了保证FPGA死机情况下DSP继续运行
********************************************************************************/
void TIM2_IRQHandler(void)
{

    /*辅助中断执行程序*/
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);       //Clear TIM1 update interrupt
        
//        //电机模块的辅助中断处理主要软件启动ST芯片的ADC和读取ADC采样值
//        MTR_System_AuxInterrupt();

        //功能模块的辅助中断处理主要处理芯片上电复位时显示rESEt, 复位完成后监控FPGA中断是否正常
        FUNC_System_AuxInterrupt();

        if( STR_PUB_Gvar.AllInitDone == 1 )  //初始化完成
        {
            TIM_Cmd(TIM2, DISABLE);                         //不使能定时器
            TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);     //不使能定时器更新中断
        }
    }
}
/*******************************************************************************
  函数名:  
  输入:    
  输出:    
  子函数:         
  描述:  Pub 外设初始化(包括系统时钟初始化，程序测试时间定时器初始化，以及TIM1定时器初始化等) 
********************************************************************************/ 
Static_Inline void Pub_PeripheralConfig_RST()
{
    Init_PeripheralClocks();      //初始化系统以及外设相关时钟

    Init_TIM2_UnRealTimer();      //定时器1辅助中断的初始化,用以配置非实时调度周期以及用于显示RESET //给非实时调用部分面板，通信等提供时基, 中断频率和电流环中断一致！                                 

    Init_TIM4AndTIM11_ProgramTimeTest();  //用于时间测试用的TIM4 TIM11
}

/*******************************************************************************
  函数名:  
  输入:    
  输出:    
  子函数:         
  描述:  初始化CPU，系统，外设时钟
********************************************************************************/
Static_Inline void Init_PeripheralClocks()
{
    PUB_PeripheralClockConfig();
}

/*******************************************************************************
  函数名:  
  输入:    
  输出:    
  子函数:         
  描述:  TIM1 辅助中断定时器初始化
********************************************************************************/ 
Static_Inline void Init_TIM2_UnRealTimer()
{
    NVIC_InitTypeDef           NVIC_InitStructure_main;    //定义TIM1中断的中断控制器的结构体变量
    TIM_TimeBaseInitTypeDef    TIM2_TimeBaseStructure;     //定义TIM1中断的中断定时器的结构体变量

    DINT;                                              //关总中断

    TIM_DeInit(TIM2);
    TIM_TimeBaseStructInit(&TIM2_TimeBaseStructure);
    // 确定定时器2的时基 
    TIM2_TimeBaseStructure.TIM_Prescaler         = 0x1;
    TIM2_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM2_TimeBaseStructure.TIM_Period            = (Uint16)((42000000L / 1000) - 1);
    TIM2_TimeBaseStructure.TIM_ClockDivision     = 0;
    TIM_TimeBaseInit(TIM2, &TIM2_TimeBaseStructure);

    TIM_Cmd(TIM2, ENABLE);     //使能定时器 

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);        // 使能定时器更新中断 

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);   //优先组配置为1位主，可抢占性;3位亚优先级，非强占性

    //配置TIM2的中断优先级
    NVIC_InitStructure_main.NVIC_IRQChannel = TIM2_IRQn;       //更新事件中断
    NVIC_InitStructure_main.NVIC_IRQChannelPreemptionPriority = TIM2_UP_IRQ_PreemptionPriority;
    NVIC_InitStructure_main.NVIC_IRQChannelSubPriority = TIM2_UP_IRQ_SubPriority;       //0~7
    NVIC_InitStructure_main.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure_main);

    EINT;                                              //开总中断
}

/*******************************************************************************
  函数名:  
  输入:    
  输出:    
  子函数:         
  描述:  用于程序时间测试的计时器初始化
********************************************************************************/
Static_Inline void Init_TIM4AndTIM11_ProgramTimeTest()
{
    TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;   //定义TIM6程序时间计时器的结构体变量

    TIM_DeInit(TIM4);    //复位定时器4
    //系统时钟  1MHz
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 65535;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_Cmd(TIM4, ENABLE);   // TIM4 counter enable

#if CODE_RUN_TIME_TEST
    TIM_DeInit(TIM11);   //复位定时器11
    //函数运算时间计算用计数器  
    TIM_TimeBaseStructure.TIM_Prescaler =  0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 65535;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInit(TIM11, &TIM_TimeBaseStructure);

    TIM_Cmd(TIM11, ENABLE);   // TIM11 counter enable
#endif
}
/*******************************************************************************
  函数名:  
  输入:    
  输出:    
  子函数:         
  描述: 
********************************************************************************/
void PUB_Interrupt_RST(void)
{
    GPIO_InitTypeDef    GPIO_InitStruct;            //定义FPGA中断的GPIO管脚的初始化结构体变量
    NVIC_InitTypeDef    NVIC_InitStructure_EXTI;    //定义FPGA中断的中断控制器的结构体变量
    EXTI_InitTypeDef    EXTI_InitStructureFPGA;     //定义FPGA中断的中断定时器的结构体变量

    //FPGA管脚中断配置   
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_0;	        //对中断管脚PB0的配置，FPGA发出的16K中断
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStruct); 

    //用于配置FPGA送过来的中断的中断定时器将PB0管脚作为 EXTI Line0 的中断来源
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource0);   //中断对应的管脚

    EXTI_InitStructureFPGA.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructureFPGA.EXTI_LineCmd = ENABLE;
    EXTI_InitStructureFPGA.EXTI_Line    = EXTI_Line0;             //PB0 ->中断
    EXTI_InitStructureFPGA.EXTI_Trigger = EXTI_Trigger_Falling;   //下降沿中断  
    EXTI_Init(&EXTI_InitStructureFPGA);                           //初始化配置

    //先清除已有的中断标志先
    EXTI_ClearITPendingBit(EXTI_Line0);                           //PB0 由FPGA发送过来的中断 

    //配置中断来源 选择芯片没有的管脚，使用了中断EXTI，却没有分配硬件中断管脚，SYSCFG默认分配第一个管脚为硬件中断信号，此时，为避免软中断被硬中断替换，必须将硬件中断信号分配到一个无效的管脚上。但若第一个管脚接地，则不存在此问题。
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOK, GPIO_PinSource1); 
    
    //软件中断 用于位置调节的中断
    EXTI_InitStructureFPGA.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructureFPGA.EXTI_LineCmd = ENABLE;
    EXTI_InitStructureFPGA.EXTI_Line = EXTI_Line1;                  //
    EXTI_InitStructureFPGA.EXTI_Trigger= EXTI_Trigger_Falling;      //
    EXTI_Init(&EXTI_InitStructureFPGA);   //初始化配置

    EXTI_ClearITPendingBit(EXTI_Line1);       //清除软中断

    //由FPGA送来的中断配置其优先级
    NVIC_InitStructure_EXTI.NVIC_IRQChannel = EXTI0_IRQn;         //用于速度转矩调节的中断
    NVIC_InitStructure_EXTI.NVIC_IRQChannelPreemptionPriority = EXTI0_UP_IRQ_PreemptionPriority;
    NVIC_InitStructure_EXTI.NVIC_IRQChannelSubPriority = EXTI0_UP_IRQ_SubPriority;
    NVIC_InitStructure_EXTI.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure_EXTI);

    NVIC_InitStructure_EXTI.NVIC_IRQChannel = EXTI1_IRQn;       //用于位置调节的中断
    NVIC_InitStructure_EXTI.NVIC_IRQChannelPreemptionPriority = EXTI1_IRQ_PreemptionPriority;
    NVIC_InitStructure_EXTI.NVIC_IRQChannelSubPriority = EXTI1_IRQ_SubPriority;
    NVIC_InitStructure_EXTI.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure_EXTI);

    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_11;         //对中断管脚PB11的配置, Z信号接入端
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    EXTI_InitStructureFPGA.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructureFPGA.EXTI_LineCmd = ENABLE;
    EXTI_InitStructureFPGA.EXTI_Line    = EXTI_Line11;             //PB11 ->Z中断
    EXTI_InitStructureFPGA.EXTI_Trigger = EXTI_Trigger_Falling;   //下降沿中断

    // 配置中断对应的管脚
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource11); 
    // 初始化Z信号中断配置
    EXTI_Init(&EXTI_InitStructureFPGA);
    // 清除已有的Z信号中断标志
    EXTI_ClearITPendingBit(EXTI_Line11);
}

/*******************************************************************************
  函数名: void EXTI9_5_IRQHandler() 
  输  入:           
  输  出:   
  子函数:                                       
  描  述: 处理Z脉冲中断
********************************************************************************/
Static_Inline void ZFalling_Handler(void)
{
    if(EXTI_GetITStatus(EXTI_Line11) != RESET)  //确定是来自PB7管脚的中断
	{
	    EXTI_ClearITPendingBit(EXTI_Line11); 

	    ZeroIndexISR();       //执行Z中断处理函数
	    
#if ECT_ENABLE_SWITCH
        CanopenZeroIndexISR();

        MCZindex();

        TouchProbeZeroIndexISR();
#endif

        AngInt_ZPosLatch();

        ZPosErrDeal();
	}
}

/*******************************************************************************
  函数名:  
  输  入:           
  输  出:   
  子函数:                                       
  描  述: 
********************************************************************************/
void SysTick_Handler(void)
{
}
/********************************* END OF FILE *********************************/
