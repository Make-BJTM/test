/*******************************************************************************
 深圳市汇川技术有限公司 版权所有（C）All rights reserved.                    
 文件名:    PUB_Main.h  
 创建人：   童文邹                 创建日期：XXXX.XX.XX
 修改人：   王治国                 修改日期：2012.03.01
 描述： 
    1.
    2.
 修改记录：  
    XXXX.XX.XX  XXXXXXX
    1.
    2.
********************************************************************************/ 
#ifndef __PUB_MAIN_H
#define __PUB_MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */
#include "PUB_GlobalPrototypes.h"

/* Exported_Constants --------------------------------------------------------*/
/* 不带参数的宏定义 */
//代码运行时间测试
#define CODE_RUN_TIME_TEST   1

//PUB模块全局变量结构体默认值
#define STR_PUB_GLOBALVARIABLE_DEFAULT     {0}

/* Exported_Macros -----------------------------------------------------------*/
/* 带参数的宏定义 */
#define GetSysTime_1MHzClk()    (*TIM4_CNT)      //系统时钟频率1MHz
#if CODE_RUN_TIME_TEST
    #define GetSysTime_168MHzClk()   (*TIM11_CNT)     //系统时钟频率168MHz
#endif
/* Exported_Types ------------------------------------------------------------*/ 
/* 结构体变量类型定义 枚举变量类型定义 */

//PUB模块全局变量结构体
typedef struct{
    Uint8    MTRAlmRst;                     //MTR故障复位 
    Uint8    AllInitDone;                   //初始化完成标志位
    Uint8    SoftInterruptEn;               //主循环功能模块调度标志位 
    Uint32   MainLoop_PRTime;               //主循环程序执行时间
    Uint32   ToqInterrupt_PRTime;           //转矩环中断程序执行时间
    Uint32   PosInterrupt_PRTime;           //位置环中断程序执行时间
    Uint32   MainLoop_PSTime;               //主循环中程序的调度时间
    Uint32   ToqInterrupt_PSTime;           //转矩环中断调度时间   正确时应为62.5us
    Uint32   PosInterrupt_PSTime;           //位置环软中断调度时间 正确时应为250us
    Uint32   McuIqCalTime;                  //MCU转矩指令计算时间 
    Uint16   ToqIntStartTime;               //转矩中断开始时刻 
    Uint16   TestStartTime;                 //测试开始时刻    
    Uint16   TestEndTime;                   //测试结束时刻
    Uint32   SYNCInterrupt_PRTime;           //SYNC中断程序执行时间
    Uint32   SYNCInterrupt_PSTime;           //SYNC中断程序调度时间
    Uint32   IRQInterrupt_PRTime;           //IRQ中断程序执行时间
    Uint32   IRQInterrupt_PSTime;           //IRQ中断程序调度时间
	int32    SYNC2IRQ_DeltaTime;            //与SYNC与IRQ相位
    Uint16   IRQ_TriggerTime;           //IRQ中断程序进入时间
}STR_PUB_GLOBALVARIABLE;



/* Exported_Variables --------------------------------------------------------*/
/* 可供其它文件调用变量的声明 */
extern Uint16 * TIM4_CNT;
extern Uint16 * TIM11_CNT;
extern STR_PUB_GLOBALVARIABLE  STR_PUB_Gvar;   //PUB模块全局变量结构体

/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */ 


#ifdef __cplusplus
}
#endif

#endif /* __PUB_MAIN_H*/

/********************************* END OF FILE *********************************/
