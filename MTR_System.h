/*******************************************************************************
 深圳市汇川技术有限公司 版权所有（C）All rights reserved.                    
 文件名:	MTR_System.h                                                           
 创建人：朱祥华            创建日期：2011.11.09 
 修改人：XX                修改日期：XX.XX.XX 
 描述：  
    1.电机模块系统调度全局变量声明
    2.
 修改记录：  
    1. xx.xx.xx      XX  
       变更内容： xxxxxxxxxxx
    2. xx.xx.xx      XX
       变更内容： xxxxxxxxxxx
********************************************************************************/
#ifndef MTR_SYSTEM_H
#define MTR_SYSTEM_H


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */  

/* Exported_Constants --------------------------------------------------------*/
/* 宏定义 常数类*/


/* Exported_Macros -----------------------------------------------------------*/
/* 宏定义 函数类 */	

/* Exported_Types ------------------------------------------------------------*/ 
/* 结构体变量类型定义 枚举变量类型定义 */ 


/* Exported_Variables --------------------------------------------------------*/
/* 可供其它文件调用变量的声明 */


/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */ 
//初始化相关调度程序
extern void MTR_PeripheralConfig_RST(void);
extern void MTR_Parameter_Frist_RST(void);
extern void MTR_Parameter_Second_RST(void);
extern void MTR_Interrupt_RST(void);
extern void MTR_UpdateSysFreqAndPrd(void);
//中断相关调度程序
extern void MTR_GetPara_ToqInterrupt(void);
extern void MTR_ReguControl_ToqInterrupt(void);
extern void MTR_System_AuxInterrupt(void);
extern void MTR_PostionControl_PosInterrupt(void);//MTR位置环控制中断处理
//主循环调度程序
extern void MTR_MainLoop(void);


extern void AngInt_ZPosLatch(void);
extern void AbsRom_InitDeal(void);
extern void ZPosErrDeal(void);

#ifdef __cplusplus
}
#endif

#endif  /* MTR_System.h */	

/********************************* END OF FILE *********************************/

