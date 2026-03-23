/*******************************************************************************
 深圳市汇川技术有限公司 版权所有（C）All rights reserved.                    
 文件名:    COMM_System.h  
 创建人：   XXXXXX                 创建日期：XXXX.XX.XX                     
 修改人：   XXXXXX                 修改日期：XXXX.XX.XX 
 描述： 
    1.
    2.
 修改记录：  
    XXXX.XX.XX  XXXXXXX
    1.
    2.
********************************************************************************/ 
#ifndef __COMM_SYSTEM_H
#define __COMM_SYSTEM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* 引用头文件 */
#include "PUB_GlobalPrototypes.h"

/* Exported_Constants --------------------------------------------------------*/
/* 不带参数的宏定义 */

/* Exported_Macros -----------------------------------------------------------*/
/* 带参数的宏定义 */

/* Exported_Types ------------------------------------------------------------*/ 
/* 结构体变量类型定义 枚举变量类型定义 */ 

/* Exported_Variables --------------------------------------------------------*/
/* 可供其它文件调用变量的声明 */


/* Exported_Functions --------------------------------------------------------*/
/* 可供其它文件调用的函数的声明 */
//Modbus调度函数
extern void COMM_ModbusDeal_MainLoop(void);
extern void COMM_SciInit(void);

//CAN调度函数
//extern void CANopenCycleRun(Uint16 addr, Uint16 baud);
//extern void CanlinkFun( Uint16 sCANlinkBaud, Uint16 sCANlinkAddr );

void USART1_RcvDealToqInt(void);  //转矩中断中查询接收到的数据

#ifdef __cplusplus
}
#endif

#endif /* __COMM_SYSTEM_H */

/********************************* END OF FILE *********************************/
