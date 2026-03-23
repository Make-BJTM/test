/*******************************************************************************
//深圳市汇川技术有限公司 版权所有（C）All rights reserved.
// 文件名: COMM_ModbusDrv.h
// 创建人: 韦水平            创建日期：2011年10月 [V.001]
// 描述: SCI底层驱动头文件  
// 修改记录:
********************************************************************************/

#ifndef COMM_MODBUSDRV_H
#define COMM_MODBUSDRV_H

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
extern void SciaReadEna(Uint16 EnaFlag);
extern void SciaSendEna(Uint16 EnaFlag);
//extern void ActivateSciSend(void);
extern void CleanUpErrFlag(void);
extern Uint16 GetSciStatus(void);
extern void COMM_UpdateSciSet(void);
extern void UART_TX_DMA_LowLevelConfig(Uint32 Buffer, Uint32 BufferSize);
extern void G_COMM_ModbusSchedual(void);

#ifdef __cplusplus
}
#endif 

#endif /* __COMM_MODBUSDRV_H */

/********************************* END OF FILE *********************************/

